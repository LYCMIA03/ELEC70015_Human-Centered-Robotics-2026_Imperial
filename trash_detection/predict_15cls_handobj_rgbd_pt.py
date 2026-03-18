# -*- coding: utf-8 -*-
"""
15cls + Hand-Object + Person 全 .pt 版：Ultralytics YOLO + PyTorch CUDA（无 ONNX）。

与 predict_15cls_handobj_rgbd.py 逻辑相同，三模型均为 .pt，默认 device=cuda。
依赖：pip install ultralytics torch torchvision opencv-python pyorbbecsdk（Jetson 用 jp6/cu126 的 torch）

运行：
  export ULTRALYTICS_AUTOINSTALL=0
  python trash_detection/predict_15cls_handobj_rgbd_pt.py --stream-enable
  python trash_detection/predict_15cls_handobj_rgbd_pt.py --weights-15cls weight_new/best.pt ...
"""
import argparse
import gc
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native"
if "--headless" in sys.argv and "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
import cv2
import torch

try:
    from pyorbbecsdk import (
        Pipeline, Config, OBSensorType, OBFormat, OBError,
        OBStreamType, OBFrameAggregateOutputMode, AlignFilter,
        FormatConvertFilter, OBConvertFormat,
    )
except (ModuleNotFoundError, ImportError) as e:
    print("未找到 pyorbbecsdk:", e)
    sys.exit(1)

try:
    import torchvision.ops as _tv_ops
    from ultralytics.utils.nms import TorchNMS

    def _safe_nms(boxes, scores, iou_threshold):
        return TorchNMS.nms(boxes, scores, iou_threshold)

    _tv_ops.nms = _safe_nms
except Exception:
    pass

from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor

ROOT = Path(__file__).resolve().parent
TRASH_ROOT = ROOT
HANDOBJ_ROOT = ROOT.parent / "handobj_detection"
WEIGHTS_HANDOBJ_DIR = ROOT.parent / "weight_handobj_40" / "weight_hand"
WEIGHTS_15CLS_PT_CANDIDATES = [
    TRASH_ROOT / "weight_new" / "best.pt",
    TRASH_ROOT / "weights3" / "epoch80.pt",
    TRASH_ROOT / "weights3" / "epoch90.pt",
    TRASH_ROOT / "weights2" / "best.pt",
]
WEIGHTS_HANDOBJ_PT_CANDIDATES = [
    HANDOBJ_ROOT / "weight" / "best.pt",
    WEIGHTS_HANDOBJ_DIR / "best.pt",
]
WEIGHTS_PERSON_DEFAULT = "yolov8m.pt"

CLASS_NAMES_15 = [
    "Metal", "Cardboard", "Glass", "Paper", "Plastic", "Tetra",
    "Apple", "Apple-core", "Apple-peel", "Bread", "Orange", "Orange-peel",
    "Pear", "Vegetable", "Human",
]
NUM_WASTE_CLASSES = 14
COCO_PERSON_CLASS_ID = 0
METAL_CLS_BIAS = 0.4
AGNOSTIC_NMS = False
NMS_IOU_OVERLAP = 0.65

DEPTH_FX, DEPTH_FY = 500.0, 500.0
DEPTH_CX, DEPTH_CY = 319.5, 287.5
COLOR_RES_MAP = {"720p": (1280, 720), "1080p": (1920, 1080), "1440p": (2560, 1440)}
MAX_DEPTH_M = 5.0


class UdpTargetSender:
    def __init__(self, host, port, frame_id, source, max_rate_hz):
        self.host = host
        self.port = int(port)
        self.frame_id = frame_id
        self.source = source
        self.min_dt = 1.0 / max(max_rate_hz, 0.1)
        self.last_sent_t = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_xyz(self, xyz, kind):
        if xyz is None:
            return False
        now = time.time()
        if now - self.last_sent_t < self.min_dt:
            return False
        payload = {
            "stamp": now,
            "frame_id": self.frame_id,
            "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
            "source": self.source,
            "kind": kind,
        }
        self.sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), (self.host, self.port))
        self.last_sent_t = now
        return True


class MjpegStreamHolder:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None

    def set_frame(self, bgr_frame, quality=85):
        if bgr_frame is None or bgr_frame.size == 0:
            return
        _, buf = cv2.imencode(".jpg", bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        with self._lock:
            self._jpeg = buf.tobytes()

    def get_jpeg(self):
        with self._lock:
            return self._jpeg


def _make_stream_handler(stream_holder):
    boundary = b"frame"

    class StreamHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path == "/" or self.path == "/stream" or self.path.startswith("/stream?"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s" % boundary.decode())
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.end_headers()
                try:
                    while True:
                        jpeg = stream_holder.get_jpeg()
                        if jpeg:
                            self.wfile.write(b"--" + boundary + b"\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.033)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                return
            self.send_response(404)
            self.end_headers()

    return StreamHandler


class MjpegStreamServer:
    def __init__(self, bind_addr="0.0.0.0", port=8765, stream_holder=None):
        self.holder = stream_holder or MjpegStreamHolder()
        self.port = port
        self.server = HTTPServer((bind_addr, port), _make_stream_handler(self.holder))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()
        return self.holder


def _first_existing_pt(candidates):
    for p in candidates:
        if p.exists() and p.suffix.lower() == ".pt":
            return p.resolve()
    return None


def _color_frame_to_bgr(color_frame):
    if color_frame is None:
        return None
    h, w = color_frame.get_height(), color_frame.get_width()
    data = np.asarray(color_frame.get_data())
    fmt = color_frame.get_format()
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return data.reshape(h, w, 3).copy()
    if fmt == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    conv_map = {
        OBFormat.I420: OBConvertFormat.I420_TO_RGB888,
        OBFormat.YUYV: OBConvertFormat.YUYV_TO_RGB888,
        OBFormat.NV12: OBConvertFormat.NV12_TO_RGB888,
        OBFormat.NV21: OBConvertFormat.NV21_TO_RGB888,
        OBFormat.UYVY: OBConvertFormat.UYVY_TO_RGB888,
    }
    if fmt not in conv_map:
        return None
    flt = FormatConvertFilter()
    flt.set_format_convert_format(conv_map[fmt])
    rgb = flt.process(color_frame)
    if rgb is None:
        return None
    img = np.asarray(rgb.get_data()).reshape(rgb.get_height(), rgb.get_width(), 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def depth_pixel_to_xyz_m(px, py, depth_mm):
    if depth_mm is None or depth_mm <= 0:
        return None
    z_m = float(depth_mm) / 1000.0
    x_m = (px - DEPTH_CX) * z_m / DEPTH_FX
    y_m = (py - DEPTH_CY) * z_m / DEPTH_FY
    return (x_m, y_m, z_m)


def sample_depth_at_box_center(depth, x1, y1, x2, y2, h, w):
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    d = int(depth[cy, cx]) if 0 <= cy < h and 0 <= cx < w else 0
    if d > 0:
        return d
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < h and 0 <= nx < w:
                d = int(depth[ny, nx])
                if d > 0:
                    return d
    return None


def _dist_sq_xyz(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def _install_metal_class_bias(bias=0.1):
    if getattr(DetectionPredictor, "_metal_bias_patched", False):
        return
    _orig = DetectionPredictor.postprocess

    def _postprocess(self, preds, img, orig_imgs, **kwargs):
        if preds is not None and getattr(preds, "dim", lambda: 0)() >= 3 and preds.shape[-2] >= 5:
            names = getattr(self.model, "names", None)
            if names is not None:
                first = names.get(0) if isinstance(names, dict) else (names[0] if len(names) > 0 else None)
                if first == "Metal":
                    preds = preds.clone()
                    preds[..., 4, :] += bias
        return _orig(self, preds, img, orig_imgs, **kwargs)

    DetectionPredictor.postprocess = _postprocess
    DetectionPredictor._metal_bias_patched = True


def _boxes_with_xyz_from_results(results, depth_mm, depth_h, depth_w, conf_threshold, max_range_mm):
    out = []
    if results.boxes is None or depth_mm is None or depth_mm.ndim != 2:
        return out
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    for xyxy, conf, cls in zip(
        results.boxes.xyxy.cpu().numpy(),
        results.boxes.conf.cpu().numpy(),
        results.boxes.cls.cpu().numpy(),
    ):
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
        d_mm = sample_depth_at_box_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm)
        if xyz is None:
            continue
        out.append((x1, y1, x2, y2, xyz, d_mm))
    return out


def get_nearest_waste_and_person_xyz_from_two_models(
    results_waste, results_person, names_list, depth_mm, depth_h, depth_w,
    conf_threshold=0.25, max_range_mm=5000,
):
    waste_items = _boxes_with_xyz_from_results(
        results_waste, depth_mm, depth_h, depth_w, conf_threshold, max_range_mm
    )
    person_items = _boxes_with_xyz_from_results(
        results_person, depth_mm, depth_h, depth_w, conf_threshold, max_range_mm
    )
    all_wastes = [(t[0], t[1], t[2], t[3], t[4]) for t in waste_items]
    all_persons = [(t[0], t[1], t[2], t[3], t[4]) for t in person_items]
    if not waste_items:
        return None, None, all_persons, all_wastes
    nearest_waste = min(waste_items, key=lambda t: t[5])
    waste_xyz = nearest_waste[4]
    if not person_items:
        return waste_xyz, None, all_persons, all_wastes
    nearest_person = min(person_items, key=lambda t: _dist_sq_xyz(t[4], waste_xyz))
    return waste_xyz, nearest_person[4], all_persons, all_wastes


def _get_class_names(model):
    names = getattr(model, "names", None)
    if names is None:
        return {}
    return names if isinstance(names, dict) else dict(enumerate(names))


def _find_class_ids(names_dict, *keywords):
    ids = []
    for idx, name in names_dict.items():
        n = (name or "").lower()
        for kw in keywords:
            if kw in n:
                ids.append(idx)
                break
    return ids


def _box_center_inside(box_a, box_b):
    cx = (box_b[0] + box_b[2]) / 2
    cy = (box_b[1] + box_b[3]) / 2
    x1, y1, x2, y2 = box_a
    return x1 <= cx <= x2 and y1 <= cy <= y2


def _iou_rect(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    inter = (ix2 - ix1) * (iy2 - iy1)
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    if area_a <= 0 or area_b <= 0:
        return 0.0
    return inter / min(area_a, area_b)


def get_holding_person_boxes_and_xyz(
    results, names_dict, depth_mm, depth_h, depth_w,
    conf_threshold=0.35, max_range_mm=5000,
    person_keywords=("person", "human"),
    hand_keywords=("hand", "left_hand", "right_hand"),
    object_keywords=("object", "obj", "thing", "item"),
):
    if results.boxes is None or len(results.boxes) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return []
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    max_range_mm = min(max_range_mm, int(MAX_DEPTH_M * 1000))
    person_ids = _find_class_ids(names_dict, *person_keywords)
    hand_ids = _find_class_ids(names_dict, *hand_keywords)
    obj_ids = _find_class_ids(names_dict, *object_keywords)
    has_person = len(person_ids) > 0
    has_hand_or_obj = len(hand_ids) > 0 or len(obj_ids) > 0
    boxes_list = []
    for xyxy, conf, cls in zip(
        results.boxes.xyxy.cpu().numpy(),
        results.boxes.conf.cpu().numpy(),
        results.boxes.cls.cpu().numpy(),
    ):
        cls_id = int(cls)
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
        d_mm = sample_depth_at_box_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm)
        if xyz is None:
            continue
        boxes_list.append((x1, y1, x2, y2, cls_id, float(conf), d_mm, xyz))
    if not boxes_list:
        return []
    if not has_person or not has_hand_or_obj:
        out = [((t[0], t[1], t[2], t[3]), t[7], t[5]) for t in boxes_list]
        out.sort(key=lambda x: x[1][2])
        return out
    person_boxes = [t for t in boxes_list if t[4] in person_ids]
    hand_obj_boxes = [t for t in boxes_list if t[4] in hand_ids or t[4] in obj_ids]
    holding = []
    for (x1, y1, x2, y2, _, conf, d_mm, xyz) in person_boxes:
        person_rect = (x1, y1, x2, y2)
        for h in hand_obj_boxes:
            h_rect = (h[0], h[1], h[2], h[3])
            if _iou_rect(person_rect, h_rect) > 0.05 or _box_center_inside(person_rect, h_rect):
                holding.append(((x1, y1, x2, y2), xyz, conf))
                break
    holding.sort(key=lambda x: x[1][2])
    return holding


def depth_vis(depth_uint16, max_mm=3000):
    d = np.asarray(depth_uint16, dtype=np.float32)
    if d.ndim != 2 or d.size == 0:
        return None
    d = np.clip(d, 0, max_mm)
    mx = float(d.max())
    if mx <= 0:
        return np.zeros((*d.shape, 3), dtype=np.uint8)
    return cv2.applyColorMap((d / mx * 255).astype(np.uint8), cv2.COLORMAP_JET)


def main():
    p = argparse.ArgumentParser(description="15cls + Hand-Object RGB-D（全 .pt + PyTorch CUDA）")
    p.add_argument("--weights-15cls", default=None, help="15 类垃圾 .pt，默认自动找 weight_new/best.pt 等")
    p.add_argument("--weights-handobj", default=None, help="Hand-Object .pt，默认 handobj_detection/weight/best.pt")
    p.add_argument("--weights-person", default=WEIGHTS_PERSON_DEFAULT, help="人检测 .pt，默认 yolov8m.pt")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1080p")
    p.add_argument("--max-depth", type=float, default=MAX_DEPTH_M)
    p.add_argument("--no-depth-window", action="store_true")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--print-xyz", action="store_true")
    p.add_argument(
        "--device",
        choices=["cuda", "cpu", "auto"],
        default="cuda",
        help="PyTorch 推理设备，默认 cuda（要求 GPU 可用）",
    )
    p.add_argument("--udp-enable", action="store_true")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=16031)
    p.add_argument("--udp-frame-id", default="camera_link")
    p.add_argument("--udp-rate", type=float, default=10.0)
    p.add_argument("--stream-enable", action="store_true")
    p.add_argument("--stream-bind", default="0.0.0.0")
    p.add_argument("--stream-port", type=int, default=8765)
    p.add_argument("--stream-quality", type=int, default=85)
    args = p.parse_args()

    if args.device == "auto":
        yolo_device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        yolo_device = args.device
    if yolo_device == "cuda" and not torch.cuda.is_available():
        print("错误: 请求 CUDA 但 torch.cuda.is_available() 为 False，请安装 GPU 版 PyTorch 或加 --device cpu")
        return 1
    if yolo_device == "cuda":
        print("推理设备: cuda | GPU:", torch.cuda.get_device_name(0))
    else:
        print("推理设备: cpu")

    if args.weights_15cls:
        w15 = Path(args.weights_15cls)
        if not w15.is_absolute():
            w15 = (TRASH_ROOT / w15) if (TRASH_ROOT / w15).exists() else Path(args.weights_15cls).resolve()
    else:
        w15 = _first_existing_pt(WEIGHTS_15CLS_PT_CANDIDATES)
    if w15 is None or not Path(w15).exists() or Path(w15).suffix.lower() != ".pt":
        print("未找到 15 类 .pt，请指定 --weights-15cls，例如 trash_detection/weight_new/best.pt")
        return 1
    w15 = Path(w15).resolve()

    if args.weights_handobj:
        whand = Path(args.weights_handobj)
        if not whand.is_absolute():
            whand = (WEIGHTS_HANDOBJ_DIR / whand) if (WEIGHTS_HANDOBJ_DIR / whand).exists() else (HANDOBJ_ROOT / whand)
            if not whand.exists():
                whand = Path(args.weights_handobj)
                if not whand.is_absolute():
                    whand = ROOT.parent / whand
    else:
        whand = _first_existing_pt(WEIGHTS_HANDOBJ_PT_CANDIDATES)
    if whand is None or not Path(whand).exists() or Path(whand).suffix.lower() != ".pt":
        print("未找到 Hand-Object .pt，请指定 --weights-handobj，例如 handobj_detection/weight/best.pt")
        return 1
    whand = Path(whand).resolve()

    weights_person = str(args.weights_person).strip()
    if not weights_person.lower().endswith(".pt"):
        print("person 须为 .pt 权重:", weights_person)
        return 1

    model_waste = YOLO(str(w15))
    print("15cls (.pt):", w15)
    model_handobj = YOLO(str(whand))
    print("Hand-Object (.pt):", whand)
    if yolo_device == "cuda" and torch.cuda.is_available():
        torch.cuda.empty_cache()
        gc.collect()
    model_person = YOLO(weights_person)
    print("Person (.pt):", weights_person)
    names_15 = getattr(model_waste, "names", None)
    _ = getattr(model_handobj, "names", None)

    if isinstance(names_15, dict):
        names_list = [names_15.get(i, "cls%d" % i) for i in range(max(names_15.keys()) + 1)]
    else:
        names_list = list(names_15) if names_15 else CLASS_NAMES_15
    names_handobj = _get_class_names(model_handobj)

    _install_metal_class_bias(METAL_CLS_BIAS)
    trash_class_ids = list(range(NUM_WASTE_CLASSES))
    max_range_mm = int((args.max_depth or MAX_DEPTH_M) * 1000)
    print("三模型 .pt PyTorch %s: 15cls(0-%d) + person + handobj" % (yolo_device, NUM_WASTE_CLASSES - 1))

    if yolo_device == "cuda":
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            model_waste.predict(dummy, imgsz=args.imgsz, verbose=False, device=yolo_device)
            model_person.predict(dummy, imgsz=args.imgsz, verbose=False, device=yolo_device)
            model_handobj.predict(dummy, imgsz=args.imgsz, verbose=False, device=yolo_device)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print("YOLO 预热完成")
        except Exception as e:
            print("预热失败:", e)

    res_wh = COLOR_RES_MAP.get(args.color_res, (1920, 1080))
    cw, ch = res_wh[0], res_wh[1]
    pipeline = Pipeline()
    config = Config()
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile = profile_list.get_video_stream_profile(cw, ch, OBFormat.RGB, 30)
        except OBError:
            color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        depth_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        config.enable_stream(depth_list.get_default_video_stream_profile())
        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
    except Exception as e:
        print("配置 RGB-D 失败:", e)
        return 1
    try:
        pipeline.start(config)
    except Exception as e:
        print("启动 RGB-D 失败:", e)
        return 1
    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    time.sleep(1.5)

    headless = args.headless or args.stream_enable
    print("RGB-D", args.color_res, "无界面 Ctrl+C" if headless else "按 q 退出")
    sender = None
    if args.udp_enable:
        sender = UdpTargetSender(
            args.udp_host, args.udp_port, args.udp_frame_id,
            "combined_15cls_handobj_rgbd_pt", args.udp_rate,
        )
        print("UDP %s:%s" % (args.udp_host, args.udp_port))

    stream_holder = None
    if args.stream_enable:
        try:
            srv = MjpegStreamServer(bind_addr=args.stream_bind, port=args.stream_port)
            stream_holder = srv.start()
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                ip = args.stream_bind
            print("MJPEG http://%s:%d/" % (ip, args.stream_port))
        except OSError as e:
            print("视频流失败:", e)

    if not headless:
        cv2.namedWindow("15cls+HandObj RGB-D (PT/CUDA)", cv2.WINDOW_NORMAL)
        if not args.no_depth_window:
            cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)

    fps_start = time.perf_counter()
    fps_count = 0
    yolo_sum = 0.0

    try:
        while True:
            frames = None
            for _ in range(15):
                frames = pipeline.wait_for_frames(500)
                if frames and frames.get_color_frame() and frames.get_depth_frame():
                    break
            if frames is None:
                if not headless and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                if headless:
                    time.sleep(0.02)
                continue

            frames = align_filter.process(frames)
            if frames is not None:
                frames = frames.as_frame_set()
            color_frame = frames.get_color_frame() if frames else None
            depth_frame = frames.get_depth_frame() if frames else None
            color_img = _color_frame_to_bgr(color_frame) if color_frame else None
            depth_copy = None
            if depth_frame is not None:
                scale = depth_frame.get_depth_scale()
                d = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
                    (depth_frame.get_height(), depth_frame.get_width()))
                depth_copy = (d.astype(np.float32) * scale).astype(np.uint16)

            if color_img is None or depth_copy is None or depth_copy.ndim != 2:
                continue

            dh, dw = depth_copy.shape[0], depth_copy.shape[1]
            if (color_img.shape[0], color_img.shape[1]) != (dh, dw):
                frame = cv2.resize(color_img, (dw, dh), interpolation=cv2.INTER_LINEAR)
            else:
                frame = color_img.copy()

            t0 = time.perf_counter()
            results_waste = model_waste.predict(
                frame, conf=args.conf, imgsz=args.imgsz, iou=NMS_IOU_OVERLAP,
                agnostic_nms=AGNOSTIC_NMS, classes=trash_class_ids,
                verbose=False, device=yolo_device,
            )[0]
            results_person = model_person.predict(
                frame, conf=args.conf, imgsz=args.imgsz, iou=NMS_IOU_OVERLAP,
                classes=[COCO_PERSON_CLASS_ID], verbose=False, device=yolo_device,
            )[0]
            results_handobj = model_handobj.predict(
                frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device,
            )[0]
            yolo_sum += time.perf_counter() - t0

            waste_xyz, person_near_waste_xyz, all_persons, all_wastes = get_nearest_waste_and_person_xyz_from_two_models(
                results_waste, results_person, names_list, depth_copy, dh, dw, args.conf, max_range_mm
            )
            holding_list = get_holding_person_boxes_and_xyz(
                results_handobj, names_handobj, depth_copy, dh, dw,
                conf_threshold=args.conf, max_range_mm=max_range_mm,
            )
            holding_xyz = holding_list[0][1] if holding_list else None

            target_xyz = None
            target_kind = None
            target_box = None
            if holding_xyz is not None and person_near_waste_xyz is not None:
                if holding_xyz[2] <= person_near_waste_xyz[2]:
                    target_xyz, target_kind = holding_xyz, "holding"
                    if holding_list:
                        target_box = holding_list[0][0]
                else:
                    target_xyz, target_kind = person_near_waste_xyz, "person"
                    if all_persons and waste_xyz is not None:
                        nearest_p = min(all_persons, key=lambda t: _dist_sq_xyz(t[4], waste_xyz))
                        target_box = (nearest_p[0], nearest_p[1], nearest_p[2], nearest_p[3])
            elif holding_xyz is not None:
                target_xyz, target_kind = holding_xyz, "holding"
                if holding_list:
                    target_box = holding_list[0][0]
            elif person_near_waste_xyz is not None:
                target_xyz, target_kind = person_near_waste_xyz, "person"
                if all_persons and waste_xyz is not None:
                    nearest_p = min(all_persons, key=lambda t: _dist_sq_xyz(t[4], waste_xyz))
                    target_box = (nearest_p[0], nearest_p[1], nearest_p[2], nearest_p[3])

            if sender is not None:
                sender.send_xyz(target_xyz, target_kind or "person")
            if args.print_xyz and target_xyz is not None:
                print("XYZ: %.3f %.3f %.3f [%s]" % (target_xyz[0], target_xyz[1], target_xyz[2], target_kind or "—"), flush=True)

            fps_count += 1
            if time.perf_counter() - fps_start >= 1.0 and fps_count > 0:
                loop_fps = fps_count / (time.perf_counter() - fps_start)
                yolo_fps = 1.0 / (yolo_sum / fps_count) if yolo_sum > 0 else 0
                line = "FPS: %.1f YOLO: %.1f" % (loop_fps, yolo_fps)
                if target_xyz is not None:
                    line += " | %.2f,%.2f,%.2f [%s]" % (target_xyz[0], target_xyz[1], target_xyz[2], target_kind or "—")
                print(line, flush=True)
                fps_start = time.perf_counter()
                fps_count = 0
                yolo_sum = 0.0

            need_vis = not headless or stream_holder is not None
            if need_vis:
                vis = frame.copy()
                if results_waste.boxes is not None:
                    for xyxy, conf, cls in zip(
                        results_waste.boxes.xyxy.cpu().numpy(),
                        results_waste.boxes.conf.cpu().numpy(),
                        results_waste.boxes.cls.cpu().numpy(),
                    ):
                        if int(cls) >= NUM_WASTE_CLASSES or float(conf) < args.conf:
                            continue
                        x1, y1, x2, y2 = map(int, xyxy)
                        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        cv2.putText(vis, names_list[int(cls)][:8], (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                if results_person.boxes is not None:
                    for xyxy, conf, cls in zip(
                        results_person.boxes.xyxy.cpu().numpy(),
                        results_person.boxes.conf.cpu().numpy(),
                        results_person.boxes.cls.cpu().numpy(),
                    ):
                        if float(conf) < args.conf:
                            continue
                        x1, y1, x2, y2 = map(int, xyxy)
                        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 255, 0), 1)
                if target_box is not None and target_xyz is not None:
                    x1, y1, x2, y2 = map(int, target_box)
                    color = (0, 255, 255) if target_kind == "holding" else (255, 165, 0)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(vis, "Nearest: %s (%.2f,%.2f,%.2f)m" % (
                        target_kind or "—", target_xyz[0], target_xyz[1], target_xyz[2]),
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if stream_holder is not None:
                    stream_holder.set_frame(vis, args.stream_quality)
            if not headless:
                if need_vis:
                    cv2.imshow("15cls+HandObj RGB-D (PT/CUDA)", vis)
                if not args.no_depth_window and depth_copy is not None:
                    dv = depth_vis(depth_copy)
                    if dv is not None:
                        cv2.imshow("Depth", dv)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        if not headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
