# -*- coding: utf-8 -*-
"""
15cls + Hand-Object + Person 全 ONNX 版（无 PyTorch/Ultralytics），避免 CUBLAS 显存冲突。

三模型均用 onnxruntime 推理；预处理与 NMS 用 numpy/OpenCV。
依赖：pip install onnxruntime-gpu opencv-python pyorbbecsdk numpy
运行前导出 person：yolo export model=yolov8m.pt format=onnx imgsz=640 → yolov8m.onnx
  python trash_detection/predict_15cls_handobj_rgbd_onnx_only.py --stream-enable --onnx-ep cuda
  # TensorRT（FP16+引擎缓存，显存/速度更好；每个 ONNX 首次编译要很久，引擎在 /tmp/ort_trt_15cls_handobj_rgbd）
  python trash_detection/predict_15cls_handobj_rgbd_onnx_only.py --stream-enable --onnx-ep trt
  python trash_detection/predict_15cls_handobj_rgbd_onnx_only.py --stream-enable --onnx-ep trt --person-gpu
"""
import argparse
import json
import os
import socket
import sys
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

if "--headless" in sys.argv and "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
import cv2

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
    import onnxruntime as ort
except ImportError:
    print("请安装 onnxruntime 或 onnxruntime-gpu")
    sys.exit(1)

# ---------- 路径与常量 ----------
ROOT = Path(__file__).resolve().parent
ONNX_DIR = ROOT / "onnx"
WEIGHTS_15CLS = ONNX_DIR / "15cls_640.onnx"
WEIGHTS_HANDOBJ = ONNX_DIR / "handobj_640.onnx"
# person：优先 onnx/yolov8m_640.onnx；否则 onnx/yolov8m.onnx；否则项目根 yolov8m.onnx（yolo export 默认保存位置）
WEIGHTS_PERSON_CANDIDATES = (
    ONNX_DIR / "yolov8m_640.onnx",
    ONNX_DIR / "yolov8m.onnx",
    ROOT.parent / "yolov8m.onnx",
)


def _resolve_person_onnx(cli_path):
    if cli_path:
        p = Path(cli_path)
        if p.is_absolute() and p.exists():
            return p
        for base in (Path.cwd(), ROOT.parent, ROOT):
            cand = (base / p).resolve()
            if cand.exists():
                return cand
        return p if p.is_absolute() else (ROOT / p).resolve()
    for c in WEIGHTS_PERSON_CANDIDATES:
        if c.exists():
            return c
    return WEIGHTS_PERSON_CANDIDATES[0]

CLASS_NAMES_15 = [
    "Metal", "Cardboard", "Glass", "Paper", "Plastic", "Tetra",
    "Apple", "Apple-core", "Apple-peel", "Bread", "Orange", "Orange-peel",
    "Pear", "Vegetable", "Human",
]
NUM_WASTE_CLASSES = 14
COCO_PERSON_CLASS_ID = 0
NMS_IOU = 0.65
IMG_SIZE = 640

DEPTH_FX, DEPTH_FY = 500.0, 500.0
DEPTH_CX, DEPTH_CY = 319.5, 287.5
COLOR_RES_MAP = {"720p": (1280, 720), "1080p": (1920, 1080), "1440p": (2560, 1440)}
MAX_DEPTH_M = 5.0


# ---------- 结果包装（兼容原脚本 _boxes_with_xyz / get_holding） ----------
class _Boxes:
    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32) if xyxy is not None else np.zeros((0, 4), dtype=np.float32)
        self.conf = np.asarray(conf, dtype=np.float32) if conf is not None else np.zeros(0, dtype=np.float32)
        self.cls = np.asarray(cls, dtype=np.float32) if cls is not None else np.zeros(0, dtype=np.float32)


class _Results:
    def __init__(self, boxes):
        self.boxes = boxes


# ---------- Letterbox & 预处理 ----------
def letterbox(bgr, new_shape=(640, 640), stride=32):
    h, w = bgr.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    else:
        img = bgr.copy()
    dh = new_shape[0] - nh
    dw = new_shape[1] - nw
    dh = dh // 2, dh - dh // 2
    dw = dw // 2, dw - dw // 2
    img = cv2.copyMakeBorder(img, dh[0], dh[1], dw[0], dw[1], cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return img, r, (dw[0], dh[0], nw, nh)


def preprocess_bgr_to_onnx(bgr_padded):
    # BGR HWC -> RGB NCHW, [0,255] -> [0,1]
    rgb = cv2.cvtColor(bgr_padded, cv2.COLOR_BGR2RGB)
    blob = rgb.astype(np.float32) / 255.0
    blob = np.transpose(blob, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)
    return blob.astype(np.float32)


# ---------- YOLOv8 ONNX 输出解码 + NMS ----------
def decode_yolo_output(output, conf_thresh, iou_thresh, num_classes, class_filter=None):
    # output: (1, 4+nc, 8400)
    if output is None or output.size == 0:
        return _Results(_Boxes(None, None, None))
    raw = np.squeeze(output, axis=0)
    if raw.ndim != 2:
        return _Results(_Boxes(None, None, None))
    # (4+nc, 8400) -> (8400, 4+nc)
    raw = raw.T
    cx, cy, w, h = raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3]
    x1 = cx - w / 2
    y1 = cy - h / 2
    x2 = cx + w / 2
    y2 = cy + h / 2
    scores = raw[:, 4:]
    if scores.shape[1] != num_classes:
        num_classes = scores.shape[1]
    cls_ids = np.argmax(scores, axis=1)
    confs = np.max(scores, axis=1)
    mask = confs >= conf_thresh
    if class_filter is not None:
        mask &= np.isin(cls_ids, class_filter)
    x1, y1, x2, y2 = x1[mask], y1[mask], x2[mask], y2[mask]
    confs = confs[mask]
    cls_ids = cls_ids[mask].astype(np.float32)
    if len(confs) == 0:
        return _Results(_Boxes(np.zeros((0, 4)), np.zeros(0), np.zeros(0)))
    # NMS
    indices = cv2.dnn.NMSBoxes(
        list(zip(x1.tolist(), y1.tolist(), (x2 - x1).tolist(), (y2 - y1).tolist())),
        confs.tolist(), conf_thresh, iou_thresh,
    )
    if len(indices) == 0:
        return _Results(_Boxes(np.zeros((0, 4)), np.zeros(0), np.zeros(0)))
    idx = np.array(indices).flatten()
    xyxy = np.stack([x1[idx], y1[idx], x2[idx], y2[idx]], axis=1)
    return _Results(_Boxes(xyxy, confs[idx], cls_ids[idx]))


def scale_boxes_to_original(xyxy, scale, pad_left, pad_top, orig_w, orig_h, in_h, in_w):
    # 从 letterbox 坐标还原到原图
    xyxy = xyxy.copy()
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad_left) / scale
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad_top) / scale
    xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, orig_w)
    xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, orig_h)
    return xyxy


# ---------- ONNX 推理封装 ----------
def _providers_for_ep(ep, trt_workspace_mb=256, trt_fp16=True):
    """构建 InferenceSession 的 providers 列表。"""
    avail = ort.get_available_providers()
    if ep == "cpu":
        return ["CPUExecutionProvider"]
    if ep == "cuda" and "CUDAExecutionProvider" in avail:
        return [("CUDAExecutionProvider", {"device_id": 0}), "CPUExecutionProvider"]
    if ep == "trt":
        cuda_fallback = [("CUDAExecutionProvider", {"device_id": 0}), "CPUExecutionProvider"]
        if "TensorrtExecutionProvider" not in avail:
            print("无 TensorrtExecutionProvider，TRT 请求退回 CUDA", flush=True)
            return cuda_fallback
        import tempfile
        cache_dir = os.path.join(tempfile.gettempdir(), "ort_trt_15cls_handobj_rgbd")
        os.makedirs(cache_dir, exist_ok=True)
        ws = max(32, int(trt_workspace_mb)) * 1024 * 1024
        trt_opts = {
            "device_id": 0,
            "trt_fp16_enable": bool(trt_fp16),
            "trt_engine_cache_enable": True,
            "trt_engine_cache_path": cache_dir,
            "trt_max_workspace_size": str(int(ws)),
        }
        return [
            ("TensorrtExecutionProvider", trt_opts),
            ("CUDAExecutionProvider", {"device_id": 0}),
            "CPUExecutionProvider",
        ]
    if "CUDAExecutionProvider" in avail:
        return [("CUDAExecutionProvider", {"device_id": 0}), "CPUExecutionProvider"]
    return ["CPUExecutionProvider"]


def create_onnx_session(path, ep, trt_workspace_mb=256, trt_fp16=True):
    providers = _providers_for_ep(ep, trt_workspace_mb, trt_fp16)
    try:
        return ort.InferenceSession(str(path), providers=providers)
    except Exception as e:
        print("ONNX 加载失败 %s: %s" % (path, e))
        raise


def run_detect(session, blob, input_name=None, output_name=None):
    if input_name is None:
        input_name = session.get_inputs()[0].name
    if output_name is None:
        output_name = session.get_outputs()[0].name
    out = session.run([output_name], {input_name: blob})
    return out[0]


# ---------- UDP / MJPEG（与原脚本一致） ----------
class UdpTargetSender:
    def __init__(self, host, port, frame_id, source, max_rate_hz):
        self.port = int(port)
        self.host = host
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
            "stamp": now, "frame_id": self.frame_id,
            "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
            "source": self.source, "kind": kind,
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

    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/stream"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s" % boundary.decode())
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
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

    return _Handler


class MjpegStreamServer:
    def __init__(self, bind_addr="0.0.0.0", port=8765, stream_holder=None):
        self.holder = stream_holder or MjpegStreamHolder()
        self.port = port
        self.server = HTTPServer((bind_addr, port), _make_stream_handler(self.holder))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()
        return self.holder


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


def _boxes_with_xyz_from_results(results, depth_mm, depth_h, depth_w, conf_threshold, max_range_mm):
    out = []
    if results.boxes is None or len(results.boxes.xyxy) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return out
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    for i in range(len(results.boxes.xyxy)):
        xyxy = results.boxes.xyxy[i]
        conf = results.boxes.conf[i]
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


def _find_class_ids(names_dict, *keywords):
    ids = []
    for idx, name in names_dict.items():
        n = (name or "").lower()
        for kw in keywords:
            if kw in n:
                ids.append(idx)
                break
    return ids


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


def _box_center_inside(box_a, box_b):
    cx = (box_b[0] + box_b[2]) / 2
    cy = (box_b[1] + box_b[3]) / 2
    x1, y1, x2, y2 = box_a
    return x1 <= cx <= x2 and y1 <= cy <= y2


def get_holding_person_boxes_and_xyz(
    results, names_dict, depth_mm, depth_h, depth_w,
    conf_threshold=0.35, max_range_mm=5000,
    person_keywords=("person", "human"),
    hand_keywords=("hand", "left_hand", "right_hand"),
    object_keywords=("object", "obj", "thing", "item"),
):
    if results.boxes is None or len(results.boxes.xyxy) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return []
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    max_range_mm = min(max_range_mm, int(MAX_DEPTH_M * 1000))
    person_ids = _find_class_ids(names_dict, *person_keywords)
    hand_ids = _find_class_ids(names_dict, *hand_keywords)
    obj_ids = _find_class_ids(names_dict, *object_keywords)
    has_person = len(person_ids) > 0
    has_hand_or_obj = len(hand_ids) > 0 or len(obj_ids) > 0
    boxes_list = []
    for i in range(len(results.boxes.xyxy)):
        xyxy = results.boxes.xyxy[i]
        conf = results.boxes.conf[i]
        cls_id = int(results.boxes.cls[i])
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


# Hand-Object 类别名（常见导出）
HANDOBJ_NAMES = {
    0: "person", 1: "left_hand", 2: "right_hand", 3: "object",
}


def main():
    p = argparse.ArgumentParser(description="15cls + Hand-Object + Person 全 ONNX（无 PyTorch）")
    p.add_argument("--weights-15cls", default=None)
    p.add_argument("--weights-handobj", default=None)
    p.add_argument("--weights-person", default=None)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--imgsz", type=int, default=IMG_SIZE)
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1080p")
    p.add_argument("--max-depth", type=float, default=MAX_DEPTH_M)
    p.add_argument("--no-depth-window", action="store_true")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--print-xyz", action="store_true")
    p.add_argument("--udp-enable", action="store_true")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=16031)
    p.add_argument("--udp-frame-id", default="camera_link")
    p.add_argument("--udp-rate", type=float, default=10.0)
    p.add_argument("--stream-enable", action="store_true")
    p.add_argument("--stream-bind", default="0.0.0.0")
    p.add_argument("--stream-port", type=int, default=8765)
    p.add_argument("--stream-quality", type=int, default=85)
    p.add_argument("--onnx-ep", choices=["trt", "cuda", "cpu"], default="cuda")
    p.add_argument(
        "--trt-workspace-mb",
        type=int,
        default=256,
        help="TensorRT 最大 workspace (MB)，显存紧可试 128",
    )
    p.add_argument(
        "--no-trt-fp16",
        action="store_true",
        help="TensorRT 不用 FP16（更占显存/慢，仅排错时用）",
    )
    p.add_argument(
        "--person-gpu",
        action="store_true",
        help="人检 ONNX 也放 GPU（默认 person 在 CPU，避免 Orin 三模型同显存 NvMap error 12）",
    )
    args = p.parse_args()
    # Jetson：yolov8m + 15cls + handobj 三会话同占 GPU 易 OOM → 默认 person 走 CPU
    person_on_cpu = args.onnx_ep in ("cuda", "trt") and not args.person_gpu

    w15 = Path(args.weights_15cls or WEIGHTS_15CLS)
    whand = Path(args.weights_handobj or WEIGHTS_HANDOBJ)
    wperson = _resolve_person_onnx(args.weights_person)
    if not w15.is_absolute():
        w15 = ROOT / w15
    if not whand.is_absolute():
        whand = ROOT / whand
    if not w15.exists() or not whand.exists() or not wperson.exists():
        print("请确保三个 ONNX 存在: 15cls, handobj, person")
        print("  person: 在项目根 yolo export model=yolov8m.pt format=onnx imgsz=640 → yolov8m.onnx")
        print("  或复制到 trash_detection/onnx/yolov8m_640.onnx，或 --weights-person /path/to/yolov8m.onnx")
        print("  当前:", w15, whand, wperson)
        return 1
    print("Person ONNX:", wperson)

    imgsz = args.imgsz
    max_range_mm = int((args.max_depth or MAX_DEPTH_M) * 1000)
    names_list = CLASS_NAMES_15[:NUM_WASTE_CLASSES]
    names_handobj = HANDOBJ_NAMES

    avail = ort.get_available_providers()
    print(
        "ONNX Runtime 可用 EP: %s | 你请求: --onnx-ep %s"
        % (avail, args.onnx_ep),
        flush=True,
    )
    if args.onnx_ep in ("cuda", "trt") and "CUDAExecutionProvider" not in avail and "TensorrtExecutionProvider" not in avail:
        print(
            "\n⚠️  当前 pip 里的 onnxruntime 多半是 **CPU 版**，没有 CUDA/TensorRT，"
            "所以 --onnx-ep cuda 仍会走 CPU，三模型串行会非常慢（~0.1 FPS 正常）。\n"
            "Jetson 请换 GPU 版（与系统 JP/CUDA 一致，示例 JP6 cu126）：\n"
            "  pip uninstall onnxruntime onnxruntime-gpu -y\n"
            "  pip install onnxruntime-gpu -f https://pypi.jetson-ai-lab.io/jp6/cu126\n"
            "装好后应看到可用 EP 含 CUDAExecutionProvider；更快可试 --onnx-ep trt（TensorRT）。\n",
            flush=True,
        )

    trt_ws = args.trt_workspace_mb
    trt_fp16 = not args.no_trt_fp16
    print("加载 ONNX（无 PyTorch）…", flush=True)
    if args.onnx_ep == "trt":
        print(
            "【TensorRT】每个模型首次会编译引擎（可能各需数～数十分钟），完成后缓存到 /tmp/ort_trt_15cls_handobj_rgbd",
            flush=True,
        )
    sess_15 = create_onnx_session(w15, args.onnx_ep, trt_ws, trt_fp16)
    print("  → 15cls 会话就绪", flush=True)
    sess_hand = create_onnx_session(whand, args.onnx_ep, trt_ws, trt_fp16)
    print("  → handobj 会话就绪", flush=True)
    if person_on_cpu:
        sess_person = create_onnx_session(wperson, "cpu")
        print("Person: CPU（默认，省 GPU；全 GPU 请加 --person-gpu）", flush=True)
    else:
        sess_person = create_onnx_session(wperson, args.onnx_ep, trt_ws, trt_fp16)
        print("  → person 会话就绪", flush=True)
    used = sess_15.get_providers()[0]
    print("15cls/handobj 实际 EP: %s" % used, flush=True)
    print("Person 实际 EP: %s" % sess_person.get_providers()[0], flush=True)
    if args.onnx_ep == "cuda" and used == "CPUExecutionProvider":
        print("→ 未用到 GPU；请按上文安装 onnxruntime-gpu。", flush=True)
    if args.person_gpu and args.onnx_ep in ("cuda", "trt"):
        print(
            "提示: 若出现 NvMap error 12 / 极慢 FPS，勿用 --person-gpu，或换 yolov8n.onnx。",
            flush=True,
        )
    in_name = sess_15.get_inputs()[0].name
    out_name = sess_15.get_outputs()[0].name
    # 15cls: 4+15=19, person: 4+80=84, handobj: 4+nc
    nc_15 = sess_15.get_outputs()[0].shape[1] - 4
    nc_person = sess_person.get_outputs()[0].shape[1] - 4
    nc_hand = sess_hand.get_outputs()[0].shape[1] - 4
    print("15cls nc=%d person nc=%d handobj nc=%d" % (nc_15, nc_person, nc_hand))

    pipeline = Pipeline()
    config = Config()
    res_wh = COLOR_RES_MAP.get(args.color_res, (1920, 1080))
    cw, ch = res_wh[0], res_wh[1]
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
    print("RGB-D 全 ONNX", "无界面 Ctrl+C" if headless else "按 q 退出")
    sender = None
    if args.udp_enable:
        sender = UdpTargetSender(
            args.udp_host, args.udp_port, args.udp_frame_id,
            "combined_15cls_handobj_rgbd_onnx", args.udp_rate,
        )
        print("UDP %s:%s" % (args.udp_host, args.udp_port))
    stream_holder = None
    if args.stream_enable:
        srv = MjpegStreamServer(bind_addr=args.stream_bind, port=args.stream_port)
        stream_holder = srv.start()
        try:
            ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            ip = args.stream_bind
        print("MJPEG http://%s:%d/" % (ip, args.stream_port))
    if not headless:
        cv2.namedWindow("15cls+HandObj RGB-D (ONNX only)", cv2.WINDOW_NORMAL)
        if not args.no_depth_window:
            cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)

    fps_start = time.perf_counter()
    fps_count = 0
    yolo_sum = 0.0
    trash_class_ids = list(range(NUM_WASTE_CLASSES))

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

            orig_h, orig_w = frame.shape[:2]
            padded, scale, (pad_left, pad_top, nw, nh) = letterbox(frame, (imgsz, imgsz))
            blob = preprocess_bgr_to_onnx(padded)

            t0 = time.perf_counter()
            out_15 = run_detect(sess_15, blob, in_name, out_name)
            out_person = run_detect(sess_person, blob, sess_person.get_inputs()[0].name, sess_person.get_outputs()[0].name)
            out_hand = run_detect(sess_hand, blob, sess_hand.get_inputs()[0].name, sess_hand.get_outputs()[0].name)
            yolo_sum += time.perf_counter() - t0

            results_waste = decode_yolo_output(out_15, args.conf, NMS_IOU, nc_15, class_filter=trash_class_ids)
            results_person = decode_yolo_output(out_person, args.conf, NMS_IOU, nc_person, class_filter=[COCO_PERSON_CLASS_ID])
            results_handobj = decode_yolo_output(out_hand, args.conf, NMS_IOU, nc_hand, class_filter=None)

            # 坐标还原到原图
            if len(results_waste.boxes.xyxy) > 0:
                results_waste.boxes.xyxy = scale_boxes_to_original(
                    results_waste.boxes.xyxy, scale, pad_left, pad_top, orig_w, orig_h, nh, nw
                )
            if len(results_person.boxes.xyxy) > 0:
                results_person.boxes.xyxy = scale_boxes_to_original(
                    results_person.boxes.xyxy, scale, pad_left, pad_top, orig_w, orig_h, nh, nw
                )
            if len(results_handobj.boxes.xyxy) > 0:
                results_handobj.boxes.xyxy = scale_boxes_to_original(
                    results_handobj.boxes.xyxy, scale, pad_left, pad_top, orig_w, orig_h, nh, nw
                )

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
                for i in range(len(results_waste.boxes.xyxy)):
                    if int(results_waste.boxes.cls[i]) >= NUM_WASTE_CLASSES or float(results_waste.boxes.conf[i]) < args.conf:
                        continue
                    x1, y1, x2, y2 = results_waste.boxes.xyxy[i].astype(int)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    cv2.putText(vis, names_list[int(results_waste.boxes.cls[i])][:8], (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                for i in range(len(results_person.boxes.xyxy)):
                    if float(results_person.boxes.conf[i]) < args.conf:
                        continue
                    x1, y1, x2, y2 = results_person.boxes.xyxy[i].astype(int)
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
                    cv2.imshow("15cls+HandObj RGB-D (ONNX only)", vis)
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
