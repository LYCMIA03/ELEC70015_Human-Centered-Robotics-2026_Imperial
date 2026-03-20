#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hand-Object RGB-D + 远程 15cls（可选）

默认：只跑 handobj_detection，输出“最近 holding_xyz”。

开启远程 15cls：
  - 每隔 --waste-call-every 帧，把当前彩色图发给 waste15cls_server 的 HTTP /predict
  - server 返回 waste bbox 后，用深度得到最近 waste_xyz
  - 再用 handobj 的所有 person 框（非 holding 限制）找“离 waste_xyz 最近的 person candidate”
  - 最终在 holding person 与 waste-associated person 之间，按 z（更近相机更小）选一个作为 target_xyz

可视化：
  - 如果 --stream-enable：MJPEG 流实时输出（浏览器可看）
  - target/holding/waste/person-associated 框会在画面里标出来
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import urllib.parse
import urllib.request

import cv2
import numpy as np
import torch
try:
    import websockets
except ImportError:  # websocket 版本为可选
    websockets = None
try:
    import onnxruntime as ort
except ImportError:
    ort = None

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Jetson CUDA 分配器（在 import torch 之前）
if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native"

# 无显示器时避免 Qt/xcb 报错（须在 import cv2 之前设置，这里尽量早）
if (("--headless" in sys.argv) or ("--stream-enable" in sys.argv)) and "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

try:
    from pyorbbecsdk import (
        Pipeline, Config, OBSensorType, OBFormat, OBError,
        OBStreamType, OBFrameAggregateOutputMode, AlignFilter,
        FormatConvertFilter, OBConvertFormat,
    )
except (ModuleNotFoundError, ImportError) as e:
    print("未找到 pyorbbecsdk:", e)
    sys.exit(1)

from ultralytics import YOLO
from common_utils.rgbd_orientation import rotate_rgbd_180


# ---------------- 路径与内参 ----------------
WEIGHT_DIR = ROOT / "weight"
DEFAULT_WEIGHTS = WEIGHT_DIR / "best.pt"
PERSON_MODEL_DEFAULT = "yolov8n.pt"  # COCO class 0 = person

COCO_PERSON_CLASS_ID = 0
COLOR_RES_MAP = {"720p": (1280, 720), "1080p": (1920, 1080), "1440p": (2560, 1440)}
DEPTH_FX_BASE, DEPTH_FY_BASE = 500.0, 500.0
MAX_DEPTH_M = 5.0

# ---------------- ONNX(Hand-Object) ----------------
ONNX_DIR = PROJECT_ROOT / "trash_detection" / "onnx"
DEFAULT_HANDOBJ_ONNX = ONNX_DIR / "handobj_640.onnx"

# hand/object 类别名（与 trash_detection 的 ONNX 导出一致）
HANDOBJ_NAMES = {0: "person", 1: "left_hand", 2: "right_hand", 3: "object"}

# YOLOv8 ONNX 常量（与 onnx-only 脚本对齐）
NMS_IOU = 0.65


class _NumpyArrWrap:
    """让 numpy 数组也支持 .cpu().numpy()，用于兼容现有 holding 逻辑。"""

    def __init__(self, arr: np.ndarray):
        self._arr = arr

    def cpu(self):
        return self

    def numpy(self):
        return self._arr


class _OnnxBoxesTorchLike:
    def __init__(self, xyxy: np.ndarray, conf: np.ndarray, cls: np.ndarray):
        self.xyxy = _NumpyArrWrap(xyxy)
        self.conf = _NumpyArrWrap(conf)
        self.cls = _NumpyArrWrap(cls)

    def __len__(self):
        return int(self.xyxy.numpy().shape[0]) if self.xyxy.numpy() is not None else 0


class _OnnxResultsTorchLike:
    def __init__(self, boxes: _OnnxBoxesTorchLike):
        self.boxes = boxes


def letterbox(bgr, new_shape=(640, 640), stride=32):
    """YOLOv8 letterbox: 保持比例 + padding 到 new_shape。"""
    h, w = bgr.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    else:
        img = bgr.copy()
    dh = new_shape[0] - nh
    dw = new_shape[1] - nw
    dh = (dh // 2, dh - dh // 2)
    dw = (dw // 2, dw - dw // 2)
    img = cv2.copyMakeBorder(
        img,
        dh[0],
        dh[1],
        dw[0],
        dw[1],
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    pad_left, pad_top = dw[0], dh[0]
    return img, r, (pad_left, pad_top, nw, nh)


def preprocess_bgr_to_onnx(bgr_padded):
    """BGR HWC -> RGB NCHW float32 [0,1]。"""
    rgb = cv2.cvtColor(bgr_padded, cv2.COLOR_BGR2RGB)
    blob = rgb.astype(np.float32) / 255.0
    blob = np.transpose(blob, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)
    return blob.astype(np.float32)


def scale_boxes_to_original(xyxy, scale, pad_left, pad_top, orig_w, orig_h, in_h, in_w):
    """从 letterbox 坐标还原到原图坐标。"""
    xyxy = xyxy.copy()
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad_left) / scale
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad_top) / scale
    xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, orig_w)
    xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, orig_h)
    return xyxy


def decode_yolo_output(output, conf_thresh, iou_thresh, num_classes, class_filter=None):
    """
    YOLOv8 ONNX 解码:
    output: (1, 4+nc, 8400) 或类似形状。
    返回: (xyxy, conf, cls) numpy。
    """
    if output is None or output.size == 0:
        return np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    raw = np.squeeze(output, axis=0)
    if raw.ndim != 2:
        return np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    raw = raw.T  # (8400, 4+nc)
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
        return np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0,), dtype=np.float32)

    # NMS: cv2 需要 (x,y,w,h)
    indices = cv2.dnn.NMSBoxes(
        list(zip(x1.tolist(), y1.tolist(), (x2 - x1).tolist(), (y2 - y1).tolist())),
        confs.tolist(),
        float(conf_thresh),
        float(iou_thresh),
    )
    if indices is None or len(indices) == 0:
        return np.zeros((0, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    idx = np.array(indices).flatten()
    xyxy = np.stack([x1[idx], y1[idx], x2[idx], y2[idx]], axis=1).astype(np.float32)
    conf = confs[idx].astype(np.float32)
    cls = cls_ids[idx].astype(np.float32)
    return xyxy, conf, cls


def _providers_for_ep(ep, trt_workspace_mb=256, trt_fp16=True):
    if ort is None:
        raise RuntimeError("onnxruntime 未安装")
    avail = ort.get_available_providers()
    if ep == "cpu":
        return ["CPUExecutionProvider"]
    if ep == "cuda":
        if "CUDAExecutionProvider" in avail:
            return [("CUDAExecutionProvider", {"device_id": 0}), "CPUExecutionProvider"]
        return ["CPUExecutionProvider"]
    if ep == "trt":
        if "TensorrtExecutionProvider" not in avail:
            # 无 TRT 时回落 CUDA
            if "CUDAExecutionProvider" in avail:
                return [("CUDAExecutionProvider", {"device_id": 0}), "CPUExecutionProvider"]
            return ["CPUExecutionProvider"]
        import tempfile

        cache_dir = os.path.join(tempfile.gettempdir(), "ort_trt_handobj")
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
    return ["CPUExecutionProvider"]


def create_onnx_session(path, ep, trt_workspace_mb=256, trt_fp16=True):
    if ort is None:
        raise RuntimeError("onnxruntime 未安装")
    providers = _providers_for_ep(ep, trt_workspace_mb, trt_fp16)
    return ort.InferenceSession(str(path), providers=providers)


def run_onnx_detect(session, blob, input_name=None, output_name=None):
    if input_name is None:
        input_name = session.get_inputs()[0].name
    if output_name is None:
        output_name = session.get_outputs()[0].name
    out = session.run([output_name], {input_name: blob})
    return out[0]


# ---------------- UDP/MJPEG ----------------
class UdpTargetSender:
    """UDP 发送 JSON {stamp, frame_id, x, y, z, source, kind}。"""

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
            "x": float(xyz[0]),
            "y": float(xyz[1]),
            "z": float(xyz[2]),
            "source": self.source,
            "kind": kind,
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(data, (self.host, self.port))
        self.last_sent_t = now
        return True


class MjpegStreamHolder:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None

    def set_frame(self, bgr_frame, quality=85):
        if bgr_frame is None or bgr_frame.size == 0:
            return
        ok, buf = cv2.imencode(".jpg", bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            return
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
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=%s" % boundary.decode(),
                )
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.end_headers()
                try:
                    while True:
                        jpeg = stream_holder.get_jpeg()
                        if jpeg:
                            self.wfile.write(b"--" + boundary + b"\r\n")
                            self.wfile.write(
                                b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                % len(jpeg)
                            )
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
        self.port = int(port)
        self.server = HTTPServer((bind_addr, self.port), _make_stream_handler(self.holder))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()
        return self.holder


def _is_cuda_allocator_error(exc):
    msg = str(exc)
    return (
        "CUDACachingAllocator" in msg
        or "NVML" in msg
        or "NvMapMemAllocInternalTagged" in msg
        or "cuda out of memory" in msg.lower()
    )


def _predict_yolo_with_fallback(model, frame, *, conf, imgsz, device, label):
    max_retries = 3 if device == "cuda" else 1
    for attempt in range(max_retries):
        try:
            result = model.predict(
                frame,
                conf=conf,
                imgsz=imgsz,
                verbose=False,
                device=device,
            )[0]
            return result, device
        except RuntimeError as e:
            if (
                attempt < max_retries - 1
                and device == "cuda"
                and _is_cuda_allocator_error(e)
            ):
                time.sleep(0.15 * (attempt + 1))
                continue
            if device == "cuda" and _is_cuda_allocator_error(e):
                print(
                    "%s CUDA 推理失败（显存/分配器异常），自动切换到 CPU 继续运行: %s"
                    % (label, e),
                    flush=True,
                )
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
                result = model.predict(
                    frame,
                    conf=conf,
                    imgsz=imgsz,
                    verbose=False,
                    device="cpu",
                )[0]
                return result, "cpu"
            raise


# ---------------- 手持物体计算（从原脚本逻辑搬运） ----------------
def _get_class_names(model):
    names = getattr(model, "names", None)
    if names is None:
        return {}
    if isinstance(names, dict):
        return names
    return dict(enumerate(names))


def _find_class_ids(names_dict, *keywords):
    ids = []
    for idx, name in names_dict.items():
        n = (name or "").lower()
        for kw in keywords:
            if kw in n:
                ids.append(idx)
                break
    return ids


def sample_depth_at_center(depth, x1, y1, x2, y2, h, w):
    """在 bbox 中心及邻域采样深度 (mm)，无效返回 None。"""
    cx = int(round((x1 + x2) / 2))
    cy = int(round((y1 + y2) / 2))
    cx = max(0, min(cx, w - 1))
    cy = max(0, min(cy, h - 1))
    d = int(depth[cy, cx])
    if d > 0:
        return d
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            ny, nx = cy + dy, cx + dx
            if 0 <= ny < h and 0 <= nx < w:
                d = int(depth[ny, nx])
                if d > 0:
                    return d
    return None


def depth_pixel_to_xyz_m(px, py, depth_mm, depth_w=640, depth_h=576):
    """像素 (px, py) + 深度 mm -> 相机系 (X,Y,Z) 米。"""
    if depth_mm is None or depth_mm <= 0:
        return None
    z_m = float(depth_mm) / 1000.0
    cx, cy = depth_w / 2.0, depth_h / 2.0
    fx = DEPTH_FX_BASE * depth_w / 640.0
    fy = DEPTH_FY_BASE * depth_h / 576.0
    x_m = (px - cx) * z_m / fx
    y_m = (py - cy) * z_m / fy
    return (x_m, y_m, z_m)


def _box_center_inside(box_a, box_b):
    x1, y1, x2, y2 = box_a
    cx = (box_b[0] + box_b[2]) / 2
    cy = (box_b[1] + box_b[3]) / 2
    return x1 <= cx <= x2 and y1 <= cy <= y2


def _iou_rect(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    inter = (ix2 - ix1) * (iy2 - iy1)
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    if area_a <= 0 or area_b <= 0:
        return 0.0
    return inter / min(area_a, area_b)


def _rect_intersection_area(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0
    return float(ix2 - ix1) * float(iy2 - iy1)


def _rect_area(a):
    x1, y1, x2, y2 = a
    return max(0.0, float(x2 - x1) * float(y2 - y1))


def get_holding_person_boxes_and_xyz_dual(
    person_results,
    handobj_results,
    handobj_names_dict,
    depth_mm,
    depth_h,
    depth_w,
    conf_threshold=0.35,
    max_range_mm=5000,
    person_keywords=("person", "human"),
    hand_keywords=("hand", "left_hand", "right_hand"),
    object_keywords=("object", "obj", "thing", "item"),
    ignore_self_touch=True,
    self_touch_inside_thr=0.85,
    ignore_self_touch_objects=True,
    self_touch_object_inside_thr=0.92,
    self_touch_object_same_size_area_thr=0.9,
    self_touch_object_same_size_iou_thr=0.6,
    hand_object_overlap_iou_thr=0.05,
    hand_object_center_inside=True,
    person_class_id=COCO_PERSON_CLASS_ID,
):
    """
    双模型 holding：
    - person model：用 person_class_id 找人框
    - hand/object model：用关键词从 handobj_names_dict 找 hand/object
    """
    if (
        person_results is None
        or handobj_results is None
        or person_results.boxes is None
        or len(person_results.boxes) == 0
        or depth_mm is None
        or depth_mm.ndim != 2
    ):
        return []

    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    max_range_mm = min(max_range_mm, int(MAX_DEPTH_M * 1000))

    hand_ids = _find_class_ids(handobj_names_dict, *hand_keywords)
    obj_ids = _find_class_ids(handobj_names_dict, *object_keywords)

    person_boxes = []
    for xyxy, conf, cls in zip(
        person_results.boxes.xyxy.cpu().numpy(),
        person_results.boxes.conf.cpu().numpy(),
        person_results.boxes.cls.cpu().numpy(),
    ):
        cls_id = int(cls)
        if cls_id != int(person_class_id):
            continue
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])

        d_mm = sample_depth_at_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm, depth_w=dw, depth_h=dh)
        if xyz is None:
            continue
        person_boxes.append(((x1, y1, x2, y2), xyz, float(conf)))

    if not person_boxes:
        return []

    hand_boxes = []
    obj_boxes = []
    if handobj_results.boxes is not None and len(handobj_results.boxes) > 0:
        for xyxy, conf, cls in zip(
            handobj_results.boxes.xyxy.cpu().numpy(),
            handobj_results.boxes.conf.cpu().numpy(),
            handobj_results.boxes.cls.cpu().numpy(),
        ):
            cls_id = int(cls)
            if float(conf) < conf_threshold:
                continue
            x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
            if cls_id in hand_ids:
                hand_boxes.append((x1, y1, x2, y2, float(conf)))
            elif cls_id in obj_ids:
                obj_boxes.append((x1, y1, x2, y2, float(conf)))

    holding = []
    for (x1, y1, x2, y2), xyz, p_conf in person_boxes:
        person_rect = (x1, y1, x2, y2)

        touch_hands = []
        for h in hand_boxes:
            h_rect = (h[0], h[1], h[2], h[3])
            if _iou_rect(person_rect, h_rect) > 0.05 or _box_center_inside(person_rect, h_rect):
                touch_hands.append(h_rect)

        if not touch_hands:
            continue

        # 是否“hand 主要在自己身上”（疑似摸自己）
        hand_inside_any = False
        if ignore_self_touch:
            for h_rect in touch_hands:
                inter_area = _rect_intersection_area(person_rect, h_rect)
                hand_area = _rect_area(h_rect)
                inside_ratio = inter_area / hand_area if hand_area > 1e-6 else 1.0
                if self_touch_inside_thr is not None and inside_ratio >= float(self_touch_inside_thr):
                    hand_inside_any = True
                    break

        # 是否手与 object 有关联（触发 holding）
        def hand_has_object_association(h_rect):
            for o in obj_boxes:
                o_rect = (o[0], o[1], o[2], o[3])
                has_assoc = False
                if _iou_rect(h_rect, o_rect) > float(hand_object_overlap_iou_thr):
                    has_assoc = True
                elif hand_object_center_inside and _box_center_inside(h_rect, o_rect):
                    has_assoc = True

                if not has_assoc:
                    continue

                # self-touch 模式下，对 object 做“同尺寸”抑制（允许 object 落在人框内）
                if ignore_self_touch and hand_inside_any and ignore_self_touch_objects:
                    person_area = _rect_area(person_rect)
                    obj_area = _rect_area(o_rect)
                    if person_area > 1e-6 and obj_area > 1e-6:
                        size_similarity = min(obj_area, person_area) / max(obj_area, person_area)
                    else:
                        size_similarity = 0.0

                    if _iou_rect(person_rect, o_rect) >= float(self_touch_object_same_size_iou_thr) and size_similarity >= float(self_touch_object_same_size_area_thr):
                        continue

                return True

            return False

        any_hand_object = False
        for h_rect in touch_hands:
            if hand_has_object_association(h_rect):
                any_hand_object = True
                break

        if any_hand_object:
            holding.append(((x1, y1, x2, y2), xyz, p_conf))

    holding.sort(key=lambda x: x[1][2])
    return holding


def get_holding_person_boxes_and_xyz(
    results,
    names_dict,
    depth_mm,
    depth_h,
    depth_w,
    conf_threshold=0.35,
    max_range_mm=5000,
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
        d_mm = sample_depth_at_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm, depth_w=dw, depth_h=dh)
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


def get_person_candidates_from_results(
    results,
    names_dict,
    depth_mm,
    depth_h,
    depth_w,
    conf_threshold=0.35,
    max_range_mm=5000,
    person_keywords=("person", "human"),
):
    if results.boxes is None or len(results.boxes) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return []
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    max_range_mm = min(max_range_mm, int(MAX_DEPTH_M * 1000))

    person_ids = _find_class_ids(names_dict, *person_keywords)
    if not person_ids:
        # fallback：若没识别到 person 类别名，按 COCO class 0（在你的模型里可能不成立）
        person_ids = [COCO_PERSON_CLASS_ID]

    candidates = []
    for xyxy, conf, cls in zip(
        results.boxes.xyxy.cpu().numpy(),
        results.boxes.conf.cpu().numpy(),
        results.boxes.cls.cpu().numpy(),
    ):
        cls_id = int(cls)
        if cls_id not in person_ids:
            continue
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
        d_mm = sample_depth_at_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm, depth_w=dw, depth_h=dh)
        if xyz is None:
            continue
        candidates.append(((x1, y1, x2, y2), xyz, float(conf), cls_id))

    # 按离相机近（z 小）排序
    candidates.sort(key=lambda t: t[1][2])
    return candidates


def _dist_sq_xyz(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


# ---------------- Orbbec 彩色帧->BGR ----------------
def _color_frame_to_bgr(color_frame):
    if color_frame is None:
        return None
    h, w = color_frame.get_height(), color_frame.get_width()
    data = np.asarray(color_frame.get_data())
    fmt = color_frame.get_format()
    if fmt == OBFormat.RGB:
        img = data.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
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


def _remote_waste_predict(
    server_url,
    bgr,
    *,
    conf,
    imgsz,
    hide_human=True,
    timeout_s=2.0,
    jpeg_quality=40,
    waste_jpeg_max_dim=None,
):
    if not server_url:
        return []

    img = bgr
    if waste_jpeg_max_dim is not None:
        h, w = img.shape[:2]
        scale = min(waste_jpeg_max_dim / float(w), waste_jpeg_max_dim / float(h), 1.0)
        if scale < 1.0:
            img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok:
        return []
    jpeg_bytes = buf.tobytes()

    boundary = "----HCRYOLOFORMBOUNDARY7d5b2f3a"
    query = {
        "conf": float(conf),
        "imgsz": int(imgsz),
        "agnostic_nms": "false",
        "hide_human": str(bool(hide_human)).lower(),
        "return_image": "false",
    }
    full_url = server_url.rstrip("/") + "/predict?" + urllib.parse.urlencode(query)

    body_parts = []
    body_parts.append(f"--{boundary}\r\n".encode("utf-8"))
    body_parts.append(b'Content-Disposition: form-data; name="file"; filename="frame.jpg"\r\n')
    body_parts.append(b"Content-Type: image/jpeg\r\n\r\n")
    body_parts.append(jpeg_bytes)
    body_parts.append(b"\r\n")
    body_parts.append(f"--{boundary}--\r\n".encode("utf-8"))
    body = b"".join(body_parts)

    req = urllib.request.Request(
        full_url,
        data=body,
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            raw = resp.read().decode("utf-8", errors="ignore")
            obj = json.loads(raw)
            return obj.get("detections", []) or []
    except Exception:
        return []


def _to_ws_uri(server_url: str) -> str:
    """把 http(s)://host:port 转成 ws(s)://host:port，并确保末尾是 /ws。"""
    u = (server_url or "").strip()
    if u.startswith("http://"):
        u = "ws://" + u[len("http://") :]
    elif u.startswith("https://"):
        u = "wss://" + u[len("https://") :]
    elif u.startswith("ws://") or u.startswith("wss://"):
        pass
    else:
        u = "ws://" + u
    if not u.rstrip("/").endswith("/ws"):
        u = u.rstrip("/") + "/ws"
    return u


def _to_http_uri(server_url: str) -> str:
    """把 ws(s)://host:port 转成 http(s)://host:port（用于 HTTP fallback）。"""
    u = (server_url or "").strip()
    if u.startswith("ws://"):
        u = "http://" + u[len("ws://") :]
    elif u.startswith("wss://"):
        u = "https://" + u[len("wss://") :]
    return u


def _remote_waste_predict_ws(
    server_url,
    bgr,
    *,
    timeout_s=2.0,
    jpeg_quality=40,
    waste_jpeg_max_dim=None,
):
    """WebSocket 方式（对齐 waste client：发送 JPEG bytes，接收 JSON 文本）。"""
    if websockets is None:
        raise RuntimeError("websockets 未安装，无法使用 ws 方式")
    if not server_url:
        return []

    img = bgr
    if waste_jpeg_max_dim is not None:
        h, w = img.shape[:2]
        scale = min(waste_jpeg_max_dim / float(w), waste_jpeg_max_dim / float(h), 1.0)
        if scale < 1.0:
            img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok:
        return []
    jpeg_bytes = buf.tobytes()

    ws_uri = _to_ws_uri(server_url)

    async def _call():
        async with websockets.connect(ws_uri, max_size=16 * 1024 * 1024) as ws:
            await ws.send(jpeg_bytes)
            raw = await ws.recv()
            if isinstance(raw, (bytes, bytearray)):
                raw = raw.decode("utf-8", errors="ignore")
            obj = json.loads(raw)
            return obj.get("detections", []) or []

    # 该脚本主循环是同步的，所以在当前线程内用 asyncio.run 做一次性 ws 调用
    import asyncio

    return asyncio.run(asyncio.wait_for(_call(), timeout=timeout_s))


def _remote_waste_predict_dispatch(
    server_url,
    bgr,
    *,
    conf,
    imgsz,
    hide_human=True,
    timeout_s=2.0,
    jpeg_quality=40,
    waste_jpeg_max_dim=None,
):
    """
    优先 WebSocket（和你的 waste client 对齐），失败则回退 HTTP `/predict`。
    注意：ws 方式的 conf/imgsz/hide_human 由 server 启动参数决定（客户端不再传 query）。
    """
    if not server_url:
        return []

    # 1) WS 优先
    if websockets is not None:
        try:
            return _remote_waste_predict_ws(
                server_url,
                bgr,
                timeout_s=timeout_s,
                jpeg_quality=jpeg_quality,
                waste_jpeg_max_dim=waste_jpeg_max_dim,
            )
        except Exception:
            # WS 失败就尝试 HTTP fallback（更稳）
            pass

    # 2) HTTP fallback
    http_url = _to_http_uri(server_url)
    return _remote_waste_predict(
        http_url,
        bgr,
        conf=conf,
        imgsz=imgsz,
        hide_human=hide_human,
        timeout_s=timeout_s,
        jpeg_quality=jpeg_quality,
        waste_jpeg_max_dim=waste_jpeg_max_dim,
    )


def main():
    p = argparse.ArgumentParser(description="Hand-Object RGB-D + remote 15cls")
    p.add_argument("--weights", "-w", default=str(DEFAULT_WEIGHTS), help="handobj weight .pt")
    p.add_argument(
        "--handobj-onnx-path",
        default=str(DEFAULT_HANDOBJ_ONNX),
        help="handobj ONNX 路径（默认 trash_detection/onnx/handobj_640.onnx）",
    )
    p.add_argument(
        "--handobj-onnx-ep",
        choices=["trt", "cuda", "cpu"],
        default=None,
        help="如果设置则用 onnxruntime 推理 handobj（例如 --handobj-onnx-ep cuda）",
    )
    p.add_argument("--person-weights", default=PERSON_MODEL_DEFAULT,
                   help="person 检测权重（默认 yolov8n.pt，COCO person=0）")
    p.add_argument("--conf", type=float, default=0.35, help="handobj conf 阈值")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO 输入尺寸（handobj）")
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1080p")
    p.add_argument("--max-depth", type=float, default=MAX_DEPTH_M)
    p.add_argument("--headless", action="store_true", help="无界面")
    p.add_argument("--print-xyz", action="store_true")
    p.add_argument(
        "--no-rotate-180",
        action="store_true",
        help="不对彩色/深度做 180 度翻转（默认开启；适用于相机上下倒装）",
    )
    # 兼容旧参数：如果你之前用过 --rotate-180，就仍然可以触发翻转
    p.add_argument("--rotate-180", action="store_true", help="(兼容) 强制开启 180 度翻转")

    p.add_argument("--udp-enable", action="store_true")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=16031)
    p.add_argument("--udp-frame-id", default="camera_link")
    p.add_argument("--udp-rate", type=float, default=10.0)
    p.add_argument("--udp-kind", default="target")

    # remote waste server
    p.add_argument("--waste-server", default=None, help="http://<ip>:8765，开启远程 15cls")
    p.add_argument("--waste-conf", type=float, default=0.2)
    p.add_argument("--waste-timeout", type=float, default=2.0)
    p.add_argument("--waste-jpeg-quality", type=int, default=40)
    p.add_argument(
        "--waste-call-every",
        type=float,
        default=1.0,
        help="控制 remote 15cls 调用频率：>=1 表示每 N 帧调用一次；<1 表示每 N 秒调用一次（例如 0.25 表示 250ms）。",
    )
    p.add_argument("--waste-async", action="store_true",
                   help="远程 15cls 预测放后台线程，避免阻塞本地推理与 MJPEG 帧更新")

    # mjpeg
    p.add_argument("--stream-enable", action="store_true", help="开启 MJPEG 流")
    p.add_argument("--stream-bind", default="0.0.0.0")
    p.add_argument("--stream-port", type=int, default=8765)
    p.add_argument("--stream-quality", type=int, default=85)

    # holding 判定（对齐 handobj_detection_rgbd.py）
    p.add_argument("--no-self-touch-filter", action="store_true", help="关闭“手摸自己”抑制（默认开启）")
    p.add_argument("--self-touch-inside-thr", type=float, default=0.85,
                   help="hand 在 person 框内的比例阈值（默认 0.85）")
    p.add_argument("--hand-object-overlap-iou-thr", type=float, default=0.05,
                   help="hand 与 object 关联 IoU 阈值（默认 0.05）")
    p.add_argument("--self-touch-object-same-size-area-thr", type=float, default=0.9,
                   help="self-touch 时 object 与 person 面积相似度阈值（默认 0.9）")
    p.add_argument("--self-touch-object-same-size-iou-thr", type=float, default=0.6,
                   help="self-touch 时 object 与 person IoU 阈值（默认 0.6）")

    p.add_argument(
        "--xyz-hold-sec",
        type=float,
        default=1.0,
        help="当 holding bbox/xyz 在某帧消失时，最多继续沿用上一帧 xyz 的时间（秒）。",
    )

    p.add_argument(
        "--remote-box-hold-sec",
        type=float,
        default=1.0,
        help="当 waste server 端检测到的 waste/person bbox 过旧时，多久后在本端画面中自动消失（秒）。",
    )

    # 两个 YOLO 分别指定 device，避免 CUDA OOM
    p.add_argument(
        "--person-device",
        choices=["cuda", "cpu", "auto"],
        default="auto",
        help="person YOLO 推理设备（默认 auto：有 CUDA 用 cuda）。建议 OOM 时改 cpu。",
    )
    p.add_argument(
        "--handobj-device",
        choices=["cuda", "cpu", "auto"],
        default="auto",
        help="handobj YOLO 推理设备（默认 auto）。",
    )

    args = p.parse_args()

    weights = Path(args.weights)
    if not weights.is_absolute():
        weights = ROOT / weights
    # 只有在未启用 handobj ONNX 时才强制检查 torch 权重
    if getattr(args, "handobj_onnx_ep", None) is None:
        if not weights.exists():
            print("未找到权重:", weights, flush=True)
            return 1
    weights = str(weights)

    cuda_available = torch.cuda.is_available()
    yolo_device = "cuda" if cuda_available else "cpu"

    def _resolve_device(dev):
        if dev == "auto":
            return "cuda" if cuda_available else "cpu"
        return dev

    person_device = _resolve_device(getattr(args, "person_device", "auto"))
    handobj_device = _resolve_device(getattr(args, "handobj_device", "auto"))

    print("person_device:", person_device)
    print("handobj_device:", handobj_device)

    # handobj: 可选 ONNXRuntime 推理，避免 PyTorch/CUDA OOM
    handobj_names_dict = HANDOBJ_NAMES
    handobj_model = None
    handobj_onnx_sess = None
    onnx_imgsz = None
    hand_nc = None
    hand_input_name = None
    hand_output_name = None

    if getattr(args, "handobj_onnx_ep", None) is not None:
        if ort is None:
            print("未安装 onnxruntime，无法使用 --handobj-onnx-ep", flush=True)
            return 1
        w_handobj_onnx = Path(args.handobj_onnx_path)
        if not w_handobj_onnx.is_absolute():
            w_handobj_onnx = ROOT / w_handobj_onnx
        if not w_handobj_onnx.exists():
            print("未找到 handobj ONNX:", w_handobj_onnx, flush=True)
            return 1
        handobj_onnx_sess = create_onnx_session(w_handobj_onnx, args.handobj_onnx_ep)
        inp_shape = handobj_onnx_sess.get_inputs()[0].shape
        if inp_shape is not None and len(inp_shape) >= 4 and inp_shape[2] not in (None, "None"):
            onnx_imgsz = int(inp_shape[2])
        else:
            onnx_imgsz = int(args.imgsz)
        out_shape = handobj_onnx_sess.get_outputs()[0].shape
        # output: (1, 4+nc, ...)，取 shape[1]-4 得 nc
        if out_shape is not None and len(out_shape) >= 2 and out_shape[1] not in (None, "None"):
            hand_nc = int(out_shape[1]) - 4
        else:
            # fallback：与 onnx-only 脚本一致 handobj nc=4
            hand_nc = 4
        hand_input_name = handobj_onnx_sess.get_inputs()[0].name
        hand_output_name = handobj_onnx_sess.get_outputs()[0].name
        print("handobj: ONNXRuntime EP=", args.handobj_onnx_ep, "imgsz=", onnx_imgsz, "nc=", hand_nc, flush=True)
    else:
        handobj_model = YOLO(weights)
        handobj_names_dict = _get_class_names(handobj_model)

    person_model = YOLO(args.person_weights)
    person_names_dict = _get_class_names(person_model)

    # Jetson 上 CUDA 分配器有时会在首次推理时触发 NVML/NvMap 异常。
    # 先做一次小图预热；若失败则降级到 CPU，避免主循环直接崩溃。
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    if person_device == "cuda":
        try:
            person_model.predict(
                dummy,
                conf=args.conf,
                imgsz=min(args.imgsz, 640),
                verbose=False,
                device=person_device,
            )
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print("person CUDA 预热完成", flush=True)
        except RuntimeError as e:
            if _is_cuda_allocator_error(e):
                print("person CUDA 预热失败，自动切换到 CPU:", e, flush=True)
                person_device = "cpu"
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            else:
                raise
    if handobj_model is not None and handobj_device == "cuda":
        try:
            handobj_model.predict(
                dummy,
                conf=args.conf,
                imgsz=min(args.imgsz, 640),
                verbose=False,
                device=handobj_device,
            )
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print("handobj CUDA 预热完成", flush=True)
        except RuntimeError as e:
            if _is_cuda_allocator_error(e):
                print("handobj CUDA 预热失败，自动切换到 CPU:", e, flush=True)
                handobj_device = "cpu"
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            else:
                raise

    max_depth_mm = int((args.max_depth or MAX_DEPTH_M) * 1000)
    res_wh = COLOR_RES_MAP.get(args.color_res, (1920, 1080))
    cw, ch = res_wh[0], res_wh[1]

    waste_remote_enabled = args.waste_server is not None and str(args.waste_server).strip() != ""
    waste_server_url = args.waste_server.strip() if waste_remote_enabled else None

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
        print("配置 RGB-D 失败:", type(e).__name__, e)
        return 1

    try:
        pipeline.start(config)
    except Exception as e:
        print("启动 RGB-D 相机失败:", type(e).__name__, e)
        return 1

    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    time.sleep(1.5)

    # stream enable 就当作 headless（避免无窗口环境导致崩）
    effective_headless = args.headless or args.stream_enable

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

    sender = None
    if args.udp_enable:
        sender = UdpTargetSender(
            host=args.udp_host,
            port=args.udp_port,
            frame_id=args.udp_frame_id,
            source="handobj_detection_rgbd_remote_15cls",
            max_rate_hz=args.udp_rate,
        )

    if not effective_headless:
        cv2.namedWindow("Hand-Object RGB-D (remote 15cls optional)", cv2.WINDOW_NORMAL)

    fps_interval_start = time.perf_counter()
    fps_frame_count = 0
    yolo_time_sum = 0.0
    yolo_frame_count = 0

    frame_idx = 0
    last_remote_person_xyz = None
    last_remote_person_box = None
    last_remote_waste_box = None
    remote_lock = threading.Lock()
    remote_inflight = threading.Event()
    remote_request_id_latest = -1
    last_waste_call_ts = 0.0

    holding_xyz = None
    holding_box = None
    target_xyz = None
    target_box = None
    target_kind = None

    # holding bbox/xyz 消失时的鲁棒性：在 1s 内沿用上一帧 xyz
    xyz_hold_sec = float(getattr(args, "xyz_hold_sec", 1.0))
    remote_box_hold_sec = float(getattr(args, "remote_box_hold_sec", xyz_hold_sec))
    last_holding_xyz = None
    last_holding_xyz_ts = 0.0
    last_seen_holding_seq = -1

    last_remote_result_ts = 0.0

    hold_lock = threading.Lock()
    yolo_stats_lock = threading.Lock()

    # yolo worker 的输入/版本号（主线程只负责采集与推流，不做推理）
    capture_lock = threading.Lock()
    new_capture_event = threading.Event()
    stop_event = threading.Event()
    latest_frame = None
    latest_depth_copy = None
    latest_dh = None
    latest_dw = None
    latest_frame_idx = 0
    latest_version = 0

    holding_seq = 0
    holding_seq_ts = 0.0

    def remote_worker(
        request_id,
        frame_snapshot,
        depth_snapshot,
        dh_snapshot,
        dw_snapshot,
        person_results_snapshot,
    ):
        nonlocal last_remote_person_xyz, last_remote_person_box, last_remote_waste_box, last_remote_result_ts
        call_ts = time.perf_counter()
        try:
            t_remote_start = time.perf_counter()
            dets = _remote_waste_predict_dispatch(
                waste_server_url,
                frame_snapshot,
                conf=args.waste_conf,
                imgsz=640,
                hide_human=True,
                timeout_s=args.waste_timeout,
                jpeg_quality=args.waste_jpeg_quality,
            )
            t_remote_ms = (time.perf_counter() - t_remote_start) * 1000.0
            print(
                "waste_server call: %.1f ms | detections=%d"
                % (t_remote_ms, len(dets) if dets is not None else 0),
                flush=True,
            )

            # 由 waste bbox 得 waste_xyz（最近 waste：z 小）
            waste_items = []
            for d in dets or []:
                try:
                    cls_id = int(d.get("class_id", -1))
                except Exception:
                    cls_id = -1
                if cls_id < 0 or cls_id >= 14:
                    continue
                x1, y1, x2, y2 = float(d["x1"]), float(d["y1"]), float(d["x2"]), float(d["y2"])
                d_mm = sample_depth_at_center(
                    depth_snapshot, x1, y1, x2, y2, dh_snapshot, dw_snapshot
                )
                if d_mm is None or d_mm <= 0 or d_mm > max_depth_mm:
                    continue
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                xyz = depth_pixel_to_xyz_m(
                    cx, cy, d_mm, depth_w=dw_snapshot, depth_h=dh_snapshot
                )
                if xyz is None:
                    continue
                waste_items.append((xyz, (x1, y1, x2, y2)))

            if not waste_items:
                return

            nearest_waste_xyz, nearest_waste_box = min(waste_items, key=lambda t: t[0][2])

            person_candidates = get_person_candidates_from_results(
                person_results_snapshot,
                person_names_dict,
                depth_snapshot,
                dh_snapshot,
                dw_snapshot,
                conf_threshold=args.conf,
                max_range_mm=max_depth_mm,
            )
            if not person_candidates:
                return

            best = min(person_candidates, key=lambda t: _dist_sq_xyz(t[1], nearest_waste_xyz))
            with remote_lock:
                # 只写入“最新一次请求”的结果
                if request_id != remote_request_id_latest:
                    return
                last_remote_person_box = best[0]
                last_remote_person_xyz = best[1]
                last_remote_waste_box = nearest_waste_box
        except Exception:
            pass
        finally:
            remote_inflight.clear()
            with remote_lock:
                # 不论成功/失败，只要发起过一次请求，就刷新“上次远程检测时间”
                # 用于在主线程端对远程 bbox/xyz 做过期处理
                last_remote_result_ts = call_ts

    def yolo_worker():
        """
        后台 YOLO 推理线程：
        - 主线程只负责采集/推流（保证视频流不会因推理阻塞）
        - 以“处理最新帧”为策略，跳过中间帧，从而实现各自 fps
        """
        nonlocal person_device, handobj_device, last_waste_call_ts, remote_request_id_latest
        nonlocal holding_xyz, holding_box, holding_seq, holding_seq_ts
        nonlocal yolo_time_sum, yolo_frame_count

        processed_version = -1
        while not stop_event.is_set():
            # 等待主线程提交新帧
            new_capture_event.wait(timeout=0.2)
            if stop_event.is_set():
                break

            with capture_lock:
                if latest_version == processed_version:
                    new_capture_event.clear()
                    continue
                frame_in = latest_frame
                depth_in = latest_depth_copy
                dh_in = latest_dh
                dw_in = latest_dw
                frame_idx_in = latest_frame_idx
                version_in = latest_version
                processed_version = version_in

            if frame_in is None or depth_in is None:
                continue

            t_yolo_start = time.perf_counter()
            person_results = None
            try:
                person_results, person_device_local = _predict_yolo_with_fallback(
                    person_model,
                    frame_in,
                    conf=args.conf,
                    imgsz=min(args.imgsz, 640),
                    device=person_device,
                    label="person",
                )
                person_device = person_device_local

                if handobj_onnx_sess is not None:
                    orig_h, orig_w = frame_in.shape[:2]
                    padded, scale, (pad_left, pad_top, nw, nh) = letterbox(
                        frame_in, (onnx_imgsz, onnx_imgsz), stride=32
                    )
                    blob = preprocess_bgr_to_onnx(padded)
                    out_hand = run_onnx_detect(
                        handobj_onnx_sess, blob, hand_input_name, hand_output_name
                    )
                    xyxy, conf, cls = decode_yolo_output(
                        out_hand, args.conf, NMS_IOU, hand_nc, class_filter=None
                    )
                    if xyxy.shape[0] > 0:
                        xyxy = scale_boxes_to_original(
                            xyxy,
                            scale,
                            pad_left,
                            pad_top,
                            orig_w,
                            orig_h,
                            nh,
                            nw,
                        )
                    handobj_results = _OnnxResultsTorchLike(_OnnxBoxesTorchLike(xyxy, conf, cls))
                else:
                    handobj_results, handobj_device_local = _predict_yolo_with_fallback(
                        handobj_model,
                        frame_in,
                        conf=args.conf,
                        imgsz=min(args.imgsz, 640),
                        device=handobj_device,
                        label="handobj",
                    )
                    handobj_device = handobj_device_local

                holding_list = get_holding_person_boxes_and_xyz_dual(
                    person_results,
                    handobj_results,
                    handobj_names_dict,
                    depth_in,
                    dh_in,
                    dw_in,
                    conf_threshold=args.conf,
                    max_range_mm=max_depth_mm,
                    ignore_self_touch=not getattr(args, "no_self_touch_filter", False),
                    self_touch_inside_thr=float(getattr(args, "self_touch_inside_thr", 0.85)),
                    ignore_self_touch_objects=True,
                    self_touch_object_same_size_area_thr=float(
                        getattr(args, "self_touch_object_same_size_area_thr", 0.9)
                    ),
                    self_touch_object_same_size_iou_thr=float(
                        getattr(args, "self_touch_object_same_size_iou_thr", 0.6)
                    ),
                    hand_object_overlap_iou_thr=float(
                        getattr(args, "hand_object_overlap_iou_thr", 0.05)
                    ),
                )
                holding_xyz_new = holding_list[0][1] if holding_list else None
                holding_box_new = holding_list[0][0] if holding_list else None
            except Exception:
                holding_xyz_new = None
                holding_box_new = None
            dt = time.perf_counter() - t_yolo_start

            with yolo_stats_lock:
                yolo_time_sum += dt
                yolo_frame_count += 1

            with hold_lock:
                holding_xyz = holding_xyz_new
                holding_box = holding_box_new
                holding_seq += 1
                holding_seq_ts = time.perf_counter()

            # 远程 waste server：放到另一个线程，避免影响主推流与本线程 fps
            if waste_remote_enabled and person_results is not None:
                try:
                    now_ts = time.perf_counter()
                    should_call_remote = False
                    if float(args.waste_call_every) >= 1.0:
                        N = max(1, int(args.waste_call_every))
                        should_call_remote = (N <= 1) or (frame_idx_in % N == 0)
                    else:
                        should_call_remote = (now_ts - last_waste_call_ts) >= float(args.waste_call_every)

                    if should_call_remote and not remote_inflight.is_set():
                        last_waste_call_ts = now_ts
                        remote_request_id_latest = frame_idx_in
                        remote_inflight.set()
                        if args.waste_async:
                            th = threading.Thread(
                                target=remote_worker,
                                args=(
                                    frame_idx_in,
                                    frame_in,
                                    depth_in,
                                    dh_in,
                                    dw_in,
                                    person_results,
                                ),
                                daemon=True,
                            )
                            th.start()
                        else:
                            # 仍在后台 YOLO 线程内执行：不会影响主线程推流
                            remote_worker(
                                frame_idx_in,
                                frame_in,
                                depth_in,
                                dh_in,
                                dw_in,
                                person_results,
                            )
                except Exception:
                    pass

    yolo_thread = threading.Thread(target=yolo_worker, daemon=True)
    yolo_thread.start()

    try:
        while True:
            frame_idx += 1
            frames = None
            for _ in range(15):
                frames = pipeline.wait_for_frames(500)
                if frames and frames.get_color_frame() and frames.get_depth_frame():
                    break
            if frames is None:
                if not effective_headless and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                if effective_headless:
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
                    (depth_frame.get_height(), depth_frame.get_width())
                )
                depth_copy = (d.astype(np.float32) * scale).astype(np.uint16)

            color_img, depth_copy = rotate_rgbd_180(
                color_img,
                depth_copy,
                enabled=(getattr(args, "rotate_180", False) or not getattr(args, "no_rotate_180", False)),
            )

            if color_img is None or depth_copy is None or depth_copy.ndim != 2:
                if effective_headless:
                    time.sleep(0.02)
                continue

            dh, dw = depth_copy.shape[0], depth_copy.shape[1]
            if (color_img.shape[0], color_img.shape[1]) != (dh, dw):
                frame = cv2.resize(color_img, (dw, dh), interpolation=cv2.INTER_LINEAR)
            else:
                frame = color_img.copy()

            # 把最新帧提交给后台 YOLO 推理线程（不阻塞本线程推流）
            with capture_lock:
                latest_frame = frame
                latest_depth_copy = depth_copy
                latest_dh = dh
                latest_dw = dw
                latest_frame_idx = frame_idx
                latest_version += 1
                new_capture_event.set()

            # 读取最新 holding（由 yolo_worker 更新）
            with hold_lock:
                holding_xyz_cur = holding_xyz
                holding_box_cur = holding_box
                holding_seq_cur = holding_seq
                holding_seq_ts_cur = holding_seq_ts

            now_ts = time.perf_counter()
            if holding_seq_cur != last_seen_holding_seq:
                if holding_xyz_cur is not None:
                    last_holding_xyz = holding_xyz_cur
                    last_holding_xyz_ts = holding_seq_ts_cur
                last_seen_holding_seq = holding_seq_cur

            # holding bbox 消失时：最多沿用上一帧 xyz
            if holding_xyz_cur is not None:
                holding_xyz_eff = holding_xyz_cur
            else:
                if last_holding_xyz is not None and (now_ts - last_holding_xyz_ts) <= xyz_hold_sec:
                    holding_xyz_eff = last_holding_xyz
                else:
                    holding_xyz_eff = None

            # 默认 target = holding
            holding_box = holding_box_cur  # bbox 不做 hold（可视化避免误导）
            target_xyz = holding_xyz_eff
            target_box = holding_box_cur if holding_xyz_cur is not None else None
            target_kind = "holding" if target_xyz is not None else "holding"

            # 如果远程 person 可用，则与 holding 比 z 选更近
            with remote_lock:
                remote_person_xyz = last_remote_person_xyz
                remote_person_box = last_remote_person_box
                remote_waste_box = last_remote_waste_box
                remote_result_ts = last_remote_result_ts

            # 垃圾离开后，上一次远程 bbox 会“过期”，避免一直残留
            remote_valid = remote_result_ts > 0.0 and (now_ts - remote_result_ts) <= remote_box_hold_sec
            if not remote_valid:
                remote_person_xyz = None
                remote_person_box = None
                remote_waste_box = None
            if remote_person_xyz is not None:
                if target_xyz is None or remote_person_xyz[2] < target_xyz[2]:
                    target_xyz = remote_person_xyz
                    target_box = remote_person_box
                    target_kind = "person_near_waste"

            if sender is not None:
                sender.send_xyz(target_xyz, target_kind)

            if getattr(args, "print_xyz", False) and target_xyz is not None:
                print(
                    "target XYZ: X=%.3f Y=%.3f Z=%.3f kind=%s"
                    % (target_xyz[0], target_xyz[1], target_xyz[2], target_kind),
                    flush=True,
                )

            fps_frame_count += 1
            elapsed_sec = time.perf_counter() - fps_interval_start
            if elapsed_sec >= 1.0 and fps_frame_count > 0:
                loop_fps = fps_frame_count / elapsed_sec
                with yolo_stats_lock:
                    yolo_count = yolo_frame_count
                    yolo_sum = yolo_time_sum
                    yolo_time_sum = 0.0
                    yolo_frame_count = 0
                avg_yolo_s = (yolo_sum / yolo_count) if yolo_count > 0 else 0.0
                yolo_fps = (1.0 / avg_yolo_s) if avg_yolo_s > 0 else 0.0
                line = "FPS: %.1f (loop) | YOLO: %.1f fps" % (loop_fps, yolo_fps)
                if target_xyz is not None:
                    line += " | XYZ: %.2f, %.2f, %.2f [%s]" % (
                        target_xyz[0],
                        target_xyz[1],
                        target_xyz[2],
                        target_kind,
                    )
                print(line, flush=True)
                fps_interval_start = time.perf_counter()
                fps_frame_count = 0

            # 画面
            if effective_headless or stream_holder is not None or not effective_headless:
                vis = frame.copy()

                # holding box（黄色）
                if holding_box is not None:
                    x1, y1, x2, y2 = map(int, holding_box)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(vis, "holding", (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # target box（橙色）
                if target_box is not None:
                    x1, y1, x2, y2 = map(int, target_box)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 128, 255), 3)
                    cv2.putText(vis, target_kind or "target", (x1, max(0, y1 - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)

                # waste box（品红）
                if remote_waste_box is not None:
                    x1, y1, x2, y2 = map(int, remote_waste_box)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    cv2.putText(vis, "waste_near", (x1, max(0, y1 - 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                if stream_holder is not None:
                    stream_holder.set_frame(vis, quality=args.stream_quality)

                if not effective_headless:
                    cv2.imshow("Hand-Object RGB-D (remote 15cls optional)", vis)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break

            if effective_headless:
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_event.set()
            new_capture_event.set()
            if "yolo_thread" in locals():
                yolo_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            pipeline.stop()
        except Exception:
            pass
        if not effective_headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
