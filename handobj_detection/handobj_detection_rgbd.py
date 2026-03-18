# -*- coding: utf-8 -*-
"""
Hand-Object RGB-D：Orbbec 相机 + 双模型，返回离相机最近的「手持物体的人」的 XYZ。

- 使用 pyorbbecsdk 取彩色与深度（对齐到彩色），与 predict_15cls_rgbd 一致。
- 双模型（与原来一致）：person 用 yolov8n.pt（COCO person），hand/object 用 weight/best.pt。
- get_holding_person_boxes(person_res, hand_res, ...) 判定人框与手/物体框重叠为「手持」。

前置：pip install pyorbbecsdk opencv-python numpy ultralytics（Jetson 需 udev 规则）。
运行（在项目根目录）：
  python handobj_detection/handobj_detection_rgbd.py
  python handobj_detection/handobj_detection_rgbd.py --headless --print-xyz
  python handobj_detection/handobj_detection_rgbd.py --udp-enable   # 与 predict_15cls_rgbd 相同目标发送 XYZ
"""
import argparse
import json
import os
import socket
import sys
import time
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Jetson CUDA 分配器（在 import torch 之前）
if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native"

# 无显示器（SSH/headless）时避免 Qt/xcb 报错，须在 import cv2 之前设置
if (("--headless" in sys.argv) or ("--stream-enable" in sys.argv)) and "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

import numpy as np
import cv2
import torch
from common_utils.rgbd_orientation import rotate_rgbd_180

try:
    from pyorbbecsdk import (
        Pipeline, Config, OBSensorType, OBFormat, OBError,
        OBStreamType, OBFrameAggregateOutputMode, AlignFilter,
        FormatConvertFilter, OBConvertFormat,
    )
except (ModuleNotFoundError, ImportError) as e:
    print("未找到 pyorbbecsdk:", e)
    sys.exit(1)

# TorchNMS 回退（Jetson 上 torchvision C++ 常不兼容）
try:
    import torchvision.ops as _tv_ops
    from ultralytics.utils.nms import TorchNMS
    def _safe_nms(boxes, scores, iou_threshold):
        return TorchNMS.nms(boxes, scores, iou_threshold)
    _tv_ops.nms = _safe_nms
except Exception:
    pass

from ultralytics import YOLO

# 路径与内参
WEIGHT_DIR = ROOT / "weight"
DEFAULT_WEIGHTS = WEIGHT_DIR / "best.pt"
PERSON_MODEL_DEFAULT = "yolov8n.pt"  # COCO class 0 = person，与原来一致
COCO_PERSON_CLASS_ID = 0
COLOR_RES_MAP = {"720p": (1280, 720), "1080p": (1920, 1080), "1440p": (2560, 1440)}
DEPTH_FX_BASE, DEPTH_FY_BASE = 500.0, 500.0
MAX_DEPTH_M = 5.0


class UdpTargetSender:
    """与 predict_15cls_rgbd 相同：UDP 发送 JSON {stamp, frame_id, x, y, z, source, kind}。"""
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

    class StreamHandler(BaseHTTPRequestHandler):
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

    return StreamHandler


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


def _color_frame_to_bgr(color_frame):
    """Orbbec 彩色帧 -> BGR numpy (H, W, 3) uint8。"""
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


def _get_class_names(model):
    """YOLO 模型类别 id -> 名称，兼容 list/dict。"""
    names = getattr(model, "names", None)
    if names is None:
        return {}
    if isinstance(names, dict):
        return names
    return dict(enumerate(names))


def _find_class_ids(names_dict, *keywords):
    """在 names_dict 中查找名称包含任一 keyword 的类别 id 列表。"""
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
    """像素 (px, py) + 深度 mm -> 相机系 (X,Y,Z) 米。对齐到彩色时 cx,cy 取图中心，fx,fy 按分辨率缩放。"""
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
    """box_b 的中心是否在 box_a 内。"""
    x1, y1, x2, y2 = box_a
    cx = (box_b[0] + box_b[2]) / 2
    cy = (box_b[1] + box_b[3]) / 2
    return x1 <= cx <= x2 and y1 <= cy <= y2


def _iou_rect(a, b):
    """两个 (x1,y1,x2,y2) 的 IoU。"""
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


def get_holding_person_boxes_and_xyz(results, names_dict, depth_mm, depth_h, depth_w,
                                     conf_threshold=0.35, max_range_mm=5000,
                                     person_keywords=("person", "human"),
                                     hand_keywords=("hand", "left_hand", "right_hand"),
                                     object_keywords=("object", "obj", "thing", "item"),
                                     ignore_self_touch=True,
                                     self_touch_inside_thr=0.85):
    """
    从 YOLO results 和深度图得到「手持物体的人」的 bbox 与 XYZ。
    返回 list of ((x1,y1,x2,y2), xyz_m, person_conf)，按 z 升序（最近在前）。
    若无法区分 person/hand/object，则把所有框当目标，按深度最近排序。
    """
    if results.boxes is None or len(results.boxes) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return []
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    max_range_mm = min(max_range_mm, int(MAX_DEPTH_M * 1000))

    person_ids = _find_class_ids(names_dict, *person_keywords)
    hand_ids = _find_class_ids(names_dict, *hand_keywords)
    obj_ids = _find_class_ids(names_dict, *object_keywords)
    has_person = len(person_ids) > 0
    has_hand_or_obj = len(hand_ids) > 0 or len(obj_ids) > 0

    # 所有框：(xyxy, conf, cls_id)
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

    # 仅一类或无法区分：全部当目标，按 z 排序
    if not has_person or not has_hand_or_obj:
        out = [((t[0], t[1], t[2], t[3]), t[7], t[5]) for t in boxes_list]
        out.sort(key=lambda x: x[1][2])
        return out

    # person 与 hand/object 重叠则视为「手持」
    # 但如果只是“手摸自己”（hand 大部分落在 person 框内部），则屏蔽成非 holding。
    person_boxes = [t for t in boxes_list if t[4] in person_ids]
    hand_boxes = [t for t in boxes_list if t[4] in hand_ids]
    obj_boxes = [t for t in boxes_list if t[4] in obj_ids]
    holding = []
    for (x1, y1, x2, y2, _, conf, d_mm, xyz) in person_boxes:
        person_rect = (x1, y1, x2, y2)
        found_holding = False

        # 优先：检测到 object 才算“拿着东西”
        for o in obj_boxes:
            o_rect = (o[0], o[1], o[2], o[3])
            if _iou_rect(person_rect, o_rect) > 0.05 or _box_center_inside(person_rect, o_rect):
                holding.append(((x1, y1, x2, y2), xyz, conf))
                found_holding = True
                break
        if found_holding:
            continue

        # 次要：hand 与 person 重叠时，若 hand 基本都在 person 框内，则视为 self-touch
        if ignore_self_touch:
            for h in hand_boxes:
                h_rect = (h[0], h[1], h[2], h[3])
                if not (_iou_rect(person_rect, h_rect) > 0.05 or _box_center_inside(person_rect, h_rect)):
                    continue
                if self_touch_inside_thr is not None:
                    inter_area = _rect_intersection_area(person_rect, h_rect)
                    hand_area = _rect_area(h_rect)
                    inside_ratio = inter_area / hand_area if hand_area > 1e-6 else 1.0
                    if inside_ratio >= float(self_touch_inside_thr):
                        continue
                holding.append(((x1, y1, x2, y2), xyz, conf))
                break
        else:
            for h in hand_boxes:
                h_rect = (h[0], h[1], h[2], h[3])
                if _iou_rect(person_rect, h_rect) > 0.05 or _box_center_inside(person_rect, h_rect):
                    holding.append(((x1, y1, x2, y2), xyz, conf))
                    break
    holding.sort(key=lambda x: x[1][2])
    return holding


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

        # 核心判定：只用“hand 与 object 的关联”来触发 holding，
        # 不再直接用 object 与 person 的重合来作为触发条件。
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

                # 如果是 self-touch 模式，则再额外抑制“object 其实只是人体内部误检”
                if ignore_self_touch and hand_inside_any and ignore_self_touch_objects:
                    person_area = _rect_area(person_rect)
                    obj_area = _rect_area(o_rect)
                    if person_area > 1e-6 and obj_area > 1e-6:
                        size_similarity = min(obj_area, person_area) / max(obj_area, person_area)
                    else:
                        size_similarity = 0.0

                    # 仅当 “object 和 person 的尺寸几乎一样大” 时抑制。
                    # 允许 object 落在人框内/与人框有重合，但只要它“没跟 person 同尺寸”，就仍然允许 holding。
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


def main():
    p = argparse.ArgumentParser(description="Hand-Object RGB-D：最近手持物体的人的 XYZ")
    p.add_argument("--weights", "-w", default=str(DEFAULT_WEIGHTS), help="权重路径")
    p.add_argument("--person-weights", default=PERSON_MODEL_DEFAULT,
                   help="人检测权重（默认 yolov8n.pt，COCO person=0）。")
    p.add_argument("--conf", type=float, default=0.35, help="置信度阈值")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO 输入尺寸")
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1080p", help="彩色分辨率")
    p.add_argument("--max-depth", type=float, default=MAX_DEPTH_M, help="有效深度范围 (m)")
    p.add_argument("--headless", action="store_true", help="无界面，Ctrl+C 退出")
    p.add_argument("--print-xyz", action="store_true", help="每帧打印最近持物者 XYZ")
    p.add_argument("--device", choices=["cuda", "cpu", "auto"], default="auto", help="推理设备")
    p.add_argument("--udp-enable", action="store_true", help="启用 UDP 发送最近持物者 XYZ（与 predict_15cls_rgbd 相同目标）")
    p.add_argument("--udp-host", default="127.0.0.1", help="UDP 目标 host，默认 127.0.0.1")
    p.add_argument("--udp-port", type=int, default=16031, help="UDP 目标端口，默认 16031")
    p.add_argument("--udp-frame-id", default="camera_link", help="发送时使用的 frame_id，默认 camera_link")
    p.add_argument("--udp-rate", type=float, default=10.0, help="UDP 最大发送频率 (Hz)，默认 10")
    p.add_argument("--udp-kind", default="holding", help="UDP 目标类型标签 (holding|person|waste)，默认 holding")
    p.add_argument("--no-rotate-180", action="store_true",
                   help="不对彩色图/深度图做 180 度翻转（默认开启；适用于相机上下倒装）")
    p.add_argument("--no-self-touch-filter", action="store_true",
                   help="关闭“手摸自己”屏蔽（默认开启）")
    p.add_argument("--self-touch-inside-thr", type=float, default=0.85,
                   help="self-touch 过滤阈值：hand 基本在 person 框内的比例 >= 该值则忽略；默认 0.85")
    p.add_argument("--self-touch-object-inside-thr", type=float, default=0.92,
                   help="当疑似摸自己（hand 基本在 person 框内）时：若 object 几乎完全落在 person 框内的比例 >= 该值，则忽略；默认 0.92")
    p.add_argument("--self-touch-object-same-size-area-thr", type=float, default=0.9,
                   help="当疑似摸自己时：若 target object 与人框(person)面积相似度 >= 该值，则忽略（抑制“像人本身”的误检）；默认 0.9")
    p.add_argument("--self-touch-object-same-size-iou-thr", type=float, default=0.6,
                   help="当疑似摸自己时：若 object 与 person 的 IoU >= 该值 且面积相似度 >= 上述阈值，则忽略；默认 0.6")
    p.add_argument("--hand-object-overlap-iou-thr", type=float, default=0.05,
                   help="hand 与 object 框的 IoU >= 该值，则认为手在拿物（默认 0.05）")
    p.add_argument("--stream-enable", action="store_true", help="开启 MJPEG 流，把带框结果发到局域网")
    p.add_argument("--stream-bind", default="0.0.0.0", help="MJPEG 监听地址，默认 0.0.0.0")
    p.add_argument("--stream-port", type=int, default=8765, help="MJPEG 端口，默认 8765")
    p.add_argument("--stream-quality", type=int, default=85, help="MJPEG JPEG 质量 1-100，默认 85")
    args = p.parse_args()

    handobj_weights = Path(args.weights)
    if not handobj_weights.is_absolute():
        handobj_weights = ROOT / handobj_weights
    if not handobj_weights.exists():
        alt = WEIGHT_DIR / "best.pt"
        if alt.exists():
            handobj_weights = alt
        else:
            print("未找到权重:", handobj_weights)
            return 1
    handobj_weights = str(handobj_weights)

    yolo_device = "cuda" if torch.cuda.is_available() else "cpu"
    if getattr(args, "device", "auto") != "auto":
        yolo_device = args.device
    print("YOLO 推理设备:", yolo_device)

    person_model = YOLO(args.person_weights)
    handobj_model = YOLO(handobj_weights)
    handobj_names_dict = _get_class_names(handobj_model)
    print("handobj 模型类别:", handobj_names_dict)
    hand_ids_vis = _find_class_ids(handobj_names_dict, "hand", "left_hand", "right_hand")
    obj_ids_vis = _find_class_ids(handobj_names_dict, "object", "obj", "thing", "item")

    # Jetson 上 CUDA 分配器有时会在相机和首次推理并发初始化时触发 NVML/NvMap 异常。
    # 先做一次小图预热，尽量把显存和执行上下文提前稳定下来；若仍失败则退回 CPU。
    if yolo_device == "cuda":
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            person_model.predict(
                dummy, conf=args.conf, imgsz=min(640, args.imgsz), verbose=False, device=yolo_device
            )
            handobj_model.predict(
                dummy, conf=args.conf, imgsz=min(640, args.imgsz), verbose=False, device=yolo_device
            )
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print("YOLO 双模型 CUDA 预热完成")
        except RuntimeError as e:
            if _is_cuda_allocator_error(e):
                print("CUDA 预热失败（显存/分配器异常），自动切换到 CPU:", e)
                yolo_device = "cpu"
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            else:
                raise

    max_depth_mm = int((args.max_depth or MAX_DEPTH_M) * 1000)
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
            print("使用默认彩色 profile:", color_profile)
        config.enable_stream(color_profile)
        depth_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        config.enable_stream(depth_list.get_default_video_stream_profile())
        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
    except Exception as e:
        print("配置 RGB-D 流失败:", type(e).__name__, e)
        return 1
    try:
        pipeline.start(config)
    except Exception as e:
        print("启动 RGB-D 相机失败:", type(e).__name__, e)
        return 1
    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    time.sleep(1.5)

    headless = getattr(args, "headless", False) or getattr(args, "stream_enable", False)
    print("RGB-D 已开（彩色 %s），%s" % (args.color_res, "Ctrl+C 退出（无界面）" if headless else "按 q 退出"))
    sender = None
    if getattr(args, "udp_enable", False):
        sender = UdpTargetSender(
            host=args.udp_host,
            port=args.udp_port,
            frame_id=args.udp_frame_id,
            source="handobj_detection_rgbd",
            max_rate_hz=args.udp_rate,
        )
        print("UDP 发送已启用: %s:%s frame_id=%s kind=%s rate<=%.1f Hz" % (
            args.udp_host, args.udp_port, args.udp_frame_id, args.udp_kind, args.udp_rate))

    stream_holder = None
    stream_server = None
    if getattr(args, "stream_enable", False):
        try:
            stream_server = MjpegStreamServer(
                bind_addr=getattr(args, "stream_bind", "0.0.0.0"),
                port=getattr(args, "stream_port", 8765),
            )
            stream_holder = stream_server.start()
            print(
                "MJPEG 流已启动: http://%s:%d/stream  (或 http://%s:%d/)"
                % (getattr(args, "stream_bind", "0.0.0.0"), int(getattr(args, "stream_port", 8765)),
                   getattr(args, "stream_bind", "0.0.0.0"), int(getattr(args, "stream_port", 8765))),
                flush=True,
            )
        except Exception as e:
            stream_holder = None
            stream_server = None
            print("MJPEG 流启动失败，已跳过:", type(e).__name__, e, flush=True)
    if not headless:
        cv2.namedWindow("Hand-Object RGB-D", cv2.WINDOW_NORMAL)

    fps_interval_start = time.perf_counter()
    fps_frame_count = 0
    yolo_time_sum = 0.0

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

            color_img, depth_copy = rotate_rgbd_180(
                color_img,
                depth_copy,
                enabled=not getattr(args, "no_rotate_180", False),
            )

            if color_img is None or depth_copy is None or depth_copy.ndim != 2:
                if headless:
                    time.sleep(0.02)
                continue

            dh, dw = depth_copy.shape[0], depth_copy.shape[1]
            if (color_img.shape[0], color_img.shape[1]) != (dh, dw):
                frame = cv2.resize(color_img, (dw, dh), interpolation=cv2.INTER_LINEAR)
            else:
                frame = color_img.copy()

            t_yolo_start = time.perf_counter()
            max_yolo_retries = 3 if yolo_device == "cuda" else 1
            for yolo_attempt in range(max_yolo_retries):
                try:
                    person_results = person_model.predict(
                        frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device
                    )[0]
                    handobj_results = handobj_model.predict(
                        frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device
                    )[0]
                    break
                except RuntimeError as e:
                    if (
                        yolo_attempt < max_yolo_retries - 1
                        and yolo_device == "cuda"
                        and _is_cuda_allocator_error(e)
                    ):
                        time.sleep(0.15 * (yolo_attempt + 1))
                        continue
                    if yolo_device == "cuda" and _is_cuda_allocator_error(e):
                        print("CUDA 推理失败（显存/分配器异常），自动切换到 CPU 继续运行:", e)
                        yolo_device = "cpu"
                        if torch.cuda.is_available():
                            torch.cuda.empty_cache()
                        person_results = person_model.predict(
                            frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device
                        )[0]
                        handobj_results = handobj_model.predict(
                            frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device
                        )[0]
                        break
                    raise
            yolo_time_sum += time.perf_counter() - t_yolo_start

            holding_list = get_holding_person_boxes_and_xyz_dual(
                person_results,
                handobj_results,
                handobj_names_dict,
                depth_copy,
                dh,
                dw,
                conf_threshold=args.conf,
                max_range_mm=max_depth_mm,
                ignore_self_touch=not getattr(args, "no_self_touch_filter", False),
                self_touch_inside_thr=float(getattr(args, "self_touch_inside_thr", 0.85)),
                ignore_self_touch_objects=True,
                self_touch_object_inside_thr=float(getattr(args, "self_touch_object_inside_thr", 0.92)),
                self_touch_object_same_size_area_thr=float(
                    getattr(args, "self_touch_object_same_size_area_thr", 0.9)
                ),
                self_touch_object_same_size_iou_thr=float(
                    getattr(args, "self_touch_object_same_size_iou_thr", 0.6)
                ),
                hand_object_overlap_iou_thr=float(getattr(args, "hand_object_overlap_iou_thr", 0.05)),
            )

            # 最近手持物体的人
            nearest_xyz = None
            nearest_box = None
            if holding_list:
                (x1, y1, x2, y2), xyz, conf = holding_list[0]
                nearest_box = (x1, y1, x2, y2)
                nearest_xyz = xyz
                if getattr(args, "print_xyz", False):
                    print("最近持物者 XYZ: X=%.3f  Y=%.3f  Z=%.3f m  (conf=%.2f)" % (
                        xyz[0], xyz[1], xyz[2], conf), flush=True)

            if sender is not None:
                sender.send_xyz(nearest_xyz, args.udp_kind)

            fps_frame_count += 1
            elapsed_sec = time.perf_counter() - fps_interval_start
            if elapsed_sec >= 1.0 and fps_frame_count > 0:
                loop_fps = fps_frame_count / elapsed_sec
                avg_yolo_s = yolo_time_sum / fps_frame_count
                yolo_fps = 1.0 / avg_yolo_s if avg_yolo_s > 0 else 0.0
                line = "FPS: %.1f (循环) | YOLO: %.1f fps (推理)" % (loop_fps, yolo_fps)
                if nearest_xyz is not None:
                    line += " | XYZ: %.2f, %.2f, %.2f m" % (nearest_xyz[0], nearest_xyz[1], nearest_xyz[2])
                else:
                    line += " | XYZ: —"
                print(line, flush=True)
                fps_interval_start = time.perf_counter()
                fps_frame_count = 0
                yolo_time_sum = 0.0

            # 可视化
            vis = frame.copy()
            # 画人框（蓝色）
            if person_results.boxes is not None:
                for xyxy, conf, cls in zip(
                    person_results.boxes.xyxy.cpu().numpy(),
                    person_results.boxes.conf.cpu().numpy(),
                    person_results.boxes.cls.cpu().numpy(),
                ):
                    if float(conf) < args.conf:
                        continue
                    cls_id = int(cls)
                    if cls_id != int(COCO_PERSON_CLASS_ID):
                        continue
                    x1, y1, x2, y2 = map(int, xyxy)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(
                        vis,
                        "person %.2f" % float(conf),
                        (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        1,
                    )

            # 画 hand/object 框（绿色）
            if handobj_results.boxes is not None:
                for xyxy, conf, cls in zip(
                    handobj_results.boxes.xyxy.cpu().numpy(),
                    handobj_results.boxes.conf.cpu().numpy(),
                    handobj_results.boxes.cls.cpu().numpy(),
                ):
                    if float(conf) < args.conf:
                        continue
                    cls_id = int(cls)
                    if cls_id not in hand_ids_vis and cls_id not in obj_ids_vis:
                        continue
                    x1, y1, x2, y2 = map(int, xyxy)
                    name = handobj_names_dict.get(int(cls), "cls%d" % int(cls))
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        vis,
                        "%s %.2f" % (name, float(conf)),
                        (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )
            if nearest_box is not None and nearest_xyz is not None:
                x1, y1, x2, y2 = map(int, nearest_box)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.putText(vis, "Nearest holding: (%.2f, %.2f, %.2f) m" % nearest_xyz,
                            (x1, y1 - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if stream_holder is not None:
                stream_holder.set_frame(vis, quality=getattr(args, "stream_quality", 85))

            if not headless:
                cv2.imshow("Hand-Object RGB-D", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if stream_server is not None:
            try:
                stream_server.server.shutdown()
            except Exception:
                pass
            try:
                stream_server.server.server_close()
            except Exception:
                pass
        try:
            pipeline.stop()
        except Exception:
            pass
        if not headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
