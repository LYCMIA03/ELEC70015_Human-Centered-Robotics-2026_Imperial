# -*- coding: utf-8 -*-
"""
15 类垃圾检测 + RGB-D 相机（Orbbec SDK v2 / pyorbbecsdk）。

与 predict_15cls 相同：Metal 类别 bias、agnostic_nms=False（多类别框可重叠）。
输入为 RGB-D 的彩色流；可选在框上显示深度距离、另开窗口显示深度图。

前置：pip install pyorbbecsdk opencv-python numpy ultralytics（Jetson/Linux 需先装 udev 规则）。
运行（在 trash_detection 或 ELEC70015 根目录）：
  python trash_detection/predict_15cls_rgbd.py
  python trash_detection/predict_15cls_rgbd.py --no-depth-window   # 不显示深度窗口
  python trash_detection/predict_15cls_rgbd.py --max-depth 3      # 只显示 3m 内并标距离
  python trash_detection/predict_15cls_rgbd.py --color-res 1080p # 彩色流 1080p（多数 Orbbec 支持）
  python trash_detection/predict_15cls_rgbd.py --nearest-person   # 每帧输出「离最近垃圾最近的人」的 XYZ（米）
  python trash_detection/predict_15cls_rgbd.py --nearest-person --person-hz 5  # 人检测低频，垃圾检测高频
  --nearest-person 时：15 类模型禁用 Human，人由 YOLOv8 预训练模型(yolov8m.pt)检测并用于跟随。
  重叠框：agnostic_nms=False 且默认 iou=0.65，重叠的垃圾/人框更易保留。
"""
import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

# Jetson 上可缓解 CUDA 分配器 NVML 断言失败（在 import torch 之前设置）
if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native"

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

from ultralytics.utils.nms import TorchNMS

# Jetson 上 torchvision 与 PyTorch 的 C++ 扩展不兼容时，nms 会报 "Couldn't load custom C++ ops"
# 对 torchvision.ops.nms 做回退：失败则用 ultralytics 自带的 TorchNMS.nms
# try:
#     import torchvision.ops as _tv_ops
#     _orig_tv_nms = _tv_ops.nms
#     def _nms_fallback(boxes, scores, iou_threshold):
#         try:
#             return _orig_tv_nms(boxes, scores, iou_threshold)
#         except RuntimeError:
#             return TorchNMS.nms(boxes, scores, iou_threshold)
#     _tv_ops.nms = _nms_fallback
# except Exception:
#     pass

try:
    import torchvision.ops as _tv_ops
    from ultralytics.utils.nms import TorchNMS

    def _safe_nms(boxes, scores, iou_threshold):
        return TorchNMS.nms(boxes, scores, iou_threshold)

    _tv_ops.nms = _safe_nms
    print("✓ Using Ultralytics TorchNMS (torchvision C++ bypassed)")
except Exception as e:
    print("TorchNMS patch failed:", e)
    
    
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor


# 与 predict_15cls 一致
ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "weights3" / "epoch80.pt"
CONF_THRESHOLD = 0.2
METAL_CLS_BIAS = 0.4
HIDE_HUMAN = True
AGNOSTIC_NMS = False
CLASS_NAMES_15 = [
    "Metal", "Cardboard", "Glass", "Paper", "Plastic", "Tetra",
    "Apple", "Apple-core", "Apple-peel", "Bread", "Orange", "Orange-peel",
    "Pear", "Vegetable", "Human",
]

# 彩色流分辨率 (width, height) for pyorbbecsdk
COLOR_RES_MAP = {
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "1440p": (2560, 1440),
}
# 深度图内参（Orbbec 深度分辨率可能与彩色不同，可按相机标定修改）
DEPTH_FX, DEPTH_FY = 500.0, 500.0
DEPTH_CX, DEPTH_CY = 319.5, 287.5

# 15 类中 Human 的 id（用于排除）；垃圾类 id 为 0..NUM_WASTE_CLASSES-1
HUMAN_CLASS_ID_15 = 14
NUM_WASTE_CLASSES = 14
# 跟随/最近人时用 YOLOv8 预训练模型检测人（COCO class 0 = person）
PERSON_MODEL_DEFAULT = "yolov8m.pt"
COCO_PERSON_CLASS_ID = 0
# NMS IoU 阈值：调高后重叠的框更不易被抑制，便于识别“与人/物重叠”的垃圾
NMS_IOU_OVERLAP = 0.65


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
    # 其他格式用 FormatConvertFilter 转 RGB 再转 BGR
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


def get_class_names(model):
    names = getattr(model, "names", None)
    if names is not None:
        return names
    return CLASS_NAMES_15


def _install_metal_class_bias(bias=0.1):
    if getattr(DetectionPredictor, "_metal_bias_patched", False):
        return
    _original_postprocess = DetectionPredictor.postprocess

    def _postprocess_with_metal_bias(self, preds, img, orig_imgs, **kwargs):
        if preds is not None and getattr(preds, "dim", lambda: 0)() >= 3 and preds.shape[-2] >= 5:
            names = getattr(self.model, "names", None)
            if names is not None:
                first_name = names.get(0) if isinstance(names, dict) else (names[0] if len(names) > 0 else None)
                if first_name == "Metal":
                    preds = preds.clone()
                    # ultralytics NMS 输入 shape (B, 4+nc, N)：前 4 为框，接着 nc 个类别得分，Metal(类别0) 在索引 4
                    preds[..., 4, :] += bias
        return _original_postprocess(self, preds, img, orig_imgs, **kwargs)

    DetectionPredictor.postprocess = _postprocess_with_metal_bias
    DetectionPredictor._metal_bias_patched = True


def sample_depth_at_box_center(depth, x1, y1, x2, y2, h, w):
    """框中心及邻域采样深度（mm），无效返回 None。"""
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    d = int(depth[cy, cx])
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


def depth_pixel_to_xyz_m(px, py, depth_mm, fx=DEPTH_FX, fy=DEPTH_FY, cx=DEPTH_CX, cy=DEPTH_CY):
    """深度图像素 (px, py) 及深度 depth_mm -> 相机系下 (x, y, z) 单位米。"""
    if depth_mm is None or depth_mm <= 0:
        return None
    z_m = float(depth_mm) / 1000.0
    x_m = (px - cx) * z_m / fx
    y_m = (py - cy) * z_m / fy
    return (x_m, y_m, z_m)


def _dist_sq_xyz(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def _is_cuda_allocator_error(exc):
    msg = str(exc)
    return (
        "CUDACachingAllocator" in msg
        or "NVML" in msg
        or "NvMapMemAllocInternalTagged" in msg
        or "cuda out of memory" in msg.lower()
    )


def get_nearest_waste_and_person_xyz(results, names_list, depth_mm, depth_h, depth_w, conf_threshold=0.25, max_range_mm=5000):
    """
    从检测结果和深度图得到：1) 离相机最近的垃圾的 XYZ；2) 离该垃圾最近的人的 XYZ（米）；
    3) 本帧所有带 XYZ 的人 all_persons；4) 本帧所有带 XYZ 的垃圾 all_wastes。
    返回 (nearest_waste_xyz, person_xyz, all_persons, all_wastes)。
    """
    if results.boxes is None or len(results.boxes) == 0 or depth_mm is None or depth_mm.ndim != 2:
        return None, None, [], []
    dh, dw = depth_mm.shape[0], depth_mm.shape[1]
    waste_list = []
    person_list = []
    for xyxy, conf, cls in zip(
        results.boxes.xyxy.cpu().numpy(),
        results.boxes.conf.cpu().numpy(),
        results.boxes.cls.cpu().numpy(),
    ):
        cls_id = int(cls)
        if cls_id < 0 or cls_id >= len(names_list) or float(conf) < conf_threshold:
            continue
        name = names_list[cls_id]
        x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
        d_mm = sample_depth_at_box_center(depth_mm, x1, y1, x2, y2, dh, dw)
        if d_mm is None or d_mm <= 0 or d_mm > max_range_mm:
            continue
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        xyz = depth_pixel_to_xyz_m(cx, cy, d_mm)
        if xyz is None:
            continue
        if name == "Human":
            person_list.append((x1, y1, x2, y2, cls_id, float(conf), d_mm, xyz))
        else:
            waste_list.append((x1, y1, x2, y2, cls_id, float(conf), d_mm, xyz))
    all_persons = [(t[0], t[1], t[2], t[3], t[7]) for t in person_list]
    all_wastes = [(t[0], t[1], t[2], t[3], t[7]) for t in waste_list]
    if not waste_list:
        return None, None, all_persons, all_wastes
    nearest_waste = min(waste_list, key=lambda t: t[6])
    waste_xyz = nearest_waste[7]
    if not person_list:
        return waste_xyz, None, all_persons, all_wastes
    nearest_person = min(person_list, key=lambda t: _dist_sq_xyz(t[7], waste_xyz))
    return waste_xyz, nearest_person[7], all_persons, all_wastes


def _boxes_with_xyz_from_results(results, depth_mm, depth_h, depth_w, conf_threshold, max_range_mm):
    """从单次检测 results 提取 (x1,y1,x2,y2, xyz) 列表，深度无效或超范围则跳过。"""
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
    results_waste, results_person, names_waste, depth_mm, depth_h, depth_w,
    conf_threshold=0.25, max_range_mm=5000,
):
    """
    垃圾来自 15 类模型（已排除 Human），人来自 YOLOv8 预训练模型。
    返回 (nearest_waste_xyz, person_xyz, all_persons, all_wastes)。
    """
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


def follow_locked_target(all_items, locked_xyz, max_follow_m=2.0):
    """
    在 all_items（list of (x1,y1,x2,y2, xyz)）中找离 locked_xyz 最近的目标（3D 距离 < max_follow_m），
    返回其 (box, xyz) 或 (None, None)。
    """
    if not all_items or locked_xyz is None:
        return None, None
    max_sq = max_follow_m * max_follow_m
    best = None
    best_sq = max_sq
    for (x1, y1, x2, y2, xyz) in all_items:
        d_sq = _dist_sq_xyz(xyz, locked_xyz)
        if d_sq < best_sq:
            best_sq = d_sq
            best = ((x1, y1, x2, y2), xyz)
    return (best[0], best[1]) if best is not None else (None, None)


def follow_locked_person(all_persons, locked_xyz, max_follow_m=2.0):
    """在 all_persons 中找离 locked_xyz 最近的人，返回其 (box, xyz) 或 (None, None)。"""
    return follow_locked_target(all_persons, locked_xyz, max_follow_m)


def depth_vis(depth_uint16, max_mm=3000):
    """深度图转伪彩色 BGR uint8。"""
    d = np.asarray(depth_uint16, dtype=np.float32)
    if d.ndim != 2 or d.size == 0:
        return None
    d = np.clip(d, 0, max_mm)
    mx = float(d.max())
    if mx <= 0:
        return np.zeros((*d.shape, 3), dtype=np.uint8)
    u8 = (d / mx * 255).astype(np.uint8)
    return cv2.applyColorMap(u8, cv2.COLORMAP_JET)


def draw_results(frame, results, names_list, hide_human=True, conf_threshold=0.25,
                 depth_mm=None, depth_h=None, depth_w=None, max_depth_mm=None):
    if results.boxes is None or len(results.boxes) == 0:
        return frame
    has_depth = depth_mm is not None and depth_mm.ndim == 2 and depth_h and depth_w
    for xyxy, conf, cls in zip(
        results.boxes.xyxy.cpu().numpy(),
        results.boxes.conf.cpu().numpy(),
        results.boxes.cls.cpu().numpy(),
    ):
        cls_id = int(cls)
        if cls_id < 0 or cls_id >= len(names_list):
            continue
        label_name = names_list[cls_id]
        if hide_human and label_name == "Human":
            continue
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = map(int, xyxy)
        label = f"{label_name} {float(conf):.2f}"
        if has_depth and max_depth_mm is not None:
            d_mm = sample_depth_at_box_center(depth_mm, x1, y1, x2, y2, depth_h, depth_w)
            if d_mm is not None and 0 < d_mm <= max_depth_mm:
                label += f" {d_mm/1000:.2f}m"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def draw_person_boxes(frame, results_person, conf_threshold=0.25, color=(255, 255, 0)):
    """绘制 YOLOv8 预训练模型检测到的人框（用于 --nearest-person 时）。"""
    if results_person is None or results_person.boxes is None or len(results_person.boxes) == 0:
        return frame
    for xyxy, conf, cls in zip(
        results_person.boxes.xyxy.cpu().numpy(),
        results_person.boxes.conf.cpu().numpy(),
        results_person.boxes.cls.cpu().numpy(),
    ):
        if float(conf) < conf_threshold:
            continue
        x1, y1, x2, y2 = map(int, xyxy)
        label = "person %.2f" % float(conf)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return frame


def main():
    p = argparse.ArgumentParser(description="15 类垃圾检测 RGB-D")
    p.add_argument("--weights", "-w", default=str(MODEL_PATH), help="权重路径")
    p.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="置信度阈值")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO 推理输入尺寸（默认 640，性能优先）")
    p.add_argument("--show-human", action="store_true", help="画出 Human 框")
    p.add_argument("--agnostic-nms", action="store_true", help="NMS 不区分类别")
    p.add_argument("--iou", type=float, default=NMS_IOU_OVERLAP, help="NMS IoU 阈值，越大重叠框越易保留，默认 %.2f" % NMS_IOU_OVERLAP)
    p.add_argument("--no-depth-window", action="store_true", help="不显示深度图窗口")
    p.add_argument("--max-depth", type=float, default=None, help="仅在框上显示该距离(m)内的深度，不设则都显示框不标距离")
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1080p", help="彩色流分辨率，默认 1080p")
    p.add_argument("--nearest-person", action="store_true", help="每帧输出「离最近垃圾最近的人」的 XYZ；人用 YOLOv8 预训练检测")
    p.add_argument("--nearest-range", type=float, default=5.0, help="参与最近垃圾/人的深度范围(m)，默认 5")
    p.add_argument("--person-weights", default=PERSON_MODEL_DEFAULT, help="检测人用的 YOLO 权重，默认 yolov8m.pt（COCO person）")
    p.add_argument("--person-hz", type=float, default=5.0, help="人检测更新频率(Hz)，默认 5（垃圾检测仍每帧）")
    p.add_argument("--print-xyz", action="store_true", help="每帧在终端打印最近垃圾/人的 XYZ（适合 SSH 无界面）")
    p.add_argument("--headless", action="store_true", help="无界面模式：不弹窗，仅推理并打印 XYZ，用 Ctrl+C 退出（适合 SSH）")
    p.add_argument("--device", choices=["cuda", "cpu", "auto"], default="auto", help="YOLO 推理设备：cuda/cpu/auto，默认 auto（有 GPU 用 cuda）")
    p.add_argument("--udp-enable", action="store_true", help="启用 UDP JSON 发送 XYZ（用于对接 Docker 内 udp_target_bridge）")
    p.add_argument("--udp-host", default="127.0.0.1", help="UDP 目标 host，默认 127.0.0.1")
    p.add_argument("--udp-port", type=int, default=16031, help="UDP 目标端口，默认 16031（与 ROS master 11311 分离）")
    p.add_argument("--udp-frame-id", default="camera_link", help="发送时使用的 frame_id，默认 camera_link")
    p.add_argument("--udp-kind", choices=["waste", "person", "auto"], default="person", help="发送对象：waste/person/auto(优先 waste)")
    p.add_argument("--udp-rate", type=float, default=10.0, help="UDP 最大发送频率(Hz)，默认 10")
    args = p.parse_args()

    weights = Path(args.weights)
    if not weights.exists():
        for alt in (ROOT.parent / "weights3" / "epoch80.pt", ROOT / "weights2" / "best.pt"):
            if alt.exists():
                weights = alt
                break
        else:
            print(f"未找到权重: {weights}")
            return 1
    weights = str(weights)

    # 显式选择设备：--device auto 时有 CUDA 用 GPU，否则 CPU；可传 --device cuda/cpu 覆盖
    if getattr(args, "device", "auto") == "auto":
        yolo_device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        yolo_device = args.device
    print("YOLO 推理设备:", yolo_device, "(CUDA 可用)" if (yolo_device == "cuda" and torch.cuda.is_available()) else "")

    model = YOLO(weights)

    class_names = get_class_names(model)
    if isinstance(class_names, dict):
        n = max(class_names.keys()) + 1 if class_names else 0
        names_list = [class_names.get(i, f"cls{i}") for i in range(n)]
    else:
        names_list = list(class_names) if class_names else CLASS_NAMES_15
    print("类别 (id -> name):", dict(enumerate(names_list)))

    hide_human = HIDE_HUMAN and not args.show_human
    agnostic_nms = args.agnostic_nms or AGNOSTIC_NMS
    _install_metal_class_bias(METAL_CLS_BIAS)
    # 15 类模型只检垃圾（不检 Human）；若 --nearest-person 则人由 YOLOv8 预训练模型检
    trash_class_ids = list(range(NUM_WASTE_CLASSES))

    person_model = None
    if getattr(args, "nearest_person", False):
        person_weights = getattr(args, "person_weights", PERSON_MODEL_DEFAULT)
        person_model = YOLO(person_weights)
        print(
            "人检测模型: %s (COCO class 0 = person)，使用设备: %s，更新频率: %.2f Hz"
            % (person_weights, yolo_device, max(getattr(args, "person_hz", 5.0), 0.1))
        )

    # CUDA 时在开相机前先做一次小图推理，预占 GPU 显存，减轻与 Orbbec/深度管线争用导致的分配器错误
    if yolo_device == "cuda":
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=min(640, args.imgsz), verbose=False, device=yolo_device)
            if person_model is not None:
                person_model.predict(dummy, imgsz=min(640, args.imgsz), verbose=False, device=yolo_device)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print("YOLO CUDA 预热完成")
        except RuntimeError as e:
            if _is_cuda_allocator_error(e):
                print("CUDA 预热失败（显存/分配器异常），自动切换到 CPU:", e)
                yolo_device = "cpu"
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            else:
                raise

    max_depth_mm = int(args.max_depth * 1000) if args.max_depth is not None else None

    # Orbbec 相机：彩色 + 深度，对齐到彩色
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
        print("若设备不支持当前分辨率，可尝试: --color-res 1080p 或 --color-res 720p")
        return 1
    align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    time.sleep(1.5)  # 等深度流稳定

    headless = getattr(args, "headless", False)
    print("RGB-D 已开（彩色 %s），%s" % (args.color_res, "Ctrl+C 退出（无界面）" if headless else "按 q 退出。"))
    if getattr(args, "nearest_person", False) and not headless:
        print("  [L] 锁定当前选中的垃圾与人并同时跟随；[U] 解锁")
    sender = None
    if getattr(args, "udp_enable", False):
        sender = UdpTargetSender(
            host=args.udp_host,
            port=args.udp_port,
            frame_id=args.udp_frame_id,
            source="trash_detection_rgbd",
            max_rate_hz=args.udp_rate,
        )
        print(
            "UDP 发送已启用: %s:%s frame_id=%s kind=%s rate<=%.1fHz"
            % (args.udp_host, args.udp_port, args.udp_frame_id, args.udp_kind, args.udp_rate)
        )
    time.sleep(1)
    win_color = "15cls Waste (RGB-D)"
    win_depth = "Depth"
    if not headless:
        cv2.namedWindow(win_color, cv2.WINDOW_NORMAL)
        if not args.no_depth_window:
            cv2.namedWindow(win_depth, cv2.WINDOW_NORMAL)

    locked_person_xyz = None
    locked_waste_xyz = None
    follow_max_m = 2.0
    person_interval_s = 1.0 / max(getattr(args, "person_hz", 5.0), 0.1)
    person_last_infer_t = -1e9
    results_person_cached = None

    # FPS 统计：每秒打印一次 循环 FPS 与 YOLO 推理 FPS
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
                depth_copy = (d.astype(np.float32) * scale).astype(np.uint16)  # mm

            if color_img is None:
                if not headless and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                if headless:
                    time.sleep(0.02)
                continue

            dh, dw = (depth_copy.shape[0], depth_copy.shape[1]) if depth_copy is not None and depth_copy.ndim == 2 else (color_img.shape[0], color_img.shape[1])
            if (color_img.shape[0], color_img.shape[1]) != (dh, dw):
                frame = cv2.resize(color_img, (dw, dh), interpolation=cv2.INTER_LINEAR)
            else:
                frame = color_img

            nms_iou = getattr(args, "iou", NMS_IOU_OVERLAP)
            t_yolo_start = time.perf_counter()
            now_t = time.perf_counter()
            should_update_person = (
                person_model is not None and (
                    results_person_cached is None or (now_t - person_last_infer_t) >= person_interval_s
                )
            )
            # Jetson 上 CUDA 分配器有时会间歇性 NVML 断言，同帧重试几次往往能通过
            max_yolo_retries = 3 if yolo_device == "cuda" else 1
            results_person = results_person_cached
            for _yolo_attempt in range(max_yolo_retries):
                try:
                    results_waste = model.predict(
                        frame,
                        conf=args.conf,
                        imgsz=args.imgsz,
                        iou=nms_iou,
                        agnostic_nms=agnostic_nms,
                        classes=trash_class_ids,
                        verbose=False,
                        device=yolo_device,
                    )[0]
                    if should_update_person:
                        results_person = person_model.predict(
                            frame, conf=args.conf, imgsz=args.imgsz, iou=nms_iou, classes=[COCO_PERSON_CLASS_ID],
                            verbose=False,
                            device=yolo_device,
                        )[0]
                        results_person_cached = results_person
                        person_last_infer_t = time.perf_counter()
                    break
                except RuntimeError as e:
                    if _yolo_attempt < max_yolo_retries - 1 and yolo_device == "cuda" and _is_cuda_allocator_error(e):
                        time.sleep(0.15 * (_yolo_attempt + 1))
                        continue
                    if yolo_device == "cuda" and _is_cuda_allocator_error(e):
                        print("CUDA 推理失败（显存/分配器异常），自动切换到 CPU 继续运行:", e)
                        yolo_device = "cpu"
                        if torch.cuda.is_available():
                            torch.cuda.empty_cache()
                        results_waste = model.predict(
                            frame,
                            conf=args.conf,
                            imgsz=args.imgsz,
                            iou=nms_iou,
                            agnostic_nms=agnostic_nms,
                            classes=trash_class_ids,
                            verbose=False,
                            device=yolo_device,
                        )[0]
                        if should_update_person and person_model is not None:
                            results_person = person_model.predict(
                                frame, conf=args.conf, imgsz=args.imgsz, iou=nms_iou, classes=[COCO_PERSON_CLASS_ID],
                                verbose=False,
                                device=yolo_device,
                            )[0]
                            results_person_cached = results_person
                            person_last_infer_t = time.perf_counter()
                        break
                    raise
            yolo_time_sum += time.perf_counter() - t_yolo_start

            draw_all_boxes = True
            send_waste_xyz = None
            send_person_xyz = None
            if getattr(args, "nearest_person", False) and depth_copy is not None and depth_copy.ndim == 2:
                max_range_mm = int(getattr(args, "nearest_range", 5.0) * 1000)
                waste_xyz, person_xyz, all_persons, all_wastes = get_nearest_waste_and_person_xyz_from_two_models(
                    results_waste, results_person, names_list, depth_copy, dh, dw, args.conf, max_range_mm
                )
                send_waste_xyz = waste_xyz
                send_person_xyz = person_xyz
                key = (cv2.waitKey(1) & 0xFF) if not headless else 0
                if key == ord("u"):
                    locked_person_xyz = None
                    locked_waste_xyz = None
                    print("[已解锁]")
                elif key == ord("l") and (waste_xyz is not None or person_xyz is not None):
                    locked_waste_xyz = waste_xyz
                    locked_person_xyz = person_xyz
                    print("[已锁定] 同时跟随选中的垃圾与人，按 U 解锁；仅显示此二人/物框")

                is_locked = locked_waste_xyz is not None or locked_person_xyz is not None
                draw_all_boxes = False
                line_y = 28
                if is_locked:
                    if locked_waste_xyz is not None:
                        wbox, wxyz = follow_locked_target(all_wastes, locked_waste_xyz, follow_max_m) if all_wastes else (None, None)
                        if wbox is not None and wxyz is not None:
                            locked_waste_xyz = wxyz
                            send_waste_xyz = wxyz
                            x1, y1, x2, y2 = int(wbox[0]), int(wbox[1]), int(wbox[2]), int(wbox[3])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                            cv2.putText(frame, "LOCKED waste", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                            x, y, z = wxyz
                            cv2.putText(frame, "Waste XYZ: %.2f, %.2f, %.2f m" % (x, y, z), (10, line_y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                            print("Waste: X=%.3f  Y=%.3f  Z=%.3f m" % (x, y, z))
                            line_y += 28
                        else:
                            cv2.putText(frame, "Waste: LOST", (10, line_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            line_y += 28
                    if locked_person_xyz is not None:
                        pbox, pxyz = follow_locked_person(all_persons, locked_person_xyz, follow_max_m) if all_persons else (None, None)
                        if pbox is not None and pxyz is not None:
                            locked_person_xyz = pxyz
                            send_person_xyz = pxyz
                            x1, y1, x2, y2 = int(pbox[0]), int(pbox[1]), int(pbox[2]), int(pbox[3])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)
                            cv2.putText(frame, "LOCKED person", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                            x, y, z = pxyz
                            cv2.putText(frame, "Person XYZ: %.2f, %.2f, %.2f m" % (x, y, z), (10, line_y),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            print("Person: X=%.3f  Y=%.3f  Z=%.3f m" % (x, y, z))
                        else:
                            cv2.putText(frame, "Person: LOST", (10, line_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    if waste_xyz is not None and all_wastes:
                        wbox, _ = follow_locked_target(all_wastes, waste_xyz, follow_max_m)
                        if wbox is not None:
                            x1, y1, x2, y2 = int(wbox[0]), int(wbox[1]), int(wbox[2]), int(wbox[3])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                            cv2.putText(frame, "nearest waste", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    if person_xyz is not None and all_persons:
                        pbox, _ = follow_locked_target(all_persons, person_xyz, follow_max_m)
                        if pbox is not None:
                            x1, y1, x2, y2 = int(pbox[0]), int(pbox[1]), int(pbox[2]), int(pbox[3])
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)
                            cv2.putText(frame, "nearest person [L]ock", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    if waste_xyz is not None:
                        cv2.putText(frame, "Waste XYZ: %.2f, %.2f, %.2f m" % (waste_xyz[0], waste_xyz[1], waste_xyz[2]), (10, line_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        line_y += 28
                    if person_xyz is not None:
                        cv2.putText(frame, "Person XYZ: %.2f, %.2f, %.2f m  [L]ock" % (person_xyz[0], person_xyz[1], person_xyz[2]), (10, line_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    # SSH 下在终端打印 XYZ
                    if getattr(args, "print_xyz", False):
                        if waste_xyz is not None:
                            print("Waste XYZ: X=%.3f  Y=%.3f  Z=%.3f m" % (waste_xyz[0], waste_xyz[1], waste_xyz[2]), flush=True)
                        if person_xyz is not None:
                            print("Person XYZ: X=%.3f  Y=%.3f  Z=%.3f m" % (person_xyz[0], person_xyz[1], person_xyz[2]), flush=True)
            else:
                # 非 nearest-person 模式也可发送“最近垃圾”的 XYZ
                if depth_copy is not None and depth_copy.ndim == 2:
                    waste_xyz, _, _, _ = get_nearest_waste_and_person_xyz(
                        results_waste, names_list, depth_copy, dh, dw, args.conf, int(getattr(args, "nearest_range", 5.0) * 1000)
                    )
                    send_waste_xyz = waste_xyz
                key = (cv2.waitKey(1) & 0xFF) if not headless else 0

            if sender is not None:
                udp_kind = getattr(args, "udp_kind", "waste")
                chosen_kind = None
                chosen_xyz = None
                if udp_kind == "waste":
                    chosen_kind, chosen_xyz = "waste", send_waste_xyz
                elif udp_kind == "person":
                    chosen_kind, chosen_xyz = "person", send_person_xyz
                else:  # auto
                    if send_waste_xyz is not None:
                        chosen_kind, chosen_xyz = "waste", send_waste_xyz
                    else:
                        chosen_kind, chosen_xyz = "person", send_person_xyz
                sender.send_xyz(chosen_xyz, chosen_kind)
            if draw_all_boxes:
                frame = draw_results(
                    frame, results_waste, names_list, hide_human=False, conf_threshold=args.conf,
                    depth_mm=depth_copy, depth_h=dh, depth_w=dw, max_depth_mm=max_depth_mm,
                )
                if person_model is not None:
                    frame = draw_person_boxes(frame, results_person, args.conf, color=(255, 255, 0))
            fps_frame_count += 1
            elapsed_sec = time.perf_counter() - fps_interval_start
            if elapsed_sec >= 1.0 and fps_frame_count > 0:
                loop_fps = fps_frame_count / elapsed_sec
                avg_yolo_s = yolo_time_sum / fps_frame_count
                yolo_fps = 1.0 / avg_yolo_s if avg_yolo_s > 0 else 0.0
                print("FPS: %.1f (循环) | YOLO: %.1f fps (推理)" % (loop_fps, yolo_fps), flush=True)
                fps_interval_start = time.perf_counter()
                fps_frame_count = 0
                yolo_time_sum = 0.0

            if not headless:
                cv2.imshow(win_color, frame)
                if not args.no_depth_window and depth_copy is not None and depth_copy.ndim == 2:
                    vis = depth_vis(depth_copy)
                    if vis is not None:
                        cv2.imshow(win_depth, vis)
            elif headless:
                time.sleep(0.03)
            if not headless and key == ord("q"):
                break
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
    sys.exit(main() or 0)
