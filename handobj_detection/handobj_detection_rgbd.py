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
from pathlib import Path

# Jetson CUDA 分配器（在 import torch 之前）
if "PYTORCH_CUDA_ALLOC_CONF" not in os.environ:
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "backend:native"

# 无显示器（SSH/headless）时避免 Qt/xcb 报错，须在 import cv2 之前设置
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
ROOT = Path(__file__).resolve().parent
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


def get_holding_person_boxes_and_xyz(results, names_dict, depth_mm, depth_h, depth_w,
                                     conf_threshold=0.35, max_range_mm=5000,
                                     person_keywords=("person", "human"),
                                     hand_keywords=("hand", "left_hand", "right_hand"),
                                     object_keywords=("object", "obj", "thing", "item")):
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


def main():
    p = argparse.ArgumentParser(description="Hand-Object RGB-D：最近手持物体的人的 XYZ")
    p.add_argument("--weights", "-w", default=str(DEFAULT_WEIGHTS), help="权重路径")
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
    args = p.parse_args()

    weights = Path(args.weights)
    if not weights.is_absolute():
        weights = ROOT / weights
    if not weights.exists():
        alt = WEIGHT_DIR / "best.pt"
        if alt.exists():
            weights = alt
        else:
            print("未找到权重:", weights)
            return 1
    weights = str(weights)

    yolo_device = "cuda" if torch.cuda.is_available() else "cpu"
    if getattr(args, "device", "auto") != "auto":
        yolo_device = args.device
    print("YOLO 推理设备:", yolo_device)

    model = YOLO(weights)
    names_dict = _get_class_names(model)
    print("模型类别:", names_dict)

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

    headless = getattr(args, "headless", False)
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
            results = model.predict(
                frame, conf=args.conf, imgsz=args.imgsz, verbose=False, device=yolo_device
            )[0]
            yolo_time_sum += time.perf_counter() - t_yolo_start

            holding_list = get_holding_person_boxes_and_xyz(
                results, names_dict, depth_copy, dh, dw,
                conf_threshold=args.conf, max_range_mm=max_depth_mm,
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
            if results.boxes is not None:
                for xyxy, conf, cls in zip(
                    results.boxes.xyxy.cpu().numpy(),
                    results.boxes.conf.cpu().numpy(),
                    results.boxes.cls.cpu().numpy(),
                ):
                    if float(conf) < args.conf:
                        continue
                    x1, y1, x2, y2 = map(int, xyxy)
                    name = names_dict.get(int(cls), "cls%d" % int(cls))
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(vis, "%s %.2f" % (name, float(conf)), (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if nearest_box is not None and nearest_xyz is not None:
                x1, y1, x2, y2 = map(int, nearest_box)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.putText(vis, "Nearest holding: (%.2f, %.2f, %.2f) m" % nearest_xyz,
                            (x1, y1 - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if not headless:
                cv2.imshow("Hand-Object RGB-D", vis)
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
