# -*- coding: utf-8 -*-
"""
15 类垃圾检测 + RGB-D 相机（Orbbec Femto Bolt / K4A Wrapper）。

与 predict_15cls 相同：Metal 类别 bias、agnostic_nms=False（多类别框可重叠）。
输入为 RGB-D 的彩色流；可选在框上显示深度距离、另开窗口显示深度图。

前置：K4A_DLL_DIR 指向 Orbbec bin；pip install pyk4a opencv-python numpy ultralytics
运行（在 trash_detection 或 ELEC70015 根目录）：
  $env:K4A_DLL_DIR = "F:\year4\HCRYOLO\OrbbecSDK_K4A_Wrapper_...\bin"
  python trash_detection/predict_15cls_rgbd.py
  python trash_detection/predict_15cls_rgbd.py --no-depth-window   # 不显示深度窗口
  python trash_detection/predict_15cls_rgbd.py --max-depth 3      # 只显示 3m 内并标距离
  python trash_detection/predict_15cls_rgbd.py --color-res 1080p # 彩色流改为 1080p（若设备不支持 1440p）
  python trash_detection/predict_15cls_rgbd.py --nearest-person  # 每帧输出「离最近垃圾最近的人」的 XYZ（米）
  --nearest-person 时：15 类模型禁用 Human，人由 YOLOv8 预训练模型(yolov8m.pt)检测并用于跟随。
  重叠框：agnostic_nms=False 且默认 iou=0.65，重叠的垃圾/人框更易保留。
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import cv2

try:
    from pyk4a import PyK4A
    from pyk4a.config import Config, ColorResolution
except (ModuleNotFoundError, ImportError) as e:
    print("未找到 pyk4a:", e)
    sys.exit(1)

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

# 彩色流分辨率与 pyk4a 枚举对应
COLOR_RES_MAP = {
    "720p": ColorResolution.RES_720P,   # 1280x720
    "1080p": ColorResolution.RES_1080P,  # 1920x1080
    "1440p": ColorResolution.RES_1440P,  # 2560x1440
}
# 深度图内参（K4A/Orbbec 深度分辨率可能与彩色不同）
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
    p.add_argument("--imgsz", type=int, default=1280, help="YOLO 推理输入尺寸（1440p 流默认 1280）")
    p.add_argument("--show-human", action="store_true", help="画出 Human 框")
    p.add_argument("--agnostic-nms", action="store_true", help="NMS 不区分类别")
    p.add_argument("--iou", type=float, default=NMS_IOU_OVERLAP, help="NMS IoU 阈值，越大重叠框越易保留，默认 %.2f" % NMS_IOU_OVERLAP)
    p.add_argument("--no-depth-window", action="store_true", help="不显示深度图窗口")
    p.add_argument("--max-depth", type=float, default=None, help="仅在框上显示该距离(m)内的深度，不设则都显示框不标距离")
    p.add_argument("--color-res", choices=["720p", "1080p", "1440p"], default="1440p", help="彩色流分辨率，默认 1440p")
    p.add_argument("--nearest-person", action="store_true", help="每帧输出「离最近垃圾最近的人」的 XYZ；人用 YOLOv8 预训练检测")
    p.add_argument("--nearest-range", type=float, default=5.0, help="参与最近垃圾/人的深度范围(m)，默认 5")
    p.add_argument("--person-weights", default=PERSON_MODEL_DEFAULT, help="检测人用的 YOLO 权重，默认 yolov8m.pt（COCO person）")
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
        print("人检测模型: %s (COCO class 0 = person)" % person_weights)

    max_depth_mm = int(args.max_depth * 1000) if args.max_depth is not None else None

    color_res = COLOR_RES_MAP.get(args.color_res, ColorResolution.RES_1440P)
    k4a_config = Config(color_resolution=color_res)
    k4a = PyK4A(config=k4a_config)
    try:
        k4a.start()
    except Exception as e:
        print("启动 RGB-D 相机失败:", type(e).__name__, e)
        print("若设备不支持 1440p，可尝试: --color-res 1080p 或 --color-res 720p")
        return 1

    print("RGB-D 已开（彩色 %s），按 q 退出。" % args.color_res)
    if getattr(args, "nearest_person", False):
        print("  [L] 锁定当前选中的垃圾与人并同时跟随；[U] 解锁")
    time.sleep(1)
    win_color = "15cls Waste (RGB-D)"
    win_depth = "Depth"
    cv2.namedWindow(win_color, cv2.WINDOW_NORMAL)
    if not args.no_depth_window:
        cv2.namedWindow(win_depth, cv2.WINDOW_NORMAL)

    locked_person_xyz = None
    locked_waste_xyz = None
    follow_max_m = 2.0

    try:
        while True:
            capture = None
            try:
                capture = k4a.get_capture()
            except Exception:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            if capture is None:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            color_img = None
            depth_copy = None
            try:
                c = capture.color
                if c is not None and c.size > 0:
                    arr = np.asarray(c)
                    if arr.ndim == 3 and arr.shape[2] == 4:
                        color_img = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_BGRA2BGR)
                    else:
                        color_img = np.asarray(c).copy()
            except Exception:
                pass
            try:
                d = capture.depth
                if d is not None and np.asarray(d).size > 0:
                    depth_copy = np.asarray(d, dtype=np.uint16).copy()
            except Exception:
                pass

            if color_img is None:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            dh, dw = (depth_copy.shape[0], depth_copy.shape[1]) if depth_copy is not None and depth_copy.ndim == 2 else (color_img.shape[0], color_img.shape[1])
            if (color_img.shape[0], color_img.shape[1]) != (dh, dw):
                frame = cv2.resize(color_img, (dw, dh), interpolation=cv2.INTER_LINEAR)
            else:
                frame = color_img.copy()

            nms_iou = getattr(args, "iou", NMS_IOU_OVERLAP)
            results_waste = model.predict(
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                iou=nms_iou,
                agnostic_nms=agnostic_nms,
                classes=trash_class_ids,
                verbose=False,
            )[0]
            results_person = None
            if person_model is not None:
                results_person = person_model.predict(
                    frame, conf=args.conf, imgsz=args.imgsz, iou=nms_iou, classes=[COCO_PERSON_CLASS_ID],
                    verbose=False,
                )[0]

            draw_all_boxes = True
            if getattr(args, "nearest_person", False) and depth_copy is not None and depth_copy.ndim == 2:
                max_range_mm = int(getattr(args, "nearest_range", 5.0) * 1000)
                waste_xyz, person_xyz, all_persons, all_wastes = get_nearest_waste_and_person_xyz_from_two_models(
                    results_waste, results_person, names_list, depth_copy, dh, dw, args.conf, max_range_mm
                )
                key = cv2.waitKey(1) & 0xFF
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
            else:
                key = cv2.waitKey(1) & 0xFF
            if draw_all_boxes:
                frame = draw_results(
                    frame, results_waste, names_list, hide_human=False, conf_threshold=args.conf,
                    depth_mm=depth_copy, depth_h=dh, depth_w=dw, max_depth_mm=max_depth_mm,
                )
                if person_model is not None:
                    frame = draw_person_boxes(frame, results_person, args.conf, color=(255, 255, 0))
            cv2.imshow(win_color, frame)
            if not args.no_depth_window and depth_copy is not None and depth_copy.ndim == 2:
                vis = depth_vis(depth_copy)
                if vis is not None:
                    cv2.imshow(win_depth, vis)
            if key == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            k4a.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
