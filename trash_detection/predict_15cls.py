"""
15 类垃圾检测推理（weights2 训练权重）。

用法：
  python predict_15cls.py                         # 摄像头
  python predict_15cls.py 0                      # 摄像头
  python predict_15cls.py path/to/image.jpg      # 图片
  python predict_15cls.py path/to/video.mp4      # 视频
  python predict_15cls.py --weights weights2/best.pt path/to/image.jpg

类别判定偏向 Metal：当某物体在 Metal 与其它类（如 Plastic）置信度接近时，
在 NMS 前给 Metal 类别得分 +0.1，使该物体被识别为 Metal。

重叠框：默认按类别做 NMS（agnostic_nms=False），不同类别（如塑料瓶与铁罐）可同时检出、框可重叠。
"""
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor
import cv2
import sys
import argparse
from pathlib import Path

# 训练好的权重（放在 weights2/best.pt 或 runs/detect/train_15cls/weights/best.pt）
MODEL_PATH = "weights3/epoch90.pt"
CONF_THRESHOLD = 0.2
# 在类别判定时给 Metal 的得分加该值，使 Metal/Plastic 等接近时偏向判为 Metal
METAL_CLS_BIAS = 0.4
# True = 不画 "Human"，只显示垃圾类；False = 人也画出来
HIDE_HUMAN = True
# NMS 按类别做（False）= 不同类别框可重叠，如塑料瓶和铁罐放得近可同时识别
AGNOSTIC_NMS = False

# 15 类（与 yolov8_merged_15cls/data.yaml 一致，仅当 checkpoint 无 names 时兜底）
CLASS_NAMES_15 = [
    "Metal", "Cardboard", "Glass", "Paper", "Plastic", "Tetra",
    "Apple", "Apple-core", "Apple-peel", "Bread", "Orange", "Orange-peel",
    "Pear", "Vegetable", "Human",
]


def get_class_names(model):
    names = getattr(model, "names", None)
    if names is not None:
        return names
    return CLASS_NAMES_15


def _install_metal_class_bias(bias=0.1):
    """在 NMS 前给 Metal（类别 0）的得分加 bias，使 Metal/Plastic 等接近时判为 Metal。"""
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


def draw_results(frame, results, names_list, hide_human=True, conf_threshold=0.25):
    if results.boxes is None or len(results.boxes) == 0:
        return frame
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
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def main():
    p = argparse.ArgumentParser(description="15 类垃圾检测")
    p.add_argument("source", nargs="?", default="0", help="摄像头(0)、图片或视频路径")
    p.add_argument("--weights", "-w", default=MODEL_PATH, help="权重路径，默认 weights2/best.pt")
    p.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="置信度阈值")
    p.add_argument("--imgsz", type=int, default=640, help="推理输入尺寸")
    p.add_argument("--show-human", action="store_true", help="同时画出 Human 框（默认只画垃圾）")
    p.add_argument("--agnostic-nms", action="store_true", help="NMS 不区分类别（重叠区域只保留一个框）")
    args = p.parse_args()

    src = args.source
    weights = args.weights
    if not Path(weights).exists():
        # 尝试相对项目根
        root = Path(__file__).resolve().parent
        alt = root / "weights2" / "best.pt"
        if alt.exists():
            weights = str(alt)
        else:
            print(f"未找到权重: {weights}")
            print("请将 best.pt 放到 weights2/best.pt 或指定 --weights path/to/best.pt")
            return 1

    model = YOLO(weights)
    class_names = get_class_names(model)
    if isinstance(class_names, dict):
        n = max(class_names.keys()) + 1 if class_names else 0
        names_list = [class_names.get(i, f"cls{i}") for i in range(n)]
    else:
        names_list = list(class_names) if class_names else CLASS_NAMES_15
    print("类别 (id -> name):", dict(enumerate(names_list)))

    hide_human = HIDE_HUMAN and not getattr(args, "show_human", False)
    agnostic_nms = getattr(args, "agnostic_nms", AGNOSTIC_NMS)
    _install_metal_class_bias(METAL_CLS_BIAS)

    is_camera = src == "0" or (isinstance(src, str) and src.isdigit())
    if is_camera:
        cap = cv2.VideoCapture(int(src))
        print("摄像头已开，按 q 退出")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, agnostic_nms=agnostic_nms, verbose=False)[0]
            frame = draw_results(frame, results, names_list, hide_human, args.conf)
            cv2.imshow("15cls Waste Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
        return 0

    path = Path(src)
    if not path.exists():
        print(f"未找到: {path}")
        return 1
    if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}:
        frame = cv2.imread(str(path))
        if frame is None:
            print("无法读取图片")
            return 1
        results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, agnostic_nms=agnostic_nms, verbose=False)[0]
        frame = draw_results(frame, results, names_list, hide_human, args.conf)
        out_path = path.parent / f"{path.stem}_det{path.suffix}"
        cv2.imwrite(str(out_path), frame)
        print(f"已保存: {out_path}")
        cv2.imshow("15cls Waste Detection", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = cv2.VideoCapture(str(path))
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, agnostic_nms=agnostic_nms, verbose=False)[0]
            frame = draw_results(frame, results, names_list, hide_human, args.conf)
            cv2.imshow("15cls Waste Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
