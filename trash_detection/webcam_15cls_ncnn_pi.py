#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
15-class NCNN trash detection on Raspberry Pi: CSI camera, MJPEG stream, MQTT.
Inference in subprocess (shared memory); stream at camera rate; labels from CLASS_NAMES_15.
"""
import argparse
import json
import multiprocessing
import socket
import sys
import time
from collections import Counter
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from threading import Condition, Thread

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import cv2
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2

ROOT = Path(__file__).resolve().parent
NCNN_DIR = ROOT / "toppi_ncnn_model"
CONF_THRESHOLD = 0.25

# MQTT
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "imperial/yh4222/esp32/test"
MQTT_CLIENT_ID_PUB = "pi_pub_trash_001"

# Toppi 15cls: same order as predict_toppi_webcam (0=Metal .. 4=Plastic .. 14=Human)
NUM_WASTE_CLASSES = 14  # 0..13 only; Human(14) not detected when using classes=trash_class_ids
CLASS_NAMES_15 = [
    "Metal", "Cardboard", "Glass", "Paper", "Plastic", "Tetra",
    "Apple", "Apple-core", "Apple-peel", "Bread", "Orange", "Orange-peel",
    "Pear", "Vegetable", "Human",
]
# 15 classes -> 4 categories: 1=paper/card, 2=general, 3=recycling, 4=food
CLASS_TO_CATEGORY = {
    "Metal": (3, "dry mixing and recycling"),
    "Cardboard": (1, "paper and card"),
    "Glass": (3, "dry mixing and recycling"),
    "Paper": (1, "paper and card"),
    "Plastic": (3, "dry mixing and recycling"),
    "Tetra": (2, "general waste"),
    "Apple": (4, "food waste"),
    "Apple-core": (4, "food waste"),
    "Apple-peel": (4, "food waste"),
    "Bread": (4, "food waste"),
    "Orange": (4, "food waste"),
    "Orange-peel": (4, "food waste"),
    "Pear": (4, "food waste"),
    "Vegetable": (4, "food waste"),
    "Human": (2, "general waste"),
}
CATEGORY_NAMES = {1: "paper and card", 2: "general waste", 3: "dry mixing and recycling", 4: "food waste"}

MQTT_DELAY_S = 2.0  # Publish only after object present for this many seconds; result = mode category in window


def ensure_ncnn_model_dir():
    """Return toppi_ncnn_model path if model.ncnn.param and model.ncnn.bin exist."""
    if (NCNN_DIR / "model.ncnn.param").exists() and (NCNN_DIR / "model.ncnn.bin").exists():
        return str(NCNN_DIR)
    return None


def _infer_worker(frame_queue, result_queue, model_dir, conf, imgsz, shm0_name, shm1_name, width, height):
    """Subprocess: read frame from shared memory, run NCNN, put (xyxy, conf, cls, infer_ms) in result_queue."""
    from ultralytics import YOLO
    shm0 = SharedMemory(name=shm0_name)
    shm1 = SharedMemory(name=shm1_name)
    buf_shape = (height, width, 3)
    model = YOLO(model_dir, task="detect")
    trash_class_ids = list(range(NUM_WASTE_CLASSES))  # 0..13 only, match predict_toppi_webcam
    while True:
        try:
            idx = frame_queue.get()
            if idx is None:
                break
            shm = shm0 if idx == 0 else shm1
            raw = np.ndarray(buf_shape, dtype=np.uint8, buffer=shm.buf).copy()
            t0 = time.perf_counter()
            results = model.predict(
                raw, conf=conf, imgsz=imgsz, verbose=False, device="cpu",
                classes=trash_class_ids,
            )[0]
            t1 = time.perf_counter()
            infer_ms = (t1 - t0) * 1000
            if results.boxes is None or len(results.boxes.xyxy) == 0:
                result_queue.put((None, None, None, infer_ms))
            else:
                xyxy = results.boxes.xyxy.cpu().numpy()
                conf_np = results.boxes.conf.cpu().numpy()
                cls_np = results.boxes.cls.cpu().numpy()
                result_queue.put((xyxy, conf_np, cls_np, infer_ms))
        except Exception as e:
            print("infer_worker error:", e)
            time.sleep(0.5)


def _draw_arrays(frame, xyxy, conf, cls_id, names_list, conf_threshold=0.25, hide_human=True):
    """Draw boxes from numpy arrays; names_list maps cls_id to label (e.g. Plastic)."""
    if xyxy is None or len(xyxy) == 0:
        return frame
    n = len(names_list)
    for i in range(len(xyxy)):
        if float(conf[i]) < conf_threshold:
            continue
        cid = int(cls_id[i])
        label_name = names_list[cid] if 0 <= cid < n else f"cls{cid}"
        if hide_human and label_name == "Human":
            continue
        x1, y1, x2, y2 = map(int, xyxy[i])
        label = f"{label_name} {float(conf[i]):.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def detections_arrays_to_category_list(xyxy, conf_np, cls_np, names_list, conf_threshold):
    """Return list of (category_id, category_name) for this frame; names_list must match CLASS_NAMES_15."""
    if xyxy is None or len(xyxy) == 0:
        return []
    n = len(names_list)
    out = []
    for i in range(len(xyxy)):
        c = float(conf_np[i])
        if c < conf_threshold:
            continue
        cid = int(cls_np[i])
        label = names_list[cid] if 0 <= cid < n else f"cls{cid}"
        if label == "Human":
            continue
        cat_id, cat_name = CLASS_TO_CATEGORY.get(label, (2, "general waste"))
        out.append((cat_id, cat_name))
    return out


def mqtt_update_and_maybe_publish(xyxy, conf_np, cls_np, names_list, conf_threshold, mqtt_client, mqtt_state, delay_s):
    """
    Append current frame categories to mqtt_state buffer. If object has been present for delay_s,
    publish the most frequent category in that window and reset.
    mqtt_state: dict with "buffer" [(timestamp, category_id), ...] and "window_start" (float or None).
    """
    cats = detections_arrays_to_category_list(xyxy, conf_np, cls_np, names_list, conf_threshold)
    now = time.perf_counter()
    if not cats:
        return 0
    buf = mqtt_state["buffer"]
    start = mqtt_state["window_start"]
    for cat_id, _ in cats:
        buf.append((now, cat_id))
    if start is None:
        mqtt_state["window_start"] = now
        return 0
    if (now - start) < delay_s:
        return 0
    # Use only entries in [window_start, window_start + delay_s]
    window_end = start + delay_s
    in_window = [(t, cid) for t, cid in buf if start <= t <= window_end]
    if not in_window:
        mqtt_state["buffer"] = []
        mqtt_state["window_start"] = None
        return 0
    counts = Counter(cid for _, cid in in_window)
    mode_cat_id = counts.most_common(1)[0][0]
    mode_cat_name = CATEGORY_NAMES.get(mode_cat_id, "general waste")
    payload = {
        "category": mode_cat_id,
        "category_name": mode_cat_name,
        "detections": [{"category": mode_cat_id, "category_name": mode_cat_name, "count_2s": counts.get(mode_cat_id, 0)}],
    }
    if mqtt_client:
        mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
    mqtt_state["buffer"] = []
    mqtt_state["window_start"] = None
    return mode_cat_id


def draw_boxes(frame, boxes, names, conf_threshold=0.25, in_place=False):
    """Draw YOLO boxes on frame; names = id->label; skip Human. in_place=True to avoid copy."""
    if boxes is None or len(boxes.xyxy) == 0:
        return frame
    out = frame if in_place else frame.copy()
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls_id = boxes.cls.cpu().numpy()
    for i in range(len(xyxy)):
        if float(conf[i]) < conf_threshold:
            continue
        cid = int(cls_id[i])
        label_name = names.get(cid, f"cls{cid}") if isinstance(names, dict) else (names[cid] if cid < len(names) else f"cls{cid}")
        if label_name == "Human":
            continue
        x1, y1, x2, y2 = map(int, xyxy[i])
        label = f"{label_name} {float(conf[i]):.2f}"
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out


def get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


def boxes_to_category_and_publish(boxes, names, conf_threshold, mqtt_client):
    """Map boxes to category 1–4 and publish; category=0 when none."""
    if boxes is None or len(boxes.xyxy) == 0:
        payload = {"category": 0, "category_name": "none", "detections": []}
        if mqtt_client:
            mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
        return 0
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls_id = boxes.cls.cpu().numpy()
    detections = []
    for i in range(len(xyxy)):
        c = float(conf[i])
        if c < conf_threshold:
            continue
        cid = int(cls_id[i])
        label = names.get(cid, f"cls{cid}") if isinstance(names, dict) else (names[cid] if cid < len(names) else f"cls{cid}")
        if label == "Human":
            continue
        cat_id, cat_name = CLASS_TO_CATEGORY.get(label, (2, "general waste"))
        detections.append({"class": label, "conf": round(c, 2), "category": cat_id, "category_name": cat_name})
    if not detections:
        payload = {"category": 0, "category_name": "none", "detections": []}
        if mqtt_client:
            mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
        return 0
    best = max(detections, key=lambda d: d["conf"])
    category = best["category"]
    category_name = best["category_name"]
    payload = {"category": category, "category_name": category_name, "detections": detections}
    if mqtt_client:
        mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
    return category


def main():
    p = argparse.ArgumentParser(description="15-class NCNN + Pi CSI camera, MJPEG stream, MQTT")
    p.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="Confidence threshold")
    p.add_argument("--imgsz", type=int, default=640, help="Model input size; 320/416 faster")
    p.add_argument("--infer-every", type=int, default=1, help="Run inference every N frames")
    p.add_argument("--port", type=int, default=8765, help="Stream port")
    p.add_argument("--width", type=int, default=640, help="Camera width")
    p.add_argument("--height", type=int, default=480, help="Camera height")
    p.add_argument("--quality", type=int, default=85, help="MJPEG quality 1-100")
    p.add_argument("--no-mqtt", action="store_true", help="Disable MQTT")
    p.add_argument("--no-stream", action="store_true", help="No HTTP stream; capture+infer+MQTT only")
    p.add_argument("--mqtt-delay", type=float, default=MQTT_DELAY_S, help="Publish MQTT after object present for N seconds; result = most frequent category in window (default 2)")
    args = p.parse_args()

    model_dir = ensure_ncnn_model_dir()
    if not model_dir:
        print("NCNN model not found; need toppi_ncnn_model/model.ncnn.param and model.ncnn.bin")
        sys.exit(1)
    print("Model:", model_dir, flush=True)
    print("Classes:", CLASS_NAMES_15, flush=True)

    mqtt_client = None
    if not args.no_mqtt and MQTT_AVAILABLE:
        try:
            mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID_PUB)
            mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            mqtt_client.loop_start()
            print("MQTT connected:", MQTT_BROKER, "topic:", MQTT_TOPIC, flush=True)
        except Exception as e:
            print("MQTT connect failed:", e, flush=True)
            mqtt_client = None
    elif not args.no_mqtt and not MQTT_AVAILABLE:
        print("Install paho-mqtt for MQTT: pip install paho-mqtt", flush=True)

    print("Starting CSI camera...", flush=True)
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    if args.no_stream:
        print("Stream disabled; capture + infer + MQTT only.", flush=True)
    else:
        print("Infer in subprocess; stream at camera rate.", flush=True)
    if args.infer_every > 1:
        print("Infer every {} frames.".format(args.infer_every), flush=True)

    latest_raw = None
    last_detections = None  # (xyxy, conf, cls) or (None, None, None)
    cond = Condition()
    frame_buf_size = args.width * args.height * 3
    shm0 = SharedMemory(create=True, size=frame_buf_size)
    shm1 = SharedMemory(create=True, size=frame_buf_size)
    try:
        arr0 = np.ndarray((args.height, args.width, 3), dtype=np.uint8, buffer=shm0.buf)
        arr1 = np.ndarray((args.height, args.width, 3), dtype=np.uint8, buffer=shm1.buf)
    except Exception:
        shm0.close()
        shm0.unlink()
        shm1.close()
        shm1.unlink()
        raise
    frame_queue = multiprocessing.Queue(maxsize=2)
    result_queue = multiprocessing.Queue(maxsize=2)
    write_idx = 0

    infer_proc = multiprocessing.Process(
        target=_infer_worker,
        args=(frame_queue, result_queue, model_dir, args.conf, args.imgsz,
              shm0.name, shm1.name, args.width, args.height),
        daemon=True,
    )
    infer_proc.start()

    capture_frame_count = [0]

    def capture_loop():
        nonlocal latest_raw, write_idx
        while True:
            try:
                rgb = picam2.capture_array()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                arr = arr0 if write_idx == 0 else arr1
                arr[:] = bgr
                if not args.no_stream:
                    with cond:
                        latest_raw = bgr.copy()
                        cond.notify_all()
                capture_frame_count[0] += 1
                if capture_frame_count[0] % args.infer_every == 0:
                    try:
                        frame_queue.put_nowait(write_idx)
                    except Exception:
                        try:
                            frame_queue.get_nowait()
                        except Exception:
                            pass
                        try:
                            frame_queue.put_nowait(write_idx)
                        except Exception:
                            pass
                write_idx = 1 - write_idx
            except Exception as e:
                print("capture error:", e, flush=True)
                break

    det_count = [0]
    mqtt_state = {"buffer": [], "window_start": None}

    def result_reader():
        nonlocal last_detections
        while True:
            try:
                data = result_queue.get()
                xyxy, conf_np, cls_np, infer_ms = data
                with cond:
                    last_detections = (xyxy, conf_np, cls_np)
                fps = 1000.0 / infer_ms if infer_ms > 0 else 0
                det_count[0] += 1
                print("[det {}] NCNN: {:.1f} ms  ({:.1f} FPS)".format(det_count[0], infer_ms, fps), flush=True)
                cat = mqtt_update_and_maybe_publish(
                    xyxy, conf_np, cls_np, CLASS_NAMES_15, args.conf,
                    mqtt_client, mqtt_state, args.mqtt_delay,
                )
                if cat > 0:
                    print("  -> MQTT category (mode in {:.1f}s): {}".format(args.mqtt_delay, cat), flush=True)
            except Exception:
                break

    Thread(target=capture_loop, daemon=True).start()
    Thread(target=result_reader, daemon=True).start()

    if args.no_stream:
        print("Capture + infer + MQTT only. Ctrl+C to exit.", flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        picam2.stop()
        return 0

    print("Waiting for first frame...", flush=True)
    with cond:
        cond.wait(timeout=5.0)
    print("Starting HTTP stream...", flush=True)

    app = Flask(__name__)

    def generate():
        nonlocal latest_raw, last_detections
        while True:
            with cond:
                cond.wait()
                raw = latest_raw
                det = last_detections
            if raw is None:
                continue
            frame = raw.copy()
            if det is not None and det[0] is not None and len(det[0]) > 0:
                _draw_arrays(frame, det[0], det[1], det[2], CLASS_NAMES_15, args.conf, hide_human=True)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            _, jpeg = cv2.imencode(".jpg", frame_rgb, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
            )

    @app.route("/")
    def index():
        return render_template_string("""
<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>15cls NCNN</title>
<style>body{margin:0;background:#111;} img{max-width:100%;height:auto;}</style>
</head><body><img src="/video_feed" alt="stream"/></body></html>
""")

    @app.route("/video_feed")
    def video_feed():
        r = Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        r.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
        r.headers["X-Accel-Buffering"] = "no"
        return r

    ip = get_lan_ip()
    print("-" * 50, flush=True)
    print("Stream: http://{}:{}".format(ip, args.port), flush=True)
    print("-" * 50, flush=True)
    app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
