#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
18-class NCNN trash detection on Raspberry Pi: CSI camera, MJPEG stream, MQTT.
Zero calibration uses the same capture + EV + LAB levels as calibration/aruco_zero_calibration.py
(only during the calib window); then exposure resets for normal trash inference.
Main loop: Ultralytics YOLO loads the NCNN export directory and runs detection on the same thread
as capture (same frame as MJPEG when streaming).
MQTT publishes only when the ArUco marker is occluded (gate open) and --mqtt-delay stability is met.
"""
import argparse
import json
import math
import socket
import sys
import time
from collections import Counter
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
NCNN_DIR = ROOT / "waste_ncnn_model"
CONF_THRESHOLD = 0.35

# MQTT
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "imperial/yh4222/esp32/test"
MQTT_TOPIC_CALIB = "imperial/yh4222/esp32/calib"  # plate zero angle deg (negated before publish)
MQTT_TOPIC_RES = "imperial/yh4222/esp32/res"  # app -> ESP32: motor preset 1–4 as text; 1->0, 2->-pi/2, 3->-pi, 4->+pi/2
MQTT_CLIENT_ID_PUB = "pi_pub_trash_001"

# Motor preset: category 1..4 -> angle (rad). Sent to res; record for last classification.
CATEGORY_TO_ANGLE_RAD = {1: 0.0, 2: -math.pi / 2, 3: -math.pi, 4: math.pi / 2}

try:
    _aruco_module = cv2.aruco
except AttributeError:
    _aruco_module = None

# waste_ncnn_model classes (derived from model.ncnn.param: nc=18)
NUM_WASTE_CLASSES = 18  # 0..17
CLASS_NAMES = [
    "Can",
    "Cardboard",
    "Carton",
    "Glass",
    "Paper",
    "Plastic",
    "Tetra",
    "Apple",
    "Apple-core",
    "Apple-peel",
    "Bone",
    "Bread",
    "Egg-shell",
    "Orange",
    "Orange-peel",
    "Pear",
    "Vegetable",
    "Human",
]
# Classes -> 4 categories: 1=paper/card, 2=general, 3=recycling, 4=food
CLASS_TO_CATEGORY = {
    # Recycling
    "Can": (3, "dry mixing and recycling"),
    "Glass": (3, "dry mixing and recycling"),
    "Plastic": (3, "dry mixing and recycling"),
    # Paper & card
    "Cardboard": (1, "paper and card"),
    "Carton": (1, "paper and card"),
    "Paper": (1, "paper and card"),
    # General waste (often non-recyclable composite)
    "Tetra": (2, "general waste"),
    # Food waste
    "Apple": (4, "food waste"),
    "Apple-core": (4, "food waste"),
    "Apple-peel": (4, "food waste"),
    "Bone": (4, "food waste"),
    "Bread": (4, "food waste"),
    "Orange": (4, "food waste"),
    "Orange-peel": (4, "food waste"),
    "Pear": (4, "food waste"),
    "Vegetable": (4, "food waste"),
    "Egg-shell": (4, "food waste"),
    "Human": (2, "general waste"),
}
CATEGORY_NAMES = {1: "paper and card", 2: "general waste", 3: "dry mixing and recycling", 4: "food waste"}

MQTT_DELAY_S = 2.0  # Publish only after object present for this many seconds; result = mode category in window


def _boost_conf_for_output(label: str, conf: float, threshold: float) -> float:
    """
    Post-process confidence for display/print only.
    If label is Orange/Orange-peel and original conf already >= threshold, add +0.3 then cap at 1.0.
    """
    if label in ("Orange", "Orange-peel") and conf >= threshold:
        return min(conf + 0.3, 1.0)
    return conf


def _set_pi_exposure_ev(picam2, ev: float, settle_s: float) -> None:
    """libcamera ExposureValue; settle_s seconds for AE to adapt (0 to skip sleep)."""
    try:
        from libcamera import controls

        picam2.set_controls({controls.ExposureValue: float(ev)})
    except Exception:
        try:
            picam2.set_controls({"ExposureValue": float(ev)})
        except Exception as e:
            print("Warning: ExposureValue not set (ev=%s): %s" % (ev, e), flush=True)
            return
    if settle_s and settle_s > 0:
        time.sleep(settle_s)


def _apply_pi_exposure_ev(picam2, ev: float) -> None:
    """Long settle (calib startup)."""
    _set_pi_exposure_ev(picam2, ev, 0.2)


def _prepare_aruco_bgr_gray(
    rgb,
    calib_flip_vertical,
    calib_no_contrast,
    calib_black_level,
    calib_white_level,
    calib_clahe_clip,
):
    """Same pipeline as calib / aruco_zero_calibration for ArUco detection."""
    # Picamera2 returns RGB888; keep everything in RGB order (no R/B swapping).
    if calib_flip_vertical:
        rgb = cv2.flip(rgb, 0)
    if not calib_no_contrast and (
        calib_black_level > 0
        or calib_white_level < 255
        or (calib_clahe_clip and calib_clahe_clip > 0)
    ):
        rgb, gray = _levels_correct_bgr(
            rgb, calib_clahe_clip, calib_black_level, calib_white_level
        )
    else:
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    return rgb, gray


def _levels_correct_bgr(frame_bgr, clahe_clip: float, black_level: int, white_level: int):
    """Same LAB levels as aruco_zero_calibration (keep frame in RGB order)."""
    # NOTE: despite the name, frame_bgr is actually RGB in this script.
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    if clahe_clip and clahe_clip > 0:
        clahe = cv2.createCLAHE(clipLimit=float(clahe_clip), tileGridSize=(8, 8))
        l_ch = clahe.apply(l_ch)
    b0 = int(np.clip(black_level, 0, 250))
    w0 = int(np.clip(white_level, 0, 255))
    bp, wp = min(b0, w0), max(b0, w0)
    if wp <= bp + 1:
        wp = min(bp + 2, 255)
    lf = l_ch.astype(np.float32)
    denom = max(float(wp - bp), 1.0)
    l_ch = np.clip((lf - float(bp)) * (255.0 / denom), 0, 255).astype(np.uint8)
    out = cv2.cvtColor(cv2.merge([l_ch, a_ch, b_ch]), cv2.COLOR_LAB2RGB)
    return out, l_ch


def ensure_ncnn_model_dir():
    """Return waste_ncnn_model path if model.ncnn.param and model.ncnn.bin exist."""
    if (NCNN_DIR / "model.ncnn.param").exists() and (NCNN_DIR / "model.ncnn.bin").exists():
        return str(NCNN_DIR)
    return None


def _aruco_get_dict(dict_name):
    if _aruco_module is None:
        return None
    name_upper = dict_name.upper().replace(" ", "")
    mapping = {
        "4X4_50": _aruco_module.DICT_4X4_50,
        "4X4_100": _aruco_module.DICT_4X4_100,
        "5X5_50": _aruco_module.DICT_5X5_50,
        "6X6_250": _aruco_module.DICT_6X6_250,
        "6X6_1000": _aruco_module.DICT_6X6_1000,
        "7X7_50": _aruco_module.DICT_7X7_50,
    }
    dict_id = mapping.get(name_upper, _aruco_module.DICT_4X4_50)
    return _aruco_module.getPredefinedDictionary(dict_id)


def _build_aruco_detector(dictionary, adapt_const, min_perimeter_rate, adapt_min, adapt_max):
    """Match calibration/aruco_zero_calibration.py; default adaptiveThreshConstant=5 (OpenCV default is 7)."""
    dp = _aruco_module.DetectorParameters()
    dp.adaptiveThreshConstant = int(adapt_const)
    dp.minMarkerPerimeterRate = float(min_perimeter_rate)
    dp.adaptiveThreshWinSizeMin = int(adapt_min)
    dp.adaptiveThreshWinSizeMax = int(adapt_max)
    if hasattr(dp, "cornerRefinementMethod") and hasattr(_aruco_module, "CORNER_REFINE_SUBPIX"):
        dp.cornerRefinementMethod = _aruco_module.CORNER_REFINE_SUBPIX
    try:
        det = _aruco_module.ArucoDetector(dictionary, dp)
        return det, dp
    except (AttributeError, TypeError):
        return None, dp


def _aruco_detect_center(
    gray,
    dictionary,
    target_id=None,
    aruco_detector=None,
    aruco_dp=None,
):
    """Return (cx, cy, corners, marker_id) or (None, None, None, None)."""
    if _aruco_module is None or dictionary is None:
        return None, None, None, None
    try:
        if aruco_detector is not None:
            corners_list, ids, _ = aruco_detector.detectMarkers(gray)
        elif aruco_dp is not None:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary, aruco_dp)
        else:
            params = _aruco_module.DetectorParameters()
            detector = _aruco_module.ArucoDetector(dictionary, params)
            corners_list, ids, _ = detector.detectMarkers(gray)
    except (AttributeError, TypeError):
        if aruco_dp is not None:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary, aruco_dp)
        else:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary)
    if ids is None or len(ids) == 0:
        return None, None, None, None
    ids_flat = ids.flatten()
    idx = 0
    if target_id is not None:
        found = np.where(ids_flat == target_id)[0]
        if len(found) == 0:
            return None, None, None, None
        idx = int(found[0])
    corners = corners_list[idx]
    c = np.asarray(corners)
    if c.ndim == 3:
        c = c.reshape(4, 2)
    cx = float(c[:, 0].mean())
    cy = float(c[:, 1].mean())
    return cx, cy, corners, int(ids_flat[idx])


def _aruco_angle_zero_up_deg(corners, cx, cy):
    """0° = up (top-right corner points up). Returns deg: 0=up, 90=right, 180=down, -90=left."""
    c = np.asarray(corners).reshape(4, 2)
    tr = c[1]
    dx = tr[0] - cx
    dy = tr[1] - cy
    return math.degrees(math.atan2(dx, -dy))


def _aruco_draw_on_bgr_with_gray(
    bgr,
    gray,
    dictionary,
    aruco_target_id,
    aruco_detector=None,
    aruco_dp=None,
):
    """Detect on gray; draw on BGR/RGB copy. Same ArUco detector setup as aruco_zero_calibration."""
    out = bgr.copy()
    angle_deg = None
    if _aruco_module is None or dictionary is None:
        return None, out
    try:
        if aruco_detector is not None:
            corners_list, ids, _ = aruco_detector.detectMarkers(gray)
        elif aruco_dp is not None:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary, aruco_dp)
        else:
            params = _aruco_module.DetectorParameters()
            detector = _aruco_module.ArucoDetector(dictionary, params)
            corners_list, ids, _ = detector.detectMarkers(gray)
    except (AttributeError, TypeError):
        if aruco_dp is not None:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary, aruco_dp)
        else:
            corners_list, ids, _ = _aruco_module.detectMarkers(gray, dictionary)
    if ids is not None and len(ids) > 0 and corners_list is not None:
        try:
            _aruco_module.drawDetectedMarkers(out, corners_list, ids)
        except Exception:
            pass
    cx, cy, corners, _ = _aruco_detect_center(
        gray, dictionary, aruco_target_id, aruco_detector, aruco_dp
    )
    if cx is not None and corners is not None:
        angle_deg = _aruco_angle_zero_up_deg(corners, cx, cy)
    return angle_deg, out


def _aruco_draw_on_bgr(bgr, dictionary, aruco_target_id):
    # bgr is actually RGB in this script
    gray = cv2.cvtColor(bgr, cv2.COLOR_RGB2GRAY)
    return _aruco_draw_on_bgr_with_gray(bgr, gray, dictionary, aruco_target_id)


def run_calibration_phase(
    picam2,
    mqtt_client,
    aruco_dict_name,
    aruco_target_id,
    skip_calib,
    no_mqtt,
    calib_sec,
    stream_cond,
    latest_raw_ref,
    stream_status_ref,
    show_stream,
    calib_ev,
    post_calib_ev,
    calib_black_level,
    calib_white_level,
    calib_clahe_clip,
    calib_no_contrast,
    calib_flip_vertical,
    aruco_detector,
    aruco_dp,
):
    """
    ArUco zero calib: same pipeline as aruco_zero_calibration (EV + BGR + optional LAB levels).
    Restores post_calib_ev after calib so trash inference sees normal exposure.
    """
    if skip_calib:
        return
    if _aruco_module is None:
        print("Calibration skipped: cv2.aruco not found (install opencv-contrib-python)", flush=True)
        return
    dictionary = _aruco_get_dict(aruco_dict_name)
    if dictionary is None:
        print("Calibration skipped: ArUco dict not available", flush=True)
        return
    print(
        "Calibration (aruco_zero_calibration-style): EV={} then post {} | L levels black={} white={} ... {:.0f}s".format(
            calib_ev, post_calib_ev, calib_black_level, calib_white_level, calib_sec,
        ),
        flush=True,
    )
    angles = []
    t_end = time.perf_counter() + calib_sec
    try:
        if calib_ev != 0.0:
            _apply_pi_exposure_ev(picam2, calib_ev)
        while time.perf_counter() < t_end:
            try:
                rgb = picam2.capture_array()
                bgr, gray = _prepare_aruco_bgr_gray(
                    rgb,
                    calib_flip_vertical,
                    calib_no_contrast,
                    calib_black_level,
                    calib_white_level,
                    calib_clahe_clip,
                )
                angle_deg, vis = _aruco_draw_on_bgr_with_gray(
                    bgr, gray, dictionary, aruco_target_id, aruco_detector, aruco_dp
                )
                remain = max(0.0, t_end - time.perf_counter())
                if angle_deg is not None:
                    angles.append(angle_deg)
                    cv2.putText(
                        vis, "Calib angle: {:.1f} deg (0=up)".format(angle_deg), (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                    )
                else:
                    cv2.putText(
                        vis, "Point ArUco at camera", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2,
                    )
                cv2.putText(
                    vis, "CALIBRATING  {:.1f}s left".format(remain), (8, vis.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2,
                )
                if show_stream and stream_cond is not None and latest_raw_ref is not None:
                    stream_status_ref[0] = "CALIBRATING"
                    with stream_cond:
                        latest_raw_ref[0] = vis
                        stream_cond.notify_all()
            except Exception as e:
                print("Calibration frame error:", e, flush=True)
            time.sleep(0.05)
    finally:
        _set_pi_exposure_ev(picam2, post_calib_ev, 0.15)
    if not angles:
        print("Calibration: no ArUco detected in window.", flush=True)
        if no_mqtt or mqtt_client is None:
            print("  MQTT angle not sent (no detections or MQTT off).", flush=True)
        return
    angle_raw = angles[-1] if len(angles) == 1 else float(np.median(angles))
    angle_mqtt = -angle_raw
    payload = "{:.2f}".format(angle_mqtt)
    if not no_mqtt and mqtt_client is not None:
        mqtt_client.publish(MQTT_TOPIC_CALIB, payload, qos=0)
        print(
            "Calibration done: MQTT {} deg (negated)  raw={:.2f} -> {}".format(
                payload, angle_raw, MQTT_TOPIC_CALIB,
            ),
            flush=True,
        )
    else:
        print(
            "Calibration done: raw={:.2f} deg, MQTT would send {:.2f} (negated) (MQTT off)".format(
                angle_raw, angle_mqtt,
            ),
            flush=True,
        )


def _draw_arrays(frame, xyxy, conf, cls_id, names_list, conf_threshold=0.25):
    """Draw boxes from numpy arrays; names_list maps cls_id to label (e.g. Plastic)."""
    if xyxy is None or len(xyxy) == 0:
        return frame
    n = len(names_list)
    for i in range(len(xyxy)):
        c0 = float(conf[i])
        if c0 < conf_threshold:
            continue
        cid = int(cls_id[i])
        label_name = names_list[cid] if 0 <= cid < n else f"cls{cid}"
        x1, y1, x2, y2 = map(int, xyxy[i])
        c_show = _boost_conf_for_output(label_name, c0, conf_threshold)
        label = f"{label_name} {c_show:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def detections_arrays_to_category_list(xyxy, conf_np, cls_np, names_list, conf_threshold):
    """Return list of (category_id, category_name) for this frame; names_list must match model class order."""
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
        cat_id, cat_name = CLASS_TO_CATEGORY.get(label, (2, "general waste"))
        out.append((cat_id, cat_name))
    return out


def mqtt_update_and_maybe_publish(
    xyxy,
    conf_np,
    cls_np,
    names_list,
    conf_threshold,
    mqtt_client,
    mqtt_state,
    delay_s,
    last_res=None,
    *,
    treat_empty_as_general: bool = False,
):
    """
    Append current frame categories to mqtt_state buffer. If object has been present for delay_s,
    publish the most frequent category in that window and reset.
    mqtt_state: dict with "buffer" [(timestamp, category_id), ...] and "window_start" (float or None).
    last_res: optional dict to record last sent category and angle_rad (CATEGORY_TO_ANGLE_RAD).

    When `treat_empty_as_general=True` (used for ArUco-occlusion gating):
    - Start the window even if there are temporarily no confident detections.
    - At publish time (after delay_s), if the window contains any *non-general* category (1/3/4),
      output the best non-general category; otherwise fall back to category 2 ("general waste").
    """
    now = time.perf_counter()
    cats = detections_arrays_to_category_list(xyxy, conf_np, cls_np, names_list, conf_threshold)
    buf = mqtt_state["buffer"]
    start = mqtt_state["window_start"]

    # For occlusion-gating, we must still start the timer even if `cats` is empty.
    if start is None:
        if not cats and not treat_empty_as_general:
            return 0
        mqtt_state["window_start"] = now
        start = now
        # If we just started and this frame has no confident detections, wait for the window.
        if not cats:
            return 0

    # Only append detections when we actually have categories.
    if cats:
        for cat_id, _ in cats:
            buf.append((now, cat_id))

    if (now - start) < delay_s:
        return 0
    # Use only entries in [window_start, window_start + delay_s]
    window_end = start + delay_s
    in_window = [(t, cid) for t, cid in buf if start <= t <= window_end]

    # Default: mode category (original behavior)
    counts = Counter(cid for _, cid in in_window)

    general_cat_id = 2
    if treat_empty_as_general:
        # If we saw any non-general category within the window, don't let empty frames push us back to general.
        non_general_items = [(cid, cnt) for cid, cnt in counts.items() if cid != general_cat_id]
        if not non_general_items:
            mode_cat_id = general_cat_id
        else:
            # Pick the most frequent non-general category in the window.
            non_general_items.sort(key=lambda x: x[1], reverse=True)
            mode_cat_id = non_general_items[0][0]
    else:
        if not in_window:
            mqtt_state["buffer"] = []
            mqtt_state["window_start"] = None
            return 0
        mode_cat_id = counts.most_common(1)[0][0]

    mode_cat_name = CATEGORY_NAMES.get(mode_cat_id, "general waste")
    payload = {
        "category": mode_cat_id,
        "category_name": mode_cat_name,
        "detections": [{"category": mode_cat_id, "category_name": mode_cat_name, "count_2s": counts.get(mode_cat_id, 0)}],
    }
    if mqtt_client:
        mqtt_client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
        # Motor preset: send "1"/"2"/"3"/"4" to res topic (moves motor to preset position)
        if 1 <= mode_cat_id <= 4:
            mqtt_client.publish(MQTT_TOPIC_RES, str(mode_cat_id), qos=0)
    if last_res is not None:
        last_res["category"] = mode_cat_id
        last_res["angle_rad"] = CATEGORY_TO_ANGLE_RAD.get(mode_cat_id)
    mqtt_state["buffer"] = []
    mqtt_state["window_start"] = None
    return mode_cat_id


def draw_boxes(frame, boxes, names, conf_threshold=0.25, in_place=False):
    """Draw YOLO boxes on frame; names = id->label; in_place=True to avoid copy."""
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
        cat_id, cat_name = CLASS_TO_CATEGORY.get(label, (2, "general waste"))
        c_show = _boost_conf_for_output(label, c, conf_threshold)
        detections.append({"class": label, "conf": round(c_show, 2), "category": cat_id, "category_name": cat_name})
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
    p = argparse.ArgumentParser(description="18-class NCNN + Pi CSI camera, MJPEG stream, MQTT")
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
    p.add_argument("--skip-calib", action="store_true", help="Skip ArUco calibration at startup")
    p.add_argument("--aruco-dict", type=str, default="4X4_50", help="ArUco dictionary for calibration")
    p.add_argument("--aruco-target-id", type=int, default=None, help="ArUco marker ID to use (default: any)")
    p.add_argument(
        "--aruco-preset",
        choices=("default", "sensitive"),
        default="default",
        help="Same as aruco_zero_calibration; shared by gate and calib (default: C=5; sensitive: small markers/low contrast)",
    )
    p.add_argument("--aruco-adapt-const", type=int, default=None, help="Adaptive threshold C (default 5, same as calib script)")
    p.add_argument("--aruco-min-perimeter-rate", type=float, default=None, help="Min marker perimeter / image size (default 0.03)")
    p.add_argument("--aruco-adapt-min", type=int, default=None, help="Adaptive window min side (default 3)")
    p.add_argument("--aruco-adapt-max", type=int, default=None, help="Adaptive window max side (default 23)")
    p.add_argument("--calib-sec", type=float, default=5.0, help="Calibration capture duration in seconds")
    p.add_argument("--calib-ev", type=float, default=-0.7, help="EV during ArUco zero calib (same as aruco_zero_calibration)")
    p.add_argument(
        "--post-calib-ev",
        type=float,
        default=0.0,
        help="Restore EV after calib for trash inference (default 0)",
    )
    p.add_argument("--calib-black-level", type=int, default=150, help="Calib LAB levels (with white, sorted)")
    p.add_argument("--calib-white-level", type=int, default=50, help="Calib LAB levels other endpoint")
    p.add_argument("--calib-clahe-clip", type=float, default=0.0, help="Optional CLAHE during calib only")
    p.add_argument("--calib-no-contrast", action="store_true", help="Disable LAB levels during calib")
    p.add_argument("--calib-flip-vertical", action="store_true", help="Flip image during calib")
    p.add_argument(
        "--dual-capture-settle",
        type=float,
        default=0.10,
        help="Sleep after EV change before capture; increase (e.g. 0.12–0.2) if gate vs calib disagree",
    )
    p.add_argument(
        "--same-frame-aruco-gate",
        action="store_true",
        help="Legacy: ArUco gate and stream/YOLO share one frame. Default off (dual exposure: ArUco at calib_ev, YOLO/MJPEG at post_calib_ev)",
    )
    p.add_argument(
        "--no-aruco-gate",
        action="store_true",
        help="Disable rule: classify even when ArUco marker is visible (default: classify only when marker is occluded)",
    )
    p.add_argument(
        "--no-print-dets",
        action="store_true",
        help="Do not print per-frame classification lines to the terminal",
    )
    p.add_argument(
        "--include-human",
        action="store_true",
        help="Include Human class in detection (default: Human is excluded)",
    )
    p.add_argument(
        "--print-empty-every",
        type=int,
        default=20,
        help="When no detections, print a line every N inferences (0 = never)",
    )
    args = p.parse_args()

    model_dir = ensure_ncnn_model_dir()
    if not model_dir:
        print("NCNN model not found; need waste_ncnn_model/model.ncnn.param and model.ncnn.bin")
        sys.exit(1)
    print("Model:", model_dir, flush=True)
    print("Classes:", CLASS_NAMES, flush=True)

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

    cond = Condition()
    latest_raw_holder = [None]  # list so nested functions can assign
    stream_status = [""]  # overlay line for browser stream
    last_detections = None

    aruco_dict_gate = None
    if not args.no_aruco_gate and _aruco_module is not None:
        aruco_dict_gate = _aruco_get_dict(args.aruco_dict)
    if not args.no_aruco_gate and aruco_dict_gate is not None:
        if args.same_frame_aruco_gate:
            print(
                "ArUco gate: same-frame mode (stream/YOLO share one frame) | MQTT delay %.1fs."
                % (args.mqtt_delay,),
                flush=True,
            )
            print(
                "  Gate preprocessing: black=%d, white=%d, clahe=%.2f"
                % (args.calib_black_level, args.calib_white_level, args.calib_clahe_clip),
                flush=True,
            )
        else:
            print(
                "ArUco gate: dual exposure. ArUco at calib_ev; YOLO/MJPEG at post_calib_ev; MQTT delay %.1fs."
                % (args.mqtt_delay,),
                flush=True,
            )
            print(
                "  ArUco visibility uses: calib_ev=%.2f, black=%d, white=%d, clahe=%.2f"
                % (
                    args.calib_ev,
                    args.calib_black_level,
                    args.calib_white_level,
                    args.calib_clahe_clip,
                ),
                flush=True,
            )
            print(
                "  Trash inference / stream use: post_calib_ev=%.2f" % (args.post_calib_ev,),
                flush=True,
            )
    elif not args.no_aruco_gate:
        print("ArUco gate unavailable (no aruco); classifying every infer frame.", flush=True)

    dict_aruco = _aruco_get_dict(args.aruco_dict)
    aruco_detector_main, aruco_dp_main = None, None
    if dict_aruco is not None and _aruco_module is not None:
        adapt_c = 5
        min_pr = 0.03
        adapt_wmin, adapt_wmax = 3, 23
        if args.aruco_preset == "sensitive":
            adapt_c, min_pr, adapt_wmax = 4, 0.012, 31
        if args.aruco_adapt_const is not None:
            adapt_c = args.aruco_adapt_const
        if args.aruco_min_perimeter_rate is not None:
            min_pr = args.aruco_min_perimeter_rate
        if args.aruco_adapt_min is not None:
            adapt_wmin = args.aruco_adapt_min
        if args.aruco_adapt_max is not None:
            adapt_wmax = args.aruco_adapt_max
        aruco_detector_main, aruco_dp_main = _build_aruco_detector(
            dict_aruco, adapt_c, min_pr, adapt_wmin, adapt_wmax
        )

    app = None
    if not args.no_stream:
        app = Flask(__name__)

        def stream_generate():
            while True:
                with cond:
                    cond.wait(timeout=1.0)
                    raw = latest_raw_holder[0]
                if raw is None:
                    continue
                frame = raw.copy()
                # frame is already RGB order; encode directly to JPEG
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
                )

        @app.route("/")
        def index():
            return render_template_string("""
<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>NCNN Trash</title>
<style>body{margin:0;background:#111;} img{max-width:100%;height:auto;}</style>
</head><body><img src="/video_feed" alt="stream"/></body></html>
""")

        @app.route("/video_feed")
        def video_feed():
            r = Response(stream_generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
            r.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
            r.headers["X-Accel-Buffering"] = "no"
            return r

        def _run_http():
            app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)

        Thread(target=_run_http, daemon=True).start()
        time.sleep(0.35)
        ip = get_lan_ip()
        print("-" * 50, flush=True)
        print("Stream (calibration + live): http://{}:{}".format(ip, args.port), flush=True)
        print("-" * 50, flush=True)

    run_calibration_phase(
        picam2,
        mqtt_client,
        args.aruco_dict,
        args.aruco_target_id,
        args.skip_calib,
        args.no_mqtt,
        args.calib_sec,
        cond if not args.no_stream else None,
        latest_raw_holder if not args.no_stream else None,
        stream_status,
        show_stream=not args.no_stream,
        calib_ev=args.calib_ev,
        post_calib_ev=args.post_calib_ev,
        calib_black_level=args.calib_black_level,
        calib_white_level=args.calib_white_level,
        calib_clahe_clip=args.calib_clahe_clip,
        calib_no_contrast=args.calib_no_contrast,
        calib_flip_vertical=args.calib_flip_vertical,
        aruco_detector=aruco_detector_main,
        aruco_dp=aruco_dp_main,
    )
    stream_status[0] = ""

    if args.no_stream:
        print("Stream disabled; capture + infer + MQTT only.", flush=True)
    if args.infer_every > 1:
        print("Infer every {} frames.".format(args.infer_every), flush=True)

    print(
        "Loading YOLO (NCNN dir) — same thread as camera; YOLO matches stream frame; ArUco gate may use a separate capture.",
        flush=True,
    )
    try:
        from ultralytics import YOLO
    except ImportError as e:
        print("Requires: pip install ultralytics torch (YOLO loads the NCNN export)", e, flush=True)
        picam2.stop()
        sys.exit(1)
    yolo_model = YOLO(model_dir, task="detect")
    human_idx = None
    try:
        human_idx = CLASS_NAMES.index("Human")
    except ValueError:
        human_idx = None
    if (human_idx is not None) and (not args.include_human):
        waste_class_ids = [i for i in range(NUM_WASTE_CLASSES) if i != human_idx]
        print("YOLO classes: excluding Human (id={})".format(human_idx), flush=True)
    else:
        waste_class_ids = list(range(NUM_WASTE_CLASSES))
    print("YOLO NCNN ready.", flush=True)

    def _publish_stream_frame(bgr, status_lines):
        if args.no_stream:
            return
        stream_status[0] = "\n".join(status_lines) if status_lines else ""
        with cond:
            latest_raw_holder[0] = bgr.copy()
            cond.notify_all()

    capture_frame_count = [0]
    det_count = [0]
    empty_streak = [0]
    mqtt_state = {"buffer": [], "window_start": None}
    last_res = {"category": None, "angle_rad": None}

    def capture_loop():
        nonlocal last_detections
        use_gate = aruco_dict_gate is not None and not args.no_aruco_gate
        st = args.dual_capture_settle
        while True:
            try:
                marker_visible = False

                if use_gate and not args.same_frame_aruco_gate:
                    _set_pi_exposure_ev(picam2, args.calib_ev, st)
                    rgb_gate = picam2.capture_array()
                    _, gray_gate = _prepare_aruco_bgr_gray(
                        rgb_gate,
                        args.calib_flip_vertical,
                        args.calib_no_contrast,
                        args.calib_black_level,
                        args.calib_white_level,
                        args.calib_clahe_clip,
                    )
                    cx, _, _, _ = _aruco_detect_center(
                        gray_gate,
                        aruco_dict_gate,
                        args.aruco_target_id,
                        aruco_detector_main,
                        aruco_dp_main,
                    )
                    marker_visible = cx is not None

                _set_pi_exposure_ev(picam2, args.post_calib_ev, st)
                rgb_main = picam2.capture_array()
                # Keep original RGB order for YOLO + stream
                bgr = rgb_main

                if use_gate and args.same_frame_aruco_gate:
                    _, gray_gate = _prepare_aruco_bgr_gray(
                        rgb_main,
                        args.calib_flip_vertical,
                        args.calib_no_contrast,
                        args.calib_black_level,
                        args.calib_white_level,
                        args.calib_clahe_clip,
                    )
                    cx, _, _, _ = _aruco_detect_center(
                        gray_gate,
                        aruco_dict_gate,
                        args.aruco_target_id,
                        aruco_detector_main,
                        aruco_dp_main,
                    )
                    marker_visible = cx is not None

                allow_mqtt = (not use_gate) or (not marker_visible)
                if use_gate and marker_visible:
                    mqtt_state["buffer"] = []
                    mqtt_state["window_start"] = None
                    if args.same_frame_aruco_gate:
                        status_lines = [
                            "ArUco DETECTED (same frame) | MQTT OFF",
                            "Main stream/YOLO uses post_calib_ev",
                            "Cover/remove marker to send MQTT (%.1fs)" % args.mqtt_delay,
                        ]
                    else:
                        status_lines = [
                            "ArUco DETECTED (gate frame, calib_ev) | MQTT OFF",
                            "Main stream/YOLO uses post_calib_ev",
                            "Cover/remove marker to send MQTT (%.1fs)" % args.mqtt_delay,
                        ]
                elif use_gate:
                    if args.same_frame_aruco_gate:
                        status_lines = [
                            "ArUco NOT DETECTED (same frame)",
                            "Main stream/YOLO uses post_calib_ev | MQTT if stable %.1fs" % args.mqtt_delay,
                        ]
                    else:
                        status_lines = [
                            "ArUco NOT DETECTED (gate frame, calib_ev)",
                            "Main stream/YOLO uses post_calib_ev | MQTT if stable %.1fs" % args.mqtt_delay,
                        ]
                else:
                    status_lines = []

                capture_frame_count[0] += 1
                xyxy = conf_np = cls_np = None
                infer_ms = 0.0
                if capture_frame_count[0] % args.infer_every == 0:
                    t0 = time.perf_counter()
                    try:
                        results = yolo_model.predict(
                            bgr,
                            conf=args.conf,
                            imgsz=args.imgsz,
                            verbose=False,
                            device="cpu",
                            classes=waste_class_ids,
                        )[0]
                        infer_ms = (time.perf_counter() - t0) * 1000
                        if results.boxes is not None and len(results.boxes) > 0:
                            xyxy = results.boxes.xyxy.cpu().numpy()
                            conf_np = results.boxes.conf.cpu().numpy()
                            cls_np = results.boxes.cls.cpu().numpy()
                    except Exception as ex:
                        infer_ms = (time.perf_counter() - t0) * 1000
                        print("YOLO predict error:", ex, flush=True)

                    with cond:
                        last_detections = (xyxy, conf_np, cls_np)

                    det_count[0] += 1
                    n = len(xyxy) if xyxy is not None else 0
                    if not args.no_print_dets:
                        fps = 1000.0 / infer_ms if infer_ms > 0 else 0
                        print(
                            "[det {}] {:.1f} ms ({:.1f} FPS) | conf≥{}".format(
                                det_count[0], infer_ms, fps, args.conf,
                            ),
                            flush=True,
                        )
                        found = False
                        if n > 0 and conf_np is not None and cls_np is not None:
                            for i in range(n):
                                c = float(conf_np[i])
                                if c < args.conf:
                                    continue
                                cid = int(cls_np[i])
                                name = (
                                    CLASS_NAMES[cid]
                                    if 0 <= cid < len(CLASS_NAMES)
                                    else "cls%d" % cid
                                )
                                cat_id, cat_name = CLASS_TO_CATEGORY.get(name, (2, "general waste"))
                                x1, y1, x2, y2 = map(int, xyxy[i])
                                tag = "" if allow_mqtt else "  [no MQTT: ArUco gate visible]"
                                c_show = _boost_conf_for_output(name, c, args.conf)
                                print(
                                    "  · {}  conf={:.2f}  -> cat{} ({})  box=[{}, {}, {}, {}]{}".format(
                                        name, c_show, cat_id, cat_name, x1, y1, x2, y2, tag,
                                    ),
                                    flush=True,
                                )
                                found = True
                        if not found:
                            empty_streak[0] += 1
                            pe = args.print_empty_every
                            if pe > 0 and (empty_streak[0] % pe == 1 or pe == 1):
                                print("  (no detections above conf this frame)", flush=True)
                        else:
                            empty_streak[0] = 0

                    if allow_mqtt:
                        cat = mqtt_update_and_maybe_publish(
                            xyxy,
                            conf_np,
                            cls_np,
                            CLASS_NAMES,
                            args.conf,
                            mqtt_client,
                            mqtt_state,
                            args.mqtt_delay,
                            last_res,
                            treat_empty_as_general=(use_gate and (not marker_visible)),
                        )
                        if cat > 0:
                            ang = last_res.get("angle_rad")
                            s = (
                                "res topic '{}' | last category {} -> angle {:.2f} rad ({:.1f} deg)".format(
                                    MQTT_TOPIC_RES, cat, ang, math.degrees(ang)
                                )
                                if ang is not None
                                else "res topic '{}' | last category {}".format(MQTT_TOPIC_RES, cat)
                            )
                            print(
                                "  -> MQTT category (mode in {:.1f}s): {} | {}".format(
                                    args.mqtt_delay, cat, s,
                                ),
                                flush=True,
                            )

                if not args.no_stream:
                    # Stream what the model "sees": publish an already-annotated frame
                    vis = bgr.copy()
                    draw_xyxy = draw_conf = draw_cls = None
                    if xyxy is not None and conf_np is not None and cls_np is not None and len(xyxy) > 0:
                        draw_xyxy, draw_conf, draw_cls = xyxy, conf_np, cls_np
                    else:
                        with cond:
                            det = last_detections
                        if det is not None and det[0] is not None and len(det[0]) > 0:
                            draw_xyxy, draw_conf, draw_cls = det[0], det[1], det[2]
                    if draw_xyxy is not None and draw_conf is not None and draw_cls is not None:
                        _draw_arrays(vis, draw_xyxy, draw_conf, draw_cls, CLASS_NAMES, args.conf)
                    if status_lines:
                        y0 = 28
                        for line in status_lines:
                            cv2.putText(
                                vis, line, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                            )
                            y0 += 22
                    _publish_stream_frame(vis, status_lines)
            except Exception as e:
                print("capture error:", e, flush=True)
                break

    Thread(target=capture_loop, daemon=True).start()

    if args.no_stream:
        print("Capture + infer + MQTT only. Ctrl+C to exit.", flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        picam2.stop()
        return 0

    print("Video stream already running; waiting for frames...", flush=True)
    with cond:
        cond.wait(timeout=5.0)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    picam2.stop()
    return 0


if __name__ == "__main__":
    main()
