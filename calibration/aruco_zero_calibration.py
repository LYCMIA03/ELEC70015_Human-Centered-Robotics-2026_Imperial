# -*- coding: utf-8 -*-
"""
ArUco real-time: webcam, 0° = up (marker top-right aligned to image up). Output: center (x,y), angle deg.
Run: python calibration/aruco_zero_calibration.py [--camera-index 0]. Press Q to quit.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import cv2

try:
    aruco_module = cv2.aruco
except AttributeError:
    aruco_module = None

ROOT = Path(__file__).resolve().parent


def _get_dict(dict_name: str):
    if aruco_module is None:
        raise RuntimeError("cv2.aruco not found. Install: pip install opencv-contrib-python")
    name_upper = dict_name.upper().replace(" ", "")
    mapping = {
        "4X4_50": aruco_module.DICT_4X4_50,
        "4X4_100": aruco_module.DICT_4X4_100,
        "5X5_50": aruco_module.DICT_5X5_50,
        "6X6_250": aruco_module.DICT_6X6_250,
        "6X6_1000": aruco_module.DICT_6X6_1000,
        "7X7_50": aruco_module.DICT_7X7_50,
    }
    dict_id = mapping.get(name_upper, aruco_module.DICT_4X4_50)
    return aruco_module.getPredefinedDictionary(dict_id)


def detect_aruco_center(gray, dictionary, target_id=None):
    """Detect ArUco; return (cx, cy, corners, marker_id). corners: [0]=tl [1]=tr [2]=br [3]=bl."""
    if aruco_module is None:
        return None, None, None, None
    try:
        params = aruco_module.DetectorParameters()
        detector = aruco_module.ArucoDetector(dictionary, params)
        corners_list, ids, _ = detector.detectMarkers(gray)
    except (AttributeError, TypeError):
        corners_list, ids, _ = aruco_module.detectMarkers(gray, dictionary)
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


def angle_zero_is_up_deg(corners, cx: float, cy: float):
    """0° = up (top-right corner points up). corners[1]=tr. Returns deg: 0=up, 90=right, 180=down, -90=left."""
    c = np.asarray(corners).reshape(4, 2)
    tr = c[1]  # top-right
    dx = tr[0] - cx
    dy = tr[1] - cy
    # image up = -y, so atan2(dx, -dy) gives 0° for (0,-1)
    return math.degrees(math.atan2(dx, -dy))


def main():
    parser = argparse.ArgumentParser(description="ArUco: 0°=up, output center (x,y) and angle deg")
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index")
    parser.add_argument("--aruco-dict", type=str, default="4X4_50", help="ArUco dict")
    parser.add_argument("--target-id", type=int, default=None, help="Only this marker ID")
    parser.add_argument("--print-rate", type=float, default=10.0, help="Print rate Hz, 0=off")
    args = parser.parse_args()

    if aruco_module is None:
        print("cv2.aruco not found. Install: pip install opencv-contrib-python")
        return 1

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        print("Cannot open camera index=%s" % args.camera_index)
        return 1

    dictionary = _get_dict(args.aruco_dict)
    last_print_t = 0.0
    min_interval = 1.0 / args.print_rate if args.print_rate > 0 else 1e9
    arrow_len = 60  # 0° reference arrow length (px)

    try:
        cv2.namedWindow("ArUco", cv2.WINDOW_NORMAL)
    except Exception:
        pass

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cx, cy, corners, marker_id = detect_aruco_center(gray, dictionary, args.target_id)

            if cx is not None and corners is not None:
                aruco_module.drawDetectedMarkers(frame, [corners], np.array([marker_id]))
                ix, iy = int(cx), int(cy)
                cv2.circle(frame, (ix, iy), 6, (0, 255, 0), 2)

                # 0° = up: yellow arrow from center
                up_x = ix
                up_y = iy - arrow_len
                cv2.arrowedLine(frame, (ix, iy), (up_x, up_y), (0, 255, 255), 2, tipLength=0.2)
                cv2.putText(frame, "0", (up_x + 5, up_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # Center to top-right = current heading (green)
                c = np.asarray(corners).reshape(4, 2)
                tr = c[1]
                tx, ty = int(tr[0]), int(tr[1])
                cv2.line(frame, (ix, iy), (tx, ty), (0, 255, 0), 2)

                angle_deg = angle_zero_is_up_deg(corners, cx, cy)
                txt = "x=%d y=%d angle=%.1f deg (0=up)" % (ix, iy, angle_deg)
                cv2.putText(frame, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                now = time.perf_counter()
                if args.print_rate > 0 and now - last_print_t >= min_interval:
                    print("aruco_xy=%d,%d angle_deg=%.2f" % (ix, iy, angle_deg))
                    last_print_t = now
            else:
                cv2.putText(frame, "No ArUco", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("ArUco", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
