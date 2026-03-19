# -*- coding: utf-8 -*-
"""
ArUco 实时：树莓派 CSI（picamera2）或 USB 摄像头，0° = 上。
默认无显示器：MJPEG 局域网视频流（浏览器打开即可）。
Run:
  python calibration/aruco_zero_calibration.py
  python calibration/aruco_zero_calibration.py --port 8766
  python calibration/aruco_zero_calibration.py --webcam
  python calibration/aruco_zero_calibration.py --window   # 本地窗口（需桌面）
  # 默认: --ev -0.7 --black-level 150 --white-level 100（色阶两端自动取 min/max）
  python calibration/aruco_zero_calibration.py --ev 0 --no-contrast   # 关掉曝光补偿与色阶
Ctrl+C 结束服务。
"""
from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from threading import Condition, Thread

import numpy as np
import cv2
from flask import Flask, Response, render_template_string

try:
    aruco_module = cv2.aruco
except AttributeError:
    aruco_module = None

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # type: ignore


def _apply_pi_exposure_ev(picam2, ev: float) -> None:
    """在自动曝光下按 EV 档整体压暗/提亮（libcamera ExposureValue）。"""
    try:
        from libcamera import controls

        picam2.set_controls({controls.ExposureValue: float(ev)})
    except Exception:
        try:
            picam2.set_controls({"ExposureValue": float(ev)})
        except Exception as e:
            print("Warning: 无法设置曝光 EV (%s): %s" % (ev, e))
            return
    time.sleep(0.2)
    print("已设置曝光 EV = %.2f（负值更暗）" % ev)


def _levels_correct_bgr(
    frame_bgr: np.ndarray,
    clahe_clip: float,
    black_level: int,
    white_level: int,
):
    """
    LAB 亮度色阶：black_level 与 white_level 先取较小者为暗端 bp、较大者为亮端 wp，
    线性把 [bp, wp] 拉到 [0,255]。可选 CLAHE。
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l, a, b_ch = cv2.split(lab)
    if clahe_clip and clahe_clip > 0:
        clahe = cv2.createCLAHE(clipLimit=float(clahe_clip), tileGridSize=(8, 8))
        l = clahe.apply(l)
    b0 = int(np.clip(black_level, 0, 250))
    w0 = int(np.clip(white_level, 0, 255))
    bp, wp = min(b0, w0), max(b0, w0)
    if wp <= bp + 1:
        wp = min(bp + 2, 255)
    lf = l.astype(np.float32)
    denom = max(float(wp - bp), 1.0)
    l = np.clip((lf - float(bp)) * (255.0 / denom), 0, 255).astype(np.uint8)
    out = cv2.cvtColor(cv2.merge([l, a, b_ch]), cv2.COLOR_LAB2BGR)
    return out, l


def _get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


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


def _build_aruco_detector(dictionary, adapt_const, min_perimeter_rate, adapt_min, adapt_max):
    """返回 (ArucoDetector|None, DetectorParameters) 供新/旧 API。"""
    dp = aruco_module.DetectorParameters()
    dp.adaptiveThreshConstant = int(adapt_const)
    dp.minMarkerPerimeterRate = float(min_perimeter_rate)
    dp.adaptiveThreshWinSizeMin = int(adapt_min)
    dp.adaptiveThreshWinSizeMax = int(adapt_max)
    if hasattr(dp, "cornerRefinementMethod") and hasattr(aruco_module, "CORNER_REFINE_SUBPIX"):
        dp.cornerRefinementMethod = aruco_module.CORNER_REFINE_SUBPIX
    try:
        det = aruco_module.ArucoDetector(dictionary, dp)
        return det, dp
    except (AttributeError, TypeError):
        return None, dp


def detect_aruco_center(gray, dictionary, target_id=None, aruco_detector=None, detector_params=None):
    """Detect ArUco; return (cx, cy, corners, marker_id). corners: [0]=tl [1]=tr [2]=br [3]=bl."""
    if aruco_module is None:
        return None, None, None, None
    if aruco_detector is not None:
        corners_list, ids, _ = aruco_detector.detectMarkers(gray)
    elif detector_params is not None:
        corners_list, ids, _ = aruco_module.detectMarkers(gray, dictionary, detector_params)
    else:
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
    """0° = up. Returns deg: 0=up, 90=right, 180=down, -90=left."""
    c = np.asarray(corners).reshape(4, 2)
    tr = c[1]
    dx = tr[0] - cx
    dy = tr[1] - cy
    return math.degrees(math.atan2(dx, -dy))


def _draw_aruco_overlay(frame, corners, marker_id, cx, cy, arrow_len=60):
    aruco_module.drawDetectedMarkers(frame, [corners], np.array([marker_id]))
    ix, iy = int(cx), int(cy)
    cv2.circle(frame, (ix, iy), 6, (0, 255, 0), 2)
    up_x, up_y = ix, iy - arrow_len
    cv2.arrowedLine(frame, (ix, iy), (up_x, up_y), (0, 255, 255), 2, tipLength=0.2)
    cv2.putText(frame, "0", (up_x + 5, up_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    c = np.asarray(corners).reshape(4, 2)
    tr = c[1]
    cv2.line(frame, (ix, iy), (int(tr[0]), int(tr[1])), (0, 255, 0), 2)
    angle_deg = angle_zero_is_up_deg(corners, cx, cy)
    txt = "x=%d y=%d angle=%.1f deg (0=up)" % (ix, iy, angle_deg)
    cv2.putText(frame, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return angle_deg


def main():
    parser = argparse.ArgumentParser(description="ArUco 标定：局域网 MJPEG 推流（默认）")
    parser.add_argument("--webcam", action="store_true", help="USB 摄像头")
    parser.add_argument("--camera-index", type=int, default=0, help="USB 摄像头序号")
    parser.add_argument("--width", type=int, default=640, help="分辨率宽")
    parser.add_argument("--height", type=int, default=480, help="分辨率高")
    parser.add_argument("--aruco-dict", type=str, default="4X4_50", help="ArUco 字典")
    parser.add_argument("--target-id", type=int, default=None, help="只跟踪该 ID")
    parser.add_argument("--print-rate", type=float, default=10.0, help="终端打印 Hz，0=关闭")
    parser.add_argument("--port", type=int, default=8766, help="HTTP 端口")
    parser.add_argument("--quality", type=int, default=85, help="MJPEG JPEG 质量 1-100")
    parser.add_argument(
        "--window",
        action="store_true",
        help="同时显示本地窗口（需图形界面；无显示器勿用）",
    )
    parser.add_argument(
        "--rgb888-convert",
        action="store_true",
        help="对 Pi 摄像头做 RGB888→BGR；默认不做（imx708 等缓冲区实为 BGR 序，再转会红蓝对调）",
    )
    parser.add_argument(
        "--flip-vertical",
        action="store_true",
        help="画面上下翻转（摄像头倒装时用）",
    )
    parser.add_argument(
        "--ev",
        type=float,
        default=-0.7,
        help="曝光补偿(EV)，默认 -0.7；仅 Pi CSI",
    )
    parser.add_argument(
        "--no-contrast",
        action="store_true",
        help="关闭黑/白场校订与 CLAHE",
    )
    parser.add_argument(
        "--clahe-clip",
        type=float,
        default=0.0,
        help="可选：CLAHE 局部对比，默认 0=不用；需要时如 3.0",
    )
    parser.add_argument(
        "--black-level",
        type=int,
        default=150,
        help="色阶一端(与 white 取 min/max 为暗端/亮端)，默认 150",
    )
    parser.add_argument(
        "--white-level",
        type=int,
        default=100,
        help="色阶另一端，默认 100；与 black-level 自动排序成 [暗,亮] 再拉满",
    )
    parser.add_argument(
        "--aruco-preset",
        choices=("default", "sensitive"),
        default="default",
        help="sensitive: 更容易检出(略增误检)，暗光/对比度差可试",
    )
    parser.add_argument(
        "--aruco-adapt-const",
        type=int,
        default=None,
        help="自适应二值化常数 C（常叫 threshold）；默认7，越小越敏感，可试 4~5 或 3",
    )
    parser.add_argument(
        "--aruco-min-perimeter-rate",
        type=float,
        default=None,
        help="标记周长占图像比例下限，默认0.03；码很小/很远可试 0.008~0.015",
    )
    parser.add_argument(
        "--aruco-adapt-min",
        type=int,
        default=None,
        help="自适应窗口最小边长，默认3",
    )
    parser.add_argument(
        "--aruco-adapt-max",
        type=int,
        default=None,
        help="自适应窗口最大边长，默认23；可试31",
    )
    args = parser.parse_args()

    if aruco_module is None:
        print("cv2.aruco not found. Install: pip install opencv-contrib-python")
        return 1

    cap = None
    picam2 = None
    if args.webcam:
        if args.ev != 0.0:
            print("Note: --ev 仅对 Pi CSI 有效，USB 摄像头已忽略 EV")
        cap = cv2.VideoCapture(args.camera_index)
        if not cap.isOpened():
            print("Cannot open USB camera index=%s" % args.camera_index)
            return 1
    else:
        if Picamera2 is None:
            print("picamera2 not found. On Pi: sudo apt install python3-picamera2")
            print("Or: --webcam")
            return 1
        try:
            picam2 = Picamera2()
            cfg = picam2.create_preview_configuration(
                main={"size": (args.width, args.height), "format": "RGB888"}
            )
            picam2.configure(cfg)
            picam2.start()
            if args.ev != 0.0:
                _apply_pi_exposure_ev(picam2, args.ev)
        except Exception as e:
            print("Pi camera failed: %s" % e)
            return 1

    dictionary = _get_dict(args.aruco_dict)
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
    aruco_detector, aruco_dp = _build_aruco_detector(
        dictionary, adapt_c, min_pr, adapt_wmin, adapt_wmax
    )

    last_print_t = 0.0
    min_interval = 1.0 / args.print_rate if args.print_rate > 0 else 1e9
    arrow_len = 60
    latest_frame = None
    cond = Condition()
    stop_flag = {"v": False}

    def capture_loop():
        nonlocal latest_frame, last_print_t
        while not stop_flag["v"]:
            try:
                if picam2 is not None:
                    rgb = picam2.capture_array()
                    if args.rgb888_convert:
                        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    else:
                        # 标称 RGB888，缓冲区常为 BGR 序；再做 RGB2BGR 会红蓝对调
                        frame = np.ascontiguousarray(rgb)
                    if args.flip_vertical:
                        frame = cv2.flip(frame, 0)
                else:
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.01)
                        continue

                if not args.no_contrast and (
                    args.black_level > 0
                    or args.white_level < 255
                    or (args.clahe_clip and args.clahe_clip > 0)
                ):
                    frame, gray = _levels_correct_bgr(
                        frame,
                        args.clahe_clip,
                        args.black_level,
                        args.white_level,
                    )
                else:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                cx, cy, corners, marker_id = detect_aruco_center(
                    gray,
                    dictionary,
                    args.target_id,
                    aruco_detector=aruco_detector,
                    detector_params=aruco_dp if aruco_detector is None else None,
                )

                if cx is not None and corners is not None:
                    angle_deg = _draw_aruco_overlay(frame, corners, marker_id, cx, cy, arrow_len)
                    now = time.perf_counter()
                    if args.print_rate > 0 and now - last_print_t >= min_interval:
                        print("aruco_xy=%d,%d angle_deg=%.2f" % (int(cx), int(cy), angle_deg))
                        last_print_t = now
                else:
                    cv2.putText(
                        frame, "No ArUco", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                    )

                if args.window:
                    cv2.imshow("ArUco", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        stop_flag["v"] = True
                        break

                with cond:
                    latest_frame = frame.copy()
                    cond.notify_all()
            except Exception as e:
                print("capture error:", e)
                time.sleep(0.05)

    th = Thread(target=capture_loop, daemon=True)
    th.start()

    with cond:
        cond.wait(timeout=5.0)
    if latest_frame is None:
        print("No frames yet; check camera. Exiting.")
        stop_flag["v"] = True
        th.join(timeout=2.0)
        _release_cam(cap, picam2)
        return 1

    app = Flask(__name__)

    def generate():
        while not stop_flag["v"]:
            with cond:
                cond.wait()
                if stop_flag["v"]:
                    break
                if latest_frame is None:
                    continue
                frame = latest_frame.copy()
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            if not ok:
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
            )

    @app.route("/")
    def index():
        return render_template_string(
            """
<!DOCTYPE html>
<html><head><meta charset="utf-8"/><title>ArUco 标定</title>
<style>
body{margin:0;background:#111;color:#aaa;font-family:sans-serif;}
header{padding:8px 12px;background:#222;}
img{display:block;max-width:100%;height:auto;}
</style></head>
<body>
<header>ArUco · 0°=上 · <a href="/video_feed" style="color:#8f8">/video_feed</a></header>
<img src="/video_feed" alt="stream"/>
</body></html>
"""
        )

    @app.route("/video_feed")
    def video_feed():
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    ip = _get_lan_ip()
    print("-" * 50)
    print("ArUco 视频流:  http://%s:%s/" % (ip, args.port))
    print(
        "  ArUco 检测: adaptiveThreshConstant=%d, minPerimeterRate=%.4f, adaptWin %d~%d"
        % (adapt_c, min_pr, adapt_wmin, adapt_wmax)
    )
    if not args.webcam and args.ev != 0.0:
        print("  曝光补偿 ExposureValue: %.2f EV" % args.ev)
    if not args.no_contrast and (
        args.black_level > 0
        or args.white_level < 255
        or (args.clahe_clip and args.clahe_clip > 0)
    ):
        eb, ew = min(args.black_level, args.white_level), max(args.black_level, args.white_level)
        if ew <= eb + 1:
            ew = min(eb + 2, 255)
        msg = "  色阶: L[%d,%d]→满幅 (black=%d white=%d)" % (
            eb,
            ew,
            args.black_level,
            args.white_level,
        )
        if args.clahe_clip and args.clahe_clip > 0:
            msg += ", CLAHE=%.1f" % args.clahe_clip
        print(msg)
    print("  (局域网内手机/电脑浏览器打开上述地址)")
    print("Ctrl+C 停止")
    print("-" * 50)

    try:
        app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag["v"] = True
        with cond:
            cond.notify_all()
        th.join(timeout=2.0)
        _release_cam(cap, picam2)
        if args.window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
    return 0


def _release_cam(cap, picam2):
    if cap is not None:
        cap.release()
    if picam2 is not None:
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
