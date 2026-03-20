# Raspberry Pi — NCNN waste classification (CSI camera)

This branch focuses on **on-device trash / waste classification** on a Raspberry Pi using an **Ultralytics YOLO model exported to NCNN**, a **CSI camera** (`picamera2`), optional **MQTT** to an ESP32 stack, and optional **ArUco** zero calibration plus an **occlusion gate** (only classify when the marker is covered).

Primary script: `trash_detection/webcam_15cls_ncnn_pi.py`.

---

## What it does

- Captures **RGB** from the Pi camera (`RGB888`, configurable resolution).
- Runs **18-class detection** via `YOLO(<ncnn_dir>, task="detect")` on **CPU** (NCNN backend inside Ultralytics).
- Maps detections to **four disposal categories** (paper/card, general, dry recycling, food) for MQTT payloads and motor presets.
- Serves an **MJPEG preview** over HTTP (Flask) on port `8765` by default.
- **Startup calibration** (unless skipped): points an ArUco marker at the camera for a few seconds; median angle is negated and published over MQTT for plate zeroing.
- **ArUco gate** (default): while the marker is **visible**, detections are shown but **MQTT is suppressed**; cover/remove the marker to allow timed MQTT publish after stable viewing (`--mqtt-delay`).

---

## Requirements

- Raspberry Pi OS (or similar) with **Picamera2** / **libcamera** working.
- Python 3.10+ recommended.
- **NCNN export** placed under:

  `trash_detection/waste_ncnn_model/model.ncnn.param`  
  `trash_detection/waste_ncnn_model/model.ncnn.bin`

- Python packages (minimal set):

  ```bash
  pip install ultralytics torch opencv-python picamera2 flask numpy
  pip install paho-mqtt   # optional, for MQTT
  ```

- **OpenCV contrib** for ArUco (`cv2.aruco`), e.g. `opencv-contrib-python-headless`, if you use calibration or the gate.

---

## Quick start

On the Raspberry Pi (using your existing virtual environment and absolute paths):

```bash
cd /home/shiroha/ELEC70015_Human-Centered-Robotics-2026_Imperial
source trash_detection/venv/bin/activate
python trash_detection/webcam_15cls_ncnn_pi.py
```

Open the printed URL (LAN IP + port), e.g. `http://<pi-ip>:8765`.

**Headless (no HTTP stream):**

```bash
python trash_detection/webcam_15cls_ncnn_pi.py --no-stream
```

**Skip ArUco calibration at startup:**

```bash
python trash_detection/webcam_15cls_ncnn_pi.py --skip-calib
```

**Disable MQTT:**

```bash
python trash_detection/webcam_15cls_ncnn_pi.py --no-mqtt
```

**Disable the ArUco gate (always allow MQTT when rules match):**

```bash
python trash_detection/webcam_15cls_ncnn_pi.py --no-aruco-gate
```

---

## MQTT topics (defaults in code)

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `imperial/yh4222/esp32/test` | Pi → broker | JSON: category + name + short detection summary (after delay window) |
| `imperial/yh4222/esp32/calib` | Pi → broker | String: **negated** zero angle in degrees (after startup calib) |
| `imperial/yh4222/esp32/res` | Pi → broker | Text `"1"`–`"4"`: motor preset from winning category |

Broker default: `broker.emqx.io:1883`. Override requires editing constants at the top of `webcam_15cls_ncnn_pi.py` or extending the script.

**Category → preset angle (radians), for reference:**  
1 → `0`, 2 → `-π/2`, 3 → `-π`, 4 → `+π/2`.

---

## Useful CLI flags

| Flag | Meaning |
|------|---------|
| `--conf` | Confidence threshold (default `0.35`) |
| `--imgsz` | Inference size (e.g. `320` for speed) |
| `--infer-every N` | Run YOLO every N-th frame |
| `--port` | MJPEG server port |
| `--mqtt-delay` | Seconds object must be present (with gate rules) before MQTT publish |
| `--calib-sec` | ArUco calibration capture duration |
| `--calib-ev` / `--post-calib-ev` | ExposureValue for calib vs normal run |
| `--same-frame-aruco-gate` | Legacy single-frame gate (same exposure as YOLO) |
| `--dual-capture-settle` | Sleep after EV change before capture |
| `--include-human` | Do not filter out the `Human` class |

---

## Class list and category mapping

The model uses **18** classes (including `Human`). Names and mapping to **four** MQTT categories are defined in `CLASS_NAMES` and `CLASS_TO_CATEGORY` inside `webcam_15cls_ncnn_pi.py`.

---

## Related material in this repository

Other folders (`catkin_ws/`, `dialogue/`, `handobj_detection/`, etc.) may contain **ROS**, **Jetson**, or **host-side** components. This README documents only the **Pi + NCNN** pipeline above. For a full system runbook, check other branches or `doc.md` if present on your checkout.

---

## License / course

ELEC70015 Human-Centered Robotics — Imperial College London. Use and attribution per your course and team policy.
