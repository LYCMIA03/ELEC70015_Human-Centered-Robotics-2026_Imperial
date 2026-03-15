# Hand-Object RGB-D

Orbbec RGB-D camera + dual-model detection for "person holding object". Outputs the nearest holder's 3D coordinates (XYZ), with UDP streaming, headless mode, and terminal FPS/XYZ printing.

For upside-down camera mounts, the script supports software correction with
`--rotate-180`. RGB and depth are rotated together so the depth sample taken at
the detection box center still corresponds to the visible target.

## Requirements

- Python 3.8+
- `pip install pyorbbecsdk opencv-python numpy ultralytics`
- On Jetson/Linux: configure Orbbec udev rules

## Models

- **Person detection**: `yolov8n.pt` (COCO person), auto-downloaded by Ultralytics on first run.
- **Hand/object detection**: `best.pt` — download manually and place under `weight/`.

### Download best.pt

Download `best.pt` and put it in `handobj_detection/weight/`.

| File     | Description                                      | Download |
|----------|--------------------------------------------------|----------|
| best.pt  | Hand-Object YOLOv8 weights (classes: targetobject, hand) | [Download best.pt](YOUR_DOWNLOAD_LINK_HERE) |

Replace `YOUR_DOWNLOAD_LINK_HERE` with your actual URL (e.g. Google Drive, GitHub Release).

Expected layout:

```
handobj_detection/
  weight/
    best.pt   # place here
  handobj_detection_rgbd.py
  README.md
```

## Run

From the project root:

```bash
# With GUI
python handobj_detection/handobj_detection_rgbd.py

# No display / SSH (headless)
python handobj_detection/handobj_detection_rgbd.py --headless

# Terminal prints FPS and nearest holder XYZ every second
python handobj_detection/handobj_detection_rgbd.py --headless

# Print XYZ every frame
python handobj_detection/handobj_detection_rgbd.py --headless --print-xyz

# Enable UDP XYZ (same target as predict_15cls_rgbd: 127.0.0.1:16031)
python handobj_detection/handobj_detection_rgbd.py --udp-enable

# Correct an upside-down camera mount in software
python handobj_detection/handobj_detection_rgbd.py --rotate-180
```

## Options

| Option        | Default       | Description |
|---------------|---------------|-------------|
| `--weights`   | weight/best.pt | Path to hand/object weights |
| `--conf`      | 0.35          | Confidence threshold |
| `--headless`  | -             | No GUI (e.g. SSH) |
| `--print-xyz` | -             | Print nearest holder XYZ every frame |
| `--device`    | auto          | Inference device: cuda / cpu / auto |
| `--udp-enable`| -             | Send XYZ over UDP |
| `--udp-host`  | 127.0.0.1     | UDP host |
| `--udp-port`  | 16031         | UDP port |
| `--udp-rate`  | 10            | Max send rate (Hz) |
| `--rotate-180`| -             | Rotate aligned RGB + depth 180° before inference |

Terminal output every second: `FPS: X.X (loop) | YOLO: X.X fps (inference) | XYZ: x, y, z m` (or `XYZ: —` when no holder is detected).
