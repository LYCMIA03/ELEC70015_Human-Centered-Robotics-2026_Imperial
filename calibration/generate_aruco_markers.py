# -*- coding: utf-8 -*-
"""
Generate printable ArUco marker images. Print and stick on disk.
Run: python calibration/generate_aruco_markers.py [--id 0] [--out-dir ./aruco_markers]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = ROOT / "aruco_markers"

DICT_NAMES = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
    "7X7_50": cv2.aruco.DICT_7X7_50,
}


def generate_one_marker(marker_id: int, dict_name: str = "4X4_50", size_px: int = 400, border_px: int = 40):
    """Generate one ArUco image (black/white + white border for print)."""
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("Install opencv-contrib-python: pip install opencv-contrib-python")
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_NAMES.get(dict_name.upper(), cv2.aruco.DICT_4X4_50))
    try:
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    except AttributeError:
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, size_px)
    total = size_px + 2 * border_px
    out = np.ones((total, total), dtype=np.uint8) * 255
    out[border_px : border_px + size_px, border_px : border_px + size_px] = marker_img
    return out


def main():
    parser = argparse.ArgumentParser(description="Generate ArUco marker images for print")
    parser.add_argument("--id", type=int, nargs="+", default=[0], help="Marker ID(s)")
    parser.add_argument("--dict", type=str, default="4X4_50", choices=list(DICT_NAMES.keys()))
    parser.add_argument("--size", type=int, default=400, help="Marker side px")
    parser.add_argument("--border", type=int, default=40, help="White border px")
    parser.add_argument("--out-dir", type=str, default=None, help="Output dir (default: calibration/aruco_markers)")
    args = parser.parse_args()
    out_dir = Path(args.out_dir) if args.out_dir else DEFAULT_OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)

    for mid in args.id:
        img = generate_one_marker(mid, args.dict, args.size, args.border)
        path = out_dir / ("aruco_%s_id%d.png" % (args.dict.lower(), mid))
        cv2.imwrite(str(path), img)
        print("Generated: %s" % path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
