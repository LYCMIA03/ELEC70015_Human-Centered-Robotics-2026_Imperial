"""Helpers for correcting RGB-D frame orientation in software."""

import cv2


def rotate_rgbd_180(color_img, depth_img=None, enabled=False):
    """Rotate aligned RGB and depth images together when the camera is upside down."""
    if not enabled:
        return color_img, depth_img

    rotated_color = None if color_img is None else cv2.rotate(color_img, cv2.ROTATE_180)
    rotated_depth = None if depth_img is None else cv2.rotate(depth_img, cv2.ROTATE_180)
    return rotated_color, rotated_depth
