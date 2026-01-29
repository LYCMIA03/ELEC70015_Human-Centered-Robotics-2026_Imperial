#!/usr/bin/env python3
"""
Publish a synthetic CameraInfo message for a depth camera in Gazebo when the gazebo_ros_depth_camera
plugin does not publish camera_info (topic is advertised but stays at 0 Hz).

Key idea:
- Subscribe to the depth image topic.
- For every incoming Image message, publish a CameraInfo message with header.stamp copied from the Image.
  This guarantees message_filters synchronization in depthimage_to_laserscan.
"""

import math
import rospy
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoFromImageStamp:
    def __init__(self):
        # Camera intrinsics configuration (match your URDF/xacro)
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.hfov = rospy.get_param("~horizontal_fov", 1.047)  # radians
        self.frame_id = rospy.get_param("~frame_id", "depth_camera_link")

        # Topics
        self.in_image_topic = rospy.get_param(
            "~in_image_topic", "/sim_p3at/depth_camera/depth/image_raw"
        )
        self.out_info_topic = rospy.get_param(
            "~out_info_topic", "/sim_p3at/depth_camera/depth/camera_info"
        )

        # Compute pinhole intrinsics from HFOV and image width (assume square pixels)
        fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        # Prepare a static CameraInfo template (only header changes per frame)
        self.info = CameraInfo()
        self.info.width = self.width
        self.info.height = self.height
        self.info.distortion_model = "plumb_bob"
        self.info.D = [0.0, 0.0, 0.0, 0.0, 0.0]

        # K (3x3) intrinsic matrix
        self.info.K = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        # R (3x3) rectification matrix (identity)
        self.info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        # P (3x4) projection matrix for a monocular camera
        self.info.P = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        # Publisher/subscriber
        self.pub = rospy.Publisher(self.out_info_topic, CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber(self.in_image_topic, Image, self._cb, queue_size=10)

    def _cb(self, img: Image):
        # Critical: align CameraInfo timestamp with the depth image timestamp
        self.info.header.stamp = img.header.stamp
        self.info.header.frame_id = self.frame_id
        self.pub.publish(self.info)


def main():
    rospy.init_node("camera_info_from_image_stamp", anonymous=True)
    CameraInfoFromImageStamp()
    rospy.spin()


if __name__ == "__main__":
    main()
