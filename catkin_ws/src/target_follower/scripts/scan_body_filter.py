#!/usr/bin/env python3
"""
scan_body_filter.py — Filter LaserScan points that fall inside the robot body.

Subscribes to a raw LaserScan, removes (sets to inf) any points whose 2-D
coordinates in the base_link frame land inside the robot body bounding box,
then re-publishes the cleaned scan.

Why:
  The Unitree L1 LiDAR is mounted on top of the P3-AT chassis.  When the
  scan is opened to a full 360° FOV, rear- and side-facing beams can hit
  the robot's own body.  This node filters those self-detections while
  preserving close-range obstacle detection everywhere else.

Geometry (configurable via ROS params):
  LiDAR position in base_link: (lidar_x, lidar_y) = (0.208, 0.0)
  Robot body box (with margin): x ∈ [-0.35, 0.35], y ∈ [-0.30, 0.30]

Topics:
  Subscribes: ~scan_raw  (sensor_msgs/LaserScan)
  Publishes:  ~scan_out  (sensor_msgs/LaserScan)

Usage in launch file:
  <node pkg="target_follower" type="scan_body_filter.py"
        name="scan_body_filter" output="screen">
    <remap from="~scan_raw" to="/unitree/scan_raw"/>
    <remap from="~scan_out" to="/unitree/scan"/>
  </node>
"""
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


class ScanBodyFilter:
    def __init__(self):
        # Robot body bounding box in base_link frame (with safety margin)
        # P3-AT footprint: ±0.32 × ±0.27  → add ~0.03 m margin each side
        self.body_x_min = float(rospy.get_param("~body_x_min", -0.35))
        self.body_x_max = float(rospy.get_param("~body_x_max",  0.35))
        self.body_y_min = float(rospy.get_param("~body_y_min", -0.30))
        self.body_y_max = float(rospy.get_param("~body_y_max",  0.30))

        # LiDAR position in base_link frame (static, from URDF)
        self.lidar_x = float(rospy.get_param("~lidar_x", 0.208))
        self.lidar_y = float(rospy.get_param("~lidar_y", 0.0))

        self._pub = rospy.Publisher("~scan_out", LaserScan, queue_size=1)
        rospy.Subscriber("~scan_raw", LaserScan, self._cb, queue_size=1)

        rospy.loginfo(
            "[ScanBodyFilter] body box x=[%.2f, %.2f] y=[%.2f, %.2f]  "
            "lidar@(%.3f, %.3f)",
            self.body_x_min, self.body_x_max,
            self.body_y_min, self.body_y_max,
            self.lidar_x, self.lidar_y,
        )

    # ------------------------------------------------------------------
    def _cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            self._pub.publish(msg)
            return

        # Pre-compute angles for every beam (only once if geometry unchanged)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # 2-D positions of each scan point in base_link
        px = self.lidar_x + ranges * np.cos(angles)
        py = self.lidar_y + ranges * np.sin(angles)

        # Mask: finite range AND inside body box → self-detection
        valid = np.isfinite(ranges) & (ranges > 0.0)
        inside = (
            valid
            & (px >= self.body_x_min) & (px <= self.body_x_max)
            & (py >= self.body_y_min) & (py <= self.body_y_max)
        )

        ranges[inside] = float("inf")

        out = msg
        out.ranges = ranges.tolist()
        self._pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("scan_body_filter")
    ScanBodyFilter()
    rospy.spin()
