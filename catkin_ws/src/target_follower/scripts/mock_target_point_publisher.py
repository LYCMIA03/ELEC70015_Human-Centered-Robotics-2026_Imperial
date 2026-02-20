#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PointStamped


class MockTargetPointPublisher:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/trash_detection/target_point")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.rate_hz = float(rospy.get_param("~rate_hz", 5.0))
        self.mode = rospy.get_param("~mode", "fixed").strip().lower()

        self.fixed_x = float(rospy.get_param("~x", 2.0))
        self.fixed_y = float(rospy.get_param("~y", 0.0))
        self.fixed_z = float(rospy.get_param("~z", 0.0))

        self.circle_radius = float(rospy.get_param("~circle_radius", 1.5))
        self.circle_center_x = float(rospy.get_param("~circle_center_x", 2.0))
        self.circle_center_y = float(rospy.get_param("~circle_center_y", 0.0))
        self.circle_z = float(rospy.get_param("~circle_z", 0.0))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.3))

        self.pub = rospy.Publisher(self.topic, PointStamped, queue_size=10)
        self.start_time = rospy.Time.now()

        rospy.loginfo(
            "mock_target_point_publisher: topic=%s frame=%s mode=%s",
            self.topic,
            self.frame_id,
            self.mode,
        )

    def _current_point(self):
        if self.mode == "fixed":
            return self.fixed_x, self.fixed_y, self.fixed_z

        if self.mode == "circle":
            t = (rospy.Time.now() - self.start_time).to_sec()
            theta = self.angular_speed * t
            x = self.circle_center_x + self.circle_radius * math.cos(theta)
            y = self.circle_center_y + self.circle_radius * math.sin(theta)
            return x, y, self.circle_z

        rospy.logwarn_throttle(2.0, "Unknown mode '%s', fallback to fixed", self.mode)
        return self.fixed_x, self.fixed_y, self.fixed_z

    def spin(self):
        rate = rospy.Rate(max(self.rate_hz, 0.1))
        while not rospy.is_shutdown():
            x, y, z = self._current_point()

            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.point.x = x
            msg.point.y = y
            msg.point.z = z
            self.pub.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mock_target_point_publisher")
    MockTargetPointPublisher().spin()
