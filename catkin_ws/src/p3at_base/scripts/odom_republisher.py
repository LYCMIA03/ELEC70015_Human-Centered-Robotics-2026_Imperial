#!/usr/bin/env python3
"""
Republish RosAria odometry (/RosAria/pose) as standard /odom topic.
Optionally broadcast odom -> base_link TF (disable if RosAria already publishes it).

Parameters:
  ~input_topic  (str)  : Input odometry topic  [default: /RosAria/pose]
  ~output_topic (str)  : Output odometry topic  [default: /odom]
  ~publish_tf   (bool) : Whether to broadcast odom -> base_link TF [default: false]
"""

import rospy
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


class OdomRepublisher:
    def __init__(self):
        rospy.init_node("odom_republisher", anonymous=False)

        self.input_topic = rospy.get_param("~input_topic", "/RosAria/pose")
        self.output_topic = rospy.get_param("~output_topic", "/odom")
        self.publish_tf = rospy.get_param("~publish_tf", False)

        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=50)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(self.input_topic, Odometry, self.odom_callback)
        rospy.loginfo(
            "[odom_republisher] %s -> %s (publish_tf=%s)",
            self.input_topic, self.output_topic, self.publish_tf,
        )

    def odom_callback(self, msg):
        # Ensure correct frame_ids
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Republish
        self.odom_pub.publish(msg)

        # Optionally broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = "base_link"
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = OdomRepublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
