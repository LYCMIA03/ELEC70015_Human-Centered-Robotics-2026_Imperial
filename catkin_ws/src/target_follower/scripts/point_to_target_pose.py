#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion


class PointToTargetPose:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_point_topic", "/trash_detection/target_point")
        self.out_topic = rospy.get_param("~out_target_topic", "/target_pose")
        self.override_frame = rospy.get_param("~override_frame", "")
        self.stamp_now = bool(rospy.get_param("~stamp_now", True))
        self.drop_if_no_frame = bool(rospy.get_param("~drop_if_no_frame", True))

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.in_topic, PointStamped, self.cb, queue_size=1)
        rospy.loginfo("Relaying PointStamped %s -> PoseStamped %s", self.in_topic, self.out_topic)

    def cb(self, msg):
        frame_id = self.override_frame.strip() if self.override_frame else msg.header.frame_id
        if self.drop_if_no_frame and not frame_id:
            rospy.logwarn_throttle(2.0, "Drop target point: empty frame_id")
            return

        out = PoseStamped()
        out.header.stamp = rospy.Time.now() if self.stamp_now else msg.header.stamp
        out.header.frame_id = frame_id
        out.pose.position.x = msg.point.x
        out.pose.position.y = msg.point.y
        out.pose.position.z = msg.point.z
        out.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("point_to_target_pose")
    PointToTargetPose()
    rospy.spin()
