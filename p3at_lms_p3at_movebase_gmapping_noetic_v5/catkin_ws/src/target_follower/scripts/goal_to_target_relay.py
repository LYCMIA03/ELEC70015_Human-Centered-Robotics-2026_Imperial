#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


class GoalToTargetRelay:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_goal_topic", "/move_base_simple/goal")
        self.out_topic = rospy.get_param("~out_target_topic", "/target_pose")
        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.in_topic, PoseStamped, self.cb, queue_size=1)
        rospy.loginfo("Relaying %s to %s", self.in_topic, self.out_topic)

    def cb(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("goal_to_target_relay")
    GoalToTargetRelay()
    rospy.spin()
