#!/usr/bin/env python3
import math
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros

# Pure-python quaternion helpers (no PyKDL required)
def q_mult(q1, q2):
    # (x,y,z,w)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return (x, y, z, w)

def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def rotate_vec_by_q(v, q):
    # v as (x,y,z), q as (x,y,z,w)
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    return q_mult(q_mult(q, qv), q_conj(q))[:3]

def dist_xy(a, b):
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)

def transform_pose_stamped(pose_in, tf_stamped, target_frame):
    """
    Apply a TransformStamped to a PoseStamped, returning PoseStamped in target_frame.
    tf_stamped is a transform from pose_in.header.frame_id -> target_frame.
    """
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation

    q_tf = (r.x, r.y, r.z, r.w)

    # position
    p = pose_in.pose.position
    v_rot = rotate_vec_by_q((p.x, p.y, p.z), q_tf)
    x = v_rot[0] + t.x
    y = v_rot[1] + t.y
    z = v_rot[2] + t.z

    # orientation
    o = pose_in.pose.orientation
    q_pose = (o.x, o.y, o.z, o.w)
    q_out = q_mult(q_tf, q_pose)

    out = PoseStamped()
    out.header.stamp = rospy.Time.now()
    out.header.frame_id = target_frame
    out.pose.position.x = x
    out.pose.position.y = y
    out.pose.position.z = z
    out.pose.orientation = Quaternion(*q_out)
    return out


class TargetFollower:
    def __init__(self):
        self.target_topic = rospy.get_param("~target_topic", "/target_pose")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 2.0))
        self.min_update_dist = float(rospy.get_param("~min_update_dist", 0.3))
        self.target_timeout_s = float(rospy.get_param("~target_timeout_s", 1.0))
        self.use_target_orientation = bool(rospy.get_param("~use_target_orientation", False))

        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        self.last_sent_goal = None
        self.last_target = None
        self.last_send_time = rospy.Time(0)

        rospy.Subscriber(self.target_topic, PoseStamped, self.cb_target, queue_size=1)

    def cb_target(self, msg):
        self.last_target = msg

    def maybe_send_goal(self):
        if self.last_target is None:
            return

        now = rospy.Time.now()
        if self.last_target.header.stamp != rospy.Time(0):
            age = (now - self.last_target.header.stamp).to_sec()
            if age > self.target_timeout_s:
                rospy.logwarn_throttle(2.0, "Target pose is stale (age=%.2fs), skipping", age)
                return

        try:
            if self.last_target.header.frame_id != self.global_frame:
                tfm = self.tfbuf.lookup_transform(
                    self.global_frame,
                    self.last_target.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
                target_g = transform_pose_stamped(self.last_target, tfm, self.global_frame)
            else:
                target_g = self.last_target
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF not ready for target transform: %s", str(e))
            return

        if (now - self.last_send_time).to_sec() < (1.0 / max(self.send_rate_hz, 0.1)):
            return

        if self.last_sent_goal is not None:
            if dist_xy(target_g, self.last_sent_goal) < self.min_update_dist:
                return

        goal = MoveBaseGoal()
        goal.target_pose = target_g
        goal.target_pose.header.stamp = rospy.Time.now()

        # In most follower setups, forcing the robot to match the target's
        # orientation causes unnecessary spinning and can trigger recovery.
        if not self.use_target_orientation:
            try:
                tf_robot = self.tfbuf.lookup_transform(
                    self.global_frame,
                    self.robot_frame,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
                goal.target_pose.pose.orientation = tf_robot.transform.rotation
            except Exception as e:
                rospy.logwarn_throttle(2.0, "TF not ready for robot pose: %s", str(e))
                goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self.client.send_goal(goal)
        self.last_sent_goal = target_g
        self.last_send_time = now

    def spin(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.maybe_send_goal()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("target_follower")
    node = TargetFollower()
    node.spin()
