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

        # Stop short of the target by this distance (metres). 0 = go all the way.
        self.standoff_distance = float(rospy.get_param("~standoff_distance", 0.0))
        # Orient the robot to face the target at the goal pose.
        self.face_target = bool(rospy.get_param("~face_target", False))

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

        if self.standoff_distance > 0:
            rospy.loginfo("standoff_distance = %.2f m", self.standoff_distance)
        if self.face_target:
            rospy.loginfo("face_target enabled")

    def cb_target(self, msg):
        self.last_target = msg

    def _get_robot_pose_in_global(self):
        """Return (x, y) of robot in global_frame, or None on failure."""
        try:
            tf_robot = self.tfbuf.lookup_transform(
                self.global_frame, self.robot_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
            rx = tf_robot.transform.translation.x
            ry = tf_robot.transform.translation.y
            return rx, ry, tf_robot
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF not ready for robot pose: %s", str(e))
            return None

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Convert a yaw angle (rad) to geometry_msgs/Quaternion."""
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

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

        # --- Compute goal pose (possibly with standoff) ---
        tx = target_g.pose.position.x
        ty = target_g.pose.position.y

        goal_x, goal_y = tx, ty

        if self.standoff_distance > 0 or self.face_target:
            robot_info = self._get_robot_pose_in_global()
            if robot_info is None:
                return
            rx, ry, tf_robot = robot_info

            dx = tx - rx
            dy = ty - ry
            d = math.hypot(dx, dy)

            if self.standoff_distance > 0:
                if d <= self.standoff_distance:
                    # Already within standoff range — no need to send a new goal.
                    return
                # Place goal standoff_distance metres back along the robot→target vector.
                ratio = (d - self.standoff_distance) / d
                goal_x = rx + dx * ratio
                goal_y = ry + dy * ratio

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0

        # --- Orientation ---
        if self.face_target:
            # Face from the goal position towards the actual target.
            face_dx = tx - goal_x
            face_dy = ty - goal_y
            if math.hypot(face_dx, face_dy) > 0.01:
                yaw = math.atan2(face_dy, face_dx)
            else:
                # Goal ≈ target (standoff ≈ 0); face from robot towards target instead.
                robot_info = self._get_robot_pose_in_global()
                if robot_info is not None:
                    yaw = math.atan2(ty - robot_info[1], tx - robot_info[0])
                else:
                    yaw = 0.0
            goal.target_pose.pose.orientation = self._yaw_to_quaternion(yaw)
        elif self.use_target_orientation:
            goal.target_pose.pose.orientation = target_g.pose.orientation
        else:
            # Keep the robot's current orientation (avoid spinning).
            robot_info = self._get_robot_pose_in_global()
            if robot_info is not None:
                goal.target_pose.pose.orientation = robot_info[2].transform.rotation
            else:
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
