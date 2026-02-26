#!/usr/bin/env python3
"""
target_follower.py — Target following state machine with Dialogue integration.

State machine:
  IDLE
   └─ target received           → TRACKING
       └─ d <= standoff         → REACHED  (publishes result=True → triggers dialogue via bridge)
           └─ /trash_action=True  → IDLE        (accepted, resume following)
           └─ /trash_action=False → RETREATING  (refused, navigate away)
           └─ timeout             → IDLE        (no response, give up)
       └─ /trash_action received during RETREATING → navigate retreat → IDLE
  LOST / FAILED                 → IDLE on next target

Dialogue bridge flow (external, started by start_dialogue_docker_bridges.sh):
  /target_follower/result=True
    → navigation_success_udp_bridge → UDP:16041 → dialogue_udp_runner
    → UDP:16032 → udp_trash_action_bridge → /trash_action (Bool)
"""
import math
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
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
    # Valid states
    STATES = ("IDLE", "TRACKING", "REACHED", "WAITING_ACTION",
              "RETREATING", "LOST", "FAILED")

    def __init__(self):
        # ── Core following params ──────────────────────────────────────────
        self.target_topic = rospy.get_param("~target_topic", "/target_pose")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame  = rospy.get_param("~robot_frame", "base_link")
        self.send_rate_hz      = float(rospy.get_param("~send_rate_hz", 2.0))
        self.min_update_dist   = float(rospy.get_param("~min_update_dist", 0.3))
        self.target_timeout_s  = float(rospy.get_param("~target_timeout_s", 1.0))
        self.use_target_orientation = bool(
            rospy.get_param("~use_target_orientation", False))

        # Stop short of the target by this distance (metres). 0 = go all the way.
        self.standoff_distance = float(rospy.get_param("~standoff_distance", 0.8))
        # Orient the robot to face the target at the goal pose.
        self.face_target = bool(rospy.get_param("~face_target", False))

        # ── Dialogue / trash-action integration params ─────────────────────
        # Topic published by udp_trash_action_bridge (Bool): True=accept, False=refuse
        self.trash_action_topic = rospy.get_param("~trash_action_topic", "/trash_action")
        # How long (s) to wait in WAITING_ACTION before giving up (no dialogue response)
        self.action_wait_timeout_s = float(rospy.get_param("~action_wait_timeout_s", 45.0))
        # How far (m) to retreat when human refuses
        self.retreat_distance = float(rospy.get_param("~retreat_distance", 1.5))
        # Max time (s) allowed for the retreat navigation goal
        self.retreat_timeout_s = float(rospy.get_param("~retreat_timeout_s", 20.0))

        # ── TF & action client ─────────────────────────────────────────────
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        # ── Internal state ─────────────────────────────────────────────────
        self.last_sent_goal  = None
        self.last_target     = None
        self.last_send_time  = rospy.Time(0)
        self._goal_active    = False
        self._target_lost_logged = False
        self._suppress_done_cb   = False  # suppress next done_cb after manual cancel

        # Dialogue integration state
        self._pending_trash_action = None   # None | True | False
        self._waiting_action_start = None   # rospy.Time when WAITING_ACTION began
        self._retreat_start        = None   # rospy.Time when RETREATING began
        self._retreat_target_pos   = None   # (tx, ty) of last target, for retreat dir

        # ── Subscribers ────────────────────────────────────────────────────
        rospy.Subscriber(self.target_topic, PoseStamped,
                         self.cb_target, queue_size=1)
        rospy.Subscriber(self.trash_action_topic, Bool,
                         self.cb_trash_action, queue_size=5)

        # ── Publishers ─────────────────────────────────────────────────────
        # result: True=REACHED (dialogue trigger), False=FAILED/LOST
        self.result_pub = rospy.Publisher("~result",  Bool,   queue_size=1, latch=True)
        # status: IDLE|TRACKING|REACHED|WAITING_ACTION|RETREATING|LOST|FAILED
        self.status_pub = rospy.Publisher("~status",  String, queue_size=1)

        self._state = "IDLE"
        self._last_status_time = rospy.Time(0)

        rospy.loginfo("standoff_distance = %.2f m", self.standoff_distance)
        rospy.loginfo("face_target = %s", self.face_target)
        rospy.loginfo("trash_action_topic = %s", self.trash_action_topic)
        rospy.loginfo("action_wait_timeout = %.1f s", self.action_wait_timeout_s)
        rospy.loginfo("retreat_distance = %.2f m", self.retreat_distance)
        rospy.loginfo("Result topic: %s/result (std_msgs/Bool)", rospy.get_name())
        rospy.loginfo("Status topic: %s/status (std_msgs/String)", rospy.get_name())

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def cb_target(self, msg):
        """Incoming target pose — always update; state machine decides when to act."""
        self.last_target = msg
        self._target_lost_logged = False

    def cb_trash_action(self, msg):
        """
        Receive dialogue result from udp_trash_action_bridge.
        True  = human accepted (trash collected).
        False = human refused  (robot should retreat).
        Only meaningful when state is WAITING_ACTION or RETREATING.
        """
        value = bool(msg.data)
        rospy.loginfo("[TargetFollower] /trash_action received: %s", value)
        self._pending_trash_action = value

    # ──────────────────────────────────────────────────────────────────────────
    # State helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _set_state(self, new_state):
        if new_state != self._state:
            rospy.loginfo("[TargetFollower] %s -> %s", self._state, new_state)
            self._state = new_state

    def _publish_result(self, success):
        self.result_pub.publish(Bool(data=success))
        rospy.loginfo(
            "[TargetFollower] Published result: %s",
            "REACHED (True)" if success else "FAILED/LOST (False)",
        )

    def _cancel_goal_if_active(self):
        if self._goal_active:
            state = self.client.get_state()
            if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self._suppress_done_cb = True
                self.client.cancel_goal()
                rospy.loginfo("Cancelled active move_base goal")
            self._goal_active = False

    def _get_robot_pose_in_global(self):
        """Return (x, y, tf_stamped) of robot in global_frame, or None."""
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
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _reload_dynamic_params(self):
        self.standoff_distance = float(rospy.get_param("~standoff_distance", self.standoff_distance))
        self.face_target       = bool(rospy.get_param("~face_target",         self.face_target))
        self.send_rate_hz      = float(rospy.get_param("~send_rate_hz",       self.send_rate_hz))
        self.min_update_dist   = float(rospy.get_param("~min_update_dist",    self.min_update_dist))
        self.retreat_distance  = float(rospy.get_param("~retreat_distance",   self.retreat_distance))

    # ──────────────────────────────────────────────────────────────────────────
    # move_base callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _goal_done_cb(self, status, result):
        """Called by actionlib when a move_base goal reaches a terminal state."""
        self._goal_active = False

        # If we manually cancelled the goal, ignore this callback entirely
        if self._suppress_done_cb:
            self._suppress_done_cb = False
            rospy.loginfo("[TargetFollower] Suppressed done_cb after manual cancel (status=%d)", status)
            return

        # RETREATING: any terminal state means retreat is finished
        if self._state == "RETREATING":
            rospy.loginfo("[TargetFollower] Retreat navigation finished (status=%d)", status)
            self._finish_retreat()
            return

        # WAITING_ACTION / REACHED: don't let stale goal callbacks change state
        if self._state in ("REACHED", "WAITING_ACTION"):
            rospy.loginfo("[TargetFollower] Ignoring done_cb in %s (status=%d)", self._state, status)
            return

        # TRACKING: react to goal outcome
        if self._state == "TRACKING":
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("[TargetFollower] move_base SUCCEEDED — checking distance in next tick")
                # Don't set REACHED here; let maybe_send_goal distance check handle it
            elif status == GoalStatus.PREEMPTED:
                rospy.loginfo("[TargetFollower] move_base PREEMPTED (new goal will be sent)")
            elif status in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                self._set_state("FAILED")
                self._publish_result(False)

    # ──────────────────────────────────────────────────────────────────────────
    # REACHED → WAITING_ACTION
    # ──────────────────────────────────────────────────────────────────────────

    def _on_reached(self):
        """Enter REACHED, publish result=True, then immediately enter WAITING_ACTION."""
        self._cancel_goal_if_active()
        self._set_state("REACHED")
        self._publish_result(True)
        # Record current target position for retreat direction
        if self.last_target is not None:
            try:
                if self.last_target.header.frame_id != self.global_frame:
                    tfm = self.tfbuf.lookup_transform(
                        self.global_frame,
                        self.last_target.header.frame_id,
                        rospy.Time(0), rospy.Duration(0.2),
                    )
                    tg = transform_pose_stamped(self.last_target, tfm, self.global_frame)
                else:
                    tg = self.last_target
                self._retreat_target_pos = (tg.pose.position.x, tg.pose.position.y)
            except Exception as e:
                rospy.logwarn("Could not get target pos for retreat: %s", str(e))
                self._retreat_target_pos = None

        self._pending_trash_action = None
        self._waiting_action_start = rospy.Time.now()
        self._set_state("WAITING_ACTION")
        rospy.loginfo("[TargetFollower] Waiting for /trash_action (timeout=%.1fs)", self.action_wait_timeout_s)

    # ──────────────────────────────────────────────────────────────────────────
    # WAITING_ACTION processing
    # ──────────────────────────────────────────────────────────────────────────

    def _tick_waiting_action(self):
        """Called every spin loop iteration while in WAITING_ACTION."""
        # Check for incoming dialogue response
        if self._pending_trash_action is not None:
            action = self._pending_trash_action
            self._pending_trash_action = None

            if action:
                rospy.loginfo("[TargetFollower] Trash action ACCEPTED — returning to IDLE")
                self._reset_to_idle()
            else:
                rospy.loginfo("[TargetFollower] Trash action REFUSED — starting retreat")
                self._start_retreat()
            return

        # Check timeout
        elapsed = (rospy.Time.now() - self._waiting_action_start).to_sec()
        if elapsed > self.action_wait_timeout_s:
            rospy.logwarn(
                "[TargetFollower] No /trash_action received after %.1f s — returning to IDLE",
                elapsed,
            )
            self._reset_to_idle()

    # ──────────────────────────────────────────────────────────────────────────
    # Retreat logic
    # ──────────────────────────────────────────────────────────────────────────

    def _start_retreat(self):
        """Plan and execute a retreat goal: move away from the target."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            rospy.logwarn("[TargetFollower] Cannot get robot pose for retreat — resetting to IDLE")
            self._reset_to_idle()
            return

        rx, ry, tf_robot = robot_info

        # Retreat direction: robot→away_from_target
        # Use last known target position if available, otherwise retreat straight back
        if self._retreat_target_pos is not None:
            tx, ty = self._retreat_target_pos
            dx = rx - tx
            dy = ry - ty
        else:
            # Fallback: reverse robot's current heading
            q = tf_robot.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            dx = -math.cos(yaw)
            dy = -math.sin(yaw)

        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            rospy.logwarn("[TargetFollower] Target too close to robot for retreat direction — using backward")
            q = tf_robot.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            dx = -math.cos(yaw)
            dy = -math.sin(yaw)
            dist = 1.0

        # Retreat goal position
        scale = self.retreat_distance / dist
        retreat_x = rx + dx * scale
        retreat_y = ry + dy * scale

        # Retreat orientation: face away from target (keep moving direction)
        retreat_yaw = math.atan2(dy, dx)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = retreat_x
        goal.target_pose.pose.position.y = retreat_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = self._yaw_to_quaternion(retreat_yaw)

        self._set_state("RETREATING")
        self._retreat_start = rospy.Time.now()
        self.client.send_goal(goal, done_cb=self._goal_done_cb)
        self._goal_active = True
        rospy.loginfo(
            "[TargetFollower] Retreat goal: (%.2f, %.2f) yaw=%.1f° dist=%.2f m",
            retreat_x, retreat_y, math.degrees(retreat_yaw), self.retreat_distance,
        )

    def _tick_retreating(self):
        """Called every spin loop iteration while in RETREATING."""
        if self._retreat_start is None:
            return
        elapsed = (rospy.Time.now() - self._retreat_start).to_sec()
        if elapsed > self.retreat_timeout_s:
            rospy.logwarn(
                "[TargetFollower] Retreat timeout (%.1f s) — cancelling and resetting",
                elapsed,
            )
            self._cancel_goal_if_active()
            self._finish_retreat()

    def _finish_retreat(self):
        """Called after retreat goal completes or times out."""
        rospy.loginfo("[TargetFollower] Retreat complete — returning to IDLE")
        self._reset_to_idle()

    # ──────────────────────────────────────────────────────────────────────────
    # Reset
    # ──────────────────────────────────────────────────────────────────────────

    def _reset_to_idle(self):
        """Reset to IDLE. Preserves last_target so next tick can re-track immediately."""
        self._cancel_goal_if_active()
        # Keep self.last_target — so next maybe_send_goal() tick can re-track
        self.last_sent_goal    = None
        self._pending_trash_action = None
        self._waiting_action_start = None
        self._retreat_start        = None
        self._retreat_target_pos   = None
        self._target_lost_logged   = False
        self._suppress_done_cb     = False
        self._set_state("IDLE")

    # ──────────────────────────────────────────────────────────────────────────
    # Main following logic
    # ──────────────────────────────────────────────────────────────────────────

    def maybe_send_goal(self):
        """Core target-following tick — only active in IDLE/TRACKING/LOST/FAILED."""
        # Don't pursue targets while waiting for dialogue or retreating
        if self._state in ("WAITING_ACTION", "RETREATING"):
            return

        self._reload_dynamic_params()

        if self.last_target is None:
            return

        now = rospy.Time.now()
        if self.last_target.header.stamp != rospy.Time(0):
            age = (now - self.last_target.header.stamp).to_sec()
            if age > self.target_timeout_s:
                if not self._target_lost_logged:
                    rospy.logwarn(
                        "Target pose stale (age=%.2fs > %.2fs), cancelling goal",
                        age, self.target_timeout_s,
                    )
                    self._target_lost_logged = True
                self._cancel_goal_if_active()
                if self._state in ("TRACKING",):
                    self._set_state("LOST")
                    self._publish_result(False)
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

        if self.last_sent_goal is not None and self._state not in ("FAILED", "LOST", "IDLE", "REACHED"):
            if dist_xy(target_g, self.last_sent_goal) < self.min_update_dist:
                return

        # ── Standoff computation ──────────────────────────────────────────
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
            d  = math.hypot(dx, dy)

            if self.standoff_distance > 0:
                if d <= self.standoff_distance:
                    # Within standoff range — trigger REACHED → WAITING_ACTION
                    if self._state not in ("REACHED", "WAITING_ACTION"):
                        self._on_reached()
                    return
                ratio  = (d - self.standoff_distance) / d
                goal_x = rx + dx * ratio
                goal_y = ry + dy * ratio

        # ── Build MoveBaseGoal ────────────────────────────────────────────
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0

        if self.face_target:
            face_dx = tx - goal_x
            face_dy = ty - goal_y
            if math.hypot(face_dx, face_dy) > 0.01:
                yaw = math.atan2(face_dy, face_dx)
            else:
                robot_info = self._get_robot_pose_in_global()
                yaw = math.atan2(ty - robot_info[1], tx - robot_info[0]) if robot_info else 0.0
            goal.target_pose.pose.orientation = self._yaw_to_quaternion(yaw)
        elif self.use_target_orientation:
            goal.target_pose.pose.orientation = target_g.pose.orientation
        else:
            robot_info = self._get_robot_pose_in_global()
            if robot_info is not None:
                goal.target_pose.pose.orientation = robot_info[2].transform.rotation
            else:
                goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self.client.send_goal(goal, done_cb=self._goal_done_cb)
        self._goal_active  = True
        self.last_sent_goal = target_g
        self.last_send_time = now
        self._set_state("TRACKING")

    # ──────────────────────────────────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────────────────────────────────

    def spin(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Primary following (skipped in WAITING_ACTION / RETREATING)
            self.maybe_send_goal()

            # Dialogue waiting state tick
            if self._state == "WAITING_ACTION":
                self._tick_waiting_action()

            # Retreat timeout tick
            if self._state == "RETREATING":
                self._tick_retreating()

            # Publish status at ~2 Hz
            now = rospy.Time.now()
            if (now - self._last_status_time).to_sec() >= 0.5:
                self._last_status_time = now
                self.status_pub.publish(String(data=self._state))
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("target_follower")
    node = TargetFollower()
    node.spin()
