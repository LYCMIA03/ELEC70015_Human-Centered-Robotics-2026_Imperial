#!/usr/bin/env python3
"""
target_follower.py — Target following state machine with Dialogue integration.

State machine:
  IDLE
   └─ target received           → TRACKING
       └─ d <= standoff         → REACHED  (publishes result=True → triggers dialogue via bridge)
           └─ /trash_action=True  → POST_ACCEPT_COOLDOWN (pause before returning to IDLE)
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

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool, Float32, String
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
              "POST_ACCEPT_COOLDOWN", "RETREATING", "LOST", "FAILED")

    def __init__(self):
        # ── Core following params ──────────────────────────────────────────
        self.target_topic = rospy.get_param("~target_topic", "/target_pose")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame  = rospy.get_param("~robot_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.send_rate_hz      = float(rospy.get_param("~send_rate_hz", 2.0))
        self.min_update_dist   = float(rospy.get_param("~min_update_dist", 0.3))
        self.target_timeout_s  = float(rospy.get_param("~target_timeout_s", 5.0))
        self.use_target_orientation = bool(
            rospy.get_param("~use_target_orientation", False))

        # Stop short of the target by this distance (metres). 0 = go all the way.
        self.standoff_distance = float(rospy.get_param("~standoff_distance", 0.6))
        # Orient the robot to face the target at the goal pose.
        self.face_target = bool(rospy.get_param("~face_target", False))

        # ── Dialogue / trash-action integration params ─────────────────────
        # Topic published by udp_trash_action_bridge (Bool): True=accept, False=refuse
        self.trash_action_topic = rospy.get_param("~trash_action_topic", "/trash_action")
        # How long (s) to wait in WAITING_ACTION before giving up (no dialogue response)
        self.action_wait_timeout_s = float(rospy.get_param("~action_wait_timeout_s", 45.0))
        # How long (s) to ignore targets after trash_action=True (allow user to drop trash)
        self.post_accept_cooldown_s = float(rospy.get_param("~post_accept_cooldown_s", 15.0))
        # How far (m) to retreat when human refuses
        self.retreat_distance = float(rospy.get_param("~retreat_distance", 1.5))
        # Max time (s) allowed for the retreat navigation goal
        self.retreat_timeout_s = float(rospy.get_param("~retreat_timeout_s", 20.0))
        # Angular speed (rad/s) for in-place 180° turn during retreat
        self.retreat_turn_speed = float(rospy.get_param("~retreat_turn_speed", 0.5))

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
        self._post_accept_start    = None   # rospy.Time when POST_ACCEPT_COOLDOWN began
        self._retreat_start        = None   # rospy.Time when RETREATING began
        self._retreat_target_pos   = None   # (tx, ty) of last target, for retreat dir
        self._retreat_phase        = None   # "TURNING" | "DRIVING"
        self._retreat_target_yaw   = None   # target yaw after 180° turn

        # ── Publishers ─────────────────────────────────────────────────────
        # cmd_vel for direct rotation control during retreat turn phase
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

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
        # debug: planar distance from base_link to target pose in global_frame (meters)
        self.target_distance_pub = rospy.Publisher("~target_distance", Float32, queue_size=10)

        self._state = "IDLE"
        self._last_status_time = rospy.Time(0)
        self._last_param_reload_time = rospy.Time(0)
        self._param_reload_interval  = 2.0  # seconds between param server checks

        rospy.loginfo("standoff_distance = %.2f m", self.standoff_distance)
        rospy.loginfo("face_target = %s", self.face_target)
        rospy.loginfo("trash_action_topic = %s", self.trash_action_topic)
        rospy.loginfo("action_wait_timeout = %.1f s", self.action_wait_timeout_s)
        rospy.loginfo("post_accept_cooldown = %.1f s", self.post_accept_cooldown_s)
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

    def _get_camera_pose_in_global(self):
        """Return (cx, cy) of camera_link in global_frame, or None."""
        try:
            tf_cam = self.tfbuf.lookup_transform(
                self.global_frame, self.camera_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
            cx = tf_cam.transform.translation.x
            cy = tf_cam.transform.translation.y
            return cx, cy
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF not ready for camera pose: %s", str(e))
            return None

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _reload_dynamic_params(self):
        now = rospy.Time.now()
        if (now - self._last_param_reload_time).to_sec() < self._param_reload_interval:
            return
        self._last_param_reload_time = now
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
                rospy.loginfo("[TargetFollower] Trash action ACCEPTED — entering POST_ACCEPT_COOLDOWN")
                self._start_post_accept_cooldown()
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
    # POST_ACCEPT_COOLDOWN processing
    # ──────────────────────────────────────────────────────────────────────────

    def _start_post_accept_cooldown(self):
        """Freeze tracking for a short period after acceptance."""
        self._cancel_goal_if_active()
        self._stop_cmd_vel()
        # Reset latched result immediately so next REACHED publish is a clean rising edge.
        self._publish_result(False)
        self._post_accept_start = rospy.Time.now()
        self._set_state("POST_ACCEPT_COOLDOWN")

    def _tick_post_accept_cooldown(self):
        """
        During cooldown:
          1) Ignore all targets for post_accept_cooldown_s.
          2) If target stream becomes stale early, clear last_target and exit cooldown.
        """
        if self._post_accept_start is None:
            self._post_accept_start = rospy.Time.now()

        now = rospy.Time.now()

        # Early exit if target stream is stale (same criterion as LOST check).
        if self.last_target is not None and self.last_target.header.stamp != rospy.Time(0):
            age = (now - self.last_target.header.stamp).to_sec()
            if age > self.target_timeout_s:
                rospy.loginfo(
                    "[TargetFollower] POST_ACCEPT_COOLDOWN early-exit: target stale (age=%.2fs > %.2fs)",
                    age, self.target_timeout_s,
                )
                self.last_target = None
                self._reset_to_idle()
                return

        elapsed = (now - self._post_accept_start).to_sec()
        if elapsed >= self.post_accept_cooldown_s:
            rospy.loginfo(
                "[TargetFollower] POST_ACCEPT_COOLDOWN complete (%.1fs) — returning to IDLE",
                elapsed,
            )
            self._reset_to_idle()

    # ──────────────────────────────────────────────────────────────────────────
    # Retreat logic — two-phase: turn 180° in place, then drive forward
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _normalize_angle(a):
        """Normalize angle to (-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a <= -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _extract_yaw(tf_stamped):
        """Extract yaw from a TransformStamped."""
        q = tf_stamped.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _stop_cmd_vel(self):
        """Publish zero velocity to stop the robot."""
        self.cmd_vel_pub.publish(Twist())

    def _start_retreat(self):
        """Start retreat: phase 1 = turn 180° in place, phase 2 = drive forward."""
        # Safety: ensure move_base is not still sending cmd_vel
        self._cancel_goal_if_active()

        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            rospy.logwarn("[TargetFollower] Cannot get robot pose for retreat — resetting to IDLE")
            self._reset_to_idle()
            return

        rx, ry, tf_robot = robot_info
        current_yaw = self._extract_yaw(tf_robot)

        self._retreat_target_yaw = self._normalize_angle(current_yaw + math.pi)
        self._set_state("RETREATING")
        self._retreat_start = rospy.Time.now()
        self._retreat_phase = "TURNING"

        rospy.loginfo(
            "[TargetFollower] Retreat phase 1: turning 180° (%.1f° → %.1f°) at %.2f rad/s",
            math.degrees(current_yaw),
            math.degrees(self._retreat_target_yaw),
            self.retreat_turn_speed,
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
            self._stop_cmd_vel()
            self._cancel_goal_if_active()
            self._finish_retreat()
            return

        if self._retreat_phase == "TURNING":
            self._tick_retreat_turning()
        # DRIVING phase: timeout above + _goal_done_cb handle completion

    def _tick_retreat_turning(self):
        """Phase 1: publish cmd_vel rotation until ~180° turn completed."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return

        _, _, tf_robot = robot_info
        current_yaw = self._extract_yaw(tf_robot)
        yaw_error = self._normalize_angle(self._retreat_target_yaw - current_yaw)

        if abs(yaw_error) < 0.15:  # ~8.6° tolerance
            rospy.loginfo(
                "[TargetFollower] Turn complete (error=%.1f°) — starting forward drive",
                math.degrees(yaw_error),
            )
            self._stop_cmd_vel()
            self._start_retreat_drive(current_yaw)
            return

        # Publish rotation command (turn in shortest direction)
        twist = Twist()
        twist.angular.z = self.retreat_turn_speed if yaw_error > 0 else -self.retreat_turn_speed
        self.cmd_vel_pub.publish(twist)

    def _start_retreat_drive(self, current_yaw):
        """Phase 2: send MoveBaseGoal to drive forward retreat_distance."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            rospy.logwarn("[TargetFollower] Cannot get robot pose for retreat drive — finishing")
            self._finish_retreat()
            return

        rx, ry, _ = robot_info

        # Goal: drive forward in the new heading for retreat_distance
        goal_x = rx + self.retreat_distance * math.cos(current_yaw)
        goal_y = ry + self.retreat_distance * math.sin(current_yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = self._yaw_to_quaternion(current_yaw)

        self._retreat_phase = "DRIVING"
        self.client.send_goal(goal, done_cb=self._goal_done_cb)
        self._goal_active = True
        rospy.loginfo(
            "[TargetFollower] Retreat phase 2: driving to (%.2f, %.2f) yaw=%.1f° dist=%.2f m",
            goal_x, goal_y, math.degrees(current_yaw), self.retreat_distance,
        )

    def _finish_retreat(self):
        """Called after retreat goal completes or times out."""
        rospy.loginfo("[TargetFollower] Retreat complete — returning to IDLE")
        self._reset_to_idle()

    # ──────────────────────────────────────────────────────────────────────────
    # Reset
    # ──────────────────────────────────────────────────────────────────────────

    def _reset_to_idle(self):
        """Reset to IDLE. Preserves last_target so next tick can re-track immediately.

        Publishes result=False to reset the latched /result topic.
        This is critical: navigation_success_udp_bridge uses rising-edge
        detection (False→True) to trigger dialogue.  Without this reset the
        latch stays True and subsequent REACHED events won't fire the bridge.
        """
        self._cancel_goal_if_active()
        self._stop_cmd_vel()  # stop any in-progress rotation
        # Reset the latched result so the next REACHED→True is a rising edge
        self._publish_result(False)
        # Keep self.last_target — so next maybe_send_goal() tick can re-track
        self.last_sent_goal    = None
        self._pending_trash_action = None
        self._waiting_action_start = None
        self._post_accept_start    = None
        self._retreat_start        = None
        self._retreat_target_pos   = None
        self._retreat_phase        = None
        self._retreat_target_yaw   = None
        self._target_lost_logged   = False
        self._suppress_done_cb     = False
        self._set_state("IDLE")

    # ──────────────────────────────────────────────────────────────────────────
    # Main following logic
    # ──────────────────────────────────────────────────────────────────────────

    def maybe_send_goal(self):
        """Core target-following tick — only active in IDLE/TRACKING/LOST/FAILED."""
        # Don't pursue targets while waiting for dialogue or retreating
        if self._state in ("WAITING_ACTION", "POST_ACCEPT_COOLDOWN", "RETREATING"):
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

        # Rate limiter — but allow immediate re-send after FAILED/LOST/IDLE
        if self._state == "TRACKING":
            if (now - self.last_send_time).to_sec() < (1.0 / max(self.send_rate_hz, 0.1)):
                return

        # Skip if target hasn't moved enough (only while actively tracking)
        if self.last_sent_goal is not None and self._state == "TRACKING":
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

            # Distance from base_link to target (for goal waypoint computation)
            dx = tx - rx
            dy = ty - ry
            d  = math.hypot(dx, dy)

            # Distance from camera_link to target (for REACHED check)
            # Camera is mounted ahead of base_link (e.g. +0.208 m in X),
            # so camera distance is always shorter than base distance.
            cam_pos = self._get_camera_pose_in_global()
            if cam_pos is not None:
                cx, cy = cam_pos
                d_cam = math.hypot(tx - cx, ty - cy)
            else:
                d_cam = d  # fallback: use base distance

            self.target_distance_pub.publish(Float32(data=d_cam))
            rospy.loginfo_throttle(
                2.0,
                "[TargetFollower] d_base=%.3f  d_cam=%.3f  standoff=%.2f",
                d, d_cam, self.standoff_distance,
            )

            if self.standoff_distance > 0:
                # REACHED check uses camera→target distance
                reach_threshold = self.standoff_distance
                if d_cam <= reach_threshold:
                    if self._state not in ("REACHED", "WAITING_ACTION"):
                        rospy.loginfo(
                            "[TargetFollower] Within standoff (d_cam=%.2f <= %.2f) — REACHED",
                            d_cam, reach_threshold,
                        )
                        self._on_reached()
                    return
                # Goal waypoint: offset from base_link so that *camera* ends
                # up at standoff.  Approximate: camera is roughly
                # (d - d_cam) metres closer to target along the
                # robot→target line, so the base should stop at
                # standoff + (d - d_cam) from target.
                cam_ahead = d - d_cam  # positive when camera is ahead
                base_standoff = self.standoff_distance + cam_ahead
                if base_standoff >= d:
                    base_standoff = d * 0.9  # safety: don't send goal behind robot
                ratio  = (d - base_standoff) / d
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
        if self._state in ("FAILED", "LOST", "IDLE"):
            rospy.loginfo(
                "[TargetFollower] Resuming from %s → TRACKING (goal dist=%.2f m)",
                self._state, math.hypot(goal_x - rx, goal_y - ry) if 'rx' in dir() else -1,
            )
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

            # Post-accept cooldown tick
            if self._state == "POST_ACCEPT_COOLDOWN":
                self._tick_post_accept_cooldown()

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
