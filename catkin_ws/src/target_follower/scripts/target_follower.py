#!/usr/bin/env python3
"""
target_follower.py — Target following state machine with Dialogue integration.

State machine:
  IDLE
   └─ target received             → TRACKING
       └─ d <= standoff           → REACHED  (publishes result=True → triggers dialogue via bridge)
           └─ /trash_action=True  → POST_ACCEPT_COOLDOWN → RETREATING
           └─ /trash_action=False → RETREATING
           └─ timeout             → IDLE
  LOST / FAILED                   → IDLE on next target

Dialogue bridge flow (external, started by start_dialogue_docker_bridges.sh):
  /target_follower/result=True
    → navigation_success_udp_bridge → UDP:16041 → dialogue_udp_runner
    → UDP:16032 → udp_trash_action_bridge → /trash_action (Bool)
"""
import math
import random
from collections import deque
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path
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
    STATES = ("IDLE", "EXPLORING", "TRACKING", "CLOSE_APPROACH", "REACHED", "WAITING_ACTION",
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
        # In pure-follow experiments we disable dialogue waiting at REACHED so
        # the robot keeps tracking a moving target continuously.
        self.enable_interaction_mode = bool(
            rospy.get_param("~enable_interaction_mode", True))

        # ── Dialogue / trash-action integration params ─────────────────────
        # Topic published by udp_trash_action_bridge (Bool): True=accept, False=refuse
        self.trash_action_topic = rospy.get_param("~trash_action_topic", "/trash_action")
        # How long (s) to wait in WAITING_ACTION before giving up (no dialogue response)
        self.action_wait_timeout_s = float(rospy.get_param("~action_wait_timeout_s", 45.0))
        # How long (s) to wait after trash_action=True so the user can finish dropping trash.
        self.post_accept_cooldown_s = float(rospy.get_param("~post_accept_cooldown_s", 15.0))
        # How far (m) to retreat when human refuses
        self.retreat_distance = float(rospy.get_param("~retreat_distance", 1.5))
        # Max time (s) allowed for the retreat navigation goal
        self.retreat_timeout_s = float(rospy.get_param("~retreat_timeout_s", 20.0))
        # Angular speed (rad/s) for the retreat turn phase
        self.retreat_turn_speed = float(rospy.get_param("~retreat_turn_speed", 0.5))
        # Use a moderate retreat turn so the robot leaves the interaction area
        # without over-rotating into a nearby obstacle pocket.
        self.retreat_turn_angle_deg = float(rospy.get_param("~retreat_turn_angle_deg", 100.0))
        self.retreat_turn_tolerance_deg = float(rospy.get_param("~retreat_turn_tolerance_deg", 10.0))

        # ── Auto explore params (active when no target is being tracked) ─────
        self.enable_auto_explore = bool(rospy.get_param("~enable_auto_explore", True))
        self.explore_goal_distance = float(rospy.get_param("~explore_goal_distance", 2.4))
        self.explore_goal_timeout_s = float(rospy.get_param("~explore_goal_timeout_s", 30.0))
        self.explore_goal_min_dist = float(rospy.get_param("~explore_goal_min_dist", 0.8))
        # Require a short stable target stream before exploration is interrupted.
        # This prevents single-frame detections from constantly cancelling explore.
        self.explore_target_confirm_s = float(rospy.get_param("~explore_target_confirm_s", 0.8))
        # Also require target updates to keep arriving; a single stale detection
        # should not keep exploration suppressed for several seconds.
        self.explore_target_max_gap_s = float(rospy.get_param("~explore_target_max_gap_s", 1.0))
        # Replan sooner when an explore goal stays active but the robot makes
        # almost no odometry progress, which often happens when DWA is blocked.
        self.explore_stuck_timeout_s = float(rospy.get_param("~explore_stuck_timeout_s", 8.0))
        self.explore_min_progress_dist = float(rospy.get_param("~explore_min_progress_dist", 0.2))
        # If move_base keeps publishing near-zero cmd_vel during exploration,
        # treat it as a blocked local planner and replan earlier.
        self.explore_zero_cmd_vel_timeout_s = float(
            rospy.get_param("~explore_zero_cmd_vel_timeout_s", 2.0))
        self.explore_cmd_vel_linear_epsilon = float(
            rospy.get_param("~explore_cmd_vel_linear_epsilon", 0.02))
        self.explore_cmd_vel_angular_epsilon = float(
            rospy.get_param("~explore_cmd_vel_angular_epsilon", 0.05))
        self.explore_revisit_window_s = float(rospy.get_param("~explore_revisit_window_s", 120.0))
        self.explore_revisit_radius = float(rospy.get_param("~explore_revisit_radius", 1.5))
        self.target_reacquire_block_s = float(rospy.get_param("~target_reacquire_block_s", 120.0))
        self.target_reacquire_radius = float(rospy.get_param("~target_reacquire_radius", 1.6))
        self.path_sample_min_dist = float(rospy.get_param("~path_sample_min_dist", 0.25))
        self.max_path_points = int(rospy.get_param("~max_path_points", 2000))

        # ── Close-approach params (bypass move_base for last-metre approach) ──
        # When d_cam_z < this threshold, switch from move_base to direct cmd_vel.
        # Must be > standoff_distance.  Default: 2× standoff.
        self.close_approach_threshold = float(
            rospy.get_param("~close_approach_threshold", 1.5))
        # Forward speed during close approach (m/s). Keep low for safety.
        self.close_approach_speed = float(
            rospy.get_param("~close_approach_speed", 0.10))
        # Max time (s) for close approach before giving up
        self.close_approach_timeout_s = float(
            rospy.get_param("~close_approach_timeout_s", 15.0))
        # Steering gain for close approach: angular.z = -gain * camera_x
        # Camera optical convention: x = right, so negative gain steers left when target is right.
        self.close_approach_steer_gain = float(
            rospy.get_param("~close_approach_steer_gain", 0.6))
        # Max depth jump (m) allowed between consecutive readings in CLOSE_APPROACH.
        # If d_cam_z increases by more than this in one tick, the reading is
        # treated as a detection glitch and the previous value is kept.
        self.close_approach_max_depth_jump = float(
            rospy.get_param("~close_approach_max_depth_jump", 0.5))

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
        self._active_goal_kind = None  # TRACKING | EXPLORING | RETREATING
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
        self._retreat_reason       = ""
        self._close_approach_start = None   # rospy.Time when CLOSE_APPROACH began
        self._ca_last_good_depth   = None   # last accepted d_cam_z during CLOSE_APPROACH (depth filter)
        self._explore_goal_start   = None
        self._explore_start_xy     = None
        self._explore_zero_cmd_start = None
        self._next_explore_time    = rospy.Time(0)
        self._last_explore_goal_xy = None
        self._target_seen_since    = None
        self._last_target_rx_time  = None
        self._last_cmd_vel_time    = None

        # Path memory and dialogue-location memory for anti-repeat behaviour.
        self._path_history = deque(maxlen=max(self.max_path_points, 100))  # (stamp, x, y)
        self._dialogue_points = deque(maxlen=200)  # (stamp, x, y, reason)
        self._last_path_pub_time = rospy.Time(0)

        # ── Publishers ─────────────────────────────────────────────────────
        # cmd_vel for direct rotation control during retreat turn phase
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # ── Subscribers ────────────────────────────────────────────────────
        rospy.Subscriber(self.target_topic, PoseStamped,
                         self.cb_target, queue_size=1)
        rospy.Subscriber(self.trash_action_topic, Bool,
                         self.cb_trash_action, queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist,
                         self.cb_cmd_vel, queue_size=20)

        # ── Publishers ─────────────────────────────────────────────────────
        # result: True=REACHED (dialogue trigger), False=FAILED/LOST
        self.result_pub = rospy.Publisher("~result",  Bool,   queue_size=1, latch=True)
        # status: IDLE|TRACKING|REACHED|WAITING_ACTION|RETREATING|LOST|FAILED
        self.status_pub = rospy.Publisher("~status",  String, queue_size=1)
        # debug: planar distance from base_link to target pose in global_frame (meters)
        self.target_distance_pub = rospy.Publisher("~target_distance", Float32, queue_size=10)
        # debug: remembered path and dialogue points for runtime inspection
        self.path_history_pub = rospy.Publisher("~path_history", Path, queue_size=1)
        self.dialogue_points_pub = rospy.Publisher("~dialogue_points", Path, queue_size=1)

        self._state = "IDLE"
        self._last_status_time = rospy.Time(0)
        self._last_param_reload_time = rospy.Time(0)
        self._param_reload_interval  = 2.0  # seconds between param server checks

        rospy.loginfo("standoff_distance = %.2f m", self.standoff_distance)
        rospy.loginfo("close_approach_threshold = %.2f m", self.close_approach_threshold)
        rospy.loginfo("close_approach_speed = %.2f m/s", self.close_approach_speed)
        rospy.loginfo("close_approach_steer_gain = %.2f", self.close_approach_steer_gain)
        rospy.loginfo("close_approach_max_depth_jump = %.2f m", self.close_approach_max_depth_jump)
        rospy.loginfo("face_target = %s", self.face_target)
        rospy.loginfo("trash_action_topic = %s", self.trash_action_topic)
        rospy.loginfo("action_wait_timeout = %.1f s", self.action_wait_timeout_s)
        rospy.loginfo("post_accept_cooldown = %.1f s", self.post_accept_cooldown_s)
        rospy.loginfo("retreat_distance = %.2f m", self.retreat_distance)
        rospy.loginfo("retreat_turn_angle = %.1f deg", self.retreat_turn_angle_deg)
        rospy.loginfo("enable_auto_explore = %s", self.enable_auto_explore)
        rospy.loginfo("explore_target_confirm = %.1f s", self.explore_target_confirm_s)
        rospy.loginfo("explore_target_max_gap = %.1f s", self.explore_target_max_gap_s)
        rospy.loginfo("explore_stuck_timeout = %.1f s", self.explore_stuck_timeout_s)
        rospy.loginfo("explore_min_progress = %.2f m", self.explore_min_progress_dist)
        rospy.loginfo("explore_zero_cmd_vel_timeout = %.1f s", self.explore_zero_cmd_vel_timeout_s)
        rospy.loginfo("explore_revisit_window = %.1f s", self.explore_revisit_window_s)
        rospy.loginfo("explore_revisit_radius = %.2f m", self.explore_revisit_radius)
        rospy.loginfo("Result topic: %s/result (std_msgs/Bool)", rospy.get_name())
        rospy.loginfo("Status topic: %s/status (std_msgs/String)", rospy.get_name())

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def cb_target(self, msg):
        """Incoming target pose — always update; state machine decides when to act."""
        now = rospy.Time.now()
        if self._last_target_rx_time is None or (now - self._last_target_rx_time).to_sec() > self.target_timeout_s:
            self._target_seen_since = now
        self._last_target_rx_time = now
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

    def cb_cmd_vel(self, msg):
        """Track recent cmd_vel output so exploration can detect blocked motion earlier."""
        self._last_cmd_vel_time = rospy.Time.now()
        if not (self._state == "EXPLORING" and self._goal_active and self._active_goal_kind == "EXPLORING"):
            self._explore_zero_cmd_start = None
            return

        linear_mag = math.hypot(msg.linear.x, msg.linear.y)
        angular_mag = abs(msg.angular.z)
        is_near_zero = (
            linear_mag <= self.explore_cmd_vel_linear_epsilon
            and angular_mag <= self.explore_cmd_vel_angular_epsilon
        )

        if is_near_zero:
            if self._explore_zero_cmd_start is None:
                self._explore_zero_cmd_start = rospy.Time.now()
        else:
            self._explore_zero_cmd_start = None

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
            was_exploring = self._active_goal_kind == "EXPLORING"
            state = self.client.get_state()
            if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self._suppress_done_cb = True
                self.client.cancel_goal()
                rospy.loginfo("Cancelled active move_base goal")
            self._goal_active = False
            self._active_goal_kind = None
            if was_exploring:
                self._explore_goal_start = None
                self._explore_start_xy = None
                self._explore_zero_cmd_start = None

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

    def _get_target_pose_in_frame(self, target_frame):
        """Return last_target transformed to target_frame, or None."""
        if self.last_target is None:
            return None
        if self.last_target.header.frame_id == target_frame:
            return self.last_target
        try:
            tfm = self.tfbuf.lookup_transform(
                target_frame,
                self.last_target.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            return transform_pose_stamped(self.last_target, tfm, target_frame)
        except Exception as e:
            rospy.logwarn_throttle(
                2.0, "TF error transforming target to %s: %s", target_frame, str(e))
            return None

    def _get_target_depth_in_camera(self):
        """Return robust forward distance of target in camera frame.

        The detection pipeline (handobj_detection_rgbd / predict_15cls_rgbd) outputs
        coordinates in the camera optical convention:
            z = forward (depth),  x = right,  y = down
        and labels the frame as camera_link.

        In Gazebo target-follow tests, target poses can come from non-optical
        frames (e.g. base_footprint) and transformed z may become non-positive,
        which causes false "reached" triggers. We therefore:
          1) prefer optical depth z when z>0;
          2) fall back to planar camera distance hypot(x,y) otherwise.
        """
        target_cam = self._get_target_pose_in_frame(self.camera_frame)
        if target_cam is None:
            return None
        x = float(target_cam.pose.position.x)
        y = float(target_cam.pose.position.y)
        z = float(target_cam.pose.position.z)

        if z > 0.05:
            return z

        planar = math.hypot(x, y)
        if planar > 1e-3:
            rospy.logwarn_throttle(
                5.0,
                "[TargetFollower] Non-positive camera depth z=%.3f in frame=%s; "
                "fallback to planar distance hypot(x,y)=%.3f",
                z, self.camera_frame, planar,
            )
            return planar
        return abs(z)

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

    def _append_path_point_if_needed(self):
        """Record robot path sparsely for revisit suppression and debugging."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return
        rx, ry, _ = robot_info
        now = rospy.Time.now()

        if not self._path_history:
            self._path_history.append((now, rx, ry))
            return

        _, lx, ly = self._path_history[-1]
        if math.hypot(rx - lx, ry - ly) >= self.path_sample_min_dist:
            self._path_history.append((now, rx, ry))

    def _publish_memory_paths(self):
        """Publish path history and dialogue points as nav_msgs/Path topics."""
        now = rospy.Time.now()
        if (now - self._last_path_pub_time).to_sec() < 0.5:
            return
        self._last_path_pub_time = now

        hist_msg = Path()
        hist_msg.header.stamp = now
        hist_msg.header.frame_id = self.global_frame
        for stamp, x, y in self._path_history:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.global_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            hist_msg.poses.append(ps)
        self.path_history_pub.publish(hist_msg)

        dlg_msg = Path()
        dlg_msg.header.stamp = now
        dlg_msg.header.frame_id = self.global_frame
        for stamp, x, y, _ in self._dialogue_points:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.global_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            dlg_msg.poses.append(ps)
        self.dialogue_points_pub.publish(dlg_msg)

    def _mark_dialogue_location(self, reason):
        """Save where dialogue happened, used to avoid repeatedly asking the same person."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return
        rx, ry, _ = robot_info
        self._dialogue_points.append((rospy.Time.now(), rx, ry, reason))
        rospy.loginfo("[TargetFollower] Dialogue point recorded at (%.2f, %.2f), reason=%s", rx, ry, reason)

    def _is_recent_path_area(self, wx, wy, window_s=None, radius_m=None):
        """Check if (wx, wy) falls in recently visited area based on path history."""
        now = rospy.Time.now()
        if window_s is None:
            window_s = self.explore_revisit_window_s
        if radius_m is None:
            radius_m = self.explore_revisit_radius
        # In simulation startup, /clock can still be near zero. Guard against
        # negative rospy.Time results when subtracting long windows.
        cutoff = rospy.Time(0)
        if now.to_sec() > window_s:
            cutoff = now - rospy.Duration(window_s)

        for stamp, px, py in reversed(self._path_history):
            if stamp < cutoff:
                break
            if math.hypot(wx - px, wy - py) <= radius_m:
                return True
        return False

    def _is_recent_dialogue_area(self, wx, wy, window_s=None, radius_m=None):
        """Check if (wx, wy) is near a recent dialogue location."""
        now = rospy.Time.now()
        if window_s is None:
            window_s = self.target_reacquire_block_s
        if radius_m is None:
            radius_m = self.target_reacquire_radius
        cutoff = rospy.Time(0)
        if now.to_sec() > window_s:
            cutoff = now - rospy.Duration(window_s)

        for stamp, px, py, _ in reversed(self._dialogue_points):
            if stamp < cutoff:
                break
            if math.hypot(wx - px, wy - py) <= radius_m:
                return True
        return False

    def _target_fresh(self):
        if self.last_target is None:
            self._target_seen_since = None
            return False
        if self.last_target.header.stamp == rospy.Time(0):
            return True
        age = (rospy.Time.now() - self.last_target.header.stamp).to_sec()
        if age > self.target_timeout_s:
            self._target_seen_since = None
        return age <= self.target_timeout_s

    def _target_confirmed_for_explore_interrupt(self):
        """Return True only after a fresh target has persisted briefly."""
        if not self._target_fresh():
            return False
        if self._last_target_rx_time is None:
            return False
        if (rospy.Time.now() - self._last_target_rx_time).to_sec() > self.explore_target_max_gap_s:
            self._target_seen_since = None
            return False
        if self._target_seen_since is None:
            self._target_seen_since = rospy.Time.now()
            return False
        return (rospy.Time.now() - self._target_seen_since).to_sec() >= self.explore_target_confirm_s

    def _pick_explore_goal(self):
        """Generate an exploration goal in odom/map, while avoiding recently visited areas."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return None
        rx, ry, tf_robot = robot_info
        yaw = self._extract_yaw(tf_robot)

        # Keep a forward-biased but still flexible heading set. This is more
        # conservative than a full rearward sweep, which helps in cluttered
        # indoor spaces where aggressive turns can trap the base.
        offsets = [0.0, 0.6, -0.6, 1.1, -1.1, 1.7, -1.7]
        random.shuffle(offsets)
        best = None
        best_score = -1e9

        for off in offsets:
            dist = self.explore_goal_distance * (0.85 + 0.3 * random.random())
            heading = yaw + off
            gx = rx + dist * math.cos(heading)
            gy = ry + dist * math.sin(heading)
            move_dist = math.hypot(gx - rx, gy - ry)
            if move_dist < self.explore_goal_min_dist:
                continue

            if self._is_recent_path_area(gx, gy):
                continue
            if self._is_recent_dialogue_area(gx, gy):
                continue

            # Keep distance as the dominant term, but only mildly reward wider
            # turns so exploration stays a bit more conservative.
            score = move_dist + 0.05 * abs(off)
            if score > best_score:
                best_score = score
                best = (gx, gy, heading)

        # Fallback if all candidates were recently visited: still rotate enough
        # to escape a local pocket, but avoid a full rearward flip.
        if best is None:
            off = 2.0 if random.random() < 0.5 else -2.0
            heading = yaw + off
            gx = rx + self.explore_goal_distance * math.cos(heading)
            gy = ry + self.explore_goal_distance * math.sin(heading)
            best = (gx, gy, heading)

        return best

    def _start_explore_goal(self):
        if not self.enable_auto_explore:
            return
        if self._goal_active:
            return

        picked = self._pick_explore_goal()
        if picked is None:
            return
        gx, gy, heading = picked

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = self._yaw_to_quaternion(heading)

        self.client.send_goal(goal, done_cb=self._goal_done_cb)
        self._goal_active = True
        self._active_goal_kind = "EXPLORING"
        self._explore_goal_start = rospy.Time.now()
        robot_info = self._get_robot_pose_in_global()
        self._explore_start_xy = (robot_info[0], robot_info[1]) if robot_info is not None else None
        self._explore_zero_cmd_start = None
        self._last_explore_goal_xy = (gx, gy)
        self._set_state("EXPLORING")
        rospy.loginfo("[TargetFollower] Exploring toward (%.2f, %.2f)", gx, gy)

    def _tick_exploring(self):
        """Keep robot moving when no valid target is available."""
        if not self.enable_auto_explore:
            return
        if self._state in ("WAITING_ACTION", "RETREATING", "CLOSE_APPROACH", "POST_ACCEPT_COOLDOWN"):
            return

        # Target appears: stop exploration and let tracking take over.
        if self._target_confirmed_for_explore_interrupt():
            if self._active_goal_kind == "EXPLORING" and self._goal_active:
                rospy.loginfo(
                    "[TargetFollower] Stable target detected for %.1fs — interrupting exploration",
                    self.explore_target_confirm_s,
                )
                self._cancel_goal_if_active()
            if self._state == "EXPLORING":
                self._set_state("IDLE")
            return

        now = rospy.Time.now()
        if now < self._next_explore_time:
            return

        if self._active_goal_kind == "EXPLORING" and self._goal_active and self._explore_goal_start is not None:
            elapsed = (now - self._explore_goal_start).to_sec()
            if self._explore_zero_cmd_start is not None and self._last_cmd_vel_time is not None:
                cmd_age = (now - self._last_cmd_vel_time).to_sec()
                zero_elapsed = (now - self._explore_zero_cmd_start).to_sec()
                # When the local planner keeps outputting near-zero velocities
                # in front of an obstacle, replan sooner instead of waiting for
                # the longer progress timeout.
                if cmd_age <= 1.0 and zero_elapsed >= self.explore_zero_cmd_vel_timeout_s:
                    rospy.logwarn(
                        "[TargetFollower] Explore cmd_vel stayed near zero for %.1fs — cancel and replan",
                        zero_elapsed,
                    )
                    self._cancel_goal_if_active()
                    self._active_goal_kind = None
                    self._set_state("IDLE")
                    self._next_explore_time = now + rospy.Duration(0.5)
                    return
            if elapsed > self.explore_stuck_timeout_s and self._explore_start_xy is not None:
                robot_info = self._get_robot_pose_in_global()
                if robot_info is not None:
                    progress = math.hypot(
                        robot_info[0] - self._explore_start_xy[0],
                        robot_info[1] - self._explore_start_xy[1],
                    )
                    if progress < self.explore_min_progress_dist:
                        rospy.logwarn(
                            "[TargetFollower] Explore goal made only %.2fm progress in %.1fs — cancel and replan",
                            progress, elapsed,
                        )
                        self._cancel_goal_if_active()
                        self._active_goal_kind = None
                        self._set_state("IDLE")
                        self._next_explore_time = now + rospy.Duration(0.5)
                        return
            if elapsed > self.explore_goal_timeout_s:
                rospy.logwarn("[TargetFollower] Explore goal timeout %.1fs, cancel and replan", elapsed)
                self._cancel_goal_if_active()
                self._active_goal_kind = None
                self._set_state("IDLE")
                self._next_explore_time = now + rospy.Duration(1.0)
            return

        self._start_explore_goal()
        self._next_explore_time = now + rospy.Duration(0.8)

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

        # EXPLORING: continue exploration loop unless interrupted by target tracking.
        if self._state == "EXPLORING" or self._active_goal_kind == "EXPLORING":
            self._active_goal_kind = None
            self._explore_goal_start = None
            self._explore_start_xy = None
            self._explore_zero_cmd_start = None
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("[TargetFollower] Explore goal reached")
            else:
                rospy.logwarn("[TargetFollower] Explore goal ended with status=%d", status)
            self._set_state("IDLE")
            self._next_explore_time = rospy.Time.now() + rospy.Duration(0.5)
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
        self._mark_dialogue_location("reached")
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
                rospy.loginfo(
                    "[TargetFollower] Trash action ACCEPTED — starting cooldown to ensure trash drop is complete"
                )
                self._start_post_accept_cooldown()
            else:
                rospy.loginfo("[TargetFollower] Trash action REFUSED — retreating immediately")
                self._start_retreat(reason="refused")
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
        """Pause after acceptance so the user can finish dropping trash before retreating."""
        self._cancel_goal_if_active()
        self._stop_cmd_vel()
        # Reset latched result immediately so next REACHED publish is a clean rising edge.
        self._publish_result(False)
        self._post_accept_start = rospy.Time.now()
        self._set_state("POST_ACCEPT_COOLDOWN")

    def _tick_post_accept_cooldown(self):
        """
        During cooldown:
          1) Ignore target tracking for post_accept_cooldown_s.
          2) If the current target stream goes stale, clear it and continue to retreat.
        """
        if self._post_accept_start is None:
            self._post_accept_start = rospy.Time.now()

        now = rospy.Time.now()

        # Early exit if target stream is stale (same criterion as LOST check).
        if self.last_target is not None and self.last_target.header.stamp != rospy.Time(0):
            age = (now - self.last_target.header.stamp).to_sec()
            if age > self.target_timeout_s:
                rospy.loginfo(
                    "[TargetFollower] POST_ACCEPT_COOLDOWN target stale (age=%.2fs > %.2fs) — continue to retreat",
                    age, self.target_timeout_s,
                )
                self.last_target = None
                self._start_retreat(reason="accepted_after_cooldown")
                return

        elapsed = (now - self._post_accept_start).to_sec()
        if elapsed >= self.post_accept_cooldown_s:
            rospy.loginfo(
                "[TargetFollower] POST_ACCEPT_COOLDOWN complete (%.1fs) — starting retreat",
                elapsed,
            )
            self._start_retreat(reason="accepted_after_cooldown")

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

    def _start_retreat(self, reason="unknown"):
        """Start retreat: phase 1 = large in-place turn, phase 2 = drive forward."""
        # Safety: ensure move_base is not still sending cmd_vel
        self._cancel_goal_if_active()

        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            rospy.logwarn("[TargetFollower] Cannot get robot pose for retreat — resetting to IDLE")
            self._reset_to_idle()
            return

        rx, ry, tf_robot = robot_info
        current_yaw = self._extract_yaw(tf_robot)
        if self._dialogue_points:
            _, lx, ly, _ = self._dialogue_points[-1]
            if math.hypot(rx - lx, ry - ly) > 0.2:
                self._mark_dialogue_location(reason)
        else:
            self._mark_dialogue_location(reason)

        turn_rad = math.radians(self.retreat_turn_angle_deg)
        self._retreat_target_yaw = self._normalize_angle(current_yaw + turn_rad)
        self._set_state("RETREATING")
        self._retreat_start = rospy.Time.now()
        self._retreat_phase = "TURNING"
        self._retreat_reason = reason
        self._active_goal_kind = "RETREATING"

        rospy.loginfo(
            "[TargetFollower] Retreat phase 1 (%s): turning %.1f° (%.1f° → %.1f°) at %.2f rad/s",
            reason,
            self.retreat_turn_angle_deg,
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

        if abs(yaw_error) < math.radians(self.retreat_turn_tolerance_deg):
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
        self._active_goal_kind = "RETREATING"
        rospy.loginfo(
            "[TargetFollower] Retreat phase 2: driving to (%.2f, %.2f) yaw=%.1f° dist=%.2f m",
            goal_x, goal_y, math.degrees(current_yaw), self.retreat_distance,
        )

    def _finish_retreat(self):
        """Called after retreat goal completes or times out."""
        rospy.loginfo("[TargetFollower] Retreat complete (%s) — returning to IDLE and resuming explore", self._retreat_reason)
        self._active_goal_kind = None
        self._next_explore_time = rospy.Time.now() + rospy.Duration(0.3)
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
        self._active_goal_kind = None
        self._pending_trash_action = None
        self._waiting_action_start = None
        self._post_accept_start    = None
        self._retreat_start        = None
        self._retreat_target_pos   = None
        self._retreat_phase        = None
        self._retreat_target_yaw   = None
        self._close_approach_start = None
        self._ca_last_good_depth   = None
        self._explore_goal_start   = None
        self._explore_start_xy     = None
        self._explore_zero_cmd_start = None
        self._target_lost_logged   = False
        self._suppress_done_cb     = False
        self._set_state("IDLE")

    # ──────────────────────────────────────────────────────────────────────────
    # Main following logic
    # ──────────────────────────────────────────────────────────────────────────

    # ──────────────────────────────────────────────────────────────────────────
    # CLOSE_APPROACH — direct cmd_vel drive when move_base can't plan (target=obstacle)
    # ──────────────────────────────────────────────────────────────────────────

    def _start_close_approach(self):
        """Cancel move_base and drive toward target with cmd_vel + steering."""
        self._cancel_goal_if_active()
        self._close_approach_start = rospy.Time.now()
        # Initialise depth filter with current reading
        d_cam_z = self._get_target_depth_in_camera()
        self._ca_last_good_depth = d_cam_z
        self._set_state("CLOSE_APPROACH")
        rospy.loginfo(
            "[TargetFollower] CLOSE_APPROACH started — driving at %.2f m/s toward target (steer_gain=%.2f)",
            self.close_approach_speed, self.close_approach_steer_gain,
        )

    def _tick_close_approach(self):
        """Drive toward target with depth filtering + proportional steering.

        Depth filter:
            Detection can jump when the tracker switches body parts or hits
            background surfaces.  If d_cam_z *increases* by more than
            ``close_approach_max_depth_jump`` compared with the last accepted
            reading, the new reading is ignored and the previous value is kept.

        Steering:
            The camera optical x-coordinate indicates how far the target is
            to the right (positive x) or left (negative x).  A proportional
            angular velocity is applied:  angular.z = -steer_gain * cam_x
            so the robot turns toward the target instead of driving blind.
        """
        if self._close_approach_start is None:
            self._close_approach_start = rospy.Time.now()

        # Timeout check
        elapsed = (rospy.Time.now() - self._close_approach_start).to_sec()
        if elapsed > self.close_approach_timeout_s:
            rospy.logwarn(
                "[TargetFollower] CLOSE_APPROACH timeout (%.1fs) — returning to IDLE",
                elapsed,
            )
            self._stop_cmd_vel()
            self._reset_to_idle()
            return

        # Check if target is still fresh
        if self.last_target is None:
            self._stop_cmd_vel()
            self._reset_to_idle()
            return
        now = rospy.Time.now()
        if self.last_target.header.stamp != rospy.Time(0):
            age = (now - self.last_target.header.stamp).to_sec()
            if age > self.target_timeout_s:
                rospy.logwarn(
                    "[TargetFollower] CLOSE_APPROACH target stale (%.2fs) — stopping",
                    age,
                )
                self._stop_cmd_vel()
                self._set_state("LOST")
                self._publish_result(False)
                return

        # ── Read depth with jump filter ──────────────────────────────────
        raw_d_cam_z = self._get_target_depth_in_camera()
        d_cam_z = raw_d_cam_z

        if d_cam_z is not None and self._ca_last_good_depth is not None:
            jump = d_cam_z - self._ca_last_good_depth
            if jump > self.close_approach_max_depth_jump:
                rospy.logwarn(
                    "[TargetFollower] CLOSE_APPROACH depth jump filtered: "
                    "raw=%.3f  last_good=%.3f  jump=+%.3f > %.2f — keeping last value",
                    d_cam_z, self._ca_last_good_depth, jump,
                    self.close_approach_max_depth_jump,
                )
                d_cam_z = self._ca_last_good_depth
            else:
                self._ca_last_good_depth = d_cam_z
        elif d_cam_z is not None:
            # First reading — accept it
            self._ca_last_good_depth = d_cam_z

        # ── REACHED check ────────────────────────────────────────────────
        if d_cam_z is not None:
            rospy.loginfo_throttle(
                1.0,
                "[TargetFollower] CLOSE_APPROACH d_cam_z=%.3f (raw=%.3f)  standoff=%.2f",
                d_cam_z,
                raw_d_cam_z if raw_d_cam_z is not None else -1.0,
                self.standoff_distance,
            )
            if d_cam_z <= self.standoff_distance:
                rospy.loginfo(
                    "[TargetFollower] CLOSE_APPROACH reached standoff "
                    "(d_cam_z=%.3f <= %.2f) — REACHED",
                    d_cam_z, self.standoff_distance,
                )
                self._stop_cmd_vel()
                if self.enable_interaction_mode:
                    self._on_reached()
                else:
                    self._set_state("TRACKING")
                return

        # ── Steering: proportional controller on lateral offset ───────────
        # Optical frame: x is right.
        # Non-optical frame (robot-like): y is left, so convert to "right-positive".
        lateral_right = 0.0
        target_cam = self._get_target_pose_in_frame(self.camera_frame)
        if target_cam is not None:
            tx = float(target_cam.pose.position.x)
            ty = float(target_cam.pose.position.y)
            tz = float(target_cam.pose.position.z)
            if tz > 0.05:
                lateral_right = tx
            else:
                lateral_right = -ty

        twist = Twist()
        twist.linear.x = self.close_approach_speed
        twist.angular.z = -self.close_approach_steer_gain * lateral_right
        # Clamp angular velocity to avoid spinning
        max_ang = 0.35  # rad/s, same as DWA max_vel_theta
        twist.angular.z = max(-max_ang, min(max_ang, twist.angular.z))
        self.cmd_vel_pub.publish(twist)

    def maybe_send_goal(self):
        """Core target-following tick — only active in IDLE/TRACKING/LOST/FAILED."""
        # Don't pursue targets while waiting for dialogue, retreating, or close-approaching
        if self._state in ("WAITING_ACTION", "POST_ACCEPT_COOLDOWN", "RETREATING", "CLOSE_APPROACH"):
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
                        "Target pose stale (age=%.2fs > %.2fs) — clearing stale target",
                        age, self.target_timeout_s,
                    )
                    self._target_lost_logged = True
                    self._target_seen_since = None
                # Clear the stale target so exploration can continue normally.
                # Only active tracking goals should be cancelled here; otherwise
                # a stale detection can repeatedly preempt exploration and leave
                # the robot stuck facing the same obstacle.
                self.last_target = None
                if self._active_goal_kind == "TRACKING":
                    self._cancel_goal_if_active()
                if self._state in ("TRACKING",):
                    self._set_state("LOST")
                    self._publish_result(False)
                return

        if self._active_goal_kind == "EXPLORING" and self._goal_active:
            self._cancel_goal_if_active()
            self._set_state("IDLE")

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

        tx = target_g.pose.position.x
        ty = target_g.pose.position.y

        # If this target is near a recent dialogue point, skip it for a while
        # to avoid repeatedly asking the same person/object.
        if self._is_recent_dialogue_area(tx, ty):
            rospy.loginfo_throttle(
                2.0,
                "[TargetFollower] Ignoring target near recent dialogue area (%.2f, %.2f)",
                tx, ty,
            )
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
        goal_x, goal_y = tx, ty

        if self.standoff_distance > 0 or self.face_target:
            robot_info = self._get_robot_pose_in_global()
            if robot_info is None:
                return
            rx, ry, tf_robot = robot_info

            # Distance from base_link to target in global frame (for goal waypoint)
            dx = tx - rx
            dy = ty - ry
            d  = math.hypot(dx, dy)

            # ── REACHED check: camera_link z-depth ────────────────────────
            # The depth camera reports the target's z-coordinate in
            # camera_link (optical convention: z = forward / depth).
            # This is the most direct and reliable distance metric.
            d_cam_z = self._get_target_depth_in_camera()

            # Also compute 2D global-frame camera distance for goal offset
            cam_pos = self._get_camera_pose_in_global()
            if cam_pos is not None:
                cx, cy = cam_pos
                d_cam_xy = math.hypot(tx - cx, ty - cy)
            else:
                d_cam_xy = d  # fallback

            # Publish camera depth (z) for monitoring; fall back to xy
            d_report = d_cam_z if d_cam_z is not None else d_cam_xy
            self.target_distance_pub.publish(Float32(data=d_report))
            rospy.loginfo_throttle(
                2.0,
                "[TargetFollower] d_base=%.3f  d_cam_z=%s  d_cam_xy=%.3f  standoff=%.2f",
                d,
                "%.3f" % d_cam_z if d_cam_z is not None else "N/A",
                d_cam_xy,
                self.standoff_distance,
            )

            if self.standoff_distance > 0:
                # REACHED: use camera depth-z (preferred) or fallback to xy
                reach_dist = d_cam_z if d_cam_z is not None else d_cam_xy
                reach_threshold = self.standoff_distance
                if reach_dist <= reach_threshold:
                    if self._state not in ("REACHED", "WAITING_ACTION"):
                        rospy.loginfo(
                            "[TargetFollower] Within standoff "
                            "(d_cam_z=%s <= %.2f) — REACHED",
                            "%.3f" % reach_dist, reach_threshold,
                        )
                        if self.enable_interaction_mode:
                            self._on_reached()
                            return
                    # Pure-follow mode: do not stop at standoff, continue
                    # sending tracking goals for moving targets.

                # ── Close approach: bypass move_base for last-metre ────
                # When target is within close_approach_threshold but
                # beyond standoff, switch to direct cmd_vel drive.
                # This avoids DWA failure when the human is an obstacle.
                if self.enable_interaction_mode and reach_dist < self.close_approach_threshold:
                    rospy.loginfo(
                        "[TargetFollower] Target within close-approach zone "
                        "(d=%.3f < %.2f) — switching to CLOSE_APPROACH",
                        reach_dist, self.close_approach_threshold,
                    )
                    self._start_close_approach()
                    return

                # Goal waypoint: offset from base_link so that *camera*
                # depth ends up at standoff.  Camera is roughly
                # (d - d_cam_xy) metres closer to target along the
                # robot→target line.
                cam_ahead = d - d_cam_xy  # positive when camera is ahead
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
        self._active_goal_kind = "TRACKING"
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
            self._append_path_point_if_needed()

            # Primary following (skipped in WAITING_ACTION / RETREATING)
            self.maybe_send_goal()

            # Auto exploration when no valid target is being tracked
            self._tick_exploring()

            # Dialogue waiting state tick
            if self._state == "WAITING_ACTION":
                self._tick_waiting_action()

            # Close approach tick
            if self._state == "CLOSE_APPROACH":
                self._tick_close_approach()

            # Post-accept cooldown tick
            if self._state == "POST_ACCEPT_COOLDOWN":
                self._tick_post_accept_cooldown()

            # Retreat timeout tick
            if self._state == "RETREATING":
                self._tick_retreating()

            self._publish_memory_paths()

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
