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
import os
import sys
from collections import deque
from functools import partial
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Float32, String
import tf2_ros

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from frontier_planner import FrontierPlanner

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

def dist_xy_pts(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)

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
    STATES = ("STARTING", "IDLE", "EXPLORING", "TRACKING", "CLOSE_APPROACH", "REACHED", "WAITING_ACTION",
              "POST_ACCEPT_COOLDOWN", "RETREATING", "REACQUIRE_TARGET", "LOST", "FAILED")

    def __init__(self):
        # ── Core following params ──────────────────────────────────────────
        self.target_topic = rospy.get_param("~target_topic", "/target_pose")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.tracking_frame = rospy.get_param("~tracking_frame", self.global_frame)
        self.tracking_action_name = self._normalize_action_name(
            rospy.get_param("~tracking_action_name", "move_base"))
        self.explore_action_name = self._normalize_action_name(
            rospy.get_param("~explore_action_name", "move_base"))
        self.robot_frame  = rospy.get_param("~robot_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.send_rate_hz      = float(rospy.get_param("~send_rate_hz", 2.0))
        self.min_update_dist   = float(rospy.get_param("~min_update_dist", 0.3))
        self.target_timeout_s  = float(rospy.get_param("~target_timeout_s", 5.0))
        # Stale handling: use receive-time continuity as primary signal.
        self.target_timeout_use_rx_time = bool(
            rospy.get_param("~target_timeout_use_rx_time", True))
        # Keep active tracking for a while after stream dropout to avoid
        # immediate cancel/reacquire oscillation.
        self.tracking_stale_hold_s = float(
            rospy.get_param("~tracking_stale_hold_s", 8.0))
        # Keep obstacle avoidance always on by default: near-target updates are
        # made more aggressive, but still through move_base.
        self.tracking_near_target_window_m = float(
            rospy.get_param("~tracking_near_target_window_m", 1.8))
        self.tracking_near_send_rate_hz = float(
            rospy.get_param("~tracking_near_send_rate_hz", 1.8))
        self.tracking_near_min_update_dist = float(
            rospy.get_param("~tracking_near_min_update_dist", 0.08))
        # If cmd_vel stays near zero for a while during TRACKING, temporarily
        # hold goal re-sends so move_base/DWA can settle instead of preempt churn.
        self.tracking_goal_hold_zero_cmd_s = float(
            rospy.get_param("~tracking_goal_hold_zero_cmd_s", 0.8))
        self.tracking_goal_hold_on_stall_s = float(
            rospy.get_param("~tracking_goal_hold_on_stall_s", 1.4))
        self.tracking_goal_hold_far_only = bool(
            rospy.get_param("~tracking_goal_hold_far_only", True))
        # Near-target aborts can loop clear-costmap/retry. Limit retries in a
        # short window, then force retreat to break oscillation.
        self.tracking_near_abort_retry_limit = int(
            rospy.get_param("~tracking_near_abort_retry_limit", 2))
        self.tracking_near_abort_retry_window_s = float(
            rospy.get_param("~tracking_near_abort_retry_window_s", 12.0))
        self.tracking_near_abort_retry_cooldown_s = float(
            rospy.get_param("~tracking_near_abort_retry_cooldown_s", 2.0))
        # In near-target zone with close-approach disabled, send goals to the
        # raw target point (still through move_base) so local-goal tolerance
        # does not terminate too early before camera-depth REACHED.
        self.tracking_near_goal_to_target = bool(
            rospy.get_param("~tracking_near_goal_to_target", True))
        # Far-range fallback: if TRACKING stays near-zero for too long while
        # target is still far, gently push forward only when local costmap
        # indicates clear space ahead.
        self.tracking_far_fallback_enabled = bool(
            rospy.get_param("~tracking_far_fallback_enabled", True))
        self.tracking_far_distance_m = float(
            rospy.get_param("~tracking_far_distance_m", 1.8))
        self.tracking_far_zero_cmd_timeout_s = float(
            rospy.get_param("~tracking_far_zero_cmd_timeout_s", 2.0))
        self.tracking_far_forward_speed = float(
            rospy.get_param("~tracking_far_forward_speed", 0.08))
        self.tracking_far_forward_burst_s = float(
            rospy.get_param("~tracking_far_forward_burst_s", 0.6))
        self.tracking_far_steer_gain = float(
            rospy.get_param("~tracking_far_steer_gain", 0.6))
        self.tracking_far_max_ang = float(
            rospy.get_param("~tracking_far_max_ang", 0.35))
        self.tracking_far_safety_check_dist = float(
            rospy.get_param("~tracking_far_safety_check_dist", 0.8))
        self.tracking_far_safety_lateral_m = float(
            rospy.get_param("~tracking_far_safety_lateral_m", 0.18))
        # Far-range resend throttle: reduce goal preemption churn so DWA can
        # run continuously instead of being reset by tiny target jitter.
        self.tracking_far_resend_interval_s = float(
            rospy.get_param("~tracking_far_resend_interval_s", 1.3))
        self.tracking_far_min_update_dist = float(
            rospy.get_param("~tracking_far_min_update_dist", 0.45))
        # After tracking failure, enforce a minimum retreat displacement before
        # allowing re-track of the same target area.
        self.tracking_failure_force_retreat_enabled = bool(
            rospy.get_param("~tracking_failure_force_retreat_enabled", True))
        self.tracking_failure_min_retreat_displacement_m = float(
            rospy.get_param("~tracking_failure_min_retreat_displacement_m", 0.70))
        self.tracking_failure_retrack_block_radius_m = float(
            rospy.get_param("~tracking_failure_retrack_block_radius_m", 1.10))
        self.tracking_failure_retrack_block_s = float(
            rospy.get_param("~tracking_failure_retrack_block_s", 20.0))
        self.tracking_failure_retrack_cooldown_s = float(
            rospy.get_param("~tracking_failure_retrack_cooldown_s", 3.0))
        # Near target, a short detector dropout should not immediately force
        # LOST if move_base is still executing a valid close-range goal.
        self.target_timeout_near_grace_s = float(
            rospy.get_param("~target_timeout_near_grace_s", 2.0))
        self.target_timeout_near_dist_m = float(
            rospy.get_param("~target_timeout_near_dist_m", 1.15))
        self.target_timeout_near_dist_fresh_s = float(
            rospy.get_param("~target_timeout_near_dist_fresh_s", 2.0))
        # Recent-dialogue-area suppression should primarily affect exploration;
        # allow nearby re-tracking from IDLE/LOST/FAILED.
        self.dialogue_block_allow_near_dist_m = float(
            rospy.get_param("~dialogue_block_allow_near_dist_m", 2.0))
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
        # How long (s) to wait after trash_action=True so the user can finish dropping trash.
        self.post_accept_cooldown_s = float(rospy.get_param("~post_accept_cooldown_s", 15.0))
        # How far (m) to retreat when human refuses
        self.retreat_distance = float(rospy.get_param("~retreat_distance", 1.5))
        # Max time (s) allowed for the retreat navigation goal
        self.retreat_timeout_s = float(rospy.get_param("~retreat_timeout_s", 10.0))
        # After retreat completion, allow a brief window to immediately
        # accept a freshly detected target even near dialogue memory.
        self.retreat_post_target_grace_s = float(
            rospy.get_param("~retreat_post_target_grace_s", 1.6))
        # Angular speed (rad/s) for the retreat turn phase
        self.retreat_turn_speed = float(rospy.get_param("~retreat_turn_speed", 0.5))
        # Use a moderate retreat turn so the robot leaves the interaction area
        # without over-rotating into a nearby obstacle pocket.
        self.retreat_turn_angle_deg = float(rospy.get_param("~retreat_turn_angle_deg", 100.0))
        self.retreat_turn_tolerance_deg = float(rospy.get_param("~retreat_turn_tolerance_deg", 10.0))
        # Reverse-first retreat behaviour:
        # if safe, back up before turning away from the dialogue area.
        self.retreat_reverse_enabled = bool(rospy.get_param("~retreat_reverse_enabled", True))
        self.retreat_reverse_distance = float(rospy.get_param("~retreat_reverse_distance", 0.50))
        self.retreat_reverse_speed = abs(float(rospy.get_param("~retreat_reverse_speed", 0.10)))
        self.retreat_reverse_safety_margin = float(
            rospy.get_param("~retreat_reverse_safety_margin", 0.20))
        self.retreat_reverse_scan_stale_s = float(
            rospy.get_param("~retreat_reverse_scan_stale_s", 0.8))
        self.retreat_reverse_rear_sector_deg = float(
            rospy.get_param("~retreat_reverse_rear_sector_deg", 35.0))
        # Check only a short segment in costmap for reverse retreat; long-range
        # checks are often over-conservative around inflated cells.
        self.retreat_reverse_costmap_check_dist = float(
            rospy.get_param("~retreat_reverse_costmap_check_dist", 0.30))
        # Reverse-retreat safety is decided solely by this local costmap.
        self.retreat_reverse_costmap_topic = rospy.get_param(
            "~retreat_reverse_costmap_topic", "/move_base/local_costmap/costmap")

        # ── Auto explore params (active when no target is being tracked) ─────
        self.enable_auto_explore = bool(rospy.get_param("~enable_auto_explore", True))
        self.explore_goal_distance = float(rospy.get_param("~explore_goal_distance", 2.2))
        self.explore_short_horizon_m = float(
            rospy.get_param("~explore_short_horizon_m", self.explore_goal_distance))
        self.explore_goal_max_dist = float(rospy.get_param("~explore_goal_max_dist", 6.0))
        self.explore_goal_timeout_s = float(rospy.get_param("~explore_goal_timeout_s", 30.0))
        self.explore_goal_min_dist = float(rospy.get_param("~explore_goal_min_dist", 0.8))
        self.explore_map_topic = rospy.get_param("~explore_map_topic", "/map")
        self.explore_costmap_topic = rospy.get_param(
            "~explore_costmap_topic", "/move_base/global_costmap/costmap")
        self.explore_scan_topic = rospy.get_param("~explore_scan_topic", "/unitree/scan")
        self.explore_min_frontier_size = int(rospy.get_param("~explore_min_frontier_size", 8))
        self.explore_occupied_threshold = int(rospy.get_param("~explore_occupied_threshold", 65))
        self.explore_map_clearance_m = float(rospy.get_param("~explore_map_clearance_m", 0.35))
        self.explore_costmap_lethal_threshold = int(
            rospy.get_param("~explore_costmap_lethal_threshold", 70))
        self.explore_frontier_approach_pull_m = float(
            rospy.get_param("~explore_frontier_approach_pull_m", 1.0))
        self.explore_blacklist_radius_m = float(rospy.get_param("~explore_blacklist_radius_m", 0.5))
        self.explore_no_frontier_spin_interval = int(
            rospy.get_param("~explore_no_frontier_spin_interval", 3))
        self.explore_require_make_plan = bool(rospy.get_param("~explore_require_make_plan", True))
        self.explore_make_plan_tolerance = float(rospy.get_param("~explore_make_plan_tolerance", 0.2))
        self.explore_clear_costmap_on_replan = bool(
            rospy.get_param("~explore_clear_costmap_on_replan", True))
        self.explore_backup_after_failures = int(
            rospy.get_param("~explore_backup_after_failures", 3))
        self.explore_backup_duration_s = float(rospy.get_param("~explore_backup_duration_s", 1.8))
        self.explore_backup_speed = float(rospy.get_param("~explore_backup_speed", -0.08))
        self.explore_initial_scan_duration_s = float(
            rospy.get_param("~explore_initial_scan_duration_s", 4.0))
        self.explore_goal_scan_duration_s = float(
            rospy.get_param("~explore_goal_scan_duration_s", 2.2))
        self.explore_scan_angular_speed = float(
            rospy.get_param("~explore_scan_angular_speed", 0.42))
        self.explore_scan_min_clearance_m = float(
            rospy.get_param("~explore_scan_min_clearance_m", 0.28))
        self.explore_scan_settle_s = float(rospy.get_param("~explore_scan_settle_s", 0.4))
        self.explore_dependency_wait_log_s = float(
            rospy.get_param("~explore_dependency_wait_log_s", 2.0))
        self.target_reacquire_duration_s = float(rospy.get_param("~target_reacquire_duration_s", 4.0))
        self.target_reacquire_turn_speed = float(rospy.get_param("~target_reacquire_turn_speed", 0.34))
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

        # ── Close-approach params (optional direct cmd_vel fallback) ──
        self.enable_close_approach = bool(
            rospy.get_param("~enable_close_approach", False))
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
        # If a large depth jump persists consistently for several cycles,
        # accept it as a new baseline instead of being stuck on stale depth.
        self.close_approach_depth_jump_accept_count = int(
            rospy.get_param("~close_approach_depth_jump_accept_count", 6))
        self.close_approach_depth_jump_consistency_m = float(
            rospy.get_param("~close_approach_depth_jump_consistency_m", 0.20))
        self.close_approach_stale_hold_s = float(
            rospy.get_param("~close_approach_stale_hold_s", 2.0))

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
        self._retreat_phase        = None   # "BACKING" | "TURNING"
        self._retreat_target_yaw   = None   # target yaw after 180° turn
        self._retreat_reason       = ""
        self._retreat_reverse_remaining_m = 0.0
        self._retreat_reverse_last_xy = None
        self._retreat_advance_remaining_m = 0.0
        self._retreat_advance_last_xy = None
        self._force_retreat_active = False
        self._force_retreat_min_displacement_m = 0.0
        self._force_retreat_start_xy = None
        self._blocked_target_xy = None
        self._blocked_target_until = rospy.Time(0)
        self._close_approach_start = None   # rospy.Time when CLOSE_APPROACH began
        self._ca_last_good_depth   = None   # last accepted d_cam_z during CLOSE_APPROACH (depth filter)
        self._ca_depth_jump_candidate = None
        self._ca_depth_jump_count = 0
        self._explore_goal_start   = None
        self._explore_start_xy     = None
        self._explore_zero_cmd_start = None
        self._next_explore_time    = rospy.Time(0)
        self._last_explore_goal_xy = None
        self._last_explore_frontier_yaw = None
        self._explore_no_frontier_count = 0
        self._explore_consecutive_failures = 0
        self._explore_scan_start = None
        self._explore_scan_end = None
        self._explore_scan_mode = None
        self._explore_scan_target_yaw = None
        self._explore_initial_scan_done = False
        self._last_explore_wait_log = rospy.Time(0)
        self._last_scan_msg_time = None
        self._latest_scan_min_range = None
        self._latest_scan_rear_min_range = None
        self._retreat_reverse_costmap = None
        self._last_seen_target_xy = None
        self._reacquire_start = None
        self._reacquire_turn_dir = 1.0
        self._last_make_plan_warn = rospy.Time(0)
        self._last_clear_costmap_try = rospy.Time(0)
        self._target_seen_since    = None
        self._last_target_rx_time  = None
        self._last_cmd_vel_time    = None
        self._tracking_zero_cmd_start = None
        self._tracking_far_burst_until = rospy.Time(0)
        self._tracking_resend_hold_until = rospy.Time(0)
        self._near_abort_retry_count = 0
        self._near_abort_retry_window_start = None
        self._last_reach_dist = None
        self._last_reach_dist_time = None
        self._retreat_post_target_grace_until = rospy.Time(0)

        self.frontier_planner = FrontierPlanner(
            min_frontier_size=self.explore_min_frontier_size,
            occupied_threshold=self.explore_occupied_threshold,
            map_clearance_m=self.explore_map_clearance_m,
            costmap_lethal_threshold=self.explore_costmap_lethal_threshold,
            approach_pull_m=self.explore_frontier_approach_pull_m,
            blacklist_radius_m=self.explore_blacklist_radius_m,
        )
        self.make_plan_srv = None
        self.clear_costmaps_srvs = {}

        # Path memory and dialogue-location memory for anti-repeat behaviour.
        self._path_history = deque(maxlen=max(self.max_path_points, 100))  # (stamp, x, y)
        self._dialogue_points = deque(maxlen=200)  # (stamp, x, y, reason)
        self._last_path_pub_time = rospy.Time(0)
        self._state = "STARTING"
        self._last_status_time = rospy.Time(0)
        self._last_param_reload_time = rospy.Time(0)
        self._param_reload_interval  = 2.0  # seconds between param server checks

        # ── Publishers ─────────────────────────────────────────────────────
        # Publish startup state before waiting on move_base so orchestration
        # scripts can observe that target_follower is booting instead of stalled.
        self.result_pub = rospy.Publisher("~result",  Bool,   queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status",  String, queue_size=1, latch=True)
        self.status_pub.publish(String(data=self._state))

        # ── TF & action client ─────────────────────────────────────────────
        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl   = tf2_ros.TransformListener(self.tfbuf)

        self.tracking_client = actionlib.SimpleActionClient(
            self.tracking_action_name, MoveBaseAction)
        self.explore_client = self.tracking_client
        if self.explore_action_name != self.tracking_action_name:
            self.explore_client = actionlib.SimpleActionClient(
                self.explore_action_name, MoveBaseAction)
        rospy.loginfo("Waiting for %s action server...", self.tracking_action_name)
        self.tracking_client.wait_for_server()
        rospy.loginfo("Connected to %s.", self.tracking_action_name)
        self._set_state("IDLE")

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
        rospy.Subscriber(self.retreat_reverse_costmap_topic, OccupancyGrid,
                         self.cb_retreat_reverse_costmap, queue_size=1)
        if self.enable_auto_explore:
            rospy.Subscriber(self.explore_map_topic, OccupancyGrid,
                             self.cb_explore_map, queue_size=1)
            rospy.Subscriber(self.explore_costmap_topic, OccupancyGrid,
                             self.cb_explore_costmap, queue_size=1)
            rospy.Subscriber(self.explore_scan_topic, LaserScan,
                             self.cb_explore_scan, queue_size=5)

        # ── Publishers ─────────────────────────────────────────────────────
        # debug: planar distance from base_link to target pose in global_frame (meters)
        self.target_distance_pub = rospy.Publisher("~target_distance", Float32, queue_size=10)
        # debug: remembered path and dialogue points for runtime inspection
        self.path_history_pub = rospy.Publisher("~path_history", Path, queue_size=1)
        self.dialogue_points_pub = rospy.Publisher("~dialogue_points", Path, queue_size=1)

        rospy.loginfo("standoff_distance = %.2f m", self.standoff_distance)
        rospy.loginfo("enable_close_approach = %s", self.enable_close_approach)
        rospy.loginfo("close_approach_threshold = %.2f m", self.close_approach_threshold)
        rospy.loginfo("close_approach_speed = %.2f m/s", self.close_approach_speed)
        rospy.loginfo("close_approach_steer_gain = %.2f", self.close_approach_steer_gain)
        rospy.loginfo("close_approach_max_depth_jump = %.2f m", self.close_approach_max_depth_jump)
        rospy.loginfo(
            "close_approach_depth_jump_accept count=%d consistency=%.2fm stale_hold=%.1fs",
            self.close_approach_depth_jump_accept_count,
            self.close_approach_depth_jump_consistency_m,
            self.close_approach_stale_hold_s,
        )
        rospy.loginfo(
            "tracking_near_target window=%.2fm send_rate=%.2fHz min_update=%.2fm goal_to_target=%s",
            self.tracking_near_target_window_m,
            self.tracking_near_send_rate_hz,
            self.tracking_near_min_update_dist,
            self.tracking_near_goal_to_target,
        )
        rospy.loginfo(
            "tracking_far_fallback enabled=%s dist>=%.2fm timeout=%.1fs v=%.2f burst=%.1fs",
            self.tracking_far_fallback_enabled,
            self.tracking_far_distance_m,
            self.tracking_far_zero_cmd_timeout_s,
            self.tracking_far_forward_speed,
            self.tracking_far_forward_burst_s,
        )
        rospy.loginfo(
            "tracking_far_resend interval=%.2fs min_update=%.2fm stale_hold=%.1fs rx_stale=%s",
            self.tracking_far_resend_interval_s,
            self.tracking_far_min_update_dist,
            self.tracking_stale_hold_s,
            self.target_timeout_use_rx_time,
        )
        rospy.loginfo(
            "tracking_failure_retreat force=%s min_disp=%.2fm block_r=%.2fm block_t=%.1fs",
            self.tracking_failure_force_retreat_enabled,
            self.tracking_failure_min_retreat_displacement_m,
            self.tracking_failure_retrack_block_radius_m,
            self.tracking_failure_retrack_block_s,
        )
        rospy.loginfo(
            "target_timeout near_grace=%.1fs near_dist=%.2fm near_dist_fresh=%.1fs",
            self.target_timeout_near_grace_s,
            self.target_timeout_near_dist_m,
            self.target_timeout_near_dist_fresh_s,
        )
        rospy.loginfo("tracking_frame = %s", self.tracking_frame)
        rospy.loginfo(
            "tracking_action = %s  explore_action = %s",
            self.tracking_action_name,
            self.explore_action_name,
        )
        rospy.loginfo(
            "dialogue_block_allow_near_dist=%.2fm reverse_costmap_check_dist=%.2fm",
            self.dialogue_block_allow_near_dist_m,
            self.retreat_reverse_costmap_check_dist,
        )
        rospy.loginfo("face_target = %s", self.face_target)
        rospy.loginfo("trash_action_topic = %s", self.trash_action_topic)
        rospy.loginfo("action_wait_timeout = %.1f s", self.action_wait_timeout_s)
        rospy.loginfo("post_accept_cooldown = %.1f s", self.post_accept_cooldown_s)
        rospy.loginfo("retreat_distance = %.2f m", self.retreat_distance)
        rospy.loginfo("retreat_turn_angle = %.1f deg", self.retreat_turn_angle_deg)
        rospy.loginfo("retreat_reverse_enabled = %s", self.retreat_reverse_enabled)
        rospy.loginfo("retreat_reverse_distance = %.2f m", self.retreat_reverse_distance)
        rospy.loginfo("retreat_reverse_costmap_topic = %s", self.retreat_reverse_costmap_topic)
        rospy.loginfo("enable_auto_explore = %s", self.enable_auto_explore)
        rospy.loginfo("explore_target_confirm = %.1f s", self.explore_target_confirm_s)
        rospy.loginfo("explore_target_max_gap = %.1f s", self.explore_target_max_gap_s)
        rospy.loginfo("explore_stuck_timeout = %.1f s", self.explore_stuck_timeout_s)
        rospy.loginfo("explore_min_progress = %.2f m", self.explore_min_progress_dist)
        rospy.loginfo("explore_zero_cmd_vel_timeout = %.1f s", self.explore_zero_cmd_vel_timeout_s)
        rospy.loginfo("explore_revisit_window = %.1f s", self.explore_revisit_window_s)
        rospy.loginfo("explore_revisit_radius = %.2f m", self.explore_revisit_radius)
        rospy.loginfo("explore_map_topic = %s", self.explore_map_topic)
        rospy.loginfo("explore_costmap_topic = %s", self.explore_costmap_topic)
        rospy.loginfo("explore_scan_topic = %s", self.explore_scan_topic)
        rospy.loginfo("explore_short_horizon = %.2f m", self.explore_short_horizon_m)
        rospy.loginfo("explore_goal_max_dist = %.2f m", self.explore_goal_max_dist)
        rospy.loginfo("explore_require_make_plan = %s", self.explore_require_make_plan)
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
        try:
            if msg.header.frame_id == self.global_frame:
                tg = msg
            else:
                tfm = self.tfbuf.lookup_transform(
                    self.global_frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
                tg = transform_pose_stamped(msg, tfm, self.global_frame)
            self._last_seen_target_xy = (tg.pose.position.x, tg.pose.position.y)
        except Exception:
            pass

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
        """Track recent cmd_vel output for exploration and tracking stall detection."""
        self._last_cmd_vel_time = rospy.Time.now()

        linear_mag = math.hypot(msg.linear.x, msg.linear.y)
        angular_mag = abs(msg.angular.z)
        is_near_zero = (
            linear_mag <= self.explore_cmd_vel_linear_epsilon
            and angular_mag <= self.explore_cmd_vel_angular_epsilon
        )

        if self._state == "TRACKING" and self._goal_active and self._active_goal_kind == "TRACKING":
            if is_near_zero:
                if self._tracking_zero_cmd_start is None:
                    self._tracking_zero_cmd_start = rospy.Time.now()
            else:
                self._tracking_zero_cmd_start = None
        else:
            self._tracking_zero_cmd_start = None

        if not (self._state == "EXPLORING" and self._goal_active and self._active_goal_kind == "EXPLORING"):
            self._explore_zero_cmd_start = None
            return

        if is_near_zero:
            if self._explore_zero_cmd_start is None:
                self._explore_zero_cmd_start = rospy.Time.now()
        else:
            self._explore_zero_cmd_start = None

    def cb_explore_map(self, msg):
        self.frontier_planner.update_map(msg)

    def cb_explore_costmap(self, msg):
        self.frontier_planner.update_costmap(msg)
    
    def cb_retreat_reverse_costmap(self, msg):
        self._retreat_reverse_costmap = msg

    def cb_explore_scan(self, msg):
        self._last_scan_msg_time = rospy.Time.now()
        finite_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self._latest_scan_min_range = min(finite_ranges) if finite_ranges else None
        self._latest_scan_rear_min_range = self._sector_min_range(
            msg,
            center_angle=math.pi,
            half_width=math.radians(self.retreat_reverse_rear_sector_deg),
        )

    # ──────────────────────────────────────────────────────────────────────────
    # State helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _set_state(self, new_state):
        if new_state != self._state:
            rospy.loginfo("[TargetFollower] %s -> %s", self._state, new_state)
            self._state = new_state
            if new_state != "TRACKING":
                self._tracking_zero_cmd_start = None
                self._tracking_far_burst_until = rospy.Time(0)

    def _publish_result(self, success):
        self.result_pub.publish(Bool(data=success))
        rospy.loginfo(
            "[TargetFollower] Published result: %s",
            "REACHED (True)" if success else "FAILED/LOST (False)",
        )

    def _cancel_goal_if_active(self):
        if self._goal_active:
            was_exploring = self._active_goal_kind == "EXPLORING"
            client = self._client_for_goal_kind(self._active_goal_kind)
            state = client.get_state() if client is not None else GoalStatus.LOST
            if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self._suppress_done_cb = True
                client.cancel_goal()
                rospy.loginfo(
                    "Cancelled active %s goal on %s",
                    str(self._active_goal_kind).lower(),
                    self._action_name_for_goal_kind(self._active_goal_kind),
                )
            self._goal_active = False
            self._active_goal_kind = None
            if was_exploring:
                self._explore_goal_start = None
                self._explore_start_xy = None
                self._explore_zero_cmd_start = None
                self._last_explore_frontier_yaw = None
            self._tracking_zero_cmd_start = None
            self._tracking_far_burst_until = rospy.Time(0)

    def _get_robot_pose_in_global(self):
        """Return (x, y, tf_stamped) of robot in global_frame, or None."""
        return self._get_robot_pose_in_frame(self.global_frame)

    def _get_robot_pose_in_frame(self, frame_id):
        """Return (x, y, tf_stamped) of robot in frame_id, or None."""
        try:
            tf_robot = self.tfbuf.lookup_transform(
                frame_id, self.robot_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
            rx = tf_robot.transform.translation.x
            ry = tf_robot.transform.translation.y
            return rx, ry, tf_robot
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF not ready for robot pose in %s: %s", frame_id, str(e))
            return None

    def _get_camera_pose_in_global(self):
        """Return (cx, cy) of camera_link in global_frame, or None."""
        return self._get_camera_pose_in_frame(self.global_frame)

    def _get_camera_pose_in_frame(self, frame_id):
        """Return (cx, cy) of camera_link in frame_id, or None."""
        try:
            tf_cam = self.tfbuf.lookup_transform(
                frame_id, self.camera_frame,
                rospy.Time(0), rospy.Duration(0.2),
            )
            cx = tf_cam.transform.translation.x
            cy = tf_cam.transform.translation.y
            return cx, cy
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF not ready for camera pose in %s: %s", frame_id, str(e))
            return None

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    @staticmethod
    def _normalize_action_name(name):
        name = str(name).strip()
        if not name:
            return "move_base"
        return name[1:] if name.startswith("/") else name

    def _action_name_for_goal_kind(self, goal_kind):
        if goal_kind == "TRACKING":
            return self.tracking_action_name
        return self.explore_action_name

    def _client_for_goal_kind(self, goal_kind):
        if goal_kind == "TRACKING":
            return self.tracking_client
        if goal_kind == "EXPLORING":
            return self.explore_client
        return None

    def _service_name_for_goal_kind(self, goal_kind, service_leaf):
        action_name = self._action_name_for_goal_kind(goal_kind)
        return "/%s/%s" % (self._normalize_action_name(action_name), service_leaf)

    def _transform_pose_to_frame(self, pose_in, target_frame):
        if pose_in.header.frame_id == target_frame:
            return pose_in
        tfm = self.tfbuf.lookup_transform(
            target_frame,
            pose_in.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.2),
        )
        return transform_pose_stamped(pose_in, tfm, target_frame)

    def _get_target_depth_in_camera(self):
        """Return the z-distance (depth) of the current target in camera_link frame.

        The detection pipeline (handobj_detection_rgbd / predict_15cls_rgbd) outputs
        coordinates in the camera optical convention:
            z = forward (depth),  x = right,  y = down
        and labels the frame as camera_link.  This z-component is the most
        reliable distance metric because it comes straight from the depth
        sensor with no TF chain involved.

        If the target is not in camera_link, we fall back to a TF lookup.
        Returns None when target or TF is unavailable.
        """
        if self.last_target is None:
            return None
        # Fast path: target already in camera_link — just read z
        if self.last_target.header.frame_id == self.camera_frame:
            return self.last_target.pose.position.z
        # Slow path: transform into camera_link
        try:
            tfm = self.tfbuf.lookup_transform(
                self.camera_frame,
                self.last_target.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            target_cam = transform_pose_stamped(
                self.last_target, tfm, self.camera_frame,
            )
            return target_cam.pose.position.z
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF error getting camera depth: %s", str(e))
            return None

    def _reload_dynamic_params(self):
        now = rospy.Time.now()
        if (now - self._last_param_reload_time).to_sec() < self._param_reload_interval:
            return
        self._last_param_reload_time = now
        self.enable_auto_explore = bool(
            rospy.get_param("~enable_auto_explore", self.enable_auto_explore)
        )
        self.standoff_distance = float(rospy.get_param("~standoff_distance", self.standoff_distance))
        self.face_target       = bool(rospy.get_param("~face_target",         self.face_target))
        self.send_rate_hz      = float(rospy.get_param("~send_rate_hz",       self.send_rate_hz))
        self.min_update_dist   = float(rospy.get_param("~min_update_dist",    self.min_update_dist))
        self.target_timeout_use_rx_time = bool(
            rospy.get_param("~target_timeout_use_rx_time", self.target_timeout_use_rx_time))
        self.tracking_stale_hold_s = float(
            rospy.get_param("~tracking_stale_hold_s", self.tracking_stale_hold_s))
        self.tracking_near_target_window_m = float(
            rospy.get_param("~tracking_near_target_window_m", self.tracking_near_target_window_m))
        self.tracking_near_send_rate_hz = float(
            rospy.get_param("~tracking_near_send_rate_hz", self.tracking_near_send_rate_hz))
        self.tracking_near_min_update_dist = float(
            rospy.get_param("~tracking_near_min_update_dist", self.tracking_near_min_update_dist))
        self.tracking_goal_hold_zero_cmd_s = float(
            rospy.get_param("~tracking_goal_hold_zero_cmd_s", self.tracking_goal_hold_zero_cmd_s))
        self.tracking_goal_hold_on_stall_s = float(
            rospy.get_param("~tracking_goal_hold_on_stall_s", self.tracking_goal_hold_on_stall_s))
        self.tracking_goal_hold_far_only = bool(
            rospy.get_param("~tracking_goal_hold_far_only", self.tracking_goal_hold_far_only))
        self.tracking_near_abort_retry_limit = int(
            rospy.get_param("~tracking_near_abort_retry_limit", self.tracking_near_abort_retry_limit))
        self.tracking_near_abort_retry_window_s = float(
            rospy.get_param("~tracking_near_abort_retry_window_s", self.tracking_near_abort_retry_window_s))
        self.tracking_near_abort_retry_cooldown_s = float(
            rospy.get_param("~tracking_near_abort_retry_cooldown_s", self.tracking_near_abort_retry_cooldown_s))
        self.tracking_near_goal_to_target = bool(
            rospy.get_param("~tracking_near_goal_to_target", self.tracking_near_goal_to_target))
        self.target_timeout_near_grace_s = float(
            rospy.get_param("~target_timeout_near_grace_s", self.target_timeout_near_grace_s))
        self.target_timeout_near_dist_m = float(
            rospy.get_param("~target_timeout_near_dist_m", self.target_timeout_near_dist_m))
        self.target_timeout_near_dist_fresh_s = float(
            rospy.get_param("~target_timeout_near_dist_fresh_s", self.target_timeout_near_dist_fresh_s))
        self.dialogue_block_allow_near_dist_m = float(
            rospy.get_param("~dialogue_block_allow_near_dist_m", self.dialogue_block_allow_near_dist_m))
        self.retreat_reverse_costmap_check_dist = float(
            rospy.get_param("~retreat_reverse_costmap_check_dist", self.retreat_reverse_costmap_check_dist))
        self.tracking_far_fallback_enabled = bool(
            rospy.get_param("~tracking_far_fallback_enabled", self.tracking_far_fallback_enabled))
        self.tracking_far_distance_m = float(
            rospy.get_param("~tracking_far_distance_m", self.tracking_far_distance_m))
        self.tracking_far_zero_cmd_timeout_s = float(
            rospy.get_param("~tracking_far_zero_cmd_timeout_s", self.tracking_far_zero_cmd_timeout_s))
        self.tracking_far_forward_speed = float(
            rospy.get_param("~tracking_far_forward_speed", self.tracking_far_forward_speed))
        self.tracking_far_forward_burst_s = float(
            rospy.get_param("~tracking_far_forward_burst_s", self.tracking_far_forward_burst_s))
        self.tracking_far_steer_gain = float(
            rospy.get_param("~tracking_far_steer_gain", self.tracking_far_steer_gain))
        self.tracking_far_max_ang = float(
            rospy.get_param("~tracking_far_max_ang", self.tracking_far_max_ang))
        self.tracking_far_safety_check_dist = float(
            rospy.get_param("~tracking_far_safety_check_dist", self.tracking_far_safety_check_dist))
        self.tracking_far_safety_lateral_m = float(
            rospy.get_param("~tracking_far_safety_lateral_m", self.tracking_far_safety_lateral_m))
        self.tracking_far_resend_interval_s = float(
            rospy.get_param("~tracking_far_resend_interval_s", self.tracking_far_resend_interval_s))
        self.tracking_far_min_update_dist = float(
            rospy.get_param("~tracking_far_min_update_dist", self.tracking_far_min_update_dist))
        self.tracking_failure_force_retreat_enabled = bool(
            rospy.get_param("~tracking_failure_force_retreat_enabled", self.tracking_failure_force_retreat_enabled))
        self.tracking_failure_min_retreat_displacement_m = float(
            rospy.get_param("~tracking_failure_min_retreat_displacement_m", self.tracking_failure_min_retreat_displacement_m))
        self.tracking_failure_retrack_block_radius_m = float(
            rospy.get_param("~tracking_failure_retrack_block_radius_m", self.tracking_failure_retrack_block_radius_m))
        self.tracking_failure_retrack_block_s = float(
            rospy.get_param("~tracking_failure_retrack_block_s", self.tracking_failure_retrack_block_s))
        self.tracking_failure_retrack_cooldown_s = float(
            rospy.get_param("~tracking_failure_retrack_cooldown_s", self.tracking_failure_retrack_cooldown_s))
        self.close_approach_depth_jump_accept_count = int(
            rospy.get_param("~close_approach_depth_jump_accept_count", self.close_approach_depth_jump_accept_count))
        self.close_approach_depth_jump_consistency_m = float(
            rospy.get_param("~close_approach_depth_jump_consistency_m", self.close_approach_depth_jump_consistency_m))
        self.close_approach_stale_hold_s = float(
            rospy.get_param("~close_approach_stale_hold_s", self.close_approach_stale_hold_s))
        self.enable_close_approach = bool(
            rospy.get_param("~enable_close_approach", self.enable_close_approach))
        self.retreat_distance  = float(rospy.get_param("~retreat_distance",   self.retreat_distance))
        self.retreat_post_target_grace_s = float(
            rospy.get_param("~retreat_post_target_grace_s", self.retreat_post_target_grace_s))

    def _should_block_dialogue_area_target(self, tx, ty):
        if not self._is_recent_dialogue_area(tx, ty):
            return False

        if self._state in ("EXPLORING", "REACQUIRE_TARGET"):
            return True
        if self._state == "TRACKING":
            return False

        if self._state in ("IDLE", "LOST", "FAILED"):
            if rospy.Time.now() < self._retreat_post_target_grace_until and self._target_fresh():
                return False
            robot_info = self._get_robot_pose_in_global()
            if robot_info is None:
                return True
            rx, ry, _ = robot_info
            return math.hypot(tx - rx, ty - ry) > self.dialogue_block_allow_near_dist_m

        return True

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
        # In sim-time startup, `now` can be smaller than the memory window.
        # Clamp cutoff to epoch to avoid negative Time subtraction exceptions.
        window_s = max(0.0, float(window_s))
        now_s = now.to_sec()
        cutoff = rospy.Time.from_sec(max(0.0, now_s - window_s))

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
        # In sim-time startup, `now` can be smaller than the memory window.
        # Clamp cutoff to epoch to avoid negative Time subtraction exceptions.
        window_s = max(0.0, float(window_s))
        now_s = now.to_sec()
        cutoff = rospy.Time.from_sec(max(0.0, now_s - window_s))

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
        now = rospy.Time.now()
        age_header = (now - self.last_target.header.stamp).to_sec()
        age_rx = None
        if self._last_target_rx_time is not None:
            age_rx = (now - self._last_target_rx_time).to_sec()

        # Keep freshness semantics consistent with maybe_send_goal():
        # use receive-time when configured, and gracefully fall back.
        if self.target_timeout_use_rx_time and age_rx is not None:
            age = age_rx
        elif age_header is not None:
            age = age_header
        elif age_rx is not None:
            age = age_rx
        else:
            self._target_seen_since = None
            return False

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

    def _goal_in_memory_blocklist(self, wx, wy):
        return self._is_recent_path_area(wx, wy) or self._is_recent_dialogue_area(wx, wy)

    def _explore_dependencies_ready(self):
        if not self.frontier_planner.has_map():
            return False, "waiting /map"
        if self._last_scan_msg_time is None:
            return False, "waiting scan"
        if (rospy.Time.now() - self._last_scan_msg_time).to_sec() > 2.5:
            return False, "scan stale"
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return False, "tf unavailable"
        return True, ""

    def _ensure_make_plan_service(self):
        if self.make_plan_srv is not None:
            return True
        service_name = self._service_name_for_goal_kind("EXPLORING", "make_plan")
        try:
            rospy.wait_for_service(service_name, timeout=0.6)
            self.make_plan_srv = rospy.ServiceProxy(service_name, GetPlan)
            return True
        except Exception:
            return False

    def _goal_reachable_with_make_plan(self, sx, sy, gx, gy):
        if not self.explore_require_make_plan:
            return True
        if not self._ensure_make_plan_service():
            now = rospy.Time.now()
            if (now - self._last_make_plan_warn).to_sec() > 2.0:
                self._last_make_plan_warn = now
                rospy.logwarn("[TargetFollower] waiting /move_base/make_plan before exploration starts")
            return False
        req = GetPlanRequest()
        req.start.header.frame_id = self.global_frame
        req.start.header.stamp = rospy.Time.now()
        req.start.pose.position.x = sx
        req.start.pose.position.y = sy
        req.start.pose.orientation.w = 1.0
        req.goal.header.frame_id = self.global_frame
        req.goal.header.stamp = req.start.header.stamp
        req.goal.pose.position.x = gx
        req.goal.pose.position.y = gy
        req.goal.pose.orientation.w = 1.0
        req.tolerance = self.explore_make_plan_tolerance
        try:
            resp = self.make_plan_srv(req)
            return len(resp.plan.poses) >= 2
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[TargetFollower] make_plan failed: %s", str(e))
            return False

    def _ensure_clear_costmaps_service(self, goal_kind):
        service_name = self._service_name_for_goal_kind(goal_kind, "clear_costmaps")
        if service_name in self.clear_costmaps_srvs:
            return True
        try:
            rospy.wait_for_service(service_name, timeout=0.4)
            self.clear_costmaps_srvs[service_name] = rospy.ServiceProxy(service_name, Empty)
            return True
        except Exception:
            return False

    def _clear_costmaps(self, reason, goal_kind=None):
        if not self.explore_clear_costmap_on_replan:
            return
        if goal_kind not in ("TRACKING", "EXPLORING"):
            goal_kind = self._active_goal_kind if self._active_goal_kind in ("TRACKING", "EXPLORING") else "EXPLORING"
        if not self._ensure_clear_costmaps_service(goal_kind):
            return
        now = rospy.Time.now()
        if (now - self._last_clear_costmap_try).to_sec() < 0.8:
            return
        self._last_clear_costmap_try = now
        try:
            service_name = self._service_name_for_goal_kind(goal_kind, "clear_costmaps")
            self.clear_costmaps_srvs[service_name]()
            rospy.loginfo("[TargetFollower] Cleared %s costmaps (%s)", goal_kind.lower(), reason)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[TargetFollower] clear_costmaps failed: %s", str(e))

    def _can_scan_in_place(self):
        if self._latest_scan_min_range is None:
            return False, "scan_unavailable"
        if self._latest_scan_min_range < self.explore_scan_min_clearance_m:
            return False, "scan_too_close"
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return False, "tf_unavailable"
        rx, ry, _ = robot_info
        if self.frontier_planner.costmap_data is not None and not self.frontier_planner._is_costmap_world_safe(rx, ry):
            # Do not hard-block in-place scans on costmap occupancy, because at
            # startup/inflated map edges this frequently becomes a false negative.
            rospy.logdebug_throttle(
                2.0,
                "[TargetFollower] scan allowed although costmap marks robot cell unsafe at (%.2f, %.2f)",
                rx,
                ry,
            )
        return True, "ok"

    def _stop_explore_scan(self):
        if self._explore_scan_end is None:
            return
        self._stop_cmd_vel()
        self._explore_scan_start = None
        self._explore_scan_end = None
        self._explore_scan_mode = None
        self._explore_scan_target_yaw = None
        self._next_explore_time = rospy.Time.now() + rospy.Duration(self.explore_scan_settle_s)

    def _start_explore_scan(self, mode, duration_s, preferred_yaw=None):
        can_scan, reason = self._can_scan_in_place()
        if not can_scan:
            rospy.logwarn_throttle(
                2.0,
                "[TargetFollower] Skip %s scan (%s): nearby clearance %.2fm < %.2fm",
                mode,
                reason,
                self._latest_scan_min_range if self._latest_scan_min_range is not None else -1.0,
                self.explore_scan_min_clearance_m,
            )
            return False
        now = rospy.Time.now()
        self._explore_scan_mode = mode
        self._explore_scan_start = now
        self._explore_scan_end = now + rospy.Duration(max(0.2, duration_s))
        self._explore_scan_target_yaw = preferred_yaw
        self._set_state("EXPLORING")
        rospy.loginfo("[TargetFollower] %s scan started for %.1fs", mode, duration_s)
        return True

    def _tick_explore_scan(self):
        if self._explore_scan_end is None:
            return False
        now = rospy.Time.now()
        if now >= self._explore_scan_end:
            self._stop_explore_scan()
            return False
        if self._target_confirmed_for_explore_interrupt():
            self._stop_explore_scan()
            return False

        twist = Twist()
        max_w = abs(self.explore_scan_angular_speed)
        turn_dir = 1.0
        if self._explore_scan_target_yaw is not None:
            robot_info = self._get_robot_pose_in_global()
            if robot_info is not None:
                yaw = self._extract_yaw(robot_info[2])
                yaw_err = self._normalize_angle(self._explore_scan_target_yaw - yaw)
                if abs(yaw_err) > math.radians(9.0):
                    twist.angular.z = max(-max_w, min(max_w, 1.2 * yaw_err))
                    self.cmd_vel_pub.publish(twist)
                    return True
                turn_dir = 1.0 if yaw_err >= 0.0 else -1.0
        phase = int((now - self._explore_scan_start).to_sec() / 0.8)
        if phase % 2 == 1:
            turn_dir *= -1.0
        twist.angular.z = turn_dir * max_w
        self.cmd_vel_pub.publish(twist)
        return True

    def _mark_explore_goal_failed(self, reason):
        if self._last_explore_goal_xy is not None:
            self.frontier_planner.blacklist_goal(self._last_explore_goal_xy[0], self._last_explore_goal_xy[1])
        self._explore_consecutive_failures += 1
        self._clear_costmaps(reason)
        if self._explore_consecutive_failures >= self.explore_backup_after_failures:
            self._try_explore_backup_recovery()
            self._explore_consecutive_failures = 0

    def _try_explore_backup_recovery(self):
        if self._state in ("WAITING_ACTION", "POST_ACCEPT_COOLDOWN", "RETREATING"):
            return
        self._cancel_goal_if_active()
        twist = Twist()
        twist.linear.x = self.explore_backup_speed
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < self.explore_backup_duration_s and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self._stop_cmd_vel()
        self._clear_costmaps("backup_recovery")

    def _pick_explore_goal(self):
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return None
        rx, ry, _ = robot_info
        for _ in range(4):
            picked = self.frontier_planner.select_goal(
                rx,
                ry,
                blocked_fn=self._goal_in_memory_blocklist,
                min_goal_dist=self.explore_goal_min_dist,
                short_horizon_m=self.explore_short_horizon_m,
                max_goal_dist=self.explore_goal_max_dist,
            )
            if picked is None:
                return None
            gx = picked["goal_x"]
            gy = picked["goal_y"]
            if self._goal_reachable_with_make_plan(rx, ry, gx, gy):
                return picked
            self.frontier_planner.blacklist_goal(gx, gy)
        return None

    def _start_explore_goal(self):
        if not self.enable_auto_explore or self._goal_active:
            return
        if not self.explore_client.wait_for_server(rospy.Duration(0.05)):
            now = rospy.Time.now()
            if (now - self._last_explore_wait_log).to_sec() >= self.explore_dependency_wait_log_s:
                self._last_explore_wait_log = now
                rospy.loginfo("[TargetFollower] Exploration waiting: move_base action server")
            return
        ready, reason = self._explore_dependencies_ready()
        if not ready:
            now = rospy.Time.now()
            if (now - self._last_explore_wait_log).to_sec() >= self.explore_dependency_wait_log_s:
                self._last_explore_wait_log = now
                rospy.loginfo("[TargetFollower] Exploration waiting: %s", reason)
            return

        # On first entry into autonomous exploration, do a short scan first to
        # seed map + camera awareness before committing to frontier motion.
        if not self._explore_initial_scan_done:
            if self._start_explore_scan("startup", self.explore_initial_scan_duration_s):
                self._explore_initial_scan_done = True
            return

        picked = self._pick_explore_goal()
        if picked is None:
            self._explore_no_frontier_count += 1
            if (
                self._explore_no_frontier_count % max(self.explore_no_frontier_spin_interval, 1) == 0
                and self._start_explore_scan("no_frontier", self.explore_goal_scan_duration_s)
            ):
                return
            self._next_explore_time = rospy.Time.now() + rospy.Duration(1.0)
            return

        self._explore_no_frontier_count = 0
        gx = picked["goal_x"]
        gy = picked["goal_y"]
        heading = picked["frontier_yaw"]

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = self._yaw_to_quaternion(heading)

        self.explore_client.send_goal(
            goal,
            done_cb=partial(self._goal_done_cb, goal_kind="EXPLORING"),
        )
        self._goal_active = True
        self._active_goal_kind = "EXPLORING"
        self._explore_goal_start = rospy.Time.now()
        robot_info = self._get_robot_pose_in_global()
        self._explore_start_xy = (robot_info[0], robot_info[1]) if robot_info is not None else None
        self._explore_zero_cmd_start = None
        self._last_explore_goal_xy = (gx, gy)
        self._last_explore_frontier_yaw = heading
        self._set_state("EXPLORING")
        rospy.loginfo(
            "[TargetFollower] Exploring frontier -> goal=(%.2f, %.2f) frontier=(%.2f, %.2f) size=%d",
            gx, gy, picked["frontier_x"], picked["frontier_y"], picked["frontier_size"],
        )

    def _tick_exploring(self):
        """Map-driven frontier exploration when no stable target is being tracked."""
        if not self.enable_auto_explore:
            return
        if self._state in (
            "WAITING_ACTION",
            "RETREATING",
            "CLOSE_APPROACH",
            "POST_ACCEPT_COOLDOWN",
            "REACQUIRE_TARGET",
            "TRACKING",
            "REACHED",
        ):
            return

        # Target appears: stop exploration and let tracking take over.
        if self._target_confirmed_for_explore_interrupt():
            if self._active_goal_kind == "EXPLORING" and self._goal_active:
                rospy.loginfo(
                    "[TargetFollower] Stable target detected for %.1fs — interrupting exploration",
                    self.explore_target_confirm_s,
                )
                self._cancel_goal_if_active()
            self._stop_explore_scan()
            if self._state == "EXPLORING":
                self._set_state("IDLE")
            return

        if self._tick_explore_scan():
            return

        now = rospy.Time.now()
        if now < self._next_explore_time:
            return

        if self._active_goal_kind == "EXPLORING" and self._goal_active and self._explore_goal_start is not None:
            elapsed = (now - self._explore_goal_start).to_sec()
            if self._explore_zero_cmd_start is not None and self._last_cmd_vel_time is not None:
                cmd_age = (now - self._last_cmd_vel_time).to_sec()
                zero_elapsed = (now - self._explore_zero_cmd_start).to_sec()
                if cmd_age <= 1.0 and zero_elapsed >= self.explore_zero_cmd_vel_timeout_s:
                    rospy.logwarn(
                        "[TargetFollower] Explore cmd_vel near zero for %.1fs — cancel and replan",
                        zero_elapsed,
                    )
                    self._cancel_goal_if_active()
                    self._active_goal_kind = None
                    self._mark_explore_goal_failed("zero_cmd")
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
                            "[TargetFollower] Explore progress only %.2fm in %.1fs — cancel and replan",
                            progress, elapsed,
                        )
                        self._cancel_goal_if_active()
                        self._active_goal_kind = None
                        self._mark_explore_goal_failed("stuck_progress")
                        self._set_state("IDLE")
                        self._next_explore_time = now + rospy.Duration(0.6)
                        return
            if elapsed > self.explore_goal_timeout_s:
                rospy.logwarn("[TargetFollower] Explore goal timeout %.1fs — cancel and replan", elapsed)
                self._cancel_goal_if_active()
                self._active_goal_kind = None
                self._mark_explore_goal_failed("goal_timeout")
                self._set_state("IDLE")
                self._next_explore_time = now + rospy.Duration(1.0)
            return

        self._start_explore_goal()
        self._next_explore_time = now + rospy.Duration(0.5)

    def _start_reacquire_target(self, reason):
        if self._state in ("WAITING_ACTION", "POST_ACCEPT_COOLDOWN", "RETREATING"):
            return
        self._cancel_goal_if_active()
        self._stop_explore_scan()
        self._reacquire_start = rospy.Time.now()
        self._reacquire_turn_dir = 1.0
        self._set_state("REACQUIRE_TARGET")
        rospy.loginfo("[TargetFollower] REACQUIRE_TARGET started (%s)", reason)

    def _tick_reacquire_target(self):
        if self._state != "REACQUIRE_TARGET":
            return
        if self._target_confirmed_for_explore_interrupt():
            self._stop_cmd_vel()
            self._set_state("IDLE")
            return
        if self._reacquire_start is None:
            self._reacquire_start = rospy.Time.now()
        elapsed = (rospy.Time.now() - self._reacquire_start).to_sec()
        if elapsed >= self.target_reacquire_duration_s:
            self._stop_cmd_vel()
            self._set_state("IDLE")
            self._next_explore_time = rospy.Time.now() + rospy.Duration(0.2)
            return

        twist = Twist()
        phase = int(elapsed / 0.8)
        direction = 1.0 if phase % 2 == 0 else -1.0
        twist.angular.z = direction * abs(self.target_reacquire_turn_speed)
        self.cmd_vel_pub.publish(twist)

    # ──────────────────────────────────────────────────────────────────────────
    # move_base callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _goal_done_cb(self, status, result, goal_kind=None):
        """Called by actionlib when a move_base goal reaches a terminal state."""
        # If we manually cancelled the goal, ignore this callback entirely
        if self._suppress_done_cb:
            self._suppress_done_cb = False
            rospy.loginfo("[TargetFollower] Suppressed done_cb after manual cancel (status=%d)", status)
            return

        if goal_kind is not None and self._active_goal_kind not in (None, goal_kind):
            rospy.loginfo(
                "[TargetFollower] Ignoring stale %s done_cb (status=%d, active=%s)",
                goal_kind,
                status,
                self._active_goal_kind,
            )
            return

        self._goal_active = False

        # RETREATING: if any stale move_base callback arrives, finish retreat safely.
        if self._state == "RETREATING":
            rospy.loginfo("[TargetFollower] Retreat callback finished (status=%d)", status)
            self._finish_retreat()
            return

        # EXPLORING: continue exploration loop unless interrupted by target tracking.
        if goal_kind == "EXPLORING" and (self._state == "EXPLORING" or self._active_goal_kind == "EXPLORING"):
            self._active_goal_kind = None
            self._explore_goal_start = None
            self._explore_start_xy = None
            self._explore_zero_cmd_start = None
            if status == GoalStatus.SUCCEEDED:
                self._explore_consecutive_failures = 0
                self._explore_no_frontier_count = 0
                rospy.loginfo("[TargetFollower] Explore frontier goal reached")
                if self._start_explore_scan(
                    "goal_reached",
                    self.explore_goal_scan_duration_s,
                    preferred_yaw=self._last_explore_frontier_yaw,
                ):
                    return
            else:
                rospy.logwarn("[TargetFollower] Explore goal ended with status=%d", status)
                self._mark_explore_goal_failed("goal_done_status_%d" % status)
            self._set_state("IDLE")
            self._next_explore_time = rospy.Time.now() + rospy.Duration(0.5)
            return

        # WAITING_ACTION / REACHED: don't let stale goal callbacks change state
        if self._state in ("REACHED", "WAITING_ACTION"):
            rospy.loginfo("[TargetFollower] Ignoring done_cb in %s (status=%d)", self._state, status)
            return

        # TRACKING: react to goal outcome
        if (goal_kind in (None, "TRACKING")) and self._state == "TRACKING":
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("[TargetFollower] move_base SUCCEEDED — checking distance in next tick")
                # Don't set REACHED here; let maybe_send_goal distance check handle it
            elif status == GoalStatus.PREEMPTED:
                rospy.loginfo("[TargetFollower] move_base PREEMPTED (new goal will be sent)")
            elif status in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                if (
                    self._last_reach_dist is not None
                    and self._last_reach_dist <= (self.tracking_near_target_window_m + 0.2)
                ):
                    now = rospy.Time.now()
                    if (
                        self._near_abort_retry_window_start is None
                        or (now - self._near_abort_retry_window_start).to_sec()
                        > self.tracking_near_abort_retry_window_s
                    ):
                        self._near_abort_retry_window_start = now
                        self._near_abort_retry_count = 0
                    self._near_abort_retry_count += 1

                    if self._near_abort_retry_count <= max(0, self.tracking_near_abort_retry_limit):
                        rospy.logwarn(
                            "[TargetFollower] move_base aborted near target (reach_dist=%.2f) — clear costmaps and retry (%d/%d)",
                            self._last_reach_dist,
                            self._near_abort_retry_count,
                            max(0, self.tracking_near_abort_retry_limit),
                        )
                        self._clear_costmaps("tracking_abort_near", goal_kind="TRACKING")
                        self._tracking_resend_hold_until = now + rospy.Duration(
                            max(0.0, self.tracking_near_abort_retry_cooldown_s)
                        )
                        self._set_state("IDLE")
                        return

                    rospy.logwarn(
                        "[TargetFollower] move_base aborted near target repeatedly (%d in %.1fs) — force retreat",
                        self._near_abort_retry_count,
                        self.tracking_near_abort_retry_window_s,
                    )
                    self._near_abort_retry_count = 0
                    self._near_abort_retry_window_start = None
                    self._start_tracking_failure_retreat("goal_aborted_near")
                    return
                self._start_tracking_failure_retreat("goal_aborted")

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
    # Retreat logic — reverse-first (if safe), then turn away
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

    def _sector_min_range(self, scan_msg, center_angle, half_width):
        if scan_msg is None or scan_msg.angle_increment == 0.0:
            return None
        best = None
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                if abs(self._normalize_angle(angle - center_angle)) <= half_width:
                    best = r if best is None else min(best, r)
            angle += scan_msg.angle_increment
        return best

    def _can_reverse_retreat(self, rx, ry, current_yaw, check_distance):
        if not self.retreat_reverse_enabled:
            return False, "reverse disabled"
        if check_distance <= 0.0:
            return False, "reverse distance <= 0"
        if self._retreat_reverse_costmap is None:
            return False, "no local costmap"

        costmap_check_dist = min(check_distance, max(0.0, self.retreat_reverse_costmap_check_dist))
        if costmap_check_dist <= 0.0:
            return True, "ok"

        check_scales = (1.0, 0.75, 0.5, 0.25)
        for scale in check_scales:
            dist = costmap_check_dist * scale
            wx = rx - dist * math.cos(current_yaw)
            wy = ry - dist * math.sin(current_yaw)
            if not self._is_local_costmap_world_safe(wx, wy):
                return False, "local costmap blocks reverse path"
        return True, "ok"

    def _is_local_costmap_world_safe(self, wx, wy):
        msg = self._retreat_reverse_costmap
        if msg is None:
            return False

        info = msg.info
        res = info.resolution
        if res <= 0.0:
            return False

        gx = int((wx - info.origin.position.x) / res)
        gy = int((wy - info.origin.position.y) / res)
        h = int(info.height)
        w = int(info.width)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False

        clearance_cells = max(1, int(self.explore_map_clearance_m / res))
        data = msg.data
        for ddy in range(-clearance_cells, clearance_cells + 1):
            for ddx in range(-clearance_cells, clearance_cells + 1):
                nx = gx + ddx
                ny = gy + ddy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                idx = ny * w + nx
                cell = int(data[idx])
                if cell < 0:
                    continue
                if cell >= self.explore_costmap_lethal_threshold:
                    return False
        return True

    def _is_tracking_forward_local_costmap_clear(self, rx, ry, yaw):
        if self._retreat_reverse_costmap is None:
            return False

        check_dist = max(0.0, self.tracking_far_safety_check_dist)
        lateral = max(0.0, self.tracking_far_safety_lateral_m)
        if check_dist <= 0.0:
            return True

        for scale in (0.35, 0.6, 1.0):
            step = check_dist * scale
            cx = rx + step * math.cos(yaw)
            cy = ry + step * math.sin(yaw)
            for side in (-lateral, 0.0, lateral):
                wx = cx + (-math.sin(yaw)) * side
                wy = cy + ( math.cos(yaw)) * side
                if not self._is_local_costmap_world_safe(wx, wy):
                    return False
        return True

    def _publish_tracking_far_forward_cmd(self, rx, ry, yaw, tx, ty):
        desired = math.atan2(ty - ry, tx - rx)
        yaw_err = self._normalize_angle(desired - yaw)

        twist = Twist()
        twist.linear.x = max(0.0, self.tracking_far_forward_speed)
        twist.angular.z = max(
            -self.tracking_far_max_ang,
            min(self.tracking_far_max_ang, self.tracking_far_steer_gain * yaw_err),
        )
        self.cmd_vel_pub.publish(twist)

    def _tick_tracking_far_fallback(self, tx, ty, far_dist_metric, now):
        if not self.tracking_far_fallback_enabled:
            return False
        if far_dist_metric is None or far_dist_metric < self.tracking_far_distance_m:
            self._tracking_far_burst_until = rospy.Time(0)
            return False

        robot_info = self._get_robot_pose_in_frame(self.tracking_frame)
        if robot_info is None:
            return False
        rx, ry, tf_robot = robot_info
        yaw = self._extract_yaw(tf_robot)

        # Continue active short burst while front remains safe.
        if now < self._tracking_far_burst_until:
            if self._is_tracking_forward_local_costmap_clear(rx, ry, yaw):
                self._publish_tracking_far_forward_cmd(rx, ry, yaw, tx, ty)
                return True
            self._tracking_far_burst_until = rospy.Time(0)
            return False

        if self._tracking_zero_cmd_start is None:
            return False
        zero_elapsed = (now - self._tracking_zero_cmd_start).to_sec()
        if zero_elapsed < self.tracking_far_zero_cmd_timeout_s:
            return False

        if not self._is_tracking_forward_local_costmap_clear(rx, ry, yaw):
            rospy.logwarn_throttle(
                1.0,
                "[TargetFollower] Far fallback blocked by local costmap ahead; keep normal tracking",
            )
            return False

        self._tracking_far_burst_until = now + rospy.Duration(self.tracking_far_forward_burst_s)
        self._publish_tracking_far_forward_cmd(rx, ry, yaw, tx, ty)
        self._tracking_zero_cmd_start = rospy.Time.now()
        rospy.logwarn_throttle(
            1.0,
            "[TargetFollower] Far-range DWA stall %.1fs: safe forward burst v=%.2f m/s",
            zero_elapsed,
            self.tracking_far_forward_speed,
        )
        return True

    def _trackable_target_available(self):
        if not self._target_fresh():
            return False
        try:
            if self.last_target.header.frame_id != self.global_frame:
                tfm = self.tfbuf.lookup_transform(
                    self.global_frame,
                    self.last_target.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )
                target_g = transform_pose_stamped(self.last_target, tfm, self.global_frame)
            else:
                target_g = self.last_target
        except Exception:
            return False
        tx = target_g.pose.position.x
        ty = target_g.pose.position.y
        return not self._is_recent_dialogue_area(tx, ty)

    def _current_target_global_xy(self):
        if self.last_target is None:
            return None
        try:
            if self.last_target.header.frame_id != self.global_frame:
                tfm = self.tfbuf.lookup_transform(
                    self.global_frame,
                    self.last_target.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.1),
                )
                tg = transform_pose_stamped(self.last_target, tfm, self.global_frame)
            else:
                tg = self.last_target
            return (tg.pose.position.x, tg.pose.position.y)
        except Exception:
            return self._last_seen_target_xy

    def _is_blocked_failed_target(self, tx, ty):
        if self._blocked_target_xy is None:
            return False
        now = rospy.Time.now()
        if now >= self._blocked_target_until:
            self._blocked_target_xy = None
            return False
        bx, by = self._blocked_target_xy
        return math.hypot(tx - bx, ty - by) <= self.tracking_failure_retrack_block_radius_m

    def _start_tracking_failure_retreat(self, reason):
        if not self.tracking_failure_force_retreat_enabled:
            self._set_state("LOST")
            self._publish_result(False)
            self._start_reacquire_target(reason)
            return

        target_xy = self._current_target_global_xy()
        if target_xy is not None:
            self._blocked_target_xy = target_xy
            self._blocked_target_until = rospy.Time.now() + rospy.Duration(
                max(1.0, self.tracking_failure_retrack_block_s)
            )

        self._force_retreat_active = True
        self._force_retreat_min_displacement_m = max(
            0.0, self.tracking_failure_min_retreat_displacement_m)
        robot_info = self._get_robot_pose_in_global()
        self._force_retreat_start_xy = (robot_info[0], robot_info[1]) if robot_info is not None else None
        self._set_state("FAILED")
        self._publish_result(False)
        self._start_retreat(reason="tracking_failed_%s" % reason)

    def _start_retreat(self, reason="unknown"):
        """Start retreat: reverse-first (if safe) then in-place turn away."""
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
        self._retreat_reverse_remaining_m = max(0.0, self.retreat_reverse_distance)
        self._retreat_reverse_last_xy = (rx, ry)
        self._retreat_advance_remaining_m = 0.0
        self._retreat_advance_last_xy = None
        can_reverse, reverse_reason = self._can_reverse_retreat(
            rx, ry, current_yaw, self._retreat_reverse_remaining_m)
        if can_reverse:
            self._retreat_phase = "BACKING"
        else:
            self._retreat_phase = "TURNING"
            rospy.loginfo("[TargetFollower] Skip reverse retreat: %s", reverse_reason)
        self._retreat_reason = reason
        self._active_goal_kind = "RETREATING"

        if self._retreat_phase == "BACKING":
            rospy.loginfo(
                "[TargetFollower] Retreat phase 1 (%s): reverse %.2fm at %.2f m/s",
                reason,
                self.retreat_reverse_distance,
                self.retreat_reverse_speed,
            )
        else:
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
        if (not self._force_retreat_active) and self._trackable_target_available():
            rospy.loginfo("[TargetFollower] Retreat interrupted by new target — resume tracking")
            self._finish_retreat(reason_override="target_detected")
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

        if self._retreat_phase == "BACKING":
            self._tick_retreat_backing()
            return
        if self._retreat_phase == "TURNING":
            self._tick_retreat_turning()
            return
        if self._retreat_phase == "ADVANCING":
            self._tick_retreat_advancing()

    def _tick_retreat_backing(self):
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            rospy.logwarn_throttle(1.0, "[TargetFollower] No robot pose during reverse retreat")
            return
        rx, ry, tf_robot = robot_info
        current_yaw = self._extract_yaw(tf_robot)
        if self._retreat_reverse_last_xy is not None:
            moved = math.hypot(rx - self._retreat_reverse_last_xy[0], ry - self._retreat_reverse_last_xy[1])
            if moved > 0.0:
                self._retreat_reverse_remaining_m = max(0.0, self._retreat_reverse_remaining_m - moved)
        self._retreat_reverse_last_xy = (rx, ry)

        if self._retreat_reverse_remaining_m <= 0.02:
            self._stop_cmd_vel()
            self._retreat_phase = "TURNING"
            rospy.loginfo("[TargetFollower] Retreat reverse complete — switching to turn phase")
            return

        can_reverse, reason = self._can_reverse_retreat(
            rx, ry, current_yaw, self._retreat_reverse_remaining_m)
        if not can_reverse:
            self._stop_cmd_vel()
            self._retreat_phase = "TURNING"
            rospy.logwarn_throttle(1.0, "[TargetFollower] Retreat reverse stopped: %s", reason)
            return

        twist = Twist()
        twist.linear.x = -self.retreat_reverse_speed
        self.cmd_vel_pub.publish(twist)

    def _tick_retreat_turning(self):
        """Retreat turn phase: publish cmd_vel rotation until turn is complete."""
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return

        _, _, tf_robot = robot_info
        current_yaw = self._extract_yaw(tf_robot)
        yaw_error = self._normalize_angle(self._retreat_target_yaw - current_yaw)

        if abs(yaw_error) < math.radians(self.retreat_turn_tolerance_deg):
            if self._force_retreat_active and self._force_retreat_start_xy is not None:
                moved = math.hypot(
                    robot_info[0] - self._force_retreat_start_xy[0],
                    robot_info[1] - self._force_retreat_start_xy[1],
                )
                remain = max(0.0, self._force_retreat_min_displacement_m - moved)
                if remain > 0.02:
                    self._retreat_phase = "ADVANCING"
                    self._retreat_advance_remaining_m = remain
                    self._retreat_advance_last_xy = (robot_info[0], robot_info[1])
                    rospy.loginfo(
                        "[TargetFollower] Retreat turning complete; force-advancing %.2fm to satisfy min displacement",
                        remain,
                    )
                    return
            rospy.loginfo(
                "[TargetFollower] Retreat turn complete (error=%.1f°) — finishing retreat",
                math.degrees(yaw_error),
            )
            self._stop_cmd_vel()
            self._finish_retreat()
            return

        # Publish rotation command (turn in shortest direction)
        twist = Twist()
        twist.angular.z = self.retreat_turn_speed if yaw_error > 0 else -self.retreat_turn_speed
        self.cmd_vel_pub.publish(twist)

    def _tick_retreat_advancing(self):
        robot_info = self._get_robot_pose_in_global()
        if robot_info is None:
            return
        rx, ry, tf_robot = robot_info
        yaw = self._extract_yaw(tf_robot)

        if self._retreat_advance_last_xy is not None:
            moved = math.hypot(rx - self._retreat_advance_last_xy[0], ry - self._retreat_advance_last_xy[1])
            if moved > 0.0:
                self._retreat_advance_remaining_m = max(0.0, self._retreat_advance_remaining_m - moved)
        self._retreat_advance_last_xy = (rx, ry)

        if self._retreat_advance_remaining_m <= 0.02:
            self._stop_cmd_vel()
            self._finish_retreat()
            return

        if not self._is_tracking_forward_local_costmap_clear(rx, ry, yaw):
            rospy.logwarn_throttle(
                1.0,
                "[TargetFollower] Forced retreat advance blocked by local costmap; finishing with block cooldown",
            )
            self._stop_cmd_vel()
            self._finish_retreat(reason_override="force_advance_blocked")
            return

        twist = Twist()
        twist.linear.x = min(0.12, max(0.05, self.retreat_reverse_speed))
        self.cmd_vel_pub.publish(twist)

    def _finish_retreat(self, reason_override=None):
        """Called after retreat goal completes or times out."""
        final_reason = reason_override if reason_override is not None else self._retreat_reason
        rospy.loginfo("[TargetFollower] Retreat complete (%s) — returning to IDLE and resuming explore", final_reason)
        if self._force_retreat_active:
            if self._blocked_target_xy is not None:
                self._blocked_target_until = rospy.Time.now() + rospy.Duration(
                    max(0.5, self.tracking_failure_retrack_cooldown_s)
                )
            self._force_retreat_active = False
            self._force_retreat_min_displacement_m = 0.0
            self._force_retreat_start_xy = None
            self._retreat_advance_remaining_m = 0.0
            self._retreat_advance_last_xy = None
        self._retreat_post_target_grace_until = rospy.Time.now() + rospy.Duration(
            max(0.0, self.retreat_post_target_grace_s)
        )
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
        self._retreat_reverse_remaining_m = 0.0
        self._retreat_reverse_last_xy = None
        self._retreat_advance_remaining_m = 0.0
        self._retreat_advance_last_xy = None
        self._force_retreat_active = False
        self._force_retreat_min_displacement_m = 0.0
        self._force_retreat_start_xy = None
        self._close_approach_start = None
        self._ca_last_good_depth   = None
        self._ca_depth_jump_candidate = None
        self._ca_depth_jump_count = 0
        self._explore_goal_start   = None
        self._explore_start_xy     = None
        self._explore_zero_cmd_start = None
        self._explore_scan_start   = None
        self._explore_scan_end     = None
        self._explore_scan_mode    = None
        self._explore_scan_target_yaw = None
        self._reacquire_start      = None
        self._target_lost_logged   = False
        self._suppress_done_cb     = False
        self._tracking_resend_hold_until = rospy.Time(0)
        self._near_abort_retry_count = 0
        self._near_abort_retry_window_start = None
        self._last_reach_dist      = None
        self._last_reach_dist_time = None
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
        self._ca_depth_jump_candidate = None
        self._ca_depth_jump_count = 0
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
        stale_age = None
        if self.target_timeout_use_rx_time and self._last_target_rx_time is not None:
            stale_age = (now - self._last_target_rx_time).to_sec()
        elif self.last_target.header.stamp != rospy.Time(0):
            stale_age = (now - self.last_target.header.stamp).to_sec()
        if stale_age is not None and stale_age > (self.target_timeout_s + self.close_approach_stale_hold_s):
            rospy.logwarn(
                "[TargetFollower] CLOSE_APPROACH target stale (%.2fs) — stopping",
                stale_age,
            )
            self._stop_cmd_vel()
            self.last_target = None
            self._set_state("LOST")
            self._publish_result(False)
            self._start_reacquire_target("close_approach_timeout")
            return

        # ── Read depth with jump filter ──────────────────────────────────
        raw_d_cam_z = self._get_target_depth_in_camera()
        d_cam_z = raw_d_cam_z

        if d_cam_z is not None and self._ca_last_good_depth is not None:
            jump = d_cam_z - self._ca_last_good_depth
            if jump > self.close_approach_max_depth_jump:
                consistent = (
                    self._ca_depth_jump_candidate is not None
                    and abs(d_cam_z - self._ca_depth_jump_candidate)
                    <= self.close_approach_depth_jump_consistency_m
                )
                if not consistent:
                    self._ca_depth_jump_candidate = d_cam_z
                    self._ca_depth_jump_count = 1
                else:
                    self._ca_depth_jump_count += 1

                if self._ca_depth_jump_count >= max(1, self.close_approach_depth_jump_accept_count):
                    rospy.logwarn(
                        "[TargetFollower] CLOSE_APPROACH accepting persistent depth jump: "
                        "new=%.3f old=%.3f count=%d",
                        d_cam_z,
                        self._ca_last_good_depth,
                        self._ca_depth_jump_count,
                    )
                    self._ca_last_good_depth = d_cam_z
                    self._ca_depth_jump_candidate = None
                    self._ca_depth_jump_count = 0
                else:
                    rospy.logwarn(
                        "[TargetFollower] CLOSE_APPROACH depth jump filtered: "
                        "raw=%.3f  last_good=%.3f  jump=+%.3f > %.2f (stable %d/%d)",
                        d_cam_z,
                        self._ca_last_good_depth,
                        jump,
                        self.close_approach_max_depth_jump,
                        self._ca_depth_jump_count,
                        max(1, self.close_approach_depth_jump_accept_count),
                    )
                    d_cam_z = self._ca_last_good_depth
            else:
                self._ca_last_good_depth = d_cam_z
                self._ca_depth_jump_candidate = None
                self._ca_depth_jump_count = 0
        elif d_cam_z is not None:
            # First reading — accept it
            self._ca_last_good_depth = d_cam_z
            self._ca_depth_jump_candidate = None
            self._ca_depth_jump_count = 0

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
                self._on_reached()
                return

        # ── Steering: proportional controller on camera x-offset ─────────
        # camera_link optical convention: x = right, so target at positive x
        # means it's to the right → robot should turn right (negative angular.z)
        cam_x = 0.0
        if self.last_target is not None and self.last_target.header.frame_id == self.camera_frame:
            cam_x = self.last_target.pose.position.x

        twist = Twist()
        twist.linear.x = self.close_approach_speed
        twist.angular.z = -self.close_approach_steer_gain * cam_x
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

        if not self.tracking_client.wait_for_server(rospy.Duration(0.05)):
            rospy.logwarn_throttle(
                2.0,
                "[TargetFollower] waiting for tracking action server %s",
                self.tracking_action_name,
            )
            return

        now = rospy.Time.now()
        age_header = None
        if self.last_target.header.stamp != rospy.Time(0):
            age_header = (now - self.last_target.header.stamp).to_sec()
        age_rx = None
        if self._last_target_rx_time is not None:
            age_rx = (now - self._last_target_rx_time).to_sec()

        stale_age = age_rx if self.target_timeout_use_rx_time else age_header
        if stale_age is None:
            stale_age = age_header if age_header is not None else age_rx

        if stale_age is not None and stale_age > self.target_timeout_s:
            stale_over = stale_age - self.target_timeout_s
            near_tracking_with_recent_depth = False
            if (
                self._state == "TRACKING"
                and self._goal_active
                and self._active_goal_kind == "TRACKING"
                and self._last_reach_dist is not None
                and self._last_reach_dist_time is not None
                and (now - self._last_reach_dist_time).to_sec() <= self.target_timeout_near_dist_fresh_s
                and self._last_reach_dist <= self.target_timeout_near_dist_m
                and stale_over <= self.target_timeout_near_grace_s
            ):
                near_tracking_with_recent_depth = True

            tracking_hold_active = (
                self._state == "TRACKING"
                and self._goal_active
                and self._active_goal_kind == "TRACKING"
                and stale_over <= self.tracking_stale_hold_s
            )

            if near_tracking_with_recent_depth:
                rospy.logwarn_throttle(
                    1.0,
                    "Target stale near goal (age=%.2fs); keep TRACKING goal for grace %.1fs",
                    stale_age,
                    self.target_timeout_near_grace_s,
                )
                return

            if tracking_hold_active:
                rospy.logwarn_throttle(
                    1.0,
                    "Target stream stale (age=%.2fs); keep active TRACKING goal for hold %.1fs",
                    stale_age,
                    self.tracking_stale_hold_s,
                )
                return

            if not self._target_lost_logged:
                rospy.logwarn(
                    "Target pose stale (age=%.2fs > %.2fs) — clearing stale target",
                    stale_age, self.target_timeout_s,
                )
                self._target_lost_logged = True
                self._target_seen_since = None
            # Clear the stale target so exploration can continue normally.
            # Only active tracking goals should be cancelled here; otherwise
            # a stale detection can repeatedly preempt exploration and leave
            # the robot stuck facing the same obstacle.
            was_tracking = self._state == "TRACKING"
            if self._active_goal_kind == "TRACKING":
                self._cancel_goal_if_active()
            if was_tracking:
                self._start_tracking_failure_retreat("target_timeout")
            else:
                self.last_target = None
            return

        if self._active_goal_kind == "EXPLORING" and self._goal_active:
            if not self._target_confirmed_for_explore_interrupt():
                return
            self._cancel_goal_if_active()
            self._set_state("IDLE")
        elif self._explore_scan_end is not None:
            if not self._target_confirmed_for_explore_interrupt():
                return
            self._stop_explore_scan()

        try:
            target_t = self._transform_pose_to_frame(self.last_target, self.tracking_frame)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF not ready for target transform: %s", str(e))
            return

        tx = target_t.pose.position.x
        ty = target_t.pose.position.y

        # If this target is near a recent dialogue point, skip it for a while
        # to avoid repeatedly asking the same person/object.
        target_g = None
        try:
            target_g = self._transform_pose_to_frame(self.last_target, self.global_frame)
        except Exception:
            pass
        block_tx = target_g.pose.position.x if target_g is not None else tx
        block_ty = target_g.pose.position.y if target_g is not None else ty
        if self._is_blocked_failed_target(block_tx, block_ty):
            rospy.loginfo_throttle(
                1.0,
                "[TargetFollower] Blocking re-track of failed target area (%.2f, %.2f)",
                block_tx,
                block_ty,
            )
            return
        if self._should_block_dialogue_area_target(block_tx, block_ty):
            rospy.loginfo_throttle(
                2.0,
                "[TargetFollower] Ignoring target near recent dialogue area (%.2f, %.2f)",
                block_tx, block_ty,
            )
            return

        active_send_rate_hz = self.send_rate_hz
        active_min_update_dist = self.min_update_dist
        force_goal_to_target = False
        far_tracking = False

        # ── Standoff computation ──────────────────────────────────────────
        goal_x, goal_y = tx, ty

        if self.standoff_distance > 0 or self.face_target:
            robot_info = self._get_robot_pose_in_frame(self.tracking_frame)
            if robot_info is None:
                return
            rx, ry, tf_robot = robot_info

            # Distance from base_link to target in tracking_frame (odom for local-only tracking).
            dx = tx - rx
            dy = ty - ry
            d  = math.hypot(dx, dy)

            # ── REACHED check: camera_link z-depth ────────────────────────
            # The depth camera reports the target's z-coordinate in
            # camera_link (optical convention: z = forward / depth).
            # This is the most direct and reliable distance metric.
            d_cam_z = self._get_target_depth_in_camera()

            # Also compute 2D tracking-frame camera distance for goal offset.
            cam_pos = self._get_camera_pose_in_frame(self.tracking_frame)
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
                self._last_reach_dist = reach_dist
                self._last_reach_dist_time = now
                reach_threshold = self.standoff_distance
                if reach_dist <= reach_threshold:
                    if self._state not in ("REACHED", "WAITING_ACTION"):
                        rospy.loginfo(
                            "[TargetFollower] Within standoff "
                            "(d_cam_z=%s <= %.2f) — REACHED",
                            "%.3f" % reach_dist, reach_threshold,
                        )
                        self._on_reached()
                    return

                if reach_dist <= self.tracking_near_target_window_m:
                    active_send_rate_hz = max(self.send_rate_hz, self.tracking_near_send_rate_hz)
                    active_min_update_dist = min(
                        self.min_update_dist,
                        self.tracking_near_min_update_dist,
                    )
                    if (not self.enable_close_approach) and self.tracking_near_goal_to_target:
                        force_goal_to_target = True
                else:
                    far_tracking = True
                    self._near_abort_retry_count = 0
                    self._near_abort_retry_window_start = None

                # Keep obstacle avoidance on unless explicitly enabled.
                if reach_dist < self.close_approach_threshold:
                    if self.enable_close_approach:
                        rospy.loginfo(
                            "[TargetFollower] Target within close-approach zone "
                            "(d=%.3f < %.2f) — switching to CLOSE_APPROACH",
                            reach_dist, self.close_approach_threshold,
                        )
                        self._start_close_approach()
                        return
                    rospy.loginfo_throttle(
                        2.0,
                        "[TargetFollower] close_approach disabled; keep move_base tracking in near-target zone",
                    )

                if force_goal_to_target:
                    goal_x, goal_y = tx, ty
                else:
                    # Goal waypoint: offset from base_link so that *camera*
                    # depth ends up at standoff.  Camera is roughly
                    # (d - d_cam_xy) metres closer to target along the
                    # robot→target line.
                    if d > 1e-6:
                        cam_ahead = d - d_cam_xy  # positive when camera is ahead
                        base_standoff = self.standoff_distance + cam_ahead
                        if base_standoff >= d:
                            base_standoff = d * 0.9  # safety: don't send goal behind robot
                        ratio  = (d - base_standoff) / d
                        goal_x = rx + dx * ratio
                        goal_y = ry + dy * ratio

                # Far-range only fallback: if tracking is stalled but local
                # costmap ahead is clear, push forward briefly toward target.
                if self._tick_tracking_far_fallback(tx, ty, reach_dist, now):
                    return

        if far_tracking:
            far_rate_hz = 1.0 / max(0.1, self.tracking_far_resend_interval_s)
            active_send_rate_hz = min(active_send_rate_hz, far_rate_hz)
            active_min_update_dist = max(active_min_update_dist, self.tracking_far_min_update_dist)

        # When tracking is stalled (near-zero cmd_vel), temporarily hold goal
        # re-sends so DWA can continue evaluating one plan instead of frequent
        # preempt/reset churn from tiny target jitter.
        if (
            self._state == "TRACKING"
            and self._goal_active
            and self._active_goal_kind == "TRACKING"
            and self._tracking_zero_cmd_start is not None
        ):
            zero_elapsed = (now - self._tracking_zero_cmd_start).to_sec()
            if zero_elapsed >= max(0.0, self.tracking_goal_hold_zero_cmd_s):
                allow_hold = (not self.tracking_goal_hold_far_only) or far_tracking
                if allow_hold:
                    hold_until = now + rospy.Duration(max(0.0, self.tracking_goal_hold_on_stall_s))
                    if hold_until > self._tracking_resend_hold_until:
                        self._tracking_resend_hold_until = hold_until
                    rospy.loginfo_throttle(
                        1.0,
                        "[TargetFollower] TRACKING stall %.1fs — hold goal re-send for %.1fs",
                        zero_elapsed,
                        max(0.0, self.tracking_goal_hold_on_stall_s),
                    )

        if self._state == "TRACKING" and now < self._tracking_resend_hold_until:
            return

        # Rate limiter — but allow immediate re-send after FAILED/LOST/IDLE
        if self._state == "TRACKING":
            if (now - self.last_send_time).to_sec() < (1.0 / max(active_send_rate_hz, 0.1)):
                return

        # While a tracking goal is active, avoid over-preempting with tiny goal
        # shifts. Once the goal is terminal, allow immediate re-send.
        if self.last_sent_goal is not None and self._state == "TRACKING" and self._goal_active:
            if dist_xy_pts(goal_x, goal_y, self.last_sent_goal[0], self.last_sent_goal[1]) < active_min_update_dist:
                return

        # ── Build MoveBaseGoal ────────────────────────────────────────────
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.header.frame_id = self.tracking_frame
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0

        if self.face_target:
            face_dx = tx - goal_x
            face_dy = ty - goal_y
            if math.hypot(face_dx, face_dy) > 0.01:
                yaw = math.atan2(face_dy, face_dx)
            else:
                robot_info = self._get_robot_pose_in_frame(self.tracking_frame)
                yaw = math.atan2(ty - robot_info[1], tx - robot_info[0]) if robot_info else 0.0
            goal.target_pose.pose.orientation = self._yaw_to_quaternion(yaw)
        elif self.use_target_orientation:
            goal.target_pose.pose.orientation = target_t.pose.orientation
        else:
            robot_info = self._get_robot_pose_in_frame(self.tracking_frame)
            if robot_info is not None:
                goal.target_pose.pose.orientation = robot_info[2].transform.rotation
            else:
                goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self.tracking_client.send_goal(
            goal,
            done_cb=partial(self._goal_done_cb, goal_kind="TRACKING"),
        )
        self._goal_active  = True
        self._active_goal_kind = "TRACKING"
        self.last_sent_goal = (goal_x, goal_y)
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

            # Short local search near last seen target before falling back to frontier exploration
            if self._state == "REACQUIRE_TARGET":
                self._tick_reacquire_target()

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
