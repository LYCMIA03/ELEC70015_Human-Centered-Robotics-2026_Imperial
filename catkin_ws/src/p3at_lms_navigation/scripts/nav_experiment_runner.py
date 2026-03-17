#!/usr/bin/env python3
"""
Run sequential waypoint navigation experiments and export structured metrics.

Usage example:
  rosrun p3at_lms_navigation nav_experiment_runner.py \
    --scenario simple \
    --output /tmp/simple_nav_metrics.json
"""

import argparse
import json
import math
import os
import time

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry


SCENARIOS = {
    "simple": [
        {"name": "WP1_open_south", "x": 0.0, "y": -2.0},
        {"name": "WP2_obstacle_detour", "x": 3.5, "y": 2.0},
        {"name": "WP3_far_east", "x": 5.0, "y": 0.0},
    ],
    "complex": [
        {"name": "WP1_south", "x": 0.0, "y": -1.5},
        {"name": "WP2_north", "x": 0.0, "y": 1.5},
        {"name": "WP3_west", "x": -1.5, "y": 0.0},
        {"name": "WP4_east_s", "x": 1.5, "y": -0.5},
        {"name": "WP5_return", "x": 0.0, "y": 0.0},
    ],
}


def parse_custom_waypoints(raw):
    """
    Parse waypoints from:
      "name:x:y;name2:x2:y2"
    or
      "x:y;x2:y2" (auto naming)
    """
    out = []
    for i, chunk in enumerate(raw.split(";"), start=1):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(":")
        if len(parts) == 2:
            name = f"WP{i}"
            x_str, y_str = parts
        elif len(parts) == 3:
            name, x_str, y_str = parts
        else:
            raise ValueError(f"Invalid waypoint format: {chunk}")
        out.append({"name": name, "x": float(x_str), "y": float(y_str)})
    if not out:
        raise ValueError("No valid waypoints provided.")
    return out


class NavExperimentRunner:
    def __init__(self, global_frame, robot_frame):
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_distance_total = 0.0
        self._last_odom_xy = None
        self._odom_samples = 0
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=50)

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._odom_samples += 1
        if self._last_odom_xy is None:
            self._last_odom_xy = (x, y)
            return
        dx = x - self._last_odom_xy[0]
        dy = y - self._last_odom_xy[1]
        self.path_distance_total += math.hypot(dx, dy)
        self._last_odom_xy = (x, y)

    def wait_ready(self, move_base_timeout_s, tf_timeout_s):
        rospy.loginfo("Waiting for move_base server...")
        deadline = time.time() + move_base_timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.client.wait_for_server(rospy.Duration(0.3)):
                break
        else:
            raise RuntimeError(
                f"move_base action server not ready after {move_base_timeout_s}s")

        rospy.loginfo("Waiting for TF %s -> %s ...", self.global_frame, self.robot_frame)
        deadline = time.time() + tf_timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.2))
                return
            except Exception:
                rospy.sleep(0.2)
        raise RuntimeError(
            f"TF {self.global_frame}->{self.robot_frame} not ready after {tf_timeout_s}s")

    def get_robot_pose(self):
        tf_stamped = self.tf_buffer.lookup_transform(
            self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.5))
        x = tf_stamped.transform.translation.x
        y = tf_stamped.transform.translation.y
        return x, y

    def run_waypoint(self, wp, goal_timeout_s):
        start_ts = time.time()
        start_pose = self.get_robot_pose()
        start_path = self.path_distance_total

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        deadline = time.time() + goal_timeout_s
        finished = False
        state = self.client.get_state()
        while not rospy.is_shutdown() and time.time() < deadline:
            state = self.client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                finished = True
                break
            time.sleep(0.25)

        if not finished:
            self.client.cancel_goal()
            state = self.client.get_state()

        end_pose = self.get_robot_pose()
        end_ts = time.time()

        goal_err = math.hypot(end_pose[0] - wp["x"], end_pose[1] - wp["y"])
        direct_dist = math.hypot(wp["x"] - start_pose[0], wp["y"] - start_pose[1])
        traveled_dist = max(0.0, self.path_distance_total - start_path)

        success = bool(finished and state == GoalStatus.SUCCEEDED)
        efficiency = (
            (direct_dist / traveled_dist) if traveled_dist > 1e-6 else 0.0
        )

        return {
            "name": wp["name"],
            "target_xy": [wp["x"], wp["y"]],
            "success": success,
            "goal_state_code": int(state),
            "goal_state": GoalStatus.to_string(state),
            "start_xy": [start_pose[0], start_pose[1]],
            "end_xy": [end_pose[0], end_pose[1]],
            "duration_s": end_ts - start_ts,
            "goal_error_m": goal_err,
            "direct_distance_m": direct_dist,
            "traveled_distance_m": traveled_dist,
            "path_efficiency": efficiency,
            "timeout_s": goal_timeout_s,
        }


def build_summary(results):
    if not results:
        return {
            "waypoints_total": 0,
            "waypoints_succeeded": 0,
            "success_rate": 0.0,
        }

    succ = [r for r in results if r["success"]]
    durations = [r["duration_s"] for r in results]
    errs = [r["goal_error_m"] for r in results]
    traveled = [r["traveled_distance_m"] for r in results]
    efficiency = [r["path_efficiency"] for r in results if r["path_efficiency"] > 0]

    return {
        "waypoints_total": len(results),
        "waypoints_succeeded": len(succ),
        "success_rate": len(succ) / float(len(results)),
        "mean_duration_s": sum(durations) / len(durations),
        "max_duration_s": max(durations),
        "mean_goal_error_m": sum(errs) / len(errs),
        "max_goal_error_m": max(errs),
        "mean_traveled_distance_m": sum(traveled) / len(traveled),
        "total_traveled_distance_m": sum(traveled),
        "mean_path_efficiency": (
            sum(efficiency) / len(efficiency) if efficiency else 0.0
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=sorted(SCENARIOS.keys()),
                        default="simple")
    parser.add_argument(
        "--waypoints",
        default="",
        help="Custom waypoints: 'name:x:y;name2:x2:y2' or 'x:y;x2:y2'",
    )
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--robot-frame", default="base_link")
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--move-base-timeout", type=float, default=60.0)
    parser.add_argument("--tf-timeout", type=float, default=30.0)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--output", required=True)
    args, _ = parser.parse_known_args()

    rospy.init_node("nav_experiment_runner", anonymous=True)

    if args.waypoints.strip():
        waypoints = parse_custom_waypoints(args.waypoints)
        scenario_name = "custom"
    else:
        waypoints = SCENARIOS[args.scenario]
        scenario_name = args.scenario

    runner = NavExperimentRunner(
        global_frame=args.global_frame,
        robot_frame=args.robot_frame,
    )

    runner.wait_ready(args.move_base_timeout, args.tf_timeout)
    rospy.loginfo("Warmup %.1fs for map/costmap stabilization...", args.warmup)
    end_warmup = time.time() + args.warmup
    while not rospy.is_shutdown() and time.time() < end_warmup:
        time.sleep(0.1)

    run_started = time.time()
    results = []

    for idx, wp in enumerate(waypoints, start=1):
        rospy.loginfo(
            "[NavExp] Goal %d/%d %s -> (%.2f, %.2f)",
            idx, len(waypoints), wp["name"], wp["x"], wp["y"])
        results.append(runner.run_waypoint(wp, args.goal_timeout))
        time.sleep(1.0)

    payload = {
        "experiment_type": "fixed_point_navigation",
        "scenario": scenario_name,
        "frames": {
            "global_frame": args.global_frame,
            "robot_frame": args.robot_frame,
        },
        "timestamps": {
            "run_start_unix_s": run_started,
            "run_end_unix_s": time.time(),
        },
        "config": {
            "goal_timeout_s": args.goal_timeout,
            "warmup_s": args.warmup,
            "waypoints": waypoints,
        },
        "summary": build_summary(results),
        "waypoint_results": results,
        "odom_samples": runner._odom_samples,
        "odom_path_distance_total_m": runner.path_distance_total,
    }

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    rospy.loginfo("[NavExp] Metrics saved to %s", args.output)
    rospy.loginfo(
        "[NavExp] success %.1f%% (%d/%d), mean error %.3fm",
        100.0 * payload["summary"]["success_rate"],
        payload["summary"]["waypoints_succeeded"],
        payload["summary"]["waypoints_total"],
        payload["summary"]["mean_goal_error_m"],
    )


if __name__ == "__main__":
    main()
