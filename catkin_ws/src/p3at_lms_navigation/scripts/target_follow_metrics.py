#!/usr/bin/env python3
"""
Collect quantitative metrics for Gazebo target-following experiments.
"""

import argparse
import json
import math
import os
import statistics
import time
from collections import defaultdict
from threading import Lock

import rospy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


def _pose_xy(pose):
    return pose.position.x, pose.position.y


class TargetFollowMetrics:
    def __init__(self, robot_model, target_model):
        self.robot_model = robot_model
        self.target_model = target_model
        self.lock = Lock()

        self.distances = []
        self.samples = 0
        self.missing_model_samples = 0

        self.robot_speed_samples = []
        self.target_speed_samples = []
        self._prev_robot = None
        self._prev_target = None
        self._prev_t = None
        self.robot_path_len = 0.0
        self.target_path_len = 0.0
        self.motion_time_s = 0.0

        self.status_current = "UNKNOWN"
        self.status_enter_t = None
        self.status_time_s = defaultdict(float)
        self.status_events = []
        self.result_events = []

        self.latest_msg_time = None
        self.clock_now_s = None

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb_model_states, queue_size=20)
        rospy.Subscriber("/target_follower/status", String, self.cb_status, queue_size=50)
        rospy.Subscriber("/target_follower/result", Bool, self.cb_result, queue_size=50)
        rospy.Subscriber("/clock", Clock, self.cb_clock, queue_size=50)

    def cb_clock(self, msg):
        with self.lock:
            self.clock_now_s = msg.clock.to_sec()

    @staticmethod
    def _wall_now_s():
        return time.time()

    def _now_s(self):
        # Prefer explicit /clock time even when /use_sim_time is false.
        # Fall back to wall time before /clock starts.
        if self.clock_now_s is not None and self.clock_now_s > 0.0:
            return self.clock_now_s
        return self._wall_now_s()

    def cb_status(self, msg):
        with self.lock:
            now = self._now_s()
            if self.status_enter_t is None:
                self.status_enter_t = now
                self.status_current = msg.data
                self.status_events.append({"time_unix_s": now, "status": msg.data})
                return

            elapsed = max(0.0, now - self.status_enter_t)
            self.status_time_s[self.status_current] += elapsed
            self.status_current = msg.data
            self.status_enter_t = now
            self.status_events.append({"time_unix_s": now, "status": msg.data})

    def cb_result(self, msg):
        with self.lock:
            self.result_events.append({
                "time_unix_s": self._now_s(),
                "result": bool(msg.data),
            })

    def cb_model_states(self, msg):
        with self.lock:
            now = self._now_s()
            self.latest_msg_time = now
            self.samples += 1

            try:
                ridx = msg.name.index(self.robot_model)
                tidx = msg.name.index(self.target_model)
            except ValueError:
                self.missing_model_samples += 1
                return

            rx, ry = _pose_xy(msg.pose[ridx])
            tx, ty = _pose_xy(msg.pose[tidx])
            d = math.hypot(tx - rx, ty - ry)
            self.distances.append(d)

            if self._prev_t is not None:
                dt = now - self._prev_t
                if dt > 1e-3:
                    if self._prev_robot is not None:
                        dr = math.hypot(rx - self._prev_robot[0], ry - self._prev_robot[1])
                        v_r = dr / dt
                        self.robot_speed_samples.append(v_r)
                        self.robot_path_len += dr
                    if self._prev_target is not None:
                        dtg = math.hypot(tx - self._prev_target[0], ty - self._prev_target[1])
                        v_t = dtg / dt
                        self.target_speed_samples.append(v_t)
                        self.target_path_len += dtg
                    self.motion_time_s += dt

            self._prev_robot = (rx, ry)
            self._prev_target = (tx, ty)
            self._prev_t = now

    def finalize_status(self):
        with self.lock:
            now = self._now_s()
            if self.status_enter_t is not None:
                self.status_time_s[self.status_current] += max(0.0, now - self.status_enter_t)


def _safe_mean(values):
    return sum(values) / len(values) if values else 0.0


def _safe_std(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def summarize(metrics, standoff, tolerance, run_start, run_end):
    with metrics.lock:
        d = list(metrics.distances)
        robot_speed_samples = list(metrics.robot_speed_samples)
        target_speed_samples = list(metrics.target_speed_samples)
        robot_path_len = float(metrics.robot_path_len)
        target_path_len = float(metrics.target_path_len)
        motion_time_s = float(metrics.motion_time_s)
        result_events = list(metrics.result_events)
        status_time_s = dict(metrics.status_time_s)
        missing_model_samples = metrics.missing_model_samples
        samples = metrics.samples

    status_total = sum(status_time_s.values())

    summary = {
        "sample_count": len(d),
        "missing_model_samples": missing_model_samples,
        "sample_dropout_rate": (
            missing_model_samples / float(samples)
            if samples > 0 else 0.0
        ),
        "distance_mean_m": _safe_mean(d),
        "distance_std_m": _safe_std(d),
        "distance_min_m": min(d) if d else 0.0,
        "distance_max_m": max(d) if d else 0.0,
        "distance_median_m": statistics.median(d) if d else 0.0,
        "standoff_target_m": standoff,
        "standoff_rmse_m": (
            math.sqrt(_safe_mean([(x - standoff) ** 2 for x in d])) if d else 0.0
        ),
        "within_tolerance_rate": (
            sum(1 for x in d if abs(x - standoff) <= tolerance) / float(len(d))
            if d else 0.0
        ),
        # Use time-weighted mean speed (path length / elapsed) so high-rate
        # repeated identical poses do not bias speed downward.
        "robot_speed_mean_mps": (
            robot_path_len / motion_time_s if motion_time_s > 1e-6 else 0.0
        ),
        "target_speed_mean_mps": (
            target_path_len / motion_time_s if motion_time_s > 1e-6 else 0.0
        ),
        # Keep legacy unweighted means for debugging/comparison.
        "robot_speed_mean_unweighted_mps": _safe_mean(robot_speed_samples),
        "target_speed_mean_unweighted_mps": _safe_mean(target_speed_samples),
        "motion_time_s": motion_time_s,
        "result_true_count": sum(1 for e in result_events if e["result"]),
        "result_false_count": sum(1 for e in result_events if not e["result"]),
        "status_time_s": status_time_s,
        "status_time_ratio": {
            k: (v / status_total if status_total > 0 else 0.0)
            for k, v in status_time_s.items()
        },
        "run_duration_s": max(0.0, run_end - run_start),
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-model", default="p3at")
    parser.add_argument("--target-model", default="target")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--standoff", type=float, default=1.0)
    parser.add_argument("--tolerance", type=float, default=0.30)
    parser.add_argument("--output", required=True)
    args, _ = parser.parse_known_args()

    rospy.init_node("target_follow_metrics", anonymous=True)
    collector = TargetFollowMetrics(args.robot_model, args.target_model)

    rospy.loginfo("Warmup %.1fs before measurement...", args.warmup)
    rospy.sleep(args.warmup)

    run_start = collector._now_s()
    end_time = run_start + args.duration
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and collector._now_s() < end_time:
        rate.sleep()
    run_end = collector._now_s()

    collector.finalize_status()
    summary = summarize(
        metrics=collector,
        standoff=args.standoff,
        tolerance=args.tolerance,
        run_start=run_start,
        run_end=run_end,
    )

    payload = {
        "experiment_type": "target_following",
        "config": {
            "robot_model": args.robot_model,
            "target_model": args.target_model,
            "duration_s": args.duration,
            "warmup_s": args.warmup,
            "standoff_m": args.standoff,
            "tolerance_m": args.tolerance,
        },
        "timestamps": {
            "run_start_unix_s": run_start,
            "run_end_unix_s": run_end,
        },
        "summary": summary,
        "status_events": collector.status_events,
        "result_events": collector.result_events,
    }

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    rospy.loginfo("[TargetFollowMetrics] saved to %s", args.output)
    rospy.loginfo(
        "[TargetFollowMetrics] mean dist=%.3fm, rmse=%.3fm, within_tol=%.1f%%",
        summary["distance_mean_m"],
        summary["standoff_rmse_m"],
        summary["within_tolerance_rate"] * 100.0,
    )


if __name__ == "__main__":
    main()
