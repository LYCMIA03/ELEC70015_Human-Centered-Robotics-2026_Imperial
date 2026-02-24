#!/usr/bin/env python3
"""
amcl_verifier.py — AMCL localization accuracy verification for P3-AT.

Compares AMCL estimated pose (from /amcl_pose) against Gazebo ground truth
(from /gazebo/model_states) while navigating a sequence of waypoints.

Generates a comprehensive report with:
  - Per-waypoint position & orientation error
  - Continuous tracking error statistics (mean, max, std)
  - Convergence time after initialization
  - Particle cloud spread analysis

Usage:
    roslaunch p3at_lms_navigation auto_amcl_verify.launch
    # The launch file starts this node automatically after map_server + AMCL.
"""

import rospy
import actionlib
import math
import numpy as np
import os
import sys
import json
import time
from threading import Lock

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty


def quaternion_to_yaw(q):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Shortest angular difference."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class AMCLVerifier:
    """Verifies AMCL localization accuracy against Gazebo ground truth."""

    def __init__(self):
        rospy.init_node('amcl_verifier', anonymous=False)

        # Parameters
        self.model_name = rospy.get_param('~model_name', 'p3at')
        self.goal_timeout = rospy.get_param('~goal_timeout', 120.0)
        self.settle_time = rospy.get_param('~settle_time', 5.0)
        self.report_path = rospy.get_param(
            '~report_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'maps', 'amcl_report.txt'))
        self.json_report_path = rospy.get_param(
            '~json_report_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'maps', 'amcl_report.json'))
        self.convergence_threshold = rospy.get_param('~convergence_threshold', 0.3)
        self.initial_wait = rospy.get_param('~initial_wait', 15.0)

        # Verification waypoints — cover different areas of the complex maze
        default_waypoints = [
            {"name": "WP1_center_south", "x": 0.0, "y": -1.0},
            {"name": "WP2_right_corridor", "x": 3.0, "y": 0.0},
            {"name": "WP3_bottom_right", "x": 4.0, "y": -3.5},
            {"name": "WP4_bottom_left", "x": -4.0, "y": -3.5},
            {"name": "WP5_upper_left_room", "x": -3.5, "y": 4.5},
            {"name": "WP6_upper_right_room", "x": 3.0, "y": 4.5},
            {"name": "WP7_return_origin", "x": 0.0, "y": 0.0},
        ]
        self.waypoints = rospy.get_param('~waypoints', default_waypoints)

        # State
        self.lock = Lock()
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.amcl_cov = None
        self.particle_spread = float('inf')
        self.amcl_received = False
        self.gt_received = False

        # Tracking data
        self.tracking_errors = []  # (timestamp, pos_err, yaw_err)
        self.waypoint_results = []
        self.convergence_time = None
        self.start_time = None

        # Subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self._gt_cb, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self._amcl_cb, queue_size=1)
        rospy.Subscriber('/particlecloud', PoseArray,
                         self._particle_cb, queue_size=1)

        # Continuous tracking timer
        self.track_timer = rospy.Timer(rospy.Duration(0.5), self._track_cb)

        # Action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[AMCL Verifier] Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("[AMCL Verifier] move_base not available!")
            sys.exit(1)
        rospy.loginfo("[AMCL Verifier] Connected to move_base.")

    def _gt_cb(self, msg):
        """Ground truth from Gazebo."""
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        with self.lock:
            p = msg.pose[idx]
            self.gt_x = p.position.x
            self.gt_y = p.position.y
            self.gt_yaw = quaternion_to_yaw(p.orientation)
            self.gt_received = True

    def _amcl_cb(self, msg):
        """AMCL estimated pose."""
        with self.lock:
            p = msg.pose.pose
            self.amcl_x = p.position.x
            self.amcl_y = p.position.y
            self.amcl_yaw = quaternion_to_yaw(p.orientation)
            self.amcl_cov = list(msg.pose.covariance)
            self.amcl_received = True

            # Check convergence
            if self.convergence_time is None and self.start_time is not None:
                pos_err = math.sqrt(
                    (self.amcl_x - self.gt_x)**2 +
                    (self.amcl_y - self.gt_y)**2)
                if pos_err < self.convergence_threshold:
                    self.convergence_time = (rospy.Time.now() - self.start_time).to_sec()
                    rospy.loginfo("[AMCL Verifier] Converged in %.1f s (err=%.3f m)"
                                  % (self.convergence_time, pos_err))

    def _particle_cb(self, msg):
        """Particle cloud for spread analysis."""
        if len(msg.poses) < 2:
            return
        xs = [p.position.x for p in msg.poses]
        ys = [p.position.y for p in msg.poses]
        self.particle_spread = math.sqrt(np.var(xs) + np.var(ys))

    def _track_cb(self, event):
        """Periodically record tracking error."""
        with self.lock:
            if not (self.amcl_received and self.gt_received):
                return
            pos_err = math.sqrt(
                (self.amcl_x - self.gt_x)**2 +
                (self.amcl_y - self.gt_y)**2)
            yaw_err = abs(angle_diff(self.amcl_yaw, self.gt_yaw))
            t = rospy.Time.now().to_sec()
            self.tracking_errors.append((t, pos_err, yaw_err))

    def get_current_error(self):
        """Get current localization error."""
        with self.lock:
            pos_err = math.sqrt(
                (self.amcl_x - self.gt_x)**2 +
                (self.amcl_y - self.gt_y)**2)
            yaw_err = abs(angle_diff(self.amcl_yaw, self.gt_yaw))
            return pos_err, yaw_err

    def send_initial_pose(self):
        """Publish initial pose estimate matching Gazebo spawn (0,0,0)."""
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(1.0)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        # Small initial covariance
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.07  # yaw

        pub.publish(msg)
        rospy.loginfo("[AMCL Verifier] Published initial pose (0, 0, 0°).")
        rospy.sleep(2.0)

    def navigate_to_waypoint(self, wp):
        """Navigate to waypoint and measure error at arrival."""
        name = wp['name']
        tx, ty = wp['x'], wp['y']

        rospy.loginfo("\n[AMCL Verifier] === Navigating to %s (%.1f, %.1f) ===" % (name, tx, ty))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = tx
        goal.target_pose.pose.position.y = ty
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
        state = self.client.get_state()

        nav_success = (finished and state == GoalStatus.SUCCEEDED)
        if not nav_success:
            self.client.cancel_goal()
            rospy.logwarn("[AMCL Verifier] %s: navigation %s (state=%d)"
                          % (name, "TIMEOUT" if not finished else "FAILED", state))

        # Wait for AMCL to settle
        rospy.sleep(self.settle_time)

        # Measure error
        with self.lock:
            gt_x, gt_y, gt_yaw = self.gt_x, self.gt_y, self.gt_yaw
            amcl_x, amcl_y, amcl_yaw = self.amcl_x, self.amcl_y, self.amcl_yaw
            cov = self.amcl_cov[:] if self.amcl_cov else [0]*36
            spread = self.particle_spread

        pos_err = math.sqrt((amcl_x - gt_x)**2 + (amcl_y - gt_y)**2)
        yaw_err_rad = abs(angle_diff(amcl_yaw, gt_yaw))
        yaw_err_deg = math.degrees(yaw_err_rad)

        result = {
            'name': name,
            'target': (tx, ty),
            'nav_success': nav_success,
            'gt_pose': (gt_x, gt_y, math.degrees(gt_yaw)),
            'amcl_pose': (amcl_x, amcl_y, math.degrees(amcl_yaw)),
            'pos_error_m': pos_err,
            'yaw_error_deg': yaw_err_deg,
            'cov_xx': cov[0],
            'cov_yy': cov[7],
            'cov_yawyaw': cov[35],
            'particle_spread': spread,
        }
        self.waypoint_results.append(result)

        status_str = "REACHED" if nav_success else "FAILED"
        rospy.loginfo("[AMCL Verifier] %s: %s | pos_err=%.3f m, yaw_err=%.1f°, spread=%.3f"
                      % (name, status_str, pos_err, yaw_err_deg, spread))
        return result

    def generate_report(self):
        """Generate a comprehensive text and JSON report."""
        # Compute tracking statistics
        if self.tracking_errors:
            pos_errs = [e[1] for e in self.tracking_errors]
            yaw_errs = [e[2] for e in self.tracking_errors]
            track_stats = {
                'mean_pos_err': float(np.mean(pos_errs)),
                'max_pos_err': float(np.max(pos_errs)),
                'std_pos_err': float(np.std(pos_errs)),
                'median_pos_err': float(np.median(pos_errs)),
                'mean_yaw_err_deg': float(np.degrees(np.mean(yaw_errs))),
                'max_yaw_err_deg': float(np.degrees(np.max(yaw_errs))),
                'samples': len(pos_errs),
            }
        else:
            track_stats = {}

        # Text report
        lines = []
        lines.append("=" * 70)
        lines.append("  AMCL LOCALIZATION VERIFICATION REPORT")
        lines.append("  Generated: %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
        lines.append("=" * 70)
        lines.append("")

        lines.append("--- Convergence ---")
        if self.convergence_time is not None:
            lines.append("  Convergence time:   %.1f s (threshold: %.2f m)"
                         % (self.convergence_time, self.convergence_threshold))
        else:
            lines.append("  Convergence:        NOT achieved within session")
        lines.append("")

        lines.append("--- Continuous Tracking Statistics ---")
        if track_stats:
            lines.append("  Samples:           %d" % track_stats['samples'])
            lines.append("  Mean pos error:    %.4f m" % track_stats['mean_pos_err'])
            lines.append("  Max pos error:     %.4f m" % track_stats['max_pos_err'])
            lines.append("  Std pos error:     %.4f m" % track_stats['std_pos_err'])
            lines.append("  Median pos error:  %.4f m" % track_stats['median_pos_err'])
            lines.append("  Mean yaw error:    %.2f°" % track_stats['mean_yaw_err_deg'])
            lines.append("  Max yaw error:     %.2f°" % track_stats['max_yaw_err_deg'])
        lines.append("")

        lines.append("--- Per-Waypoint Results ---")
        lines.append("  %-25s %-8s %-12s %-12s %-10s %-10s" %
                     ("Waypoint", "NavOK", "Pos Err(m)", "Yaw Err(°)", "Cov(xy)", "Spread"))
        lines.append("  " + "-" * 77)
        for r in self.waypoint_results:
            lines.append("  %-25s %-8s %-12.4f %-12.2f %-10.4f %-10.4f" % (
                r['name'],
                "YES" if r['nav_success'] else "NO",
                r['pos_error_m'],
                r['yaw_error_deg'],
                math.sqrt(r['cov_xx']**2 + r['cov_yy']**2),
                r['particle_spread'],
            ))
        lines.append("")

        # Summary statistics for waypoints
        if self.waypoint_results:
            wp_pos_errs = [r['pos_error_m'] for r in self.waypoint_results]
            wp_yaw_errs = [r['yaw_error_deg'] for r in self.waypoint_results]
            nav_ok = sum(1 for r in self.waypoint_results if r['nav_success'])
            lines.append("--- Waypoint Summary ---")
            lines.append("  Navigation success:   %d / %d" % (nav_ok, len(self.waypoint_results)))
            lines.append("  Mean position error:  %.4f m" % np.mean(wp_pos_errs))
            lines.append("  Max position error:   %.4f m" % np.max(wp_pos_errs))
            lines.append("  Mean yaw error:       %.2f°" % np.mean(wp_yaw_errs))
            lines.append("  Max yaw error:        %.2f°" % np.max(wp_yaw_errs))
            lines.append("")

            # Pass/Fail criteria
            mean_err = np.mean(wp_pos_errs)
            max_err = np.max(wp_pos_errs)
            lines.append("--- PASS/FAIL Criteria ---")
            lines.append("  Mean position error < 0.30 m:  %s (%.4f m)"
                         % ("PASS" if mean_err < 0.30 else "FAIL", mean_err))
            lines.append("  Max position error  < 0.50 m:  %s (%.4f m)"
                         % ("PASS" if max_err < 0.50 else "FAIL", max_err))
            lines.append("  Navigation success  >= 80%%:    %s (%d%%)"
                         % ("PASS" if nav_ok / len(self.waypoint_results) >= 0.8 else "FAIL",
                            int(100 * nav_ok / len(self.waypoint_results))))

            overall = (mean_err < 0.30 and max_err < 0.50 and
                       nav_ok / len(self.waypoint_results) >= 0.8)
            lines.append("")
            lines.append("  OVERALL: %s" % ("*** PASS ***" if overall else "*** FAIL ***"))

        lines.append("")
        lines.append("=" * 70)

        report_text = "\n".join(lines)

        # Save text report
        report_dir = os.path.dirname(self.report_path)
        if not os.path.exists(report_dir):
            os.makedirs(report_dir)

        with open(self.report_path, 'w') as f:
            f.write(report_text)
        rospy.loginfo("[AMCL Verifier] Text report saved to: %s" % self.report_path)

        # Save JSON report
        json_data = {
            'convergence_time': self.convergence_time,
            'tracking_stats': track_stats,
            'waypoint_results': [],
        }
        for r in self.waypoint_results:
            jr = dict(r)
            jr['target'] = list(jr['target'])
            jr['gt_pose'] = list(jr['gt_pose'])
            jr['amcl_pose'] = list(jr['amcl_pose'])
            json_data['waypoint_results'].append(jr)

        with open(self.json_report_path, 'w') as f:
            json.dump(json_data, f, indent=2)
        rospy.loginfo("[AMCL Verifier] JSON report saved to: %s" % self.json_report_path)

        # Print to console
        print("\n" + report_text)

    def run(self):
        """Main verification loop."""
        rospy.loginfo("[AMCL Verifier] Waiting %.0f s for AMCL + move_base to initialize..."
                      % self.initial_wait)
        rospy.sleep(self.initial_wait)

        self.start_time = rospy.Time.now()

        # Wait for ground truth and AMCL
        rospy.loginfo("[AMCL Verifier] Waiting for Gazebo ground truth and AMCL pose...")
        timeout = rospy.Time.now() + rospy.Duration(30.0)
        while not (self.gt_received and self.amcl_received) and not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("[AMCL Verifier] Timeout waiting for topics. "
                              "GT=%s, AMCL=%s" % (self.gt_received, self.amcl_received))
                break
            rospy.sleep(0.5)

        # Send initial pose
        self.send_initial_pose()

        # Let AMCL converge with a small spin
        from geometry_msgs.msg import Twist
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.5)
        twist = Twist()
        twist.angular.z = 0.6
        rospy.loginfo("[AMCL Verifier] Gentle spin to help AMCL converge...")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < 8.0 and not rospy.is_shutdown():
            cmd_pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0
        cmd_pub.publish(twist)
        rospy.sleep(3.0)

        pos_err, yaw_err = self.get_current_error()
        rospy.loginfo("[AMCL Verifier] Initial error: pos=%.3f m, yaw=%.1f°"
                      % (pos_err, math.degrees(yaw_err)))

        # Navigate waypoints
        for wp in self.waypoints:
            if rospy.is_shutdown():
                break
            self.navigate_to_waypoint(wp)
            rospy.sleep(2.0)

        # Generate report
        self.generate_report()


def main():
    try:
        verifier = AMCLVerifier()
        verifier.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
