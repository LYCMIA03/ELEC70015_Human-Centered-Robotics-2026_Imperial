#!/usr/bin/env python3
"""
autonomous_explorer.py — Frontier-based autonomous exploration for P3-AT.

Subscribes to the gmapping occupancy grid, detects frontiers (boundaries between
known-free and unknown space), clusters them, and sends the nearest large frontier
centroid as a MoveBaseGoal. Repeats until no reachable frontiers remain or a time
limit is hit.

On completion (or Ctrl-C), saves the map via the map_saver service.

Usage:
    roslaunch p3at_lms_navigation auto_mapping.launch
    # The launch file starts this node automatically.

    # Or run standalone after mapping.launch:
    rosrun p3at_lms_navigation autonomous_explorer.py
"""

import rospy
import actionlib
import math
import numpy as np
import subprocess
import os
import sys
import time
import random
from collections import deque

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class FrontierExplorer:
    """Frontier-based autonomous exploration node."""

    # Occupancy grid values
    FREE = 0
    UNKNOWN = -1
    OCCUPIED = 100

    def __init__(self):
        rospy.init_node('autonomous_explorer', anonymous=False)

        # Parameters
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 8)
        self.goal_timeout = rospy.get_param('~goal_timeout', 90.0)
        self.stuck_progress_timeout = rospy.get_param('~stuck_progress_timeout', 8.0)
        self.stuck_min_progress = rospy.get_param('~stuck_min_progress', 0.15)
        self.exploration_timeout = rospy.get_param('~exploration_timeout', 600.0)
        self.save_map = rospy.get_param('~save_map', True)
        self.map_save_path = rospy.get_param(
            '~map_save_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'maps', 'explored_map'))
        self.robot_radius = rospy.get_param('~robot_radius', 0.35)
        self.frontier_blacklist_radius = rospy.get_param('~frontier_blacklist_radius', 0.5)
        self.goal_reached_threshold = rospy.get_param('~goal_reached_threshold', 0.5)
        self.initial_wait = rospy.get_param('~initial_wait', 10.0)
        self.spin_in_place_first = rospy.get_param('~spin_in_place_first', True)

        # State
        self.map_data = None
        self.map_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.blacklisted_goals = []  # (x,y) goals that failed
        self.exploration_start = None
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_goal = None  # track last goal to prevent re-sending same point
        self.same_goal_count = 0

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)

        # Publisher for visualization
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=1)

        # Action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[Explorer] Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("[Explorer] move_base not available!")
            sys.exit(1)
        rospy.loginfo("[Explorer] Connected to move_base.")

    def _map_cb(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        dx = self.robot_x - self.last_x
        dy = self.robot_y - self.last_y
        self.total_distance += math.sqrt(dx*dx + dy*dy)
        self.last_x = self.robot_x
        self.last_y = self.robot_y

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid indices."""
        mi = self.map_info
        gx = int((wx - mi.origin.position.x) / mi.resolution)
        gy = int((wy - mi.origin.position.y) / mi.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates."""
        mi = self.map_info
        wx = mi.origin.position.x + (gx + 0.5) * mi.resolution
        wy = mi.origin.position.y + (gy + 0.5) * mi.resolution
        return wx, wy

    def find_frontiers(self):
        """Detect frontier cells — free cells adjacent to unknown cells."""
        if self.map_data is None:
            return []

        h, w = self.map_data.shape
        frontier_mask = np.zeros((h, w), dtype=bool)

        # A frontier cell is a FREE cell (value == 0) with at least one
        # UNKNOWN neighbour (value == -1)
        free_mask = (self.map_data == self.FREE)

        # Check 4-connected neighbours for unknown
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(self.map_data, -dy, axis=0), -dx, axis=1)
            frontier_mask |= (free_mask & (shifted == self.UNKNOWN))

        # Zero out borders to avoid wrap-around artifacts from np.roll
        frontier_mask[0, :] = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0] = False
        frontier_mask[:, -1] = False

        return frontier_mask

    def cluster_frontiers(self, frontier_mask):
        """BFS cluster frontier cells. Returns list of [(cx, cy, size), ...]."""
        h, w = frontier_mask.shape
        visited = np.zeros((h, w), dtype=bool)
        clusters = []

        ys, xs = np.where(frontier_mask)
        for i in range(len(xs)):
            x0, y0 = int(xs[i]), int(ys[i])
            if visited[y0, x0]:
                continue
            # BFS
            queue = deque()
            queue.append((x0, y0))
            visited[y0, x0] = True
            cells = []
            while queue:
                cx, cy = queue.popleft()
                cells.append((cx, cy))
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx] and frontier_mask[ny, nx]:
                        visited[ny, nx] = True
                        queue.append((nx, ny))

            if len(cells) >= self.min_frontier_size:
                # Centroid in grid coords
                mean_x = sum(c[0] for c in cells) / len(cells)
                mean_y = sum(c[1] for c in cells) / len(cells)
                clusters.append((mean_x, mean_y, len(cells)))

        return clusters

    def is_blacklisted(self, wx, wy):
        """Check if a world-coordinate goal is near a blacklisted position."""
        for bx, by in self.blacklisted_goals:
            if math.sqrt((wx - bx)**2 + (wy - by)**2) < self.frontier_blacklist_radius:
                return True
        return False

    def is_reachable(self, gx, gy):
        """Light check: the target cell itself is not OCCUPIED.
        Leave full collision checking to move_base — if the goal is
        unreachable move_base will ABORT and we blacklist it."""
        h, w = self.map_data.shape
        igx, igy = int(gx), int(gy)
        if not (0 <= igx < w and 0 <= igy < h):
            return False
        if self.map_data[igy, igx] == self.OCCUPIED:
            return False
        return True

    def _find_approach_point(self, gx, gy):
        """Find a safe FREE cell near (gx, gy) pulled toward the robot.

        Frontier centroids can land on unknown or wall-adjacent cells.
        Walk a short vector from the frontier centroid toward the robot
        until we land on a cell that is FREE and well away from walls.
        """
        h, w = self.map_data.shape
        rx_g, ry_g = self.world_to_grid(self.robot_x, self.robot_y)
        dx = rx_g - gx
        dy = ry_g - gy
        length = math.hypot(dx, dy)
        if length < 1.0:
            return gx, gy
        ux, uy = dx / length, dy / length

        # Step toward robot to find a safe spot.
        # Check radius: ~4 cells (0.20m) — enough to avoid wall-adjacent goals
        # without being so strict that all maze corridors are rejected.
        check_r = 4
        max_steps = max(10, int(0.6 / self.map_info.resolution))  # up to ~0.6m back
        for step in range(2, max_steps + 1):
            cx = int(gx + ux * step)
            cy = int(gy + uy * step)
            if not (0 <= cx < w and 0 <= cy < h):
                continue
            if self.map_data[cy, cx] != self.FREE:
                continue
            # Check neighbourhood for occupied cells (inscribed radius clearance)
            ok = True
            for ddy in range(-check_r, check_r + 1):
                if not ok:
                    break
                for ddx in range(-check_r, check_r + 1):
                    nx, ny = cx + ddx, cy + ddy
                    if 0 <= nx < w and 0 <= ny < h:
                        if self.map_data[ny, nx] == self.OCCUPIED:
                            ok = False
                            break
            if ok:
                return cx, cy
        return gx, gy

    def select_goal(self):
        """Select the best frontier goal. Strategy: nearest large frontier.

        Key improvement: instead of navigating directly to the frontier
        centroid (which is often ON the boundary of unknown space), we
        compute an *approach point* — a nearby free cell pulled slightly
        toward the robot. This prevents goals inside walls or unknown
        cells, which is critical in maze environments.
        """
        frontier_mask = self.find_frontiers()
        clusters = self.cluster_frontiers(frontier_mask)

        if not clusters:
            return None

        # Score: prefer closer, larger frontiers
        scored = []
        for gx, gy, size in clusters:
            wx, wy = self.grid_to_world(gx, gy)
            if self.is_blacklisted(wx, wy):
                continue
            # Find safe approach point for this frontier
            ax, ay = self._find_approach_point(gx, gy)
            awx, awy = self.grid_to_world(ax, ay)
            if self.is_blacklisted(awx, awy):
                continue
            if not self.is_reachable(ax, ay):
                continue
            dist = math.sqrt((awx - self.robot_x)**2 + (awy - self.robot_y)**2)
            if dist < 0.6:
                continue  # must be > xy_goal_tolerance to avoid instant-success loop
            # Score: STRONGLY prefer closer frontiers (crucial for maze navigation)
            # Using 1/(dist+0.5)^2 makes the explorer greedily navigate
            # to nearby frontiers first, gradually expanding outward.
            score = size / (dist + 0.5)**2
            scored.append((score, awx, awy, size, dist))

        if not scored:
            return None

        scored.sort(reverse=True)
        # Prefer goals within 1.8m; only take farther goals if nothing close
        close_goals = [s for s in scored if s[4] <= 1.8]
        if close_goals:
            best = close_goals[0]
        else:
            # All far away — pick the closest one regardless of score
            scored_by_dist = sorted(scored, key=lambda x: x[4])
            best = scored_by_dist[0]
        rospy.loginfo("[Explorer] Selected frontier: (%.2f, %.2f), size=%d, dist=%.2f, score=%.1f"
                      % (best[1], best[2], best[3], best[4], best[0]))
        return best[1], best[2]

    def send_nav_goal(self, wx, wy):
        """Send a MoveBaseGoal with progress watchdog.

        Returns:
            (success: bool, reason: str)
              reason in {"succeeded", "timeout", "aborted", "stuck_no_progress", ...}
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wx
        goal.target_pose.pose.position.y = wy
        goal.target_pose.pose.orientation.w = 1.0

        # Publish for RViz visualization
        vis = PoseStamped()
        vis.header = goal.target_pose.header
        vis.pose = goal.target_pose.pose
        self.goal_pub.publish(vis)

        self.client.send_goal(goal)
        self.goals_sent += 1
        start_wall = time.time()
        last_progress_wall = start_wall
        progress_anchor = (self.robot_x, self.robot_y)
        reason = "unknown"

        terminal_fail_states = {
            GoalStatus.ABORTED: "aborted",
            GoalStatus.REJECTED: "rejected",
            GoalStatus.PREEMPTED: "preempted",
            GoalStatus.RECALLED: "recalled",
            GoalStatus.LOST: "lost",
        }

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.goals_succeeded += 1
                return True, "succeeded"

            if state in terminal_fail_states:
                reason = terminal_fail_states[state]
                break

            now_wall = time.time()
            if now_wall - start_wall > self.goal_timeout:
                reason = "timeout"
                break

            progress = math.hypot(
                self.robot_x - progress_anchor[0],
                self.robot_y - progress_anchor[1],
            )
            if progress >= self.stuck_min_progress:
                progress_anchor = (self.robot_x, self.robot_y)
                last_progress_wall = now_wall
            elif now_wall - last_progress_wall > self.stuck_progress_timeout:
                reason = "stuck_no_progress"
                break

            rate.sleep()

        self.goals_failed += 1
        self.client.cancel_goal()
        rospy.logwarn(
            "[Explorer] Goal (%.2f, %.2f) failed (%s), blacklisting.",
            wx, wy, reason)
        self.blacklisted_goals.append((wx, wy))
        # Recovery: clear costmaps so the planner can re-plan from scratch
        self._clear_costmaps()
        return False, reason

    def _clear_costmaps(self):
        """Clear costmaps via the move_base service to recover from stuck states."""
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
            clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_srv()
            rospy.loginfo("[Explorer] Cleared costmaps for recovery.")
        except Exception as e:
            rospy.logwarn("[Explorer] Could not clear costmaps: %s" % str(e))

    def _try_backup(self, duration=2.0, speed=-0.10):
        """Physically reverse the robot to escape a stuck position near walls.

        When the robot is trapped (footprint overlapping lethal cells),
        move_base cannot plan any path. The only escape is to command
        raw velocity to back up, then clear costmaps.
        """
        from geometry_msgs.msg import Twist
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.3)

        # First cancel any active move_base goal
        self.client.cancel_all_goals()
        rospy.sleep(0.5)

        twist = Twist()
        twist.linear.x = speed  # reverse
        rospy.loginfo("[Explorer] Backing up for %.1fs at %.2f m/s..." % (duration, speed))
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            cmd_pub.publish(twist)
            rate.sleep()
        twist.linear.x = 0.0
        cmd_pub.publish(twist)
        rospy.sleep(0.5)

        # Then do a gentle rotation to face a new direction
        turn_sign = random.choice([-1.0, 1.0])
        twist.angular.z = turn_sign * 0.35
        rospy.loginfo("[Explorer] Rotating to find new direction (sign=%+.0f)...", turn_sign)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < 3.0 and not rospy.is_shutdown():
            cmd_pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0
        cmd_pub.publish(twist)
        rospy.sleep(0.5)

        # Clear costmaps after physical recovery
        self._clear_costmaps()
        rospy.loginfo("[Explorer] Backup recovery complete.")

    def do_initial_spin(self):
        """Rotate in place to get initial scan coverage."""
        from geometry_msgs.msg import Twist
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.5)
        twist = Twist()
        twist.angular.z = 0.8
        rospy.loginfo("[Explorer] Initial 360° scan spin...")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < 9.0 and not rospy.is_shutdown():
            cmd_pub.publish(twist)
            rate.sleep()
        twist.angular.z = 0.0
        cmd_pub.publish(twist)
        rospy.sleep(1.0)
        rospy.loginfo("[Explorer] Spin complete.")

    def save_map_file(self):
        """Save the current map using map_saver."""
        rospy.loginfo("[Explorer] Saving map to: %s" % self.map_save_path)
        map_dir = os.path.dirname(self.map_save_path)
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
        try:
            subprocess.call([
                'rosrun', 'map_server', 'map_saver',
                '-f', self.map_save_path,
                '__name:=map_saver_explorer'
            ], timeout=30)
            rospy.loginfo("[Explorer] Map saved successfully.")
        except Exception as e:
            rospy.logerr("[Explorer] Failed to save map: %s" % str(e))

    def compute_coverage(self):
        """Compute map coverage percentage."""
        if self.map_data is None:
            return 0.0, 0, 0, 0
        total = self.map_data.size
        free = int(np.sum(self.map_data == self.FREE))
        occupied = int(np.sum(self.map_data == self.OCCUPIED))
        unknown = int(np.sum(self.map_data == self.UNKNOWN))
        known = free + occupied
        coverage = known / total * 100.0 if total > 0 else 0.0
        return coverage, free, occupied, unknown

    def print_summary(self):
        """Print exploration summary."""
        elapsed = (rospy.Time.now() - self.exploration_start).to_sec() if self.exploration_start else 0
        coverage, free, occupied, unknown = self.compute_coverage()
        print("\n" + "=" * 60)
        print("  AUTONOMOUS EXPLORATION SUMMARY")
        print("=" * 60)
        print("  Duration:          %.1f s" % elapsed)
        print("  Distance traveled: %.2f m" % self.total_distance)
        print("  Goals sent:        %d" % self.goals_sent)
        print("  Goals succeeded:   %d" % self.goals_succeeded)
        print("  Goals failed:      %d" % self.goals_failed)
        print("  Map coverage:      %.1f%% known" % coverage)
        print("    Free cells:      %d" % free)
        print("    Occupied cells:  %d" % occupied)
        print("    Unknown cells:   %d" % unknown)
        if self.map_info:
            area = self.map_info.width * self.map_info.height * self.map_info.resolution**2
            print("  Map area:          %.1f m²" % area)
        print("  Map saved to:      %s" % self.map_save_path)
        print("=" * 60 + "\n")

    def run(self):
        """Main exploration loop."""
        rospy.loginfo("[Explorer] Waiting %.0f s for gmapping/move_base to initialize..."
                      % self.initial_wait)
        rospy.sleep(self.initial_wait)

        self.exploration_start = rospy.Time.now()

        # Wait for first map
        rospy.loginfo("[Explorer] Waiting for /map topic...")
        while self.map_data is None and not rospy.is_shutdown():
            rospy.sleep(0.5)
        rospy.loginfo("[Explorer] Map received (%d x %d)."
                      % (self.map_info.width, self.map_info.height))

        # Initial spin to seed the map
        if self.spin_in_place_first:
            self.do_initial_spin()

        consecutive_no_frontier = 0
        max_no_frontier = 8
        re_spin_interval = 3  # do a re-spin every N no-frontier attempts
        consecutive_failures = 0  # track consecutive goal failures
        max_consecutive_failures = 3  # trigger backup recovery after this many

        while not rospy.is_shutdown():
            # Check time limit
            elapsed = (rospy.Time.now() - self.exploration_start).to_sec()
            if elapsed > self.exploration_timeout:
                rospy.loginfo("[Explorer] Exploration timeout (%.0f s). Stopping."
                              % self.exploration_timeout)
                break

            # If too many consecutive failures, robot is likely stuck
            if consecutive_failures >= max_consecutive_failures:
                rospy.logwarn("[Explorer] %d consecutive failures — attempting backup recovery."
                              % consecutive_failures)
                self._try_backup()
                consecutive_failures = 0
                continue

            # Select next frontier
            goal = self.select_goal()
            if goal is None:
                consecutive_no_frontier += 1
                rospy.loginfo("[Explorer] No frontier found (%d/%d)."
                              % (consecutive_no_frontier, max_no_frontier))
                if consecutive_no_frontier >= max_no_frontier:
                    rospy.loginfo("[Explorer] No more reachable frontiers. Exploration complete!")
                    break
                # Every re_spin_interval failures, do a 360° spin to
                # update the map (lidar may reveal new frontiers from
                # the current position after the map has updated).
                if consecutive_no_frontier % re_spin_interval == 0:
                    rospy.loginfo("[Explorer] Re-spinning to discover new frontiers...")
                    self.do_initial_spin()
                else:
                    rospy.sleep(2.0)
                continue

            consecutive_no_frontier = 0
            wx, wy = goal

            # Detect if we keep selecting the same goal (infinite loop guard)
            if self.last_goal is not None:
                lg_dist = math.sqrt((wx - self.last_goal[0])**2 + (wy - self.last_goal[1])**2)
                if lg_dist < 0.3:
                    self.same_goal_count += 1
                    if self.same_goal_count >= 2:
                        rospy.logwarn("[Explorer] Same goal (%.2f, %.2f) selected %d times, blacklisting."
                                      % (wx, wy, self.same_goal_count))
                        self.blacklisted_goals.append((wx, wy))
                        self.same_goal_count = 0
                        self.last_goal = None
                        continue
                else:
                    self.same_goal_count = 0
            self.last_goal = (wx, wy)

            # Navigate to frontier
            coverage, _, _, _ = self.compute_coverage()
            rospy.loginfo("[Explorer] >>> Goal #%d: (%.2f, %.2f) | Coverage: %.1f%%"
                          % (self.goals_sent + 1, wx, wy, coverage))
            success, fail_reason = self.send_nav_goal(wx, wy)
            if success:
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if fail_reason == "stuck_no_progress":
                    rospy.logwarn("[Explorer] Stuck detected during goal execution, immediate backup recovery.")
                    self._try_backup()
                    consecutive_failures = 0
            rospy.sleep(1.0)

        # Save map
        if self.save_map:
            self.save_map_file()

        self.print_summary()


def main():
    try:
        explorer = FrontierExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
