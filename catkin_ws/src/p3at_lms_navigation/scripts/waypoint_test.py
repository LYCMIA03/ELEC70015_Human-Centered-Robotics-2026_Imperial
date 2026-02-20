#!/usr/bin/env python3
"""
waypoint_test.py — Sequential 3-waypoint navigation test for P3-AT + LMS200 simulation.

Usage:
    1. Launch the simulation:
           roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false
    2. Wait ~30 s for gmapping / move_base to initialise, then run:
           python3 src/p3at_lms_navigation/scripts/waypoint_test.py
       OR (from any sourced terminal):
           rosrun p3at_lms_navigation waypoint_test.py

Waypoints (all in the 'map' frame, robot starts at ~origin):
    WP1  (  0.0, -2.0)  Open space south, ~2 m straight run
    WP2  (  3.5,  2.0)  North of obstacle_1 @ (2, 1); robot must navigate around
    WP3  (  5.0,  0.0)  Far east, past both obstacles

Tested result (commit 5d9e4d2):
    WP1  SUCCEEDED  error 0.14 m
    WP2  SUCCEEDED  error 0.11 m
    WP3  SUCCEEDED  error 0.19 m
"""

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry


def get_pos():
    """Return current (x, y) from /odom."""
    m = rospy.wait_for_message('/odom', Odometry, timeout=3)
    return m.pose.pose.position.x, m.pose.pose.position.y


def send_goal(client, x, y, label):
    """Send a single MoveBaseGoal and block until result or 65 s timeout."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    px, py = get_pos()
    print("\n========== Waypoint %s: (%.1f, %.1f) ==========" % (label, x, y))
    print("  Start position : (%.3f, %.3f)" % (px, py))
    print("  Distance to goal: %.2f m" % math.sqrt((x - px) ** 2 + (y - py) ** 2))

    client.send_goal(goal)

    for elapsed in range(0, 65, 5):
        finished = client.wait_for_result(rospy.Duration(5.0))
        px2, py2 = get_pos()
        state = client.get_state()
        print("  t=%2ds: pos=(%.3f, %.3f)  state=%d" % (elapsed + 5, px2, py2, state))
        if finished:
            break

    state = client.get_state()
    px_f, py_f = get_pos()
    dist = math.sqrt((x - px_f) ** 2 + (y - py_f) ** 2)
    result_str = {1: "ACTIVE", 3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED"}.get(
        state, "state=%d" % state
    )
    print("  RESULT: %s | Final pos: (%.3f, %.3f) | Error: %.3f m" % (result_str, px_f, py_f, dist))
    return state == 3


def main():
    rospy.init_node('waypoint_test', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("Waiting for move_base server...")
    if not client.wait_for_server(rospy.Duration(15)):
        rospy.logerr("move_base action server not available after 15 s. Is mapping.launch running?")
        return
    print("Connected to move_base!")

    # fmt: off
    waypoints = [
        (0.0, -2.0, "1 [open south, ~2 m]"),
        (3.5,  2.0, "2 [N of obstacle_1@(2,1), ~5 m total, must avoid]"),
        (5.0,  0.0, "3 [far east past both obstacles, ~7 m total]"),
    ]
    # fmt: on

    results = []
    for x, y, label in waypoints:
        ok = send_goal(client, x, y, label)
        results.append((label, ok))
        if not ok:
            print("  WARNING: waypoint %s did not SUCCEED, continuing..." % label)
        rospy.sleep(1.0)

    print("\n========== SUMMARY ==========")
    for label, ok in results:
        status = "PASS" if ok else "FAIL"
        print("  WP%s : %s" % (label, status))

    px, py = get_pos()
    print("Final robot position: (%.3f, %.3f)" % (px, py))


if __name__ == '__main__':
    main()
