#!/usr/bin/env python3
"""
Move the 'target' model in Gazebo along a sequence of waypoints.

This simulates a moving person/object for the target_follower to chase.
The target moves at a configurable speed, pauses at each waypoint,
and loops continuously.

Usage:
    rosrun target_follower move_target.py
    rosrun target_follower move_target.py _speed:=0.3 _pause:=2.0

Parameters:
    ~speed      (float, default 0.4)  target movement speed (m/s)
    ~pause      (float, default 1.0)  pause duration at each waypoint (s)
    ~model_name (str,   default "target")  Gazebo model name
    ~loop       (bool,  default True) loop through waypoints continuously
"""

import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


def yaw_to_quat(yaw):
    """Convert yaw (rad) to (x, y, z, w) quaternion."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rospy.init_node("move_target")

    model_name = rospy.get_param("~model_name", "target")
    speed = float(rospy.get_param("~speed", 0.4))        # m/s
    pause_time = float(rospy.get_param("~pause", 1.0))    # seconds at each waypoint
    loop = bool(rospy.get_param("~loop", True))

    # Default waypoints: a loop around the obstacle area
    # Obstacles: box_obstacle_1 at (2, 1), box_obstacle_2 at (3, -1)
    # Target spawns at (4, 0)
    default_waypoints = [
        (4.0, 0.0),    # start position (where target spawns)
        (4.0, 2.0),    # move north
        (1.0, 2.0),    # move west (north of obstacle_1)
        (1.0, -2.0),   # move south
        (4.0, -2.0),   # move east (south of obstacle_2)
        (4.0, 0.0),    # back to start
    ]

    # Allow override via parameter (list of [x, y] pairs)
    wp_param = rospy.get_param("~waypoints", "")
    if wp_param and isinstance(wp_param, list):
        waypoints = [(w[0], w[1]) for w in wp_param]
        rospy.loginfo("Using %d waypoints from parameter", len(waypoints))
    else:
        waypoints = default_waypoints
        rospy.loginfo("Using %d default waypoints", len(waypoints))

    rospy.loginfo(
        "move_target: model='%s', speed=%.2f m/s, pause=%.1f s, loop=%s",
        model_name, speed, pause_time, loop,
    )

    # Wait for Gazebo services
    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/set_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.loginfo("Gazebo services ready.")

    # Get initial target position from Gazebo
    try:
        resp = get_state(model_name, "world")
        if resp.success:
            cx = resp.pose.position.x
            cy = resp.pose.position.y
            rospy.loginfo("Target initial position: (%.2f, %.2f)", cx, cy)
        else:
            rospy.logwarn("Could not get model state, using first waypoint")
            cx, cy = waypoints[0]
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s, using first waypoint", str(e))
        cx, cy = waypoints[0]

    rate = rospy.Rate(50)  # 50 Hz for smooth motion
    dt = 1.0 / 50.0

    first_run = True

    while not rospy.is_shutdown():
        for i, (tx, ty) in enumerate(waypoints):
            if rospy.is_shutdown():
                break

            # Skip first waypoint on first run if already there
            if first_run and i == 0:
                if math.hypot(tx - cx, ty - cy) < 0.3:
                    rospy.loginfo("Already at WP0, skipping to next")
                    continue
            first_run = False

            dx = tx - cx
            dy = ty - cy
            dist = math.hypot(dx, dy)

            if dist < 0.05:
                continue

            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = yaw_to_quat(yaw)

            rospy.loginfo(
                "Moving target to WP%d (%.1f, %.1f), dist=%.2f m", i, tx, ty, dist
            )

            # Move at constant speed toward waypoint
            travelled = 0.0
            while not rospy.is_shutdown() and travelled < dist:
                # Re-read speed so it can be tuned at runtime
                speed = float(rospy.get_param("~speed", speed))

                step = min(speed * dt, dist - travelled)
                cx += (dx / dist) * step
                cy += (dy / dist) * step
                travelled += step

                state_msg = ModelState()
                state_msg.model_name = model_name
                state_msg.pose.position.x = cx
                state_msg.pose.position.y = cy
                state_msg.pose.position.z = 0.0
                state_msg.pose.orientation.x = qx
                state_msg.pose.orientation.y = qy
                state_msg.pose.orientation.z = qz
                state_msg.pose.orientation.w = qw
                state_msg.reference_frame = "world"

                try:
                    set_state(state_msg)
                except rospy.ServiceException:
                    rospy.logwarn_throttle(2.0, "set_model_state failed")

                rate.sleep()

            rospy.loginfo(
                "Reached WP%d (%.2f, %.2f), pausing %.1f s", i, cx, cy, pause_time
            )

            # Pause at waypoint
            if pause_time > 0 and not rospy.is_shutdown():
                rospy.sleep(pause_time)

        if not loop:
            rospy.loginfo("All waypoints visited, loop=False, stopping.")
            break

        rospy.loginfo("Completed waypoint loop, restarting...")

    rospy.loginfo("move_target shutting down.")


if __name__ == "__main__":
    main()
