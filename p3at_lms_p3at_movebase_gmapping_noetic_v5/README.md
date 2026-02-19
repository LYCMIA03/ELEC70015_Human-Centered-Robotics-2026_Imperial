# P3-AT + LMS200 (2D lidar) ROS1 Noetic Simulation and Navigation

This zip contains a minimal ROS1 Noetic (Ubuntu 20.04) catkin workspace that supports:
- Gazebo simulation of a P3-AT style skid-steer robot
- 2D lidar (LMS200-like) publishing /scan
- gmapping (slam_gmapping node) for mapping (and map->odom)
- move_base for global planning and local obstacle avoidance
- Target following that feeds dynamic goals to move_base
  - In simulation the target is a Gazebo model named "target"
  - In real robot the target can come from your detector (YOLO), by publishing /target_pose

## Tested with
- ROS Noetic
- Gazebo (gazebo_ros)

## Install dependencies
On Ubuntu 20.04 with ROS Noetic installed:
- sudo apt update
- sudo apt install -y ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-xacro ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher ros-noetic-slam-gmapping ros-noetic-move-base ros-noetic-map-server ros-noetic-amcl \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-rviz ros-noetic-gazebo-msgs

Optional teleop:
- sudo apt install -y ros-noetic-teleop-twist-keyboard

## Build
- cd catkin_ws
- catkin_make
- source devel/setup.bash

## Run mapping + navigation + target following
This runs Gazebo, gmapping, move_base, RViz, and the target follower.
The follower reads a moving target pose and sends goals to move_base.

- roslaunch p3at_lms_navigation mapping.launch

### Move the target in simulation
In Gazebo:
- In the left panel, select the model "target"
- Use the translate tool to drag it around

The node target_follower/gazebo_target_publisher.py publishes /target_pose from Gazebo model state.

Implementation note
- /target_pose from Gazebo is published in the base_link frame by default.
- This avoids mixing Gazebo world coordinates with ROS odometry frames.

### Manual target without Gazebo model (optional)
If you want to set the target by RViz "2D Nav Goal", run:
- roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false use_rviz_goal_relay:=true

Now RViz "2D Nav Goal" publishes /move_base_simple/goal, which is relayed to /target_pose.

## Run navigation on a saved map (AMCL)
This uses a placeholder demo_map.
Replace map_file with your saved map.

- roslaunch p3at_lms_navigation nav.launch map_file:=/path/to/your_map.yaml

## How to connect your YOLO target later
Publish a PoseStamped to /target_pose.
The pose can be in map, odom, or base_link frame.
target_follower will transform it to the move_base global frame (map) before sending the goal.

Example:
- rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"

## Notes
- This is a minimal simulation. The robot geometry and dynamics are approximate.
- Tune move_base and gmapping parameters for your real P3-AT and LMS200.

move_base note
- For stable mapping, this config disables move_base recovery behaviors and relaxes goal yaw tolerance.
- LMS200 in real hardware is typically serial (RS232/RS422). In real robot you will replace Gazebo lidar with your driver publishing /scan.


## Conda note
If you run inside a conda Python, avoid nodes that require PyKDL. This workspace's target follower avoids PyKDL.
