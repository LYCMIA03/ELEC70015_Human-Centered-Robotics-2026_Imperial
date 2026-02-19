# P3-AT + LMS200 Navigation System (ROS1 Noetic)

A ROS1 Noetic workspace for the **ELEC70015 Human-Centered Robotics** course at Imperial College London. It provides both **Gazebo simulation** and **real robot** launch files for a Pioneer 3-AT mobile robot equipped with a SICK LMS200 2D lidar.

The LMS200 lidar handles **all** mapping, localization, path planning, and local obstacle avoidance. A depth camera (Orbbec Femto Bolt) is reserved for future target detection only and is not involved in navigation.

## Table of Contents

- [System Overview](#system-overview)
- [Hardware](#hardware)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Build](#build)
- [Usage: Gazebo Simulation](#usage-gazebo-simulation)
  - [Mapping (gmapping)](#1-mapping-gmapping)
  - [Navigation on a Saved Map (AMCL)](#2-navigation-on-a-saved-map-amcl)
  - [Target Following](#3-target-following)
- [Usage: Real Robot](#usage-real-robot)
  - [Multi-Machine Setup](#multi-machine-setup)
  - [Step 1 - Start Base Driver (Raspberry Pi)](#step-1---start-base-driver-raspberry-pi)
  - [Step 2a - Mapping (Jetson)](#step-2a---mapping-jetson)
  - [Step 2b - Navigation (Jetson)](#step-2b---navigation-jetson)
- [Key Topics and TF Frames](#key-topics-and-tf-frames)
- [Parameter Tuning](#parameter-tuning)
- [Connecting a YOLO Target Detector](#connecting-a-yolo-target-detector)
- [Known Issues and Notes](#known-issues-and-notes)

---

## System Overview

```
                        Gazebo Simulation                    Real Robot
                        ─────────────────                    ──────────
Lidar driver     :  Gazebo ray sensor plugin        sicktoolbox_wrapper (LMS200)
Odometry         :  Gazebo skid-steer plugin        RosAria (P3-AT encoders)
SLAM mapping     :  slam_gmapping                   slam_gmapping
Localization     :  AMCL                            AMCL
Path planning    :  move_base (NavfnROS + DWA)      move_base (NavfnROS + DWA)
Target following :  target_follower node             target_follower node
```

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| Mobile base | Pioneer 3-AT (P3-AT) | Skid-steer chassis, encoder odometry |
| 2D Lidar | SICK LMS200 | Mapping, localization, obstacle avoidance (serial RS-232/422) |
| Depth camera | Orbbec Femto Bolt | Future YOLO target detection (depth + RGB) |
| Main compute | Jetson ORIN NANO | Runs lidar driver, SLAM, navigation, target detection |
| Base compute | Raspberry Pi | Runs RosAria chassis driver, publishes /odom and /cmd_vel |

## Package Structure

```
catkin_ws/src/
├── p3at_lms_description/    # URDF/Xacro robot model (P3-AT + laser link)
│   └── urdf/p3at_lms.urdf.xacro
├── p3at_lms_gazebo/         # Gazebo world, target model, simulation launch
│   ├── launch/sim.launch
│   └── worlds/p3at_lms.world
├── p3at_lms_navigation/     # Navigation config, params, and all launch files
│   ├── launch/
│   │   ├── mapping.launch              # Sim: Gazebo + gmapping + move_base
│   │   ├── nav.launch                  # Sim: Gazebo + AMCL + move_base
│   │   ├── real_robot_mapping.launch   # Real: LMS200 + gmapping + move_base
│   │   └── real_robot_nav.launch       # Real: LMS200 + AMCL + move_base
│   └── param/
│       ├── gmapping.yaml           # gmapping SLAM parameters
│       ├── amcl.yaml               # AMCL localization parameters
│       ├── move_base.yaml          # NavfnROS + DWA planner config
│       ├── costmap_common.yaml     # Shared costmap (footprint, inflation)
│       ├── global_costmap.yaml     # Global costmap layers
│       └── local_costmap.yaml      # Local costmap layers (obstacle + inflation)
├── target_follower/         # Target following system
│   ├── scripts/
│   │   ├── target_follower.py          # Subscribes /target_pose, sends MoveBaseGoal
│   │   ├── gazebo_target_publisher.py  # Publishes Gazebo model pose as /target_pose
│   │   └── goal_to_target_relay.py     # Relays RViz 2D Nav Goal to /target_pose
│   └── launch/target_follow.launch
└── p3at_base/               # Real robot base driver (runs on Raspberry Pi)
    ├── launch/base.launch              # RosAria + odom republisher
    └── scripts/odom_republisher.py     # Republishes /RosAria/pose as /odom
```

## Dependencies

### System Requirements

- Ubuntu 20.04
- ROS Noetic (full desktop or ros-base)

### ROS Packages (Simulation)

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-msgs \
  ros-noetic-xacro \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-slam-gmapping \
  ros-noetic-move-base \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-rviz
```

### ROS Packages (Real Robot - Jetson)

In addition to the packages above (excluding Gazebo if not needed):

```bash
sudo apt install -y ros-noetic-sicktoolbox-wrapper
```

If `sicktoolbox_wrapper` is not available via apt, build from source:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/sicktoolbox_wrapper.git
cd ~/catkin_ws && catkin_make
```

### ROS Packages (Real Robot - Raspberry Pi)

```bash
sudo apt install -y ros-noetic-rosaria
```

If not available via apt, build RosAria from source with AriaCoda:
```bash
cd ~/catkin_ws/src
git clone https://github.com/reedhedges/AriaCoda.git
cd AriaCoda && make && sudo make install
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
cd ~/catkin_ws && catkin_make
```

### Optional

```bash
sudo apt install -y ros-noetic-teleop-twist-keyboard
```

## Build

```bash
cd catkin_ws
catkin_make
source devel/setup.bash    # or setup.zsh for zsh users
```

## Usage: Gazebo Simulation

All simulation launch files start Gazebo, spawn the robot, and open RViz automatically.

### 1. Mapping (gmapping)

Launch Gazebo with gmapping SLAM, move_base, and target following:

```bash
roslaunch p3at_lms_navigation mapping.launch
```

This starts:
- Gazebo with P3-AT robot and two box obstacles
- `slam_gmapping` for real-time map building
- `move_base` with NavfnROS global planner + DWA local planner
- `target_follower` tracking a Gazebo model named "target"
- RViz for visualization

**Drive the robot** by moving the "target" model in Gazebo (select it in the left panel, use the translate tool to drag). The robot will follow automatically.

**Alternatively**, use keyboard teleop in a separate terminal:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Save the map** once you are satisfied with the coverage:
```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

This creates `my_map.pgm` and `my_map.yaml` in the maps directory.

### 2. Navigation on a Saved Map (AMCL)

Load a previously saved map and use AMCL for localization:

```bash
roslaunch p3at_lms_navigation nav.launch map_file:=/absolute/path/to/my_map.yaml
```

In RViz:
- Use **2D Pose Estimate** to set the robot's initial pose on the map
- Use **2D Nav Goal** to send navigation goals (when using `use_rviz_goal_relay:=true`)

### 3. Target Following

**Gazebo model tracking** (default): The `gazebo_target_publisher` node reads the "target" model pose from Gazebo and publishes it as `/target_pose`. The `target_follower` node sends this as a `MoveBaseGoal`.

**RViz goal relay** (manual target): Use RViz "2D Nav Goal" as the target source:
```bash
roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false use_rviz_goal_relay:=true
```

**Custom target source**: Publish a `geometry_msgs/PoseStamped` to `/target_pose` from any node (e.g., your YOLO detector). The pose can be in `map`, `odom`, or `base_link` frame -- `target_follower` handles the TF transform.

---

## Usage: Real Robot

The real robot system is split across two machines:
- **Raspberry Pi**: Runs the P3-AT chassis driver (`p3at_base`)
- **Jetson ORIN NANO**: Runs the lidar driver, SLAM/localization, and navigation stack

### Multi-Machine Setup

On the **Raspberry Pi**, configure ROS networking before launching:
```bash
export ROS_MASTER_URI=http://<jetson_ip>:11311
export ROS_IP=<raspi_ip>
```

On the **Jetson** (runs `roscore`):
```bash
export ROS_MASTER_URI=http://<jetson_ip>:11311
export ROS_IP=<jetson_ip>
```

### Step 1 - Start Base Driver (Raspberry Pi)

Connect the P3-AT via serial cable, then launch:

```bash
roslaunch p3at_base base.launch
# Or specify a different serial port:
roslaunch p3at_base base.launch serial_port:=/dev/ttyUSB0
```

This starts:
- `RosAria` node: reads encoder odometry, accepts velocity commands
- `odom_republisher`: republishes `/RosAria/pose` as the standard `/odom` topic
- TF broadcast: `odom` -> `base_link` (published by RosAria with `publish_aria_tf=true`)
- Topic remapping: `/RosAria/cmd_vel` remapped to `/cmd_vel`

### Step 2a - Mapping (Jetson)

Connect the LMS200 via serial (typically `/dev/ttyUSB0`), then launch:

```bash
roslaunch p3at_lms_navigation real_robot_mapping.launch
```

Launch arguments:
| Argument | Default | Description |
|----------|---------|-------------|
| `lms200_port` | `/dev/ttyUSB0` | LMS200 serial port |
| `lms200_baud` | `38400` | LMS200 baud rate |
| `use_rviz` | `true` | Start RViz |
| `use_target_follower` | `false` | Enable target following |
| `use_rviz_goal_relay` | `false` | Relay RViz 2D Nav Goal to /target_pose |

This starts:
- `sicklms` (sicktoolbox_wrapper): LMS200 driver publishing `/scan` in the `laser` frame
- `robot_state_publisher`: publishes URDF-based TF (including `base_link` -> `laser`)
- `slam_gmapping`: builds the occupancy grid map
- `move_base`: path planning and obstacle avoidance
- RViz (optional)

Save the map after mapping:
```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

### Step 2b - Navigation (Jetson)

After mapping, use the saved map for AMCL-based navigation:

```bash
roslaunch p3at_lms_navigation real_robot_nav.launch map_file:=/absolute/path/to/my_map.yaml
```

Launch arguments are the same as mapping, plus:
| Argument | Default | Description |
|----------|---------|-------------|
| `map_file` | `...maps/demo_map.yaml` | Path to the saved map YAML |

This starts: `sicklms`, `robot_state_publisher`, `map_server`, `amcl`, `move_base`, and optionally RViz.

---

## Key Topics and TF Frames

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo plugin / sicklms | 2D lidar scan |
| `/odom` | `nav_msgs/Odometry` | Gazebo plugin / odom_republisher | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base | Velocity commands |
| `/map` | `nav_msgs/OccupancyGrid` | gmapping / map_server | Occupancy grid map |
| `/target_pose` | `geometry_msgs/PoseStamped` | gazebo_target_publisher / YOLO node | Target position for following |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | target_follower | Navigation goal |

### TF Tree

```
map -> odom -> base_footprint -> base_link -> laser
                                           -> (4x wheel links)
```

- `map` -> `odom`: Published by gmapping (during mapping) or AMCL (during navigation)
- `odom` -> `base_footprint`: Published by Gazebo skid-steer plugin / RosAria
- `base_footprint` -> `base_link`: Static, from URDF (identity transform)
- `base_link` -> `laser`: Static, from URDF

## Parameter Tuning

Key parameter files are in `p3at_lms_navigation/param/`:

**gmapping.yaml** - SLAM parameters:
- `particles: 30` - number of particles (increase for larger environments)
- `delta: 0.05` - map resolution (meters/pixel)
- `maxUrange: 8.0` - max usable range
- `linearUpdate: 0.2` / `angularUpdate: 0.2` - minimum movement before scan processing

**move_base.yaml** - Planner parameters:
- `max_vel_x: 0.6` - maximum forward velocity (m/s)
- `max_vel_theta: 1.2` - maximum rotational velocity (rad/s)
- `xy_goal_tolerance: 0.30` - position tolerance at goal (m)
- `yaw_goal_tolerance: 3.14` - orientation tolerance at goal (rad, relaxed for following)
- `recovery_behavior_enabled: false` - recovery disabled for stable mapping

**costmap_common.yaml** - Costmap parameters:
- `footprint: [[0.30, 0.25], [0.30, -0.25], [-0.30, -0.25], [-0.30, 0.25]]` - P3-AT footprint (m)
- `inflation_radius: 0.50` - inflation around obstacles (m)

**amcl.yaml** - Localization parameters:
- `min_particles: 300` / `max_particles: 2000`
- `laser_model_type: likelihood_field`
- `odom_model_type: diff`

These parameters are tuned for simulation. Adjust them for real hardware based on actual robot dynamics and LMS200 characteristics.

## Connecting a YOLO Target Detector

The `target_follower` node subscribes to `/target_pose` (`geometry_msgs/PoseStamped`). To integrate a YOLO-based detector:

1. Create a node that:
   - Subscribes to the depth camera RGB + depth topics
   - Runs YOLO inference to detect the target object
   - Computes the 3D position from the depth image
   - Publishes a `PoseStamped` to `/target_pose`

2. Enable target following in the launch file:
   ```bash
   roslaunch p3at_lms_navigation real_robot_nav.launch use_target_follower:=true
   ```

3. Manual test without a detector:
   ```bash
   rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

## Known Issues and Notes

- **Simulation fidelity**: The URDF model and Gazebo dynamics are approximate. Real P3-AT behavior (especially skid-steer turning) will differ.
- **LMS200 FOV**: The real LMS200 has a 180-degree field of view. The Gazebo simulation uses a similar configuration. Verify `resolution` and `measuring_units` match your LMS200 firmware settings.
- **move_base recovery**: Recovery behaviors are disabled (`recovery_behavior_enabled: false`) to prevent erratic behavior during mapping. Re-enable for production navigation if needed.
- **Conda environments**: If running inside a conda Python environment, avoid nodes that require `PyKDL`. The `target_follower` in this workspace uses pure-Python quaternion math and does not depend on PyKDL.
- **Gazebo reference frame**: The `target_follow.launch` uses `base_footprint` as the Gazebo reference frame (not `base_link`) because Gazebo merges fixed joints -- the frame `p3at::base_link` does not exist in the Gazebo model, only `p3at::base_footprint`.

## Status

- [x] Gazebo simulation (gmapping + move_base + target following) -- verified
- [x] Map saving and loading (map_server + AMCL) -- verified
- [x] Real robot launch files (mapping + navigation) -- created, pending hardware test
- [x] Raspberry Pi base driver package (p3at_base) -- created, pending hardware test
- [ ] YOLO target detection node -- not started
- [ ] Multi-machine network configuration scripts -- not started
- [ ] Real hardware parameter tuning -- not started
