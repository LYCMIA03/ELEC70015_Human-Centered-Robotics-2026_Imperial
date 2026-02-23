# ELEC70015 Human-Centered Robotics 2026 — Imperial College London

Pioneer 3-AT navigation stack for simulation and real-robot deployment.  
ROS1 Noetic · Ubuntu 20.04 · Gazebo 11.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Repository & Package Structure](#repository--package-structure)
4. [Prerequisites & Dependencies](#prerequisites--dependencies)
5. [Build](#build)
6. [Part A — Simulation](#part-a--simulation)
   - [A-1 Launch Gazebo Simulation](#a-1-launch-gazebo-simulation)
   - [A-2 Mapping with Manual Control](#a-2-mapping-with-manual-control)
   - [A-3 Send Navigation Goals](#a-3-send-navigation-goals)
   - [A-4 Target Following](#a-4-target-following)
   - [A-5 Autonomous Frontier Exploration](#a-5-autonomous-frontier-exploration)
   - [A-6 AMCL Verification](#a-6-amcl-verification)
   - [A-7 Full Pipeline Script](#a-7-full-pipeline-script)
7. [Simulation Verification Tests](#simulation-verification-tests)
8. [Part B — Real Robot Deployment](#part-b--real-robot-deployment)
   - [B-1 Network Architecture](#b-1-network-architecture)
   - [B-2 Hardware Setup](#b-2-hardware-setup)
   - [B-3 Option A — Unitree L1 (Primary)](#b-3-option-a--unitree-l1-primary)
   - [B-4 Option B — SICK LMS200 (Backup)](#b-4-option-b--sick-lms200-backup)
9. [Key Topics Reference](#key-topics-reference)
   - [Node List](#node-list)
   - [Topic Pub/Sub Reference](#topic-pubsub-reference)
   - [TF Tree](#tf-tree)
10. [Parameter Tuning Guide](#parameter-tuning-guide)
11. [Autonomous Explorer Algorithm](#autonomous-explorer-algorithm)
12. [Connecting a YOLO Target Detector](#connecting-a-yolo-target-detector)
13. [Known Issues and Notes](#known-issues-and-notes)
14. [Git Workflow](#git-workflow)
15. [Resources](#resources)
16. [Status](#status)
17. [Post-Installation Checklist](#post-installation-checklist)

---

## System Overview

This workspace implements **two parallel navigation stacks** for the Pioneer 3-AT robot.

| | Stack A — Unitree (Primary) | Stack B — SICK (Backup) |
|---|---|---|
| **Sensor** | Unitree 4D Lidar L1 | SICK LMS200 |
| **FOV** | 360° | 180° |
| **Max range** | 30 m | 80 m |
| **Scan topic** | `/unitree/scan` | `/scan` |
| **Sensor frame** | `unitree_lidar` | `laser` |
| **Param directory** | `param/unitree/` | `param/` |
| **RViz config** | `rviz/nav_unitree.rviz` | `rviz/nav.rviz` |
| **Priority** | **Primary — use this first** | Fallback if Unitree unavailable |

**Unitree L1** provides 360° coverage, enabling faster frontier discovery and better obstacle avoidance.  
**SICK LMS200** is the fallback when the Unitree hardware is unavailable; all functionality is preserved with a 180° FOV.

Both stacks share the same `autonomous_explorer.py` frontier explorer, `target_follower` nodes, and underlying move_base planner. The only differences are sensor topic remappings and parameter files.

### Workspace Layout

```
catkin_ws/   <- Primary workspace (both stacks built here)
ros_ws/      <- Secondary workspace (AMR driver config, real-robot support)
tools/       <- Utility scripts (camera relay, depth inspector, source helpers)
```

---

## Hardware Configuration

### Robot: Pioneer 3-AT

| Property | Value |
|----------|-------|
| Drive type | 4-wheel skid-steer |
| Footprint | `[[0.32, 0.27], [0.32, -0.27], [-0.32, -0.27], [-0.32, 0.27]]` m |
| Inscribed radius | 0.27 m |
| Circumscribed radius | 0.42 m |
| Base link to footprint | Identity (ROS convention) |

### Sensor A: Unitree 4D Lidar L1 (Primary)

| Property | Value |
|----------|-------|
| FOV | 360° horizontal |
| Range | 0.1 – 30 m |
| Scan frequency | ~10 Hz |
| ROS topic | `/unitree/scan` (`sensor_msgs/LaserScan`) |
| TF frame | `unitree_lidar` |
| Driver package | Unitree ROS SDK |

### Sensor B: SICK LMS200 (Backup)

| Property | Value |
|----------|-------|
| FOV | 180° horizontal |
| Resolution | 0.5° or 1° (firmware-dependent) |
| Range | 0.1 – 80 m |
| Scan frequency | ~75 Hz |
| ROS topic | `/scan` (`sensor_msgs/LaserScan`) |
| TF frame | `laser` |
| Driver package | `sicktoolbox_wrapper` |
| Interface | RS-232/RS-422 serial |

### Compute Platform

| Node | Hardware | Role |
|------|----------|------|
| Jetson | NVIDIA Jetson Orin/Xavier | ROS master, SLAM, navigation, Unitree driver |
| Pi | Raspberry Pi 4 | P3-AT base driver (RosAria) |
| (Simulation) | Developer laptop/workstation | All nodes in one process |

---

## Repository & Package Structure

```
catkin_ws/src/
├── CMakeLists.txt                    # Catkin top-level
├── p3at_base/                        # Pi-side P3-AT base driver (real robot)
│   └── launch/base.launch
├── p3at_lms_description/             # URDF/Xacro robot models
│   ├── urdf/p3at_with_lms.urdf.xacro       (SICK model)
│   └── urdf/p3at_with_unitree.urdf.xacro   (Unitree model)
├── p3at_lms_gazebo/                  # Gazebo worlds and sim launch files
│   ├── launch/
│   │   ├── sim.launch                (SICK simulation)
│   │   └── sim_unitree.launch        (Unitree simulation)
│   └── worlds/
│       └── complex_maze.world        (12.2×12.2 m test maze)
└── p3at_lms_navigation/              # Navigation stack (both sensors)
    ├── launch/
    │   ├── mapping.launch                    # SICK: manual mapping
    │   ├── mapping_unitree.launch            # Unitree: manual mapping
    │   ├── nav.launch                        # SICK: AMCL navigation
    │   ├── nav_unitree.launch                # Unitree: AMCL navigation
    │   ├── auto_mapping.launch               # SICK: autonomous exploration
    │   ├── auto_mapping_unitree.launch       # Unitree: autonomous exploration  <- PRIMARY
    │   ├── auto_amcl_verify.launch           # SICK: AMCL verifier
    │   ├── auto_amcl_verify_unitree.launch   # Unitree: AMCL verifier
    │   ├── real_robot_mapping.launch         # SICK: real-robot mapping
    │   ├── real_robot_mapping_unitree.launch # Unitree: real-robot mapping
    │   ├── real_robot_nav.launch             # SICK: real-robot nav
    │   └── real_robot_nav_unitree.launch     # Unitree: real-robot nav
    ├── param/                        # SICK / default parameters
    │   ├── gmapping.yaml
    │   ├── costmap_common.yaml
    │   ├── global_costmap.yaml
    │   ├── local_costmap.yaml
    │   ├── move_base.yaml
    │   └── amcl.yaml
    ├── param/unitree/                # Unitree-specific parameters (tuned)
    │   ├── gmapping.yaml
    │   ├── costmap_common.yaml       # inflation 0.45 / scale 5.0
    │   ├── global_costmap.yaml       # 360° obstacle source (unitree_lidar frame)
    │   ├── local_costmap.yaml        # inflation 0.35 / scale 8.0 (split from global)
    │   ├── move_base.yaml            # DWA tuned for narrow corridors
    │   └── amcl.yaml
    ├── rviz/
    │   ├── nav.rviz                  # SICK RViz config
    │   └── nav_unitree.rviz          # Unitree RViz config
    ├── scripts/
    │   ├── autonomous_explorer.py    # Frontier exploration node (shared by both stacks)
    │   ├── amcl_verifier.py          # AMCL accuracy verifier (shared)
    │   ├── waypoint_test.py          # 3-waypoint navigation test
    │   └── test_standoff_face.py     # Unit tests (21 tests)
    └── maps/                         # Saved maps (git-ignored)
```

---

## Prerequisites & Dependencies

### System

```bash
sudo apt-get install -y \
  ros-noetic-desktop-full \
  ros-noetic-gmapping \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-slam-gmapping \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-tf2-ros \
  ros-noetic-actionlib \
  python3-catkin-tools \
  python3-rospy \
  python3-numpy
```

### Real Robot Only

```bash
# SICK LMS200 driver
sudo apt-get install ros-noetic-sicktoolbox-wrapper

# P3-AT base driver (RosAria)
# Clone and build from: https://github.com/amor-ros-pkg/rosaria

# Unitree L1 driver
# Follow Unitree ROS SDK instructions for your Jetson platform
```

---

## Build

```bash
cd catkin_ws
catkin_make
source devel/setup.bash   # or setup.zsh
```

Verify:

```bash
rospack find p3at_lms_navigation   # should print the package path
```

---

## Part A — Simulation

### A-1 Launch Gazebo Simulation

**Option A — Unitree Stack (Primary, recommended):**

```bash
roslaunch p3at_lms_gazebo sim_unitree.launch \
  world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world
```

**Option B — SICK Stack (Backup):**

```bash
roslaunch p3at_lms_gazebo sim.launch \
  world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world
```

Both launch Gazebo physics simulator, the robot URDF model with the respective lidar sensor, and `robot_state_publisher` for TF.

### A-2 Mapping with Manual Control

**Option A — Unitree:**

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch
```

**Option B — SICK:**

```bash
roslaunch p3at_lms_navigation mapping.launch
```

Both include: Gazebo + slam_gmapping + move_base + RViz.  
Default: `use_gazebo_target:=true`, `move_target:=false`.

**Save the map when coverage is sufficient:**

```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

### A-3 Send Navigation Goals

In RViz, use **"2D Nav Goal"**, or publish directly:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{ header: {frame_id: 'map'},
     pose: { position: {x: 2.0, y: 1.0, z: 0.0},
             orientation: {w: 1.0} } }"
```

### A-4 Target Following

**Static target (track Gazebo model):**

```bash
# Unitree
roslaunch p3at_lms_navigation mapping_unitree.launch use_gazebo_target:=true

# SICK
roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=true
```

Drag the `target` model in Gazebo — the robot follows.

**Dynamic moving target with standoff and face:**

```bash
roslaunch p3at_lms_navigation mapping.launch \
  move_target:=true \
  target_speed:=0.3 \
  target_pause:=2.0 \
  standoff_distance:=1.0 \
  face_target:=true
```

**Manual target via RViz goal relay:**

```bash
roslaunch p3at_lms_navigation mapping.launch \
  use_gazebo_target:=false \
  use_rviz_goal_relay:=true
```

Then use **"2D Nav Goal"** in RViz — the robot follows that pose as a target.

### A-5 Autonomous Frontier Exploration

The `autonomous_explorer.py` node implements frontier-based exploration with an improved algorithm (see [Autonomous Explorer Algorithm](#autonomous-explorer-algorithm)).

#### Option A — Unitree Stack (Primary)

```bash
roslaunch p3at_lms_navigation auto_mapping_unitree.launch
```

With custom timeout:

```bash
roslaunch p3at_lms_navigation auto_mapping_unitree.launch \
  exploration_timeout:=300 \
  gui:=true
```

Headless (no Gazebo rendering window):

```bash
roslaunch p3at_lms_navigation auto_mapping_unitree.launch \
  gui:=false \
  exploration_timeout:=300
```

#### Option B — SICK Stack (Backup)

```bash
roslaunch p3at_lms_navigation auto_mapping.launch \
  exploration_timeout:=300 \
  gui:=true
```

#### Launch File Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `exploration_timeout` | 600 s | Stop exploring after this many seconds; 0 = no limit |
| `min_frontier_size` | 3 cells | Ignore frontier clusters smaller than this |
| `initial_wait` | 12 s | Wait for map to initialise before first goal |
| `gui` | true | Show Gazebo rendering window |
| `map_save_name` | `explored_map_unitree` | Base filename for the auto-saved map |

#### Explorer Node ROS Parameters (set in launch file `<param>`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `robot_radius` | 0.25 m | Clearance radius for reachability checks |
| `frontier_blacklist_radius` | 0.2 m | Exclusion radius around blacklisted goals |
| `goal_timeout` | 30 s | Abandon current goal after this time |
| `spin_in_place_first` | true | Initial 360° scan before exploring (Unitree only) |
| `save_map` | true | Auto-save map on completion |
| `map_save_path` | `maps/explored_map_unitree` | Map save path |

#### Expected Output

```
[Explorer] Coverage: 12.6% | Goals sent: 17 | Successes: 2 | Traveled: 17.2m
```

**Best observed result (Unitree, 300 s):** ~12.6% coverage.  
Maze theoretical maximum coverage is ~18% (limited by 0.2 m-thick walls in 0.05 m/cell map).

### A-6 AMCL Verification

Runs after autonomous mapping using the saved map.

**Option A — Unitree:**

```bash
roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true
```

**Option B — SICK:**

```bash
roslaunch p3at_lms_navigation auto_amcl_verify.launch gui:=true
```

The `amcl_verifier.py` node:

1. Loads the saved map via `map_server`
2. Teleports the robot to 5 predefined waypoints
3. Measures AMCL convergence time and position error at each waypoint
4. Saves `maps/amcl_report.txt` and `maps/amcl_report.json`

**Reference result (SICK stack):** mean position error 0.089 m, convergence 1.0 s.

### A-7 Full Pipeline Script

Run both phases (autonomous mapping → AMCL verification) in sequence:

```bash
bash run_full_pipeline.sh
```

---

## Simulation Verification Tests

### Test 1 — gmapping + Waypoint Navigation

**Purpose:** Verify SLAM and navigation stack end-to-end.

```bash
# Terminal 1: launch simulation
roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false

# Terminal 2: run 3-waypoint sequential test
python3 catkin_ws/src/p3at_lms_navigation/scripts/waypoint_test.py
```

Expected: all 3 waypoints SUCCEEDED, position errors < 0.2 m.

### Test 2 — 3-Waypoint Sequential Navigation

**Purpose:** Confirm the navigator chains multiple goals.

Same as Test 1. `waypoint_test.py` sends 3 sequential goals and prints PASS/FAIL.

### Test 3 — Unit Tests (Standoff + Face Target Logic)

**Purpose:** Validate standoff and face_target math independent of the robot.

```bash
python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py
```

Expected: **21/21 tests pass**.

### Test 4 — *(Removed — superseded by Test 6)*

### Test 5 — Dynamic Target Following

**Purpose:** Verify the robot follows a moving target with standoff.

```bash
roslaunch p3at_lms_navigation mapping.launch \
  move_target:=true \
  target_speed:=0.3 \
  target_pause:=2.0 \
  standoff_distance:=1.0 \
  face_target:=true
```

Verify: robot maintains ~1 m separation, faces target, no collision.

### Test 6a — Autonomous Frontier Exploration

**Purpose:** Verify the autonomous explorer maps the complex maze.

```bash
# Unitree (primary)
roslaunch p3at_lms_navigation auto_mapping_unitree.launch exploration_timeout:=300

# SICK (backup)
roslaunch p3at_lms_navigation auto_mapping.launch exploration_timeout:=300
```

Expected: terminal prints coverage updates; map file saved on completion.

### Test 6b — AMCL Accuracy Verification

**Purpose:** Verify AMCL localisation accuracy on the explored map.

```bash
roslaunch p3at_lms_navigation auto_amcl_verify.launch gui:=true
```

Expected: mean position error < 0.30 m, convergence time < 5 s.

---

## Part B — Real Robot Deployment

### B-1 Network Architecture

```
+-------------------+      Direct Ethernet     +------------------+
|  Jetson (Orin)    | <-----------------------> |  Raspberry Pi    |
|  192.168.50.1     |     192.168.50.0/24       |  192.168.50.2    |
|  ROS Master       |                           |  P3-AT driver    |
|  Unitree L1       |                           |  RosAria         |
+-------------------+                           +------------------+
```

Both machines run ROS Noetic inside Docker containers using `--net=host`.

**Environment variables — Jetson:**

```bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
```

**Environment variables — Pi:**

```bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
```

Configure static IPs with Netplan; verify with `ping 192.168.50.x` from each side.

### B-2 Hardware Setup

#### Start roscore (Jetson)

```bash
roscore
```

#### Start P3-AT Base Driver (Pi)

```bash
roslaunch p3at_base base.launch
```

| Direction | Topics |
|-----------|--------|
| Publishes | `/odom`, `/tf (odom->base_link)`, `/battery_voltage` |
| Subscribes | `/cmd_vel` |

#### Source Helper Scripts

```bash
source tools/source_ros.sh    # bash
source tools/source_ros.zsh   # zsh
```

---

### B-3 Option A — Unitree L1 (Primary)

> **Use this stack for all real-robot deployments unless Unitree hardware is unavailable.**

#### Nodes — Unitree Real-Robot Stack

| Node | Machine | Key Topics |
|------|---------|------------|
| `roscore` | Jetson | — |
| `p3at_base` (RosAria) | Pi | pub: `/odom`, `/tf`; sub: `/cmd_vel` |
| `unitree_lidar_ros` | Jetson | pub: `/unitree/scan` |
| `robot_state_publisher` | Jetson | pub: `/tf_static` |
| `slam_gmapping` | Jetson | sub: `/unitree/scan`, `/tf`; pub: `/map`, `/tf (map->odom)` |
| `move_base` | Jetson | sub: `/map`, `/unitree/scan`, `/odom`, `/tf`; pub: `/cmd_vel` |
| `autonomous_explorer` | Jetson | sub: `/map`, `/odom`; pub: action goals to `/move_base` |

#### Communication Interfaces — Unitree Stack

| Interface | ROS Mechanism | Details |
|-----------|--------------|---------|
| Frontier navigation goals | actionlib `SimpleActionClient` | MoveBaseAction on `/move_base` |
| Goal cancellation | actionlib | `/move_base/cancel` |
| Costmap clearing | Service call | `/move_base/clear_costmaps` |
| Scan data | Topic (10 Hz) | `/unitree/scan` — `sensor_msgs/LaserScan` |
| Map data | Topic | `/map` — `nav_msgs/OccupancyGrid` |
| Velocity commands | Topic | `/cmd_vel` — `geometry_msgs/Twist` |
| Odometry | Topic | `/odom` — `nav_msgs/Odometry` |
| Target following | Topic | `/target_pose` — `geometry_msgs/PoseStamped` |

#### Phase 1 — Real-Robot Mapping (Unitree)

```bash
# On Jetson
roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch
```

Optional — launch autonomous explorer for unmanned mapping:

```bash
rosrun p3at_lms_navigation autonomous_explorer.py \
  _exploration_timeout:=300 \
  _robot_radius:=0.25 \
  _goal_timeout:=30.0
```

Save the map:

```bash
rosrun map_server map_saver -f $HOME/maps/real_robot_map
```

#### Phase 2 — Real-Robot Navigation (Unitree)

```bash
roslaunch p3at_lms_navigation real_robot_nav_unitree.launch \
  map_file:=$HOME/maps/real_robot_map.yaml
```

1. In RViz, use **"2D Pose Estimate"** to set the initial position
2. Wait for the AMCL particle cloud to converge
3. Send goals with **"2D Nav Goal"**

#### Parameter Files — Unitree Stack

| File | Key Tuned Values |
|------|-----------------|
| `param/unitree/costmap_common.yaml` | `inflation_radius: 0.45`, `cost_scaling_factor: 5.0`, `footprint_padding: 0.02` |
| `param/unitree/global_costmap.yaml` | Obstacle source: `/unitree/scan`, frame `unitree_lidar` |
| `param/unitree/local_costmap.yaml` | `inflation_radius: 0.35`, `cost_scaling_factor: 8.0` (smaller than global) |
| `param/unitree/move_base.yaml` | `clearing_rotation_allowed: false`, `vx_samples: 20`, `vtheta_samples: 40`, `xy_goal_tolerance: 0.55` |
| `param/unitree/gmapping.yaml` | `maxUrange: 10.0` (extended for 30 m sensor) |

---

### B-4 Option B — SICK LMS200 (Backup)

> **Use this stack only if Unitree hardware is unavailable.**

#### Connect SICK LMS200

```bash
ls /dev/ttyUSB0              # confirm device
sudo chmod 666 /dev/ttyUSB0  # set permissions
# Persistent: sudo usermod -aG dialout $USER
```

#### Nodes — SICK Real-Robot Stack

| Node | Machine | Key Topics |
|------|---------|------------|
| `roscore` | Jetson | — |
| `p3at_base` (RosAria) | Pi | pub: `/odom`, `/tf`; sub: `/cmd_vel` |
| `sicktoolbox_wrapper` | Jetson | pub: `/scan` |
| `robot_state_publisher` | Jetson | pub: `/tf_static` |
| `slam_gmapping` | Jetson | sub: `/scan`, `/tf`; pub: `/map`, `/tf (map->odom)` |
| `move_base` | Jetson | sub: `/map`, `/scan`, `/odom`, `/tf`; pub: `/cmd_vel` |
| `autonomous_explorer` | Jetson | sub: `/map`, `/odom`; pub: action goals to `/move_base` |

Interfaces are identical to the Unitree stack, with `/scan` instead of `/unitree/scan`.

#### Phase 1 — Real-Robot Mapping (SICK)

```bash
roslaunch p3at_lms_navigation real_robot_mapping.launch
```

Save the map:

```bash
rosrun map_server map_saver -f $HOME/maps/real_robot_map_sick
```

#### Phase 2 — Real-Robot Navigation (SICK)

```bash
roslaunch p3at_lms_navigation real_robot_nav.launch \
  map_file:=$HOME/maps/real_robot_map_sick.yaml
```

#### LMS200 Firmware Settings to Verify

| Setting | Typical value |
|---------|--------------|
| Baud rate | 500000 bps |
| Measuring units | cm |
| Resolution | 0.5° or 1° |
| FOV | 180° |

Match these to the `sicktoolbox_wrapper` launch arguments.

---

## Key Topics Reference

### Node List

#### Simulation — Unitree Stack

| Node | Package | Role |
|------|---------|------|
| `/gazebo` | `gazebo_ros` | Physics simulator; pub `/unitree/scan`, `/odom`, `/tf (odom->base_footprint)` |
| `/joint_state_publisher` | `joint_state_publisher` | pub `/joint_states` |
| `/robot_state_publisher` | `robot_state_publisher` | pub `/tf_static` and dynamic `/tf` from URDF |
| `/slam_gmapping` | `slam_gmapping` | sub `/unitree/scan`+`/tf`; pub `/map`, `/tf (map->odom)` |
| `/move_base` | `move_base` | sub `/map`,`/unitree/scan`,`/odom`,`/tf`; pub `/cmd_vel` |
| `/autonomous_explorer` ¹ | `p3at_lms_navigation` | sub `/map`,`/odom`; pub MoveBase action goals |
| `/rviz` | `rviz` | Visualisation |
| `/gazebo_target_publisher` ² | `target_follower` | Calls Gazebo service; pub `/target_pose` |
| `/target_follower` ² | `target_follower` | sub `/target_pose`; sends MoveBaseAction goals |
| `/move_target` ³ | `target_follower` | Moves Gazebo target model along waypoints |
| `/goal_to_target_relay` ⁴ | `target_follower` | Relays `/move_base_simple/goal` to `/target_pose` |

¹ Active only in `auto_mapping_unitree.launch` / `auto_mapping.launch`.  
² Active only when `use_gazebo_target:=true`.  
³ Active only when `move_target:=true`.  
⁴ Active only when `use_rviz_goal_relay:=true`.

**SICK Stack:** identical, with `/unitree/scan` replaced by `/scan` and frame `unitree_lidar` → `laser`.

### Topic Pub/Sub Reference

#### Unitree Stack

| Topic | Message Type | Publisher(s) | Subscriber(s) |
|-------|-------------|--------------|---------------|
| `/unitree/scan` | `sensor_msgs/LaserScan` | `/gazebo` (Unitree plugin) | `/slam_gmapping`, `/move_base`, `/rviz` |
| `/odom` | `nav_msgs/Odometry` | `/gazebo` (skid-steer plugin) | `/move_base` |
| `/cmd_vel` | `geometry_msgs/Twist` | `/move_base` | `/gazebo` |
| `/map` | `nav_msgs/OccupancyGrid` | `/slam_gmapping` | `/move_base`, `/rviz`, `/autonomous_explorer` |
| `/map_metadata` | `nav_msgs/MapMetaData` | `/slam_gmapping` | — |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | `/slam_gmapping` | `/move_base`, `/rviz` |
| `/joint_states` | `sensor_msgs/JointState` | `/joint_state_publisher` | `/robot_state_publisher` |
| `/tf` | `tf2_msgs/TFMessage` | `/gazebo`, `/robot_state_publisher`, `/slam_gmapping` | all TF-aware nodes |
| `/tf_static` | `tf2_msgs/TFMessage` | `/robot_state_publisher` | all TF-aware nodes |
| `/clock` | `rosgraph_msgs/Clock` | `/gazebo` | all time-aware nodes |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz / user | `/move_base`, `/goal_to_target_relay` |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | `/autonomous_explorer`, `/target_follower` | `/move_base` |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | `/move_base` | `/autonomous_explorer`, `/target_follower` |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | `/move_base` | `/autonomous_explorer`, `/target_follower` |
| `/move_base/cancel` | `actionlib_msgs/GoalID` | `/autonomous_explorer`, `/target_follower` | `/move_base` |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | `/move_base` | — |
| `/move_base/NavfnROS/plan` | `nav_msgs/Path` | `/move_base` | `/rviz` |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | `/move_base` | `/rviz` |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `/move_base` | `/rviz` |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `/move_base` | `/rviz` |
| `/target_pose` | `geometry_msgs/PoseStamped` | `/gazebo_target_publisher` / YOLO node / relay | `/target_follower` |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | `/gazebo` | — |
| `/slam_gmapping/entropy` | `std_msgs/Float64` | `/slam_gmapping` | — |

**SICK Stack:** identical, with `/unitree/scan` replacing `/scan`.

### TF Tree

#### Simulation — Unitree Model

```
map
+-- odom                          [/slam_gmapping  ~20 Hz]
    +-- base_footprint            [/gazebo (skid-steer plugin)  ~50 Hz]
        +-- base_link             [/robot_state_publisher  static]
            +-- unitree_lidar     [/robot_state_publisher  static]   <- Unitree-specific
            +-- top_plate         [/robot_state_publisher  static]
            +-- front_sonar       [/robot_state_publisher  static]
            +-- back_sonar        [/robot_state_publisher  static]
            +-- p3at_front_left_axle   [static]
            |   +-- p3at_front_left_hub
            |       +-- p3at_front_left_wheel    [~10 Hz]
            +-- p3at_front_right_axle  [static]
            |   +-- p3at_front_right_hub
            |       +-- p3at_front_right_wheel   [~10 Hz]
            +-- p3at_back_left_axle    [static]
            |   +-- p3at_back_left_hub
            |       +-- p3at_back_left_wheel     [~10 Hz]
            +-- p3at_back_right_axle   [static]
                +-- p3at_back_right_hub
                    +-- p3at_back_right_wheel    [~10 Hz]
```

**SICK Model:** identical, with `unitree_lidar` replaced by `laser`.

#### TF Edge Summary

| TF Edge | Broadcaster | Rate | Notes |
|---------|-------------|------|-------|
| `map -> odom` | `/slam_gmapping` | ~20 Hz | Replaced by `/amcl` during navigation |
| `odom -> base_footprint` | `/gazebo` (sim) / RosAria (real robot) | ~50 Hz | |
| `base_footprint -> base_link` | `/robot_state_publisher` | static | Identity from URDF |
| `base_link -> unitree_lidar` | `/robot_state_publisher` | static | Unitree mount offset |
| `base_link -> laser` | `/robot_state_publisher` | static | SICK mount offset |
| Wheel frames ×4 | `/robot_state_publisher` | ~10 Hz | Driven by joint states |

> During **AMCL navigation**, `/slam_gmapping` is replaced by `map_server` + `/amcl`.
> The `map -> odom` edge is then maintained by `/amcl` at the laser scan rate.

---

## Parameter Tuning Guide

### Unitree Stack (`param/unitree/`) — Iteratively Tuned

These parameters were tuned for `complex_maze.world` — corridors ~1–3 m wide with P3-AT ~0.54 m wide.

#### `costmap_common.yaml`

```yaml
obstacle_range: 6.0
raytrace_range: 8.0
footprint: [[0.32, 0.27], [0.32, -0.27], [-0.32, -0.27], [-0.32, 0.27]]
footprint_padding: 0.02       # minimal padding to avoid excess expansion
inflation_radius: 0.45        # > inscribed_radius (0.27 m) for safety margin
cost_scaling_factor: 5.0      # exponential decay rate (global costmap)
transform_tolerance: 0.4
```

#### `local_costmap.yaml` — smaller inflation than global

```yaml
inflation_layer:
  inflation_radius: 0.35      # smaller than global: gives DWA more room in corridors
  cost_scaling_factor: 8.0    # steeper decay: keeps narrow lanes passable
```

**Why split global/local inflation?**  
The global costmap (0.45 m) gives NavFn a conservative path that avoids walls.  
The local costmap (0.35 m) gives DWA more room to find a valid trajectory in tight corridors.  
Without this split, DWA frequently aborts even when the global plan is valid.

#### `move_base.yaml` — DWA planner

```yaml
planner_patience: 15.0          # allow 15 s to find a global plan
controller_patience: 20.0       # allow 20 s for DWA to execute
max_planning_retries: 5
clearing_rotation_allowed: false  # CRITICAL: prevents P3-AT from tipping over

DWAPlannerROS:
  max_vel_x: 0.4
  max_vel_theta: 0.4
  acc_lim_theta: 0.8
  vx_samples: 20              # increased: better trajectory search in corridors
  vtheta_samples: 40          # increased: richer turning decisions
  sim_time: 1.5
  path_distance_bias: 32.0    # strong path-following preference
  goal_distance_bias: 28.0
  occdist_scale: 0.02         # low: allows close approach to walls in tight spaces
  xy_goal_tolerance: 0.55     # generous: P3-AT is a large robot
  latch_xy_goal_tolerance: true

NavfnROS:
  allow_unknown: true          # plan through unknown space (essential for exploration)
  default_tolerance: 0.3
```

### SICK Stack (`param/`) — Conservative Defaults

| Parameter | SICK value | Unitree value | Note |
|-----------|-----------|---------------|------|
| `inflation_radius` | 0.50 m | 0.45 m (global) / 0.35 m (local) | SICK slightly more conservative |
| `max_vel_x` | 0.6 m/s | 0.4 m/s | |
| `clearing_rotation_allowed` | true | false | Unitree has higher CoM |
| Costmap inflation split | No | Yes | Unitree uses different global/local values |

### `gmapping.yaml` Key Parameters

| Parameter | SICK | Unitree | Effect |
|-----------|------|---------|--------|
| `particles` | 30 | 30 | Increase for larger/more complex environments |
| `delta` | 0.05 m | 0.05 m | Map resolution |
| `linearUpdate` | 0.2 m | 0.2 m | Min linear motion before scan processing |
| `angularUpdate` | 0.2 rad | 0.2 rad | Min angular motion before scan processing |
| `maxUrange` | 8.0 m | 10.0 m | Max usable sensor range |

---

## Autonomous Explorer Algorithm

`scripts/autonomous_explorer.py` — frontier-based exploration with multiple improvements over the baseline.

### Architecture Overview

```
/map  ──► find_frontiers() ──► cluster frontiers
/odom ──► robot position

select_goal():
  score = size / (dist + 0.5)^2
  prefer frontiers within 1.8 m (near-first bias)
  skip if dist < 0.6 m (avoid instant-success loop)

is_reachable(gx, gy):
  lightweight: check target cell != OCCUPIED only
  [original: 23x23 costmap window check — too strict, blocked ~90% of valid goals]

_find_approach_point(gx, gy):
  walk from frontier centroid toward robot (check_r=4, up to ~12 steps)
  returns closest free cell to the frontier wall

send_nav_goal(wx, wy):
  send MoveBaseAction goal via actionlib SimpleActionClient
  SUCCEEDED  -> continue to next frontier
  ABORTED / PREEMPTED -> blacklist + call /move_base/clear_costmaps

_try_backup():
  2 s reverse at -0.10 m/s   (gentle — prevents tipping)
  3 s in-place rotation
  /move_base/clear_costmaps service call

run() main loop:
  same-goal detection: blacklist after 3 repeats within 0.3 m
  consecutive_failures >= 3  -> trigger _try_backup()
```

### Improvements vs Baseline

| Feature | Baseline | Improved |
|---------|----------|----------|
| `is_reachable()` | 23×23 costmap window (too strict) | Single-cell occupancy check |
| Approach point | Raw frontier centroid (often on wall) | Walk frontier toward robot (`check_r=4`) |
| Recovery mechanism | None | Gentle reverse −0.10 m/s + rotation + clear costmaps |
| Goal scoring | Distance only | `size / (dist + 0.5)²` with near-frontier bias |
| Min distance filter | None | Skip goals < 0.6 m (prevent instant-SUCCESS loop) |
| Same-goal loop | No detection (robot could re-send same goal 145+ times) | Blacklist after 3 repeats within 0.3 m |
| Failed-goal blacklist | Timeout goals NOT blacklisted | ALL failures (abort + timeout) blacklisted |
| Tipping prevention | `clearing_rotation_allowed: true` | `false` — prevents P3-AT tipping |

### Performance Benchmarks

Tested in `complex_maze.world` (12.2×12.2 m, corridors ~1–3 m wide):

| Stack | Timeout | Best Coverage | Goals | Notes |
|-------|---------|--------------|-------|-------|
| Unitree | 300 s | **12.6%** | 17 | Best observed result |
| Unitree | 300 s | 10.2–11.3% | 13–17 | Typical range |
| SICK | 300 s | ~10% | — | 180° FOV, slower frontier discovery |

Maze theoretical maximum ~18%. Accepted baseline performance: 12.6% in 300 s.

---

## Connecting a YOLO Target Detector

The `target_follower` node subscribes to `/target_pose` (`geometry_msgs/PoseStamped`).

To integrate a YOLO detector:

1. Create a node that:
   - Subscribes to depth camera RGB + depth topics
   - Runs YOLO inference to detect the target object
   - Computes the 3D position from the depth image
   - Publishes `PoseStamped` to `/target_pose`

2. Enable target following:

```bash
roslaunch p3at_lms_navigation real_robot_nav_unitree.launch use_target_follower:=true
```

3. Manual test without a detector:

```bash
rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## Known Issues and Notes

- **Simulation fidelity**: URDF and Gazebo dynamics are approximate. Real P3-AT skid-steer turning differs notably from simulation.

- **LMS200 FOV**: Real LMS200 has a 180° FOV. Verify `resolution` and `measuring_units` match your LMS200 firmware before deploying.

- **Narrow corridor margins**: P3-AT footprint ~0.54 m wide; complex maze corridors ~1 m wide. After inflation (0.35–0.45 m), margins are very tight. If the robot gets stuck frequently, try reducing `inflation_radius` to 0.30 m, but expect more wall contacts.

- **`clearing_rotation_allowed: false`**: This is **critical** for the Unitree stack. In-place rotation recovery was observed to cause the P3-AT to tip over (high centre of mass with Unitree L1 mounted on top). Do not re-enable without anti-tip analysis.

- **Same-goal infinite loop**: Fixed in `autonomous_explorer.py`. If logs show repeated goals, check `frontier_blacklist_radius` — reduce it if valid frontiers are being prematurely excluded.

- **Conda environments**: Avoid nodes requiring `PyKDL` inside conda. `autonomous_explorer.py` and `amcl_verifier.py` use pure-Python math only.

- **Gazebo reference frame**: `target_follow.launch` uses `base_footprint` (not `base_link`) because Gazebo merges fixed joints — `p3at::base_link` does not exist in the Gazebo model.

- **Unitree driver on Jetson**: The Unitree ROS SDK requires kernel modules compiled for your Jetson kernel. Allow ~30 min for first-time kernel module setup.

---

## Git Workflow

### What's Tracked

- Source packages: `catkin_ws/src/`
- Launch files, URDF/Xacro models, RViz configs
- Navigation parameters: `param/` and `param/unitree/` YAML files
- Scripts: `autonomous_explorer.py`, `amcl_verifier.py`, `waypoint_test.py`, `test_standoff_face.py`
- Submodule: `ros_ws/src/amr-ros-config/`
- Helper scripts: `build_and_hint.sh`, `run_full_pipeline.sh`, `tools/`
- Documentation: `README.md`

### What's Ignored (`.gitignore`)

- Build artifacts: `catkin_ws/build/`, `catkin_ws/devel/`, `ros_ws/build/`, `ros_ws/devel/`
- Generated maps: `*.pgm`, `*.yaml` in `maps/`
- TF frame outputs: `frames.gv`, `frames.pdf`
- Python caches: `__pycache__/`
- Catkin workspace marker: `.catkin_workspace`

### Setup After Clone

```bash
git clone <repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial
git submodule update --init --recursive
git checkout real_robot_navigation
cd catkin_ws && catkin_make
source devel/setup.bash   # or setup.zsh
```

---

## Resources

### ROS & Navigation
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [GMapping SLAM](http://wiki.ros.org/gmapping)
- [AMCL Localization](http://wiki.ros.org/amcl)
- [move_base Navigation](http://wiki.ros.org/move_base)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [NavfnROS Global Planner](http://wiki.ros.org/navfn)
- [map_server Package](http://wiki.ros.org/map_server)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [ROS Multi-Machine Setup](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

### Simulation
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [WSL2 GPU Rendering Guide](https://zhuanlan.zhihu.com/p/19575977500)

### Real Robot — Hardware Drivers
- [sicktoolbox_wrapper (LMS200)](http://wiki.ros.org/sicktoolbox_wrapper)
- [sicktoolbox_wrapper GitHub](https://github.com/ros-drivers/sicktoolbox_wrapper)
- [SICK LMS200 Specs](https://www.sick.com/de/en/lidar-sensors/2d-lidar-sensors/lms2xx/c/g91901)
- [RosAria (P3-AT driver)](http://wiki.ros.org/ROSARIA)
- [RosAria GitHub](https://github.com/amor-ros-pkg/rosaria)
- [AriaCoda (ARIA replacement)](https://github.com/reedhedges/AriaCoda)
- [Orbbec Femto Bolt SDK (ROS1)](https://github.com/orbbec/OrbbecSDK_ROS1)

### Docker & Networking
- [ROS Docker Images](https://hub.docker.com/_/ros)
- [Docker Host Networking](https://docs.docker.com/network/host/)
- [Ubuntu Netplan Configuration](https://netplan.readthedocs.io/)

---

## Status

### Simulation (Verified)
- [x] Gazebo simulation (gmapping + move_base + target following) — verified
- [x] Simulation laser self-collision bug fixed — verified (commit `5d9e4d2`)
- [x] Obstacle detection (obstacle_layer + inflation_layer) — verified
- [x] 3-waypoint sequential navigation — all SUCCEEDED (errors < 0.2 m)
- [x] Map saving and loading (map_server + AMCL) — verified
- [x] `standoff_distance` feature — verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] `face_target` feature — verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] Unit test suite (standoff + face_target) — 21/21 pass (commit `8e189ba`)
- [x] Dynamic target following (`move_target` node) — verified in Gazebo
- [x] Collision-free target model (ghost marker) — target no longer flips the robot
- [x] Target-lost goal cancellation — `target_follower` cancels `move_base` on stale target
- [x] Autonomous frontier exploration — verified in `complex_maze.world` (best 12.6% / 300 s)
- [x] `auto_mapping.launch` (SICK stack) — verified
- [x] `auto_mapping_unitree.launch` (Unitree stack) — verified  <- PRIMARY
- [x] AMCL accuracy verifier (`amcl_verifier.py`) — mean pos error 0.089 m, convergence 1.0 s
- [x] `auto_amcl_verify.launch` / `auto_amcl_verify_unitree.launch` — verified
- [x] Complex maze world (`complex_maze.world`, 12×12 m) — created and verified
- [x] `run_full_pipeline.sh` — one-click two-phase script — created and verified
- [x] Explorer: `is_reachable()` lightweight single-cell check — done
- [x] Explorer: `_find_approach_point()` walks toward robot (check_r=4) — done
- [x] Explorer: `_try_backup()` gentle reverse −0.10 m/s + rotation + clear costmaps — done
- [x] Explorer: `select_goal()` `size/(dist+0.5)²` scoring + near-first bias — done
- [x] Explorer: same-goal detection — blacklist after 3 repeats within 0.3 m — done
- [x] Explorer: ALL failed goals blacklisted (abort + timeout) — done
- [x] Unitree costmap: split global/local inflation (0.45 vs 0.35 m) — done
- [x] Unitree DWA: increased sampling (vx_samples=20, vtheta_samples=40) — done
- [x] `clearing_rotation_allowed: false` — prevents P3-AT tipping — done
- [x] `param/unitree/` parameter directory — created and tuned

### Real Robot (Pending Hardware Test)
- [x] Real robot launch files — all 4 variants (mapping + nav, both stacks) — created
- [x] Raspberry Pi base driver package (`p3at_base`) — created
- [x] Multi-machine network setup documented and verified (cross-machine topic test)
- [ ] YOLO target detection node (Orbbec Femto Bolt + YOLO) — not started
- [ ] Real hardware parameter tuning — not started
- [ ] End-to-end real robot navigation test — Unitree (primary) — not started
- [ ] End-to-end real robot navigation test — SICK (backup) — not started

---

## Post-Installation Checklist

### Environment Setup
- [ ] Build: `cd catkin_ws && catkin_make` — no errors
- [ ] Source: `source devel/setup.zsh && rospack find p3at_lms_navigation` — prints path
- [ ] Submodule: `ls ros_ws/src/amr-ros-config/` — directory exists

### Simulation — Unitree Stack (Primary)
- [ ] Launches: `roslaunch p3at_lms_navigation mapping_unitree.launch use_gazebo_target:=false`
- [ ] Robot spawns without errors
- [ ] `rostopic hz /unitree/scan` — ~10 Hz
- [ ] `rostopic hz /odom` — ~50 Hz
- [ ] `rostopic hz /map` — publishes
- [ ] `rosrun tf tf_echo map base_link` — TF chain complete
- [ ] RViz shows robot model, lidar scan, map
- [ ] Manual 2D Nav Goal succeeds

### Simulation — SICK Stack (Backup)
- [ ] Launches: `roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false`
- [ ] `rostopic hz /scan` — publishes
- [ ] Manual 2D Nav Goal succeeds

### Navigation Tests
- [ ] Waypoint test: `python3 catkin_ws/src/p3at_lms_navigation/scripts/waypoint_test.py` — 3/3 SUCCEEDED
- [ ] Map saving: `rosrun map_server map_saver -f /tmp/test_map` — files created

### Target Following
- [ ] Static: `mapping_unitree.launch use_gazebo_target:=true` — drag target in Gazebo, robot follows
- [ ] Dynamic: add `move_target:=true target_speed:=0.3 target_pause:=2.0`
- [ ] Standoff + face: add `standoff_distance:=1.0 face_target:=true`
- [ ] Unit tests: `python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py` — 21/21

### Autonomous Mapping (Unitree)
- [ ] Explorer: `roslaunch p3at_lms_navigation auto_mapping_unitree.launch exploration_timeout:=300`
  - [ ] Terminal prints coverage updates
  - [ ] Map saved: `ls catkin_ws/src/p3at_lms_navigation/maps/explored_map_unitree.pgm`
- [ ] AMCL verify: `roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true`
  - [ ] `grep "Mean position" catkin_ws/src/p3at_lms_navigation/maps/amcl_report.txt` — < 0.30 m
- [ ] Full pipeline: `bash run_full_pipeline.sh`

### Real Robot — Unitree (Primary) — Complete These First

#### Network
- [ ] Jetson: `ip -br addr show eth0` shows `192.168.50.1`
- [ ] Pi: `ip -br addr show eth0` shows `192.168.50.2`
- [ ] Ping both directions succeed
- [ ] Jetson Docker `--net=host`: `docker inspect -f '{{.HostConfig.NetworkMode}}' ros_noetic`
- [ ] Pi Docker `--net=host`: `docker inspect -f '{{.HostConfig.NetworkMode}}' noetic_pi`
- [ ] `roscore` on Jetson: `ss -lntp | grep 11311`
- [ ] `rosnode list` from Pi returns at least `/rosout`
- [ ] Cross-machine topic test passes

#### Unitree Hardware
- [ ] Unitree L1 powered and connected to Jetson USB
- [ ] `rostopic hz /unitree/scan` — publishing from hardware
- [ ] P3-AT base driver on Pi: `roslaunch p3at_base base.launch`
- [ ] `rostopic hz /odom` — publishing
- [ ] Teleop: `teleop_twist_keyboard` — robot moves and stops

#### Unitree Mapping & Navigation
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch` — no errors
- [ ] `rostopic hz /map` — map building
- [ ] Map saved: `rosrun map_server map_saver -f ~/maps/unitree_map`
- [ ] `roslaunch p3at_lms_navigation real_robot_nav_unitree.launch map_file:=~/maps/unitree_map.yaml`
- [ ] AMCL particle cloud converges after "2D Pose Estimate"
- [ ] Navigation goal SUCCEEDED

### Real Robot — SICK (Backup) — Only If Unitree Unavailable
- [ ] LMS200 detected: `ls /dev/ttyUSB0`
- [ ] Permissions: `sudo chmod 666 /dev/ttyUSB0`
- [ ] P3-AT base driver active (same steps as Unitree above)
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping.launch` — no errors
- [ ] `rostopic hz /scan` — ~75 Hz from LMS200
- [ ] `rosrun tf tf_echo base_link laser` — TF active
- [ ] Map saved
- [ ] AMCL navigation goal SUCCEEDED
