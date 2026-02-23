# P3-AT + LMS200 Navigation System (ROS1 Noetic)

A ROS1 Noetic workspace for the **ELEC70015 Human-Centered Robotics** course at Imperial College London. It provides both **Gazebo simulation** and **real robot** launch files for a Pioneer 3-AT mobile robot equipped with a SICK LMS200 2D lidar.

The LMS200 lidar handles **all** mapping, localization, path planning, and local obstacle avoidance. A depth camera (Orbbec Femto Bolt) is reserved for future target detection only and is not involved in navigation.

> **Note for WSL2 Users:** If the robot model displays correctly in RViz but the P3AT visual mesh does not appear in Gazebo, this may be due to GPU rendering limitations in WSL2. See the [Troubleshooting](#troubleshooting) section for solutions.

The repository contains two workspaces:
- **`catkin_ws/`** — LMS200 LiDAR navigation stack (this branch, documented below)
- **`ros_ws/`** — Depth camera simulation with `depthimage_to_laserscan` (see `main` branch)

Quick launcher guide: [`doc.md`](doc.md)

## Demo v1 Guide (Current Branch)

For `release/demo-v1`, use this three-part structure:

1. Navigation + Raspberry Pi communication (ROS1 multi-machine)
2. Trash detection (non-ROS host inference) + UDP bridge into ROS1 Docker
3. Lidar-only part (TBD for this branch)

Practical runbook is maintained in `doc.md`: [`doc.md`](doc.md)

Key notes:
- ROS master uses port `11311`.
- Trash detection bridge uses UDP `TRASH_UDP_PORT` (default `16031`) from `scripts/deploy.env`.
- Detection and navigation can run on the same branch, but in different runtimes (host + Docker).

### Demo v1 Node Tree

Current runtime split (`release/demo-v1`):
- Host: only trash detection inference (non-ROS)
- Docker: all ROS nodes

```text
Host (non-ROS)
└── predict_15cls_rgbd.py
    └── UDP JSON -> 127.0.0.1:${TRASH_UDP_PORT}  (default 16031)

Docker (ROS1 Noetic)
├── roscore / rosmaster
├── sicklms (LMS200 driver)  [real robot]
├── robot_state_publisher
├── map_server + amcl        [nav mode]
├── slam_gmapping            [mapping mode]
├── move_base
├── udp_target_bridge
│   └── publishes /trash_detection/target_point (PointStamped)
├── point_to_target_pose
│   ├── subscribes /trash_detection/target_point
│   └── publishes  /target_pose (PoseStamped)
└── target_follower
    ├── subscribes /target_pose
    └── sends MoveBaseGoal to move_base (action)

Raspberry Pi (external, real robot base)
└── p3at_base/base.launch
    ├── RosAria
    │   ├── subscribes /cmd_vel
    │   └── publishes  /RosAria/pose (odometry)
    └── odom_republisher.py
        ├── subscribes /RosAria/pose
        └── publishes  /odom
```

Key topic/action flow:
```text
Host detector (UDP) -> /udp_target_bridge -> /trash_detection/target_point
-> /point_to_target_pose -> /target_pose -> /target_follower -> /move_base (action)
-> /cmd_vel -> RosAria (Raspberry Pi Base) -> /RosAria/pose -> /odom
```

If Raspberry Pi base is not connected/running, `/odom` may be missing in real-robot mode.

## Table of Contents

- [System Overview](#system-overview)
- [Hardware](#hardware)
- [Repository Structure](#repository-structure)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Dependencies](#dependencies)
- [Build](#build)
- [Usage: Gazebo Simulation](#usage-gazebo-simulation)
  - [Mapping + Manual Navigation (gmapping)](#1-mapping--manual-navigation-gmapping)
  - [Navigation on a Saved Map (AMCL)](#2-navigation-on-a-saved-map-amcl)
  - [Target Following](#3-target-following)
  - [Testing Navigation (3-Waypoint Test)](#4-testing-navigation-3-waypoint-test)
  - [Testing Target Follower Features (Unit Tests)](#5-testing-target-follower-features-unit-tests)
- [Verifying the System](#verifying-the-system)
- [Troubleshooting](#troubleshooting)
- [Simulation Calibration Notes (Bug Fixes)](#simulation-calibration-notes-bug-fixes)
- [Usage: Real Robot](#usage-real-robot)
  - [Multi-Machine Setup](#multi-machine-setup)
  - [Deployment Helper Scripts](#deployment-helper-scripts)
  - [Step 1 - Start Base Driver (Raspberry Pi)](#step-1---start-base-driver-raspberry-pi)
  - [Step 2a - Mapping (Jetson)](#step-2a---mapping-jetson)
  - [Step 2b - Navigation (Jetson)](#step-2b---navigation-jetson)
- [Key Topics and TF Frames](#key-topics-and-tf-frames)
- [Parameter Tuning](#parameter-tuning)
- [Connecting a YOLO Target Detector](#connecting-a-yolo-target-detector)
- [Known Issues and Notes](#known-issues-and-notes)
- [Git Workflow](#git-workflow)
- [Resources](#resources)
- [Post-Installation Checklist](#post-installation-checklist)

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

## Repository Structure

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── catkin_ws/                 # LMS200 LiDAR navigation workspace (this branch)
│   ├── src/
│   │   ├── p3at_lms_description/   # URDF/Xacro robot model (P3-AT + laser link)
│   │   ├── p3at_lms_gazebo/        # Gazebo world, target model, simulation launch
│   │   ├── p3at_lms_navigation/    # Navigation stack (gmapping, AMCL, move_base)
│   │   ├── target_follower/        # Target following system
│   │   └── p3at_base/              # Raspberry Pi base driver
│   ├── build/                  # Build artifacts (not tracked)
│   └── devel/                  # Development space (not tracked)
├── ros_ws/                    # Depth camera simulation workspace (main branch)
│   └── src/
│       └── amr-ros-config/     # AMR configuration (git submodule)
├── build_and_hint.sh          # Quick-build helper script
├── .gitmodules                # Submodule declarations
├── .gitignore
└── README.md
```

> **Two workspaces:** `catkin_ws/` uses a physical LMS200 LiDAR (simulated via Gazebo ray
> sensor plugin) for mapping/navigation. `ros_ws/` (on the `main` branch) uses depth
> camera data converted to laser scans via `depthimage_to_laserscan`. Both workspaces
> share the same robot platform (Pioneer 3-AT).

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
│   ├── param/
│   │   ├── gmapping.yaml           # gmapping SLAM parameters
│   │   ├── amcl.yaml               # AMCL localization parameters
│   │   ├── move_base.yaml          # NavfnROS + DWA planner config
│   │   ├── costmap_common.yaml     # Shared costmap (footprint, inflation)
│   │   ├── global_costmap.yaml     # Global costmap layers (obstacle + inflation)
│   │   └── local_costmap.yaml      # Local costmap layers (obstacle + inflation)
│   ├── scripts/
│   │   └── waypoint_test.py        # Sequential 3-waypoint navigation test
│   └── maps/                       # Saved maps (created by map_saver)
├── target_follower/         # Target following system
│   ├── scripts/
│   │   ├── target_follower.py          # Subscribes /target_pose, sends MoveBaseGoal
│   │   │                               #   (supports standoff_distance and face_target)
│   │   ├── gazebo_target_publisher.py  # Publishes Gazebo model pose as /target_pose
│   │   ├── goal_to_target_relay.py     # Relays RViz 2D Nav Goal to /target_pose
│   │   └── test_standoff_face.py       # Unit tests for standoff & face_target logic
│   └── launch/target_follow.launch
└── p3at_base/               # Real robot base driver (runs on Raspberry Pi)
    ├── launch/base.launch              # RosAria + odom republisher
    └── scripts/odom_republisher.py     # Republishes /RosAria/pose as /odom
```

## Prerequisites

### System Requirements

- **Ubuntu 20.04 LTS**
- **Python 3.8+**
- **ROS Noetic** (full desktop or ros-base)
- **Gazebo 11** (for simulation)

**Verify ROS installation:**
```bash
which roscore  # Should return: /opt/ros/noetic/bin/roscore
gazebo --version  # Should be 11.x
```

### Python Dependencies

Some scripts and tools require additional Python packages:

```bash
pip install numpy>=1.24.4
pip install matplotlib>=3.1.2
pip install opencv-python>=4.13.0
pip install opencv-contrib-python>=4.13.0
pip install defusedxml  # Required for xacro URDF processing
pip install scipy>=1.10.1
pip install pillow>=10.4.0
```

**Optional:** Add Python user scripts to PATH:
```bash
# For bash users:
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc

# For zsh users:
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc && source ~/.zshrc
```

### Clone Repository

```bash
git clone <your-repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial

# Initialize submodules (amr-ros-config)
git submodule update --init --recursive

# Switch to the navigation branch
git checkout real_robot_navigation
```

## Dependencies

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
cd ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws
catkin_make
source devel/setup.bash    # or setup.zsh for zsh users
```

Alternatively, use the provided helper script from the repository root:
```bash
./build_and_hint.sh
```

> You must `source devel/setup.bash` (or `.zsh`) in every new terminal before running any `roslaunch` or `rostopic` commands.

> **Important for zsh users:** ROS setup scripts have both `.bash` and `.zsh` versions. Always use `.zsh` files if you are using zsh shell.

If `catkin_make` fails with missing dependencies:
```bash
cd ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

---

## Usage: Gazebo Simulation

All simulation launch files start Gazebo, spawn the robot, and open RViz automatically. Each step below runs in a separate terminal. Source the workspace at the start of each terminal:

```bash
source ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.zsh
```

### 1. Mapping + Manual Navigation (gmapping)

#### Option A — Manual goal sending (recommended for testing)

**Terminal 1** — Launch Gazebo + gmapping + move_base + RViz **without** target_follower:

```bash
roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false
```

> **Important**: pass `use_gazebo_target:=false` when you want to send goals manually or run
> the waypoint test script. Without this flag, the `target_follower` node starts and will
> cancel any manual goals after ~2 seconds.

Wait ~30 seconds for Gazebo, gmapping, and move_base to fully initialise (you will see
`odom received` and `Registering First Scan` messages in the terminal).

**Send a single navigation goal from any sourced terminal:**

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Or click **"2D Nav Goal"** in RViz.

#### Option B — Gazebo target following (default)

```bash
roslaunch p3at_lms_navigation mapping.launch
```

This starts:
- Gazebo with the P3-AT robot and two box obstacles (1.0 m tall)
- `slam_gmapping` building a 2D occupancy grid map in real time
- `move_base` (NavfnROS global planner + DWA local planner)
- `target_follower` tracking a Gazebo model named "target"
- RViz showing the live map, robot pose, and laser scan

Drive the robot by moving the **"target"** model in Gazebo:
1. In Gazebo, click the model named **"target"** in the left panel
2. Select the **Translate** tool (T key)
3. Drag the target to a new position; the robot will navigate toward it

#### Option C — Keyboard teleop

```bash
roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false
# In a second terminal:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Key bindings:
```
u  i  o        move forward + turn
j  k  l        turn left / stop / turn right
m  ,  .        move backward + turn

q / z  : increase / decrease max linear speed
w / x  : increase / decrease max angular speed
```

**Save the map** once coverage is complete (new terminal):
```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

This creates `my_map.pgm` and `my_map.yaml` in `p3at_lms_navigation/maps/`.

### 4. Testing Navigation (3-Waypoint Test)

A Python script is provided to verify end-to-end navigation automatically:
`p3at_lms_navigation/scripts/waypoint_test.py`

**Prerequisites:**
- The simulation is running: `roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false`
- Wait at least **30 seconds** after launch before running the test
- Source the workspace in the terminal where you run the script

```bash
# From the catkin_ws directory:
source devel/setup.zsh   # (or setup.bash)
python3 src/p3at_lms_navigation/scripts/waypoint_test.py
```

**What the script does:**

Sends three sequential `MoveBaseGoal` actions via the `actionlib` interface and reports
progress and final position error for each waypoint:

| Waypoint | Target (map frame) | Notes |
|----------|--------------------|-------|
| WP1 | (0.0, -2.0) | Open space south, ~2 m straight run |
| WP2 | (3.5, 2.0) | North of obstacle_1 at (2, 1) — robot must navigate around |
| WP3 | (5.0, 0.0) | Far east, past both obstacles — tests narrow-corridor traversal |

**Expected output (verified on commit `5d9e4d2`):**

```
Connected to move_base!

========== Waypoint 1 [open south, ~2 m]: (0.0, -2.0) ==========
  Start position : (0.000, 0.000)
  RESULT: SUCCEEDED | Final pos: (...) | Error: 0.14 m

========== Waypoint 2 [N of obstacle_1@(2,1), ~5 m total, must avoid]: (3.5, 2.0) ==========
  RESULT: SUCCEEDED | Final pos: (...) | Error: 0.11 m

========== Waypoint 3 [far east past both obstacles, ~7 m total]: (5.0, 0.0) ==========
  RESULT: SUCCEEDED | Final pos: (...) | Error: 0.19 m

========== SUMMARY ==========
  WP1 [open south, ~2 m] : PASS
  WP2 [N of obstacle_1@(2,1), ~5 m total, must avoid] : PASS
  WP3 [far east past both obstacles, ~7 m total] : PASS
```

Each waypoint has a **65-second timeout**. The script prints position and action state every 5 seconds.
If a waypoint is `ABORTED`, check `rostopic echo /move_base/status` and ensure the goal is not
inside an obstacle inflation zone.

### 5. Testing Target Follower Features (Unit Tests)

A self-contained unit test script verifies the core maths of `standoff_distance` and `face_target`
**without requiring a running ROS system or Gazebo**. It replicates the exact logic from
`target_follower.py` and checks all edge cases with known inputs.

Script: `target_follower/scripts/test_standoff_face.py`

#### Running the tests

```bash
# No ROS sourcing needed — pure Python, no dependencies
python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py
```

#### What is tested (21 tests, all expected to PASS)

| Group | Test | What is verified |
|-------|------|------------------|
| **Standoff geometry** | T1 | `standoff=0` → goal equals target exactly |
| | T2a–d | `standoff=1.5`, robot at origin, target 4 m away → goal is exactly 1.5 m from target, lies on the robot→target line |
| | T3 | Robot already inside standoff zone (d=1.0 < standoff=2.0) → returns `skip=True`, no goal sent |
| | T4a–c | `standoff=1.0`, oblique direction → goal exactly 1 m from target, collinear with robot→target |
| **face_target yaw** | F1 | Target due east → yaw = 0° |
| | F2 | `standoff=1 m` + target due east → yaw still = 0° |
| | F3 | Target due north → yaw = 90° |
| | F4 | Target south-west → yaw = −135° |
| | F5 | `standoff=2 m`, target NE at 45° → yaw = 45° |
| | F6 | `yaw_to_quaternion` round-trip: recover original yaw from (z, w) |
| **Edge cases** | E1 | `d == standoff` (exact boundary) → skip |
| | E2 | `d < standoff` → skip |
| | E3 | `d` just above `standoff` → no skip, goal exactly `standoff` metres from target |
| | E4 | goal ≈ target (standoff ≈ 0) → falls back to robot→target direction for yaw |

#### Expected output

```
════════════════════════════════════════════════════════════
UNIT TEST: standoff_distance geometry
════════════════════════════════════════════════════════════
  [PASS] T1: standoff=0, no skip
  [PASS] T1: standoff=0, goal == target  (goal=(3.0000,4.0000), target=(3.0,4.0))
  [PASS] T2a: standoff=1.5, no skip (d=4 > 1.5)
  [PASS] T2b: goal is exactly standoff distance from target  (dist=1.5000m (want 1.5m))
  [PASS] T2c: goal is on the robot→target line  (goal=(2.5000,0.0000))
  [PASS] T2d: goal_x = 4.0 - 1.5 = 2.5  (gx=2.5000)
  [PASS] T3: inside standoff zone → skip=True  (d=1.0 <= standoff=2.0, skip=True)
  ...
════════════════════════════════════════════════════════════
SUMMARY
════════════════════════════════════════════════════════════
  [PASS] ...  (×21)

  21/21 unit tests passed
  All unit tests PASSED ✓
```

> **Note:** The unit tests verify the mathematical correctness of the standoff and
> face_target logic in isolation. End-to-end navigation tests (robot actually moving to
> the computed goal) are covered by `waypoint_test.py` for the baseline case. Full
> integration of `standoff_distance` + `face_target` was also verified manually in
> Gazebo simulation (commit `7d87bbb`).

### 2. Navigation on a Saved Map (AMCL)

**Terminal 1** — Launch Gazebo + AMCL + move_base + RViz:

```bash
roslaunch p3at_lms_navigation nav.launch \
  map_file:=$(rospack find p3at_lms_navigation)/maps/my_map.yaml
```

#### Initialize robot pose in RViz

AMCL needs an approximate starting pose before it can localize.

1. In RViz, click **"2D Pose Estimate"** in the top toolbar
2. Click on the map at the robot's **actual current position**
3. Hold and drag to set the **heading direction**, then release
4. The green particle cloud should converge around the robot — localization is active

If the particle cloud does not converge, drive the robot a short distance and repeat.

#### Send a navigation goal in RViz

1. In RViz, click **"2D Nav Goal"** in the top toolbar
2. Click on the map at the **desired destination**
3. Hold and drag to set the **goal heading**, then release
4. Observe:
   - A green global path appears in RViz
   - The robot starts moving in Gazebo
   - The path updates as the robot progresses

To send a goal from the command line instead:
```bash
rostopic pub /target_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.0
    z: 0.0
  orientation:
    w: 1.0" -1
```

### 3. Target Following

**Gazebo model tracking** (default in `mapping.launch`): The `gazebo_target_publisher` reads the Gazebo "target" model pose and publishes it as `/target_pose`. The `target_follower` node converts this into a continuous stream of `MoveBaseGoal` actions.

**RViz 2D Nav Goal relay**: Use RViz to set the target instead of a Gazebo model:
```bash
roslaunch p3at_lms_navigation mapping.launch \
  use_gazebo_target:=false use_rviz_goal_relay:=true
```
Now every "2D Nav Goal" click in RViz is forwarded to `/target_pose` and triggers the follower.

**Programmatic target** (e.g., from a YOLO detector): Publish to `/target_pose` from any node. The pose can be in `map`, `odom`, or `base_link` frame -- `target_follower` handles the TF transform automatically.

#### Standoff Distance

The `standoff_distance` parameter makes the robot stop a fixed distance *short* of the target instead of driving all the way to it. This is useful when following a person — you want the robot to keep a comfortable gap rather than crowd the target.

```bash
# Stop 1.5 m short of the target
roslaunch p3at_lms_navigation mapping.launch standoff_distance:=1.5
```

**How it works:**
- The robot computes the vector from its current position to the target.
- The goal sent to `move_base` is placed `standoff_distance` metres back along that vector.
- If the robot is already *inside* the standoff zone (current distance ≤ `standoff_distance`), no new goal is sent — the robot stays put.
- The standoff goal is recomputed each time the target moves, so it tracks correctly as both robot and target move.

```
  Robot ──────── Goal ─ [standoff_distance] ─ Target
```

**Change at runtime** without restarting:
```bash
rosparam set /target_follower/standoff_distance 2.0
```

#### Face Target

The `face_target` parameter orients the robot to face *toward* the target when it arrives at the goal pose. Without this, the robot keeps its current heading (default DWA behaviour).

```bash
# Robot faces the target at the goal
roslaunch p3at_lms_navigation mapping.launch face_target:=true
```

**How it works:**
- After computing the goal position (with or without standoff), the goal orientation is set to `atan2(target_y - goal_y, target_x - goal_x)`.
- When `standoff_distance > 0`, the robot faces from the standoff goal toward the target — i.e., it points at the target from a distance.
- When `standoff_distance = 0` and `face_target = true`, the robot faces the direction it approached from (robot → target vector).
- The yaw is converted to a unit quaternion and passed directly to `move_base`; actual pointing accuracy is limited by DWA's `yaw_goal_tolerance` (0.5 rad ≈ 29°).

**Change at runtime** without restarting:
```bash
rosparam set /target_follower/face_target true
```

#### Combined Example

```bash
# Stop 1.0 m from the target AND face it
roslaunch p3at_lms_navigation mapping.launch \
  standoff_distance:=1.0 face_target:=true
```

This is the recommended mode for human-following: the robot stops at a comfortable distance and turns to face the person.

#### All `target_follower` Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~target_topic` | string | `/target_pose` | Topic to subscribe for target pose |
| `~global_frame` | string | `map` | Global reference frame for goals |
| `~robot_frame` | string | `base_link` | Robot body frame (for standoff TF lookup) |
| `~send_rate_hz` | float | `2.0` | Max rate at which new goals are sent (Hz) |
| `~min_update_dist` | float | `0.3` | Min target movement (m) before re-sending goal |
| `~target_timeout_s` | float | `1.0` | Ignore stale target poses older than this (s) |
| `~use_target_orientation` | bool | `false` | Use the orientation from the target message |
| `~standoff_distance` | float | `0.0` | Stop this many metres short of the target (0 = go all the way) |
| `~face_target` | bool | `false` | Orient robot to face the target at the goal pose |

---

## Verifying the System

Use these commands in any sourced terminal to check that everything is running correctly.

### Quick System Check

```bash
# List all active nodes (expect ~10 nodes with Gazebo + gmapping + move_base)
rosnode list

# List all active topics
rostopic list
```

### Sensor Frequencies

```bash
# Check lidar data rate (should be ~10 Hz in simulation, ~75 Hz real LMS200)
rostopic hz /scan

# Check odometry rate (should be ~100 Hz)
rostopic hz /odom

# Check map update rate (~1 Hz during exploration, less when stationary)
rostopic hz /map
```

### Navigation Status

```bash
# View the current robot pose estimate from AMCL
rostopic echo /amcl_pose

# Check move_base goal status
rostopic echo /move_base/status

# Monitor velocity commands (robot should be publishing when moving)
timeout 5 rostopic echo /cmd_vel
```

### TF Tree

```bash
# Print the full TF tree to a PDF
rosrun tf2_tools view_frames.py && evince frames.pdf

# Check specific transforms
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link laser
```

### Quick Diagnostic (one-liner)

```bash
echo "=== Nodes ===" && rosnode list && \
echo "=== Scan Hz ===" && timeout 3 rostopic hz /scan && \
echo "=== Odom Hz ===" && timeout 3 rostopic hz /odom && \
echo "=== TF map->base_link ===" && rosrun tf tf_echo map base_link 2>&1 | head -5
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| RViz shows no map | `map_server` not running, or wrong `map_file` path | Check the `map_file` argument; ensure the `.yaml` and `.pgm` files exist |
| Robot does not move after nav goal | AMCL not yet localized | Use "2D Pose Estimate" in RViz first, then resend the goal |
| Navigation goal immediately aborted | Target is inside an obstacle or too close to a wall | Choose a goal in open space; increase `inflation_radius` if needed |
| Manual goal cancelled after 2 s | `target_follower` is running and timing out | Launch with `use_gazebo_target:=false` to disable it |
| All lidar readings < 0.15 m (self-hit) | `<collision>` on laser link causing ray self-hit | Already fixed in URDF; check no collision element was re-added |
| Robot cannot see obstacles (detour missing) | Obstacles shorter than laser height (0.366 m) | Already fixed in world file; obstacles are now 1.0 m tall |
| waypoint_test.py: `move_base not available` | Simulation not ready yet | Wait 30 s after launch start; ensure `use_gazebo_target:=false` |
| Robot spins in place indefinitely | DWA planner cannot find a feasible velocity | Lower `max_vel_theta` or increase `sim_time` in `move_base.yaml` |
| `roslaunch` reports package not found | Workspace not sourced | Run `source devel/setup.zsh` in that terminal |
| Gazebo opens but robot falls through floor | Mesh collision geometry issue | Restart Gazebo; this is intermittent with complex STL meshes |
| `catkin_make` fails with missing package | ROS dependency not installed | Run `rosdep install --from-paths src --ignore-src -r -y` |
| P3AT mesh invisible in Gazebo (WSL2) | GPU rendering limitation in WSL2 | See [WSL2 GPU Acceleration Guide](https://zhuanlan.zhihu.com/p/19575977500); install WSL2 GPU drivers or configure X11 forwarding |
| Processes not terminating cleanly | Stale ROS/Gazebo processes | `killall -9 gzserver gzclient roscore rosmaster roslaunch rviz; sleep 3` |
| `evince` command not found | PDF viewer not installed | Use alternatives: `xdg-open frames.pdf`, `eog frames.pdf`, or `okular frames.pdf` |

---

## Simulation Calibration Notes (Bug Fixes)

The following issues were identified and fixed during simulation testing (all changes committed in `5d9e4d2`):

### 1 — Lidar Self-Collision (URDF)

**Problem:** Gazebo merges fixed-joint child links into the parent link for physics simulation.
This caused the `laser` link's collision geometry to be treated as part of `base_link`,
so all 361 lidar rays were hitting the robot body (readings 0.05–0.14 m).

**Fix:** Removed the `<collision>` element from the `laser` link in
`p3at_lms_description/urdf/p3at_lms.urdf.xacro`. The visual and sensor elements are
retained; only the collision box was removed.

### 2 — Obstacles Below Laser Plane (World File)

**Problem:** The two box obstacles were 0.5 m / 0.6 m tall. The laser is mounted at
z = 0.366 m, so the obstacle tops were at 0.5 m / 0.6 m but the lidar plane swept above
the shorter one. In practice **all scan rays passed over the obstacles** and the costmap
had no obstacle markings.

**Fix:** Raised both obstacle heights to **1.0 m** (centre at z = 0.5 m) in
`p3at_lms_gazebo/worlds/p3at_lms.world`.

### 3 — target_follower Cancelling Manual Goals

**Problem:** `target_follower.py` subscribes to `/target_pose` and, if no message arrives
within 2 seconds, cancels any active move_base goal. This silently cancelled every
manually-sent goal and made it impossible to test navigation without the Gazebo target.

**Fix:** Added `use_target_follower` argument to `mapping.launch`; the node is wrapped in
`<group if="$(arg use_target_follower)">`. It defaults to off when
`use_gazebo_target:=false`.

### 4 — DWA Goal Tolerance Too Large

**Problem:** `xy_goal_tolerance: 0.30` with `latch_xy_goal_tolerance: true` caused
move_base to declare success while the robot was still 0.3 m from the target. With `yaw_goal_tolerance: 3.14` the orientation was irrelevant.

**Fix (move_base.yaml):** `xy_goal_tolerance: 0.10`, `yaw_goal_tolerance: 0.5`,
`latch_xy_goal_tolerance: false`.

### 5 — TF Frame Mismatch in Costmap

**Problem:** `robot_base_frame: base_link` in both costmap YAML files, but the skid-steer
Gazebo plugin publishes odometry with `child_frame_id: base_footprint`. This caused a TF
lookup failure that prevented the local costmap from updating.

**Fix:** Set `robot_base_frame: base_footprint` in both `global_costmap.yaml` and
`local_costmap.yaml`.

### 6 — Missing obstacle_layer in Global Costmap

**Problem:** `global_costmap.yaml` only had a `static_layer`. Without an `obstacle_layer`,
the planner could not see live sensor obstacles — it only knew about walls from the static
map. Newly-seen obstacles (the two boxes) were never inscribed into the global costmap.

**Fix:** Added explicit `obstacle_layer` and `inflation_layer` configurations to both
`global_costmap.yaml` and `local_costmap.yaml`.

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

### Deployment Helper Scripts

To avoid repeatedly typing `source` and long launch commands, helper scripts are provided in `scripts/`.

1. Copy and edit IP config:
```bash
cp scripts/deploy.env.example scripts/deploy.env
```

2. Fill in `JETSON_IP`, `RASPI_IP`, `LAPTOP_IP` in `scripts/deploy.env`.

3. Run with wrappers (from repo root or any directory):
```bash
# Local WSL simulation (laptop as ROS master)
scripts/start_master.sh laptop
scripts/start_sim_mapping.sh
# or
scripts/start_sim_nav.sh

# Real robot topology (Jetson as ROS master)
# Jetson terminal 1:
scripts/start_master.sh jetson
# Jetson terminal 2:
scripts/start_real_mapping.sh
# or
scripts/start_real_nav.sh map_file:=/absolute/path/to/my_map.yaml
# Real target following (local RGBD+YOLO sends UDP JSON x/y/z):
scripts/start_real_nav.sh map_file:=/absolute/path/to/my_map.yaml use_target_follower:=true use_udp_target_bridge:=true point_target_frame:=camera_link
# RasPi terminal:
scripts/start_base.sh
```

Notes:
- `scripts/env.sh` is the single environment entrypoint and is sourced by all `start_*.sh` scripts.
- `MASTER_HOST` can be overridden when needed, for example:
  `MASTER_HOST=laptop scripts/start_base.sh`
- `scripts/deploy.env` is git-ignored by default so each machine can keep its own local IP config.

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
| `standoff_distance` | `0.0` | Stop this many metres short of target (see [Standoff Distance](#standoff-distance)) |
| `face_target` | `false` | Orient robot to face the target at goal (see [Face Target](#face-target)) |

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
| `standoff_distance` | `0.0` | Stop this many metres short of target |
| `face_target` | `false` | Orient robot to face the target at goal |

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
- `xy_goal_tolerance: 0.10` - position tolerance at goal (m) — tightened from 0.30
- `yaw_goal_tolerance: 0.50` - orientation tolerance at goal (rad) — tightened from 3.14
- `latch_xy_goal_tolerance: false` - re-evaluate tolerance each iteration
- `recovery_behavior_enabled: true` - enabled; `clearing_rotation_allowed: true`
- `planner_patience: 10.0` / `controller_patience: 10.0` - allow planning retries
- NavfnROS: `allow_unknown: true`, `default_tolerance: 0.10`

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

3. If your detector publishes `geometry_msgs/PointStamped` (for example a 3D point in camera frame), enable the bridge node:
   ```bash
   roslaunch p3at_lms_navigation real_robot_nav.launch \
     use_target_follower:=true \
     use_point_target_bridge:=true \
     point_target_topic:=/trash_detection/target_point
   ```
   The bridge converts `PointStamped` to `PoseStamped` on `/target_pose`.  
   `target_follower` will transform it to `map` using TF and publish `move_base` goals.

4. Manual test without a detector (`PoseStamped` directly):
   ```bash
   rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

5. Manual test detector-like input (`PointStamped`):
   ```bash
   rostopic pub -r 5 /trash_detection/target_point geometry_msgs/PointStamped \
     "{header: {frame_id: 'base_link'}, point: {x: 2.0, y: 0.0, z: 0.0}}"
   ```

6. Mock detector mode (no depth camera required):
   ```bash
   roslaunch p3at_lms_navigation mapping.launch \
     use_gazebo_target:=false \
     use_target_follower:=true \
     use_mock_detector:=true \
     use_point_target_bridge:=true \
     point_target_topic:=/trash_detection/target_point \
     mock_target_frame:=base_link \
     mock_mode:=fixed \
     mock_x:=2.0 mock_y:=0.0 mock_z:=0.0 \
     standoff_distance:=0.6 face_target:=true
   ```
   `mock_mode:=circle` is also supported for moving-target stress tests.

## Known Issues and Notes

- **Simulation fidelity**: The URDF model and Gazebo dynamics are approximate. Real P3-AT behavior (especially skid-steer turning) will differ.
- **LMS200 FOV**: The real LMS200 has a 180-degree field of view. The Gazebo simulation uses a similar configuration. Verify `resolution` and `measuring_units` match your LMS200 firmware settings.
- **move_base recovery**: Recovery behaviors are enabled (`recovery_behavior_enabled: true`) with clearing rotations. If the robot gets stuck during mapping, disable with `recovery_behavior_enabled: false` in `move_base.yaml`.
- **Conda environments**: If running inside a conda Python environment, avoid nodes that require `PyKDL`. The `target_follower` in this workspace uses pure-Python quaternion math and does not depend on PyKDL.
- **Gazebo reference frame**: The `target_follow.launch` uses `base_footprint` as the Gazebo reference frame (not `base_link`) because Gazebo merges fixed joints -- the frame `p3at::base_link` does not exist in the Gazebo model, only `p3at::base_footprint`.

---

## Git Workflow

### What's Tracked

- Source packages: `catkin_ws/src/` (all navigation, description, gazebo, target_follower, base packages)
- Launch files, URDF/Xacro models, RViz configs
- Navigation parameters: `param/` YAML files
- Test scripts: `waypoint_test.py`, `test_standoff_face.py`
- Submodule: `ros_ws/src/amr-ros-config/` (MobileRobots AMR configuration)
- Helper scripts: `build_and_hint.sh`
- Documentation: `README.md`

### What's Ignored (`.gitignore`)

- Build artifacts: `catkin_ws/build/`, `catkin_ws/devel/`, `ros_ws/build/`, `ros_ws/devel/`
- Generated maps: `*.pgm`, `*.yaml` in `maps/` (optional: commit reference maps)
- TF frame outputs: `frames.gv`, `frames.pdf`
- Python caches: `__pycache__/`
- Catkin workspace marker: `.catkin_workspace`

### Setup After Clone

```bash
# 1. Clone repository
git clone <repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial

# 2. Initialize submodules
git submodule update --init --recursive

# 3. Switch to navigation branch
git checkout real_robot_navigation

# 4. Build workspace
cd catkin_ws
catkin_make
source devel/setup.bash  # or setup.zsh
```

---

## Resources

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [GMapping SLAM](http://wiki.ros.org/gmapping)
- [AMCL Localization](http://wiki.ros.org/amcl)
- [move_base Navigation](http://wiki.ros.org/move_base)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [NavfnROS Global Planner](http://wiki.ros.org/navfn)
- [map_server Package](http://wiki.ros.org/map_server)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [depthimage_to_laserscan Package](http://wiki.ros.org/depthimage_to_laserscan) (used on `main` branch)
- [sicktoolbox_wrapper (LMS200)](http://wiki.ros.org/sicktoolbox_wrapper)
- [RosAria (P3-AT driver)](http://wiki.ros.org/ROSARIA)
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [WSL2 GPU Rendering Guide](https://zhuanlan.zhihu.com/p/19575977500)

---

## Status

- [x] Gazebo simulation (gmapping + move_base + target following) -- verified
- [x] Simulation laser self-collision bug fixed -- verified (commit `5d9e4d2`)
- [x] Obstacle detection (obstacle_layer, inflation_layer in costmap) -- verified
- [x] 3-waypoint sequential navigation test -- all SUCCEEDED (errors < 0.2 m)
- [x] Map saving and loading (map_server + AMCL) -- verified
- [x] `standoff_distance` feature -- implemented and verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] `face_target` feature -- implemented and verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] Unit test suite for standoff + face_target logic -- 21/21 pass (commit `8e189ba`)
- [x] Real robot launch files (mapping + navigation) -- created, pending hardware test
- [x] Raspberry Pi base driver package (p3at_base) -- created, pending hardware test
- [ ] YOLO target detection node -- not started
- [ ] Multi-machine network configuration scripts -- not started
- [ ] Real hardware parameter tuning -- not started

---

## Post-Installation Checklist

Verify everything works after cloning and building:

### Environment Setup
- [ ] Workspace builds without errors: `catkin_make`
- [ ] ROS sourcing works: `source devel/setup.zsh && rospack find p3at_lms_navigation`
- [ ] Submodule initialized: `ls ros_ws/src/amr-ros-config/`

### Gazebo Simulation
- [ ] Gazebo launches: `roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false`
- [ ] Robot spawns in simulation (no errors in terminal)
- [ ] Wait ~30 seconds, then verify core nodes: `rosnode list`
- [ ] Lidar scan publishes at ~10 Hz: `rostopic hz /scan`
- [ ] Odometry publishes at ~100 Hz: `rostopic hz /odom`
- [ ] Map is being built: `rostopic hz /map`
- [ ] TF tree complete: `rosrun tf tf_echo map base_link`
- [ ] RViz shows robot model, laser scan, and map

### Navigation
- [ ] Manual goal succeeds: send a `2D Nav Goal` in RViz or publish to `/move_base_simple/goal`
- [ ] Robot moves toward the goal and stops near it
- [ ] Waypoint test passes: `python3 src/p3at_lms_navigation/scripts/waypoint_test.py`
- [ ] Map saving works: `rosrun map_server map_saver -f /tmp/test_map`

### Target Following
- [ ] Target follower works: `roslaunch p3at_lms_navigation mapping.launch` (default mode)
- [ ] Drag "target" model in Gazebo; robot follows
- [ ] Unit tests pass: `python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py` (21/21)

### AMCL Navigation (requires a saved map)
- [ ] AMCL launches: `roslaunch p3at_lms_navigation nav.launch map_file:=<path_to_map.yaml>`
- [ ] Set initial pose with "2D Pose Estimate" in RViz
- [ ] Particle cloud converges around robot
- [ ] Navigation goal succeeds with AMCL localization
