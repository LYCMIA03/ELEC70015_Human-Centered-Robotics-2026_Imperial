# P3-AT + LMS200 Navigation System (ROS1 Noetic)

A ROS1 Noetic workspace for the **ELEC70015 Human-Centered Robotics** course at Imperial College London. It provides both **Gazebo simulation** and **real robot** launch files for a Pioneer 3-AT mobile robot equipped with a SICK LMS200 2D lidar.

The LMS200 lidar handles **all** mapping, localization, path planning, and local obstacle avoidance. A depth camera (Orbbec Femto Bolt) is reserved for future target detection only and is not involved in navigation.

> **Note for WSL2 Users:** If the robot model displays correctly in RViz but the P3AT visual mesh does not appear in Gazebo, this may be due to GPU rendering limitations in WSL2. See the [Troubleshooting](#troubleshooting) section for solutions.

The repository contains two workspaces:
- **`catkin_ws/`** — LMS200 LiDAR navigation stack (this branch, documented below)
- **`ros_ws/`** — Depth camera simulation with `depthimage_to_laserscan` (see `main` branch)

## Table of Contents

- [System Overview](#system-overview)
- [Hardware](#hardware)
- [Repository Structure](#repository-structure)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Dependencies](#dependencies)
- [Build](#build)
- **Part A — Gazebo Simulation**
  - [Usage: Gazebo Simulation](#usage-gazebo-simulation)
    - [Mapping + Manual Navigation (gmapping)](#1-mapping--manual-navigation-gmapping)
    - [Testing Navigation (3-Waypoint Test)](#2-testing-navigation-3-waypoint-test)
    - [Testing Target Follower Features (Unit Tests)](#3-testing-target-follower-features-unit-tests)
    - [Navigation on a Saved Map (AMCL)](#4-navigation-on-a-saved-map-amcl)
    - [Target Following](#5-target-following)
    - [Autonomous Mapping + AMCL Verification](#6-autonomous-mapping--amcl-verification)
  - [Verifying the System (Simulation)](#verifying-the-system)
  - [Troubleshooting (Simulation)](#troubleshooting)
  - [Simulation Calibration Notes (Bug Fixes)](#simulation-calibration-notes-bug-fixes)
- **Part B — Real Robot Deployment**
  - [Real Robot Architecture Overview](#real-robot-architecture-overview)
  - [Hardware Wiring & Connectivity](#hardware-wiring--connectivity)
  - [Real Robot Dependencies (Complete List)](#real-robot-dependencies-complete-list)
    - [Jetson ORIN NANO Dependencies](#jetson-orin-nano-dependencies)
    - [Raspberry Pi Dependencies](#raspberry-pi-dependencies)
    - [Orbbec Femto Bolt (Future YOLO)](#orbbec-femto-bolt-future-yolo)
  - [Multi-Machine Network Setup (Jetson + Raspberry Pi)](#multi-machine-network-setup-jetson--raspberry-pi)
    - [Step 0 — Static IP Configuration](#step-0--static-ip-configuration)
    - [Step 1 — Docker Container Setup](#step-1--docker-container-setup)
    - [Step 2 — Start ROS Master (Jetson)](#step-2--start-ros-master-jetson)
    - [Step 3 — Connect Pi to Master](#step-3--connect-pi-to-master)
    - [Step 4 — Cross-Machine Topic Test](#step-4--cross-machine-topic-test)
    - [Reconnecting Scripts](#reconnecting-scripts)
  - [Real Robot Launch Procedures](#real-robot-launch-procedures)
    - [Step 1 — Start Base Driver (Raspberry Pi)](#step-1---start-base-driver-raspberry-pi)
    - [Step 2a — Mapping (Jetson)](#step-2a---mapping-jetson)
    - [Step 2b — Navigation (Jetson)](#step-2b---navigation-jetson)
  - [Real Robot TF Tree & Differences from Simulation](#real-robot-tf-tree--differences-from-simulation)
  - [Real Robot Parameter Tuning Notes](#real-robot-parameter-tuning-notes)
  - [Common Pitfalls (Real Robot)](#common-pitfalls-real-robot)
- [Key Topics and TF Frames](#key-topics-and-tf-frames)
  - [Node Graph](#node-graph)
  - [Complete Node List](#complete-node-list-mapping-mode-observed-at-runtime)
  - [Topic Pub/Sub Reference](#topic-pubsub-reference)
  - [TF Tree](#tf-tree)
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
├── catkin_ws/                 # LMS200 LiDAR navigation workspace (real_robot_navigation branch)
│   ├── src/
│   │   ├── p3at_lms_description/   # URDF/Xacro robot model (P3-AT + laser link)
│   │   ├── p3at_lms_gazebo/        # Gazebo world, target model, simulation launch
│   │   ├── p3at_lms_navigation/    # Navigation stack (gmapping, AMCL, move_base)
│   │   ├── target_follower/        # Target following system
│   │   └── p3at_base/              # Raspberry Pi base driver
│   ├── build/                  # Build artifacts (not tracked)
│   └── devel/                  # Development space (not tracked)
├── ros_ws/                    # Depth camera simulation workspace (in main branch)
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
│   └── worlds/
│       ├── p3at_lms.world              # Default world (2 box obstacles)
│       └── complex_maze.world          # 12×12 m maze for autonomous mapping test
├── p3at_lms_navigation/     # Navigation config, params, and all launch files
│   ├── launch/
│   │   ├── mapping.launch              # Sim: Gazebo + gmapping + move_base
│   │   ├── nav.launch                  # Sim: Gazebo + AMCL + move_base
│   │   ├── auto_mapping.launch         # Autonomous frontier exploration + map save
│   │   ├── auto_amcl_verify.launch     # Automated AMCL accuracy verification
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
│   │   ├── waypoint_test.py        # Sequential 3-waypoint navigation test
│   │   ├── autonomous_explorer.py  # Frontier-based autonomous exploration node
│   │   └── amcl_verifier.py        # AMCL accuracy verifier (vs. Gazebo ground truth)
│   └── maps/                       # Saved maps (created by map_saver / auto_mapping)
├── target_follower/         # Target following system
│   ├── scripts/
│   │   ├── target_follower.py          # Subscribes /target_pose, sends MoveBaseGoal
│   │   │                               #   (supports standoff_distance and face_target)
│   │   ├── gazebo_target_publisher.py  # Publishes Gazebo model pose as /target_pose
│   │   ├── move_target.py              # Moves Gazebo target model along waypoints (dynamic target)
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

# Part A — Gazebo Simulation

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

### 2. Testing Navigation (3-Waypoint Test)

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

### 3. Testing Target Follower Features (Unit Tests)

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

### 4. Navigation on a Saved Map (AMCL)

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

### 5. Target Following

**Gazebo model tracking** (default in `mapping.launch`): The `gazebo_target_publisher` reads the Gazebo "target" model pose and publishes it as `/target_pose`. The `target_follower` node converts this into a continuous stream of `MoveBaseGoal` actions.

**RViz 2D Nav Goal relay**: Use RViz to set the target instead of a Gazebo model:
```bash
roslaunch p3at_lms_navigation mapping.launch \
  use_gazebo_target:=false use_rviz_goal_relay:=true
```
Now every "2D Nav Goal" click in RViz is forwarded to `/target_pose` and triggers the follower.

**Programmatic target** (e.g., from a YOLO detector): Publish to `/target_pose` from any node. The pose can be in `map`, `odom`, or `base_link` frame -- `target_follower` handles the TF transform automatically.

#### Dynamic Target Following

The `move_target` parameter enables automatic movement of the Gazebo "target" model along a predefined waypoint loop. This simulates a moving person or object for the robot to chase, providing a fully autonomous test of the target-following pipeline without manual Gazebo interaction.

```bash
# Launch with dynamic target (recommended for testing)
roslaunch p3at_lms_navigation mapping.launch \
  move_target:=true target_speed:=0.3 target_pause:=2.0
```

This starts the `move_target` node, which calls Gazebo's `/gazebo/set_model_state` service to move the target model at `target_speed` m/s, pausing `target_pause` seconds at each waypoint.

**Default waypoint loop** (avoids the two box obstacles in the world):

| Waypoint | Position (x, y) | Notes |
|----------|-----------------|-------|
| WP0 | (4.0, 0.0) | Start (target spawn position) |
| WP1 | (4.0, 2.0) | North |
| WP2 | (1.0, 2.0) | West, north of obstacle_1 |
| WP3 | (1.0, -2.0) | South |
| WP4 | (4.0, -2.0) | East, south of obstacle_2 |
| WP5 | (4.0, 0.0) | Back to start (loops) |

**Data flow:**

```
move_target ──set_model_state──► Gazebo target model
                                       │
                              get_model_state
                                       ▼
                          gazebo_target_publisher
                            /target_pose (10 Hz)
                                       ▼
                             target_follower
                            MoveBaseGoal (2 Hz)
                                       ▼
                              move_base
                     NavfnROS + DWA → /cmd_vel
```

**Combined with standoff + face_target** (recommended for human-following demo):

```bash
roslaunch p3at_lms_navigation mapping.launch \
  move_target:=true target_speed:=0.3 target_pause:=2.0 \
  standoff_distance:=1.0 face_target:=true
```

The robot will maintain a 1.0 m gap and face the target as it follows.

**Runtime tuning** (no restart needed):

```bash
# Change target speed
rosparam set /move_target/speed 0.5

# Change follower parameters
rosparam set /target_follower/standoff_distance 2.0
rosparam set /target_follower/send_rate_hz 4.0
rosparam set /target_follower/min_update_dist 0.15
```

**`move_target` parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~model_name` | string | `target` | Gazebo model name to move |
| `~speed` | float | `0.4` | Movement speed (m/s), tunable at runtime |
| `~pause` | float | `1.0` | Pause duration at each waypoint (s) |
| `~loop` | bool | `true` | Loop through waypoints continuously |
| `~waypoints` | list | (see above) | Override waypoints as list of `[x, y]` pairs |

> **Note:** The target model is a collision-free visual marker (red cylinder). It will
> not physically interact with the robot or appear as an obstacle in the laser scan /
> costmap. This is intentional — the robot navigates around real obstacles while
> following the ghost target.

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

> **See also:** [Section 6 — Autonomous Mapping + AMCL Verification](#6-autonomous-mapping--amcl-verification) for the fully automated two-phase pipeline.

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

### 6. Autonomous Mapping + AMCL Verification

This pipeline automates two-phase testing inside the purpose-built `complex_maze.world` environment:

- **Phase 1** — The robot explores the maze autonomously using a frontier-based algorithm (`autonomous_explorer.py`) and saves the resulting map to `maps/`.
- **Phase 2** — The saved map is loaded, the robot navigates a waypoint circuit, and `amcl_verifier.py` continuously compares the AMCL position estimate to Gazebo ground truth — generating a `PASS`/`FAIL` accuracy report.

#### Environment: `complex_maze.world`

The maze is a **12 × 12 m** enclosed arena featuring:
- Horizontal and vertical dividing walls (0.20 m thick) creating rooms and corridors
- Door gaps of 2.5 m throughout for reliable passage
- Varied obstacles: table, pillar, box, narrow column, L-shaped corridor segment

The robot spawns at the origin (0, 0) facing north.

#### Option A — One-click full pipeline

`run_full_pipeline.sh` at the repository root runs both phases automatically:

```bash
cd ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
bash run_full_pipeline.sh
```

What it does:
1. Launches Gazebo + gmapping + move_base + `autonomous_explorer` (Phase 1, up to 10 min)
2. Kills all ROS/Gazebo processes when exploration completes or times out
3. Launches Gazebo + map_server + AMCL + move_base + `amcl_verifier` (Phase 2)
4. Waits for the verification to finish and prints the report path

Both Gazebo and RViz windows open automatically. The pipeline exits when Phase 2 finishes.

> **Tip:** For a faster run, reduce `exploration_timeout` to 180–240 s inside
> `auto_mapping.launch` or pass it as argument:
> ```bash
> # Edit the script to pass exploration_timeout:=240, or run phases manually (Options B + C)
> ```

#### Option B — Phase 1 only: Autonomous Mapping

```bash
source ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.zsh
roslaunch p3at_lms_navigation auto_mapping.launch \
  gui:=true \
  exploration_timeout:=300 \
  map_save_name:=complex_maze_map
```

**Key launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `true` | Show Gazebo and RViz windows |
| `exploration_timeout` | `600` | Maximum exploration time (seconds) |
| `min_frontier_size` | `8` | Minimum frontier cluster size (cells) to consider |
| `goal_timeout` | `45.0` | Abort an individual navigation goal after this many seconds |
| `robot_radius` | `0.55` | Inflation radius used by explorer to reject goals near walls (m) |
| `frontier_blacklist_radius` | `0.8` | Blacklist radius around a failed frontier centre (m) |
| `map_save_name` | `explored_map` | Output filename prefix (saved to `maps/`) |
| `initial_wait` | `12` | Seconds to wait after Gazebo loads before starting exploration |

**What to expect in RViz:**
- The occupancy grid map grows progressively as the robot moves
- Terminal logs print coverage percentage and goal outcomes in real time

**When Phase 1 finishes**, the explorer calls `map_saver` automatically:
```
[INFO] Map saved: .../maps/complex_maze_map.pgm
[INFO] Map saved: .../maps/complex_maze_map.yaml
```
Killing the launch with `Ctrl+C` also triggers the save via the node's shutdown hook.

> **Note on coverage:** A 300 s run typically achieves 30–60% coverage of a 12 × 12 m maze.
> A full exploration pass takes 8–10 minutes (`exploration_timeout:=600`). Higher coverage
> means more of the map is known to the global planner, which improves navigation success
> during AMCL verification.

#### Option C — Phase 2 only: AMCL Verification (requires Phase 1 map)

```bash
source ~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.zsh
roslaunch p3at_lms_navigation auto_amcl_verify.launch \
  gui:=true \
  map_file:=$(rospack find p3at_lms_navigation)/maps/complex_maze_map.yaml
```

This launches: Gazebo (`complex_maze.world`), `map_server`, `amcl`, `move_base`, `amcl_verifier`, and RViz.

**Initialization sequence** (fully automated — no user input needed):
1. Wait 15 s for all nodes to settle
2. Publish initial pose at (0, 0, 0°) to `/initialpose`
3. Spin gently in place for 8 s to spread AMCL particles
4. Begin waypoint navigation circuit

**Default waypoints:**

| Waypoint | Position (x, y) | Purpose |
|----------|-----------------|--------|
| `WP1_south` | (0.0, −1.5) | Short run south in main corridor |
| `WP2_north` | (0.0, +1.5) | Run north through same corridor |
| `WP3_west` | (−1.5, 0.0) | Cross to west side |
| `WP4_east_s` | (+1.5, −0.5) | East side, mild detour |
| `WP5_north2` | (0.0, +2.5) | Approach north wall gap |
| `WP6_return` | (0.0, 0.0) | Return to origin |

At each waypoint the verifier records position error (AMCL vs. Gazebo ground truth), yaw error, and particle cloud covariance. Continuous error sampling at 0.5 Hz runs in the background throughout the full mission.

> **Note:** If some waypoints report navigation failed, this is most likely because the
> saved map does not yet cover those regions due to limited exploration time. The AMCL
> accuracy metrics are still recorded and remain valid regardless of navigation outcome.

#### Reading the Verification Report

Two files are written to `maps/` when Phase 2 finishes:
- `amcl_report.txt` — human-readable summary
- `amcl_report.json` — machine-readable (for scripting or plotting)

**Sample report (verified run, 300 s exploration):**

```
======================================================================
  AMCL LOCALIZATION VERIFICATION REPORT
  Generated: 2026-02-20 23:13:08
======================================================================

--- Convergence ---
  Convergence time:   1.0 s (threshold: 0.30 m)

--- Continuous Tracking Statistics ---
  Samples:           706
  Mean pos error:    0.0924 m
  Max pos error:     0.5996 m
  Std pos error:     0.0884 m
  Median pos error:  0.0731 m
  Mean yaw error:    4.33°
  Max yaw error:     33.72°

--- Per-Waypoint Results ---
  Waypoint            NavOK  Pos Err(m)  Yaw Err(°)  Cov(xy)  Spread
  -------------------------------------------------------------------
  WP1_south           NO     0.1062      0.13        0.4137   0.7367
  WP2_north           YES    0.1838      5.12        0.1564   0.4687
  WP3_west            NO     0.1293      0.84        0.7874   1.0542
  WP4_east_s          YES    0.0731      5.62        0.0087   0.1063
  WP5_north2          NO     0.0065      3.03        0.2508   0.5658
  WP6_return          YES    0.0378      9.38        0.1429   0.3932

--- Waypoint Summary ---
  Navigation success:   3 / 6
  Mean position error:  0.0894 m
  Max position error:   0.1838 m
  Mean yaw error:       4.02°

--- PASS/FAIL Criteria ---
  Mean position error < 0.30 m:  PASS (0.0894 m)
  Max position error  < 0.50 m:  PASS (0.1838 m)
  Navigation success  >= 80%:    FAIL (50%)

  OVERALL: *** FAIL ***
======================================================================
```

**Interpreting the results:**

| Field | What it means |
|-------|---------------|
| `Convergence time` | How quickly AMCL found the robot after the initial pose was published. < 2 s is excellent. |
| `Mean pos error` | Average metres between AMCL belief and Gazebo ground truth. < 0.15 m is excellent; < 0.30 m is acceptable. |
| `Max pos error` | Worst single error sample. Can spike during fast turns or in narrow corridors. |
| `Median pos error` | More robust central tendency; unaffected by transient spikes. |
| `Cov(xy)` | AMCL's own uncertainty estimate (xy covariance trace). Lower = more confident. |
| `Spread` | Particle cloud spread (m). Low spread on arrival = tight convergence. |
| `NavOK` | Whether `move_base` successfully drove the robot to this waypoint. |
| Overall PASS/FAIL | Passes when mean pos error < 0.30 m **and** max pos error < 0.50 m **and** navigation success ≥ 80%. |

**Key insight:** The AMCL accuracy metrics (mean/max position error) are computed independently of navigation success — even if the robot does not physically reach a waypoint, the verifier records the localization error at the time of the goal attempt. A `FAIL` on navigation success combined with `PASS` on both accuracy metrics confirms that **AMCL localization itself is working correctly**; the navigation shortfall is caused by insufficient map coverage, not a localization problem.

#### How `autonomous_explorer.py` Works

The explorer implements a **frontier-based** exploration loop:

1. Subscribes to `/map` (gmapping `OccupancyGrid`, updated at ~1 Hz)
2. Finds **frontier cells** — free cells (value 0) adjacent to unknown cells (value −1)
3. Clusters nearby frontiers via BFS; discards clusters below `min_frontier_size` cells
4. Scores each cluster: `score = cluster_size / (distance_to_robot + 1)`
5. Blacklists centres of recently-failed frontiers within `frontier_blacklist_radius`
6. Sends the best frontier as a `MoveBaseGoal`; waits up to `goal_timeout` seconds
7. Repeats until `exploration_timeout` expires or no frontiers remain
8. Calls `map_saver` to write the map and shuts down

**ROS interface:**

| Topic / Interface | Direction | Purpose |
|-------------------|-----------|---------|
| `/map` | Subscribe | `OccupancyGrid` from gmapping |
| `/odom` | Subscribe | Robot position for distance scoring |
| `move_base` action | Client | Send frontier navigation goals |
| `map_saver` (subprocess) | Spawn on exit | Save final map to disk |

#### How `amcl_verifier.py` Works

1. Waits for `move_base` action server, `/amcl_pose`, and `/gazebo/model_states`
2. Publishes initial pose → waits for error to drop below `convergence_threshold` (0.30 m)
3. Sends a gentle spin via `/cmd_vel` for 8 s to spread particles
4. Loops over each waypoint:
   - Sends `MoveBaseGoal`; waits up to `goal_timeout` seconds
   - On arrival **or** timeout, records AMCL error vs. Gazebo ground truth
5. Continuously samples error at 0.5 Hz throughout all navigation
6. Writes `amcl_report.txt` and `amcl_report.json` on shutdown

**Pass/fail thresholds:**

| Criterion | Threshold |
|-----------|-----------|
| Mean position error | < 0.30 m |
| Max position error | < 0.50 m |
| Navigation success rate | ≥ 80% |

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

# Part B — Real Robot Deployment

---

## Real Robot Architecture Overview

The real robot system uses a **dual-machine architecture** connected via a dedicated point-to-point Ethernet link. Both machines run ROS Noetic inside Docker containers with host networking.

```
┌───────────────────────────────────────────────────────────────────────────┐
│                          PHYSICAL CONNECTIONS                            │
│                                                                          │
│  ┌──────────────────────┐    Ethernet (direct)    ┌──────────────────┐   │
│  │  Jetson ORIN NANO    │◄───────────────────────►│  Raspberry Pi    │   │
│  │  Ubuntu 22.04        │    192.168.50.1/24       │  Ubuntu 24.04   │   │
│  │  Docker: ros_noetic  │    192.168.50.2/24       │  Docker: noetic_pi│  │
│  │  (ROS Master)        │                          │                 │   │
│  ├──────────────────────┤                          ├─────────────────┤   │
│  │  • roscore           │                          │  • RosAria      │   │
│  │  • sicklms (LMS200)  │                          │  • odom_repub   │   │
│  │  • gmapping / AMCL   │◄── /odom, TF ───────────│                 │   │
│  │  • move_base         │─── /cmd_vel ────────────►│                 │   │
│  │  • robot_state_pub   │                          │                 │   │
│  │  • RViz              │                          │                 │   │
│  │  • target_follower*  │                          │                 │   │
│  ├──────────────────────┤                          ├─────────────────┤   │
│  │  SICK LMS200 (RS-232)│                          │  P3-AT (Serial) │   │
│  │  /dev/ttyUSB0        │                          │  /dev/ttyS0     │   │
│  │                      │                          │  or /dev/ttyUSB0│   │
│  │  Orbbec Femto Bolt** │                          │                 │   │
│  │  (USB3, future YOLO) │                          │                 │   │
│  └──────────────────────┘                          └─────────────────┘   │
│                                                                          │
│  * Optional   ** Future — not used in current navigation                 │
└───────────────────────────────────────────────────────────────────────────┘
```

**Simulation vs. Real Robot — Key Differences:**

| Aspect | Simulation | Real Robot |
|--------|------------|------------|
| **Lidar driver** | Gazebo `libgazebo_ros_laser.so` plugin | `sicktoolbox_wrapper/sicklms` node |
| **Odometry source** | Gazebo `libgazebo_ros_skid_steer_drive.so` | `RosAria` + `odom_republisher.py` |
| **`/cmd_vel` consumer** | Gazebo plugin (simulated motors) | `RosAria` → P3-AT hardware |
| **TF: odom→base** | Gazebo → `odom→base_footprint` | RosAria → `odom→base_link` |
| **Clock** | `use_sim_time=true` (Gazebo clock) | System wall clock |
| **Network** | Single machine (localhost) | Dual machine (Ethernet P2P, Docker `--net=host`) |
| **Target following source** | `gazebo_target_publisher.py` (Gazebo model) | YOLO detector → `/target_pose` (future) |
| **Launch files** | `mapping.launch`, `nav.launch` | `real_robot_mapping.launch`, `real_robot_nav.launch` |

## Hardware Wiring & Connectivity

| Connection | From | To | Cable / Interface | Notes |
|------------|------|----|--------------------|-------|
| Ethernet (ROS network) | Jetson `eth0` | Pi `eth0` | CAT5/6 Ethernet (direct P2P) | Static IP `192.168.50.x/24`, no switch needed |
| LMS200 lidar | Jetson USB port | SICK LMS200 | RS-232 → USB adapter (FTDI) | Appears as `/dev/ttyUSB0`; baud `38400` |
| P3-AT chassis | Pi serial port | P3-AT microcontroller | Serial RS-232 (or USB-Serial) | Default `/dev/ttyS0` (GPIO UART) or `/dev/ttyUSB0` |
| Orbbec Femto Bolt | Jetson USB3 port | Femto Bolt camera | USB 3.0 Type-C | Future YOLO only; not used for navigation |
| Power | P3-AT battery | Jetson + Pi | DC-DC converter or external battery | Ensure stable 5V for Pi, 5–19V for Jetson |
| Wi-Fi (optional) | Jetson / Pi | Access point | Wi-Fi adapter | For SSH access during development; **NOT** used for ROS traffic |

## Real Robot Dependencies (Complete List)

### Jetson ORIN NANO Dependencies

**Host OS:** Ubuntu 22.04 (JetPack)

**Docker Image:** `ros:noetic-ros-base` (or `ros:noetic-ros-core`)

> All ROS packages below are installed **inside** the Docker container (`ros_noetic`), not on the host OS.

#### ROS Packages (inside Docker)

```bash
# Core navigation stack
sudo apt update && sudo apt install -y \
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
  ros-noetic-tf \
  ros-noetic-actionlib \
  ros-noetic-move-base-msgs

# Visualization (requires X11 forwarding or local display)
sudo apt install -y ros-noetic-rviz

# ★ LMS200 lidar driver — real robot specific
sudo apt install -y ros-noetic-sicktoolbox-wrapper

# Optional: keyboard teleop for manual testing
sudo apt install -y ros-noetic-teleop-twist-keyboard
```

If `sicktoolbox_wrapper` is not available via apt (common on arm64/Jetson):
```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/sicktoolbox_wrapper.git
# May also need sicktoolbox itself:
git clone https://github.com/ros-drivers/sicktoolbox.git
cd ~/catkin_ws && catkin_make
```

#### Serial Port Permissions (Host OS)

The Docker container needs access to `/dev/ttyUSB0` for the LMS200. Grant permissions on the host:
```bash
# Option A: Add user to dialout group (persistent)
sudo usermod -aG dialout $USER

# Option B: Set udev rule for FTDI adapter (persistent, more specific)
sudo tee /etc/udev/rules.d/99-lms200.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="lms200"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# Option C: Quick one-time permission
sudo chmod 666 /dev/ttyUSB0
```

The Docker container must map the device (use `--privileged` or `-v /dev:/dev`).

#### Python Dependencies (inside Docker)

```bash
pip install numpy>=1.24.4 scipy>=1.10.1 defusedxml
```

#### Orbbec Femto Bolt Driver (Future — YOLO Target Detection)

> **Not needed for the current navigation stack.** Install only when integrating the YOLO target detector.

The Femto Bolt requires the Orbbec SDK and ROS wrapper:

```bash
# Host OS (Jetson): Install Orbbec SDK udev rules
# Download from: https://github.com/orbbec/OrbbecSDK_ROS1
sudo bash install_udev_rules.sh

# Inside Docker container:
cd ~/catkin_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
cd ~/catkin_ws && catkin_make

# Topics published:
#   /camera/color/image_raw      (sensor_msgs/Image, RGB)
#   /camera/depth/image_raw      (sensor_msgs/Image, 16UC1 depth)
#   /camera/depth/camera_info    (sensor_msgs/CameraInfo)
```

### Raspberry Pi Dependencies

**Host OS:** Ubuntu 24.04

**Docker Image:** `ros:noetic-ros-base`

> All ROS packages below are installed **inside** the Docker container (`noetic_pi`).

#### ROS Packages (inside Docker)

```bash
sudo apt update && sudo apt install -y \
  ros-noetic-rospy \
  ros-noetic-nav-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-tf2-ros \
  ros-noetic-tf

# ★ P3-AT chassis driver — real robot specific
sudo apt install -y ros-noetic-rosaria
```

If `rosaria` is not available via apt (common on arm64/Pi):
```bash
# Install AriaCoda (ARIA replacement, no license issues)
cd ~/catkin_ws/src
git clone https://github.com/reedhedges/AriaCoda.git
cd AriaCoda && make -j$(nproc) && sudo make install

# Install RosAria
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
cd ~/catkin_ws && catkin_make
```

#### Serial Port Permissions (Host OS)

```bash
# For GPIO UART (/dev/ttyS0):
sudo usermod -aG dialout $USER

# For USB-Serial adapter (/dev/ttyUSB0):
sudo chmod 666 /dev/ttyUSB0
# Or add udev rule similar to the Jetson section
```

The Docker container must be started with `--privileged` and `-v /dev:/dev` to access serial ports.

### Orbbec Femto Bolt (Future YOLO)

| Item | Details |
|------|---------|
| **Interface** | USB 3.0 Type-C → Jetson |
| **Driver** | [OrbbecSDK_ROS1](https://github.com/orbbec/OrbbecSDK_ROS1) |
| **Topics** | `/camera/color/image_raw`, `/camera/depth/image_raw` |
| **Purpose** | RGB+Depth for YOLO person detection → publish `/target_pose` |
| **When to install** | Only when implementing the YOLO target detection node |
| **Note** | Camera is **not** involved in mapping, localization, or obstacle avoidance |

### Complete Dependency Summary Table

| Package / Driver | Jetson (Docker) | Pi (Docker) | Purpose |
|-----------------|:-:|:-:|---------|
| `ros-noetic-roscore` | ✅ | (uses Jetson's) | ROS master |
| `ros-noetic-rospy` | ✅ | ✅ | Python ROS client |
| `ros-noetic-sicktoolbox-wrapper` | ✅ | — | LMS200 lidar driver |
| `ros-noetic-rosaria` | — | ✅ | P3-AT chassis driver |
| `ros-noetic-slam-gmapping` | ✅ | — | SLAM mapping |
| `ros-noetic-amcl` | ✅ | — | Localization (on saved map) |
| `ros-noetic-move-base` | ✅ | — | Navigation planner |
| `ros-noetic-map-server` | ✅ | — | Static map server |
| `ros-noetic-dwa-local-planner` | ✅ | — | DWA local planner |
| `ros-noetic-navfn` | ✅ | — | NavfnROS global planner |
| `ros-noetic-xacro` | ✅ | — | URDF processing |
| `ros-noetic-robot-state-publisher` | ✅ | — | TF from URDF |
| `ros-noetic-tf2-ros` | ✅ | ✅ | TF2 library |
| `ros-noetic-tf` | ✅ | ✅ | TF library |
| `ros-noetic-actionlib` | ✅ | — | Action client (target_follower) |
| `ros-noetic-move-base-msgs` | ✅ | — | MoveBase action messages |
| `ros-noetic-rviz` | ✅ | — | Visualization |
| `ros-noetic-nav-msgs` | ✅ | ✅ | Odometry messages |
| `ros-noetic-geometry-msgs` | ✅ | ✅ | Geometry messages |
| `ros-noetic-teleop-twist-keyboard` | ✅ (optional) | — | Manual driving |
| AriaCoda (source) | — | ✅ (if apt unavailable) | ARIA library for RosAria |
| OrbbecSDK_ROS1 (source) | Future | — | Femto Bolt camera driver |

---

## Multi-Machine Network Setup (Jetson + Raspberry Pi)

This section describes the complete multi-machine ROS1 setup using:
- **Direct point-to-point Ethernet** link between Jetson and Pi
- **Static IPs** via netplan on both hosts
- **ROS Noetic inside Docker** on both machines (`--net=host`)
- **ROS master (`roscore`)** runs on Jetson

```
┌─────────────────────────┐                    ┌─────────────────────────┐
│  Jetson ORIN NANO        │    Direct Ethernet  │  Raspberry Pi           │
│  Host: Ubuntu 22.04      │◄──────────────────►│  Host: Ubuntu 24.04     │
│  Container: ros_noetic   │                    │  Container: noetic_pi   │
│  IP: 192.168.50.1        │                    │  IP: 192.168.50.2       │
│  Role: ROS Master        │                    │  Role: Base Driver      │
└─────────────────────────┘                    └─────────────────────────┘
```

> **Important:** Always set `ROS_IP` to the Ethernet IP (`192.168.50.x`), **never** to a Wi-Fi IP. ROS1 uses additional dynamic TCP/UDP ports beyond 11311, so a dedicated Ethernet link is the most stable option.

### Step 0 — Static IP Configuration

#### 0.1 Identify the Ethernet interface name (run on both hosts)

```bash
ip -br link
ip -br addr
```

You should see something like `eth0` or `enp...`. Replace `eth0` below if yours differs.

#### 0.2 Raspberry Pi host (Ubuntu 24.04) — netplan

```bash
sudo tee /etc/netplan/99-p2p-ros.yaml > /dev/null <<'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.50.2/24]
      optional: true
EOF

sudo netplan generate
sudo netplan apply
ip -br addr show eth0
```

#### 0.3 Jetson host (Ubuntu 22.04) — netplan

```bash
sudo tee /etc/netplan/99-p2p-ros.yaml > /dev/null <<'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.50.1/24]
      optional: true
EOF

sudo netplan generate
sudo netplan apply
ip -br addr show eth0
```

#### 0.4 Verify link

```bash
# From Pi host:
ping -c 3 192.168.50.1

# From Jetson host:
ping -c 3 192.168.50.2
```

### Step 1 — Docker Container Setup

Both ROS containers **must** use `--net=host` for ROS1 multi-machine communication.

#### 1.1 Verify existing containers are host-network

```bash
# Jetson host:
docker inspect -f '{{.HostConfig.NetworkMode}}' ros_noetic

# Pi host:
docker inspect -f '{{.HostConfig.NetworkMode}}' noetic_pi
```

Both should print: `host`

#### 1.2 Create containers (if they don't exist)

**Jetson host:**
```bash
docker run -it -d --name ros_noetic \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v $HOME:/root \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros:noetic-ros-base bash
```

**Pi host:**
```bash
docker run -it -d --name noetic_pi \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v $HOME:/root \
  ros:noetic-ros-base bash
```

> **Note:** `--privileged` and `-v /dev:/dev` are required for serial port access (LMS200 on Jetson, P3-AT on Pi). The `-e DISPLAY` and X11 mount on Jetson enable RViz GUI rendering.

### Step 2 — Start ROS Master (Jetson)

```bash
# 2.1 Start container
docker start ros_noetic

# 2.2 Launch roscore in background (survives SSH disconnect)
docker exec -d ros_noetic bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
roscore
"

# 2.3 Verify roscore is listening
docker exec -it ros_noetic bash -lc "ss -lntp | grep 11311 || true"
```

### Step 3 — Connect Pi to Master

```bash
# 3.1 Start container
docker start noetic_pi

# 3.2 Verify Pi container can reach the master port
docker exec -it noetic_pi bash -lc \
  'timeout 2 bash -lc "cat < /dev/null > /dev/tcp/192.168.50.1/11311" && echo PORT_OK || echo PORT_FAIL'

# 3.3 ROS-level check
docker exec -it noetic_pi bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
rosnode list
"
# Expected: at least /rosout
```

### Step 4 — Cross-Machine Topic Test

Test bidirectional ROS communication **without** the robot:

**4.1 Publish from Jetson container:**
```bash
docker exec -it ros_noetic bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
rostopic pub -r 5 /chatter std_msgs/String \"data: 'hello_from_jetson'\"
"
```

**4.2 Echo on Pi container:**
```bash
docker exec -it noetic_pi bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
rostopic echo /chatter
"
```

If Pi prints the messages, **multi-machine ROS is working**.

### Reconnecting Scripts

Convenience scripts for quick reconnection after reboot.

#### Jetson host: `~/start_roscore.sh`

```bash
cat > ~/start_roscore.sh <<'SCRIPT'
#!/usr/bin/env bash
set -e
docker start ros_noetic >/dev/null

# If roscore already listening, do nothing
if docker exec ros_noetic bash -lc "ss -lntp | grep -q ':11311'"; then
  echo "roscore already listening on 11311"
  exit 0
fi

docker exec -d ros_noetic bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
roscore
"
echo "started roscore in ros_noetic"
SCRIPT
chmod +x ~/start_roscore.sh
```

#### Pi host: `~/check_ros_link.sh`

```bash
cat > ~/check_ros_link.sh <<'SCRIPT'
#!/usr/bin/env bash
set -e
docker start noetic_pi >/dev/null

docker exec noetic_pi bash -lc \
  'timeout 2 bash -lc "cat < /dev/null > /dev/tcp/192.168.50.1/11311"' \
  && echo "PORT 11311 OK" || (echo "PORT 11311 FAIL" && exit 1)

docker exec -it noetic_pi bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
rosnode list
"
SCRIPT
chmod +x ~/check_ros_link.sh
```

---

## Real Robot Launch Procedures

> **Prerequisites for all steps below:**
> 1. Static IPs configured (Step 0)
> 2. Docker containers running with `--net=host` (Step 1)
> 3. `roscore` running on Jetson (Step 2)
> 4. Pi container can reach Jetson master (Step 3)
> 5. Project workspace cloned and built inside **both** Docker containers

### Step 1 — Start Base Driver (Raspberry Pi)

Connect the P3-AT via serial cable, then launch **inside the Pi Docker container**:

```bash
docker exec -it noetic_pi bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
source ~/catkin_ws/devel/setup.bash
roslaunch p3at_base base.launch
"
```

Or specify a different serial port:
```bash
roslaunch p3at_base base.launch serial_port:=/dev/ttyUSB0
```

**What this starts:**

| Node | Function |
|------|----------|
| `RosAria` | Reads P3-AT encoder odometry, accepts `/cmd_vel`, publishes `odom→base_link` TF |
| `odom_republisher` | Republishes `/RosAria/pose` as standard `/odom` topic |

**Verify from Jetson:**
```bash
# Should see /RosAria and /odom_republisher
rosnode list

# Should show odometry data
rostopic echo /odom --noarr | head -20

# Should show TF: odom → base_link
rosrun tf tf_echo odom base_link
```

### Step 2a — Mapping (Jetson)

Connect the LMS200 via RS-232/USB adapter, then launch **inside the Jetson Docker container**:

```bash
docker exec -it ros_noetic bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source ~/catkin_ws/devel/setup.bash
roslaunch p3at_lms_navigation real_robot_mapping.launch
"
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

**What this starts:**

| Node | Package | Function |
|------|---------|----------|
| `sicklms` | `sicktoolbox_wrapper` | LMS200 driver → `/scan` in `laser` frame |
| `robot_state_publisher` | `robot_state_publisher` | URDF TF (including `base_link→laser`) |
| `slam_gmapping` | `gmapping` | Builds occupancy grid map |
| `move_base` | `move_base` | NavfnROS + DWA path planning |
| `rviz` | `rviz` | Visualization (optional) |

**Drive the robot** using keyboard teleop (in a separate terminal):
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Save the map** once coverage is complete:
```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

### Step 2b — Navigation (Jetson)

After mapping, use the saved map for AMCL-based navigation:

```bash
docker exec -it ros_noetic bash -lc "
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source ~/catkin_ws/devel/setup.bash
roslaunch p3at_lms_navigation real_robot_nav.launch \
  map_file:=\$(rospack find p3at_lms_navigation)/maps/my_map.yaml
"
```

Additional launch arguments (beyond mapping):

| Argument | Default | Description |
|----------|---------|-------------|
| `map_file` | `...maps/demo_map.yaml` | Path to the saved map YAML |

**What this starts:** `sicklms`, `robot_state_publisher`, `map_server`, `amcl`, `move_base`, and optionally RViz.

**Initialize AMCL localization:**
1. In RViz, click **"2D Pose Estimate"**
2. Click on the map at the robot's actual position, drag to set heading
3. The green particle cloud should converge around the robot

**Send a navigation goal:**
- Click **"2D Nav Goal"** in RViz, or
- Publish from command line:
  ```bash
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
  ```

---

## Real Robot TF Tree & Differences from Simulation

```
Real Robot TF Tree:
  map
  └── odom                         [gmapping (~20 Hz) or AMCL (~10 Hz)]
      └── base_link                [RosAria (publish_aria_tf=true, ~50 Hz)]
          └── laser                [robot_state_publisher (static, from URDF)]

Simulation TF Tree:
  map
  └── odom                         [gmapping (~20 Hz) or AMCL (~10 Hz)]
      └── base_footprint           [Gazebo skid-steer plugin (~50 Hz)]
          └── base_link            [robot_state_publisher (static, identity)]
              └── laser            [robot_state_publisher (static, from URDF)]
```

**Key difference:** In simulation, Gazebo publishes `odom→base_footprint`, and URDF provides an identity transform `base_footprint→base_link`. On the real robot, RosAria publishes `odom→base_link` directly (no `base_footprint` in the chain).

Since the URDF defines `base_footprint→base_link` as an identity transform, the costmap parameter `robot_base_frame: base_footprint` still works on the real robot — `robot_state_publisher` provides the missing static TF `base_footprint→base_link`. However, if you encounter TF issues, you may change it to `robot_base_frame: base_link` in both `global_costmap.yaml` and `local_costmap.yaml`.

## Real Robot Parameter Tuning Notes

The parameters in `p3at_lms_navigation/param/` are initially tuned for Gazebo simulation. Real hardware typically requires adjustments:

| Parameter | Simulation Value | Real Robot Suggestion | File | Reason |
|-----------|-----------------|----------------------|------|--------|
| `max_vel_x` | 0.5 m/s | 0.3–0.4 m/s | `move_base.yaml` | Real P3-AT may overshoot; lower for safety |
| `max_vel_theta` | 1.0 rad/s | 0.6–0.8 rad/s | `move_base.yaml` | Skid-steer turning is aggressive on carpet/tile |
| `acc_lim_x` | 0.5 m/s² | 0.2–0.3 m/s² | `move_base.yaml` | Match actual motor limits |
| `xy_goal_tolerance` | 0.10 m | 0.15–0.20 m | `move_base.yaml` | Real odometry drift requires larger tolerance |
| `particles` | 30 | 50–80 | `gmapping.yaml` | Real environments are larger/noisier |
| `linearUpdate` | 0.2 m | 0.1 m | `gmapping.yaml` | More frequent scan processing for real lidar |
| `min_particles` | 300 | 500 | `amcl.yaml` | Better localization in noisy real conditions |
| `max_particles` | 2000 | 5000 | `amcl.yaml` | Better initial convergence |
| `transform_tolerance` | 0.4 s | 0.5–0.8 s | `costmap_common.yaml` | Network latency between machines |

## Common Pitfalls (Real Robot)

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `rosnode list` fails from Pi | `ROS_MASTER_URI`/`ROS_IP` not set | Ensure both env vars are exported; use Ethernet IP only |
| `rosnode list` shows nodes but `rostopic echo` hangs | `ROS_IP` set to Wi-Fi IP instead of Ethernet | Set `ROS_IP=192.168.50.x` on both machines |
| `/scan` not publishing | LMS200 serial port wrong or permissions | Check `ls /dev/ttyUSB*`; ensure `chmod 666` or `dialout` group |
| `/odom` not publishing | P3-AT not connected or wrong serial port | Check `ls /dev/ttyS0 /dev/ttyUSB*`; verify P3-AT is powered on |
| TF: `odom→base_link` missing | RosAria not started or `publish_aria_tf=false` | Verify `base.launch` has `publish_aria_tf=true` |
| TF: `base_link→laser` missing | `robot_state_publisher` not running on Jetson | Check Jetson launch output for errors |
| Navigation aborted immediately | AMCL not yet localized | Use "2D Pose Estimate" in RViz first |
| Robot oscillates at goal | DWA tolerances too tight | Increase `xy_goal_tolerance` and `yaw_goal_tolerance` |
| Docker container not `--net=host` | Container was created without `--net=host` | Recreate container with `--net=host` |
| `roscore` not found in container | ROS not sourced | Add `source /opt/ros/noetic/setup.bash` before commands |
| RViz cannot display (Jetson) | X11 forwarding not configured | Set `DISPLAY`, mount `/tmp/.X11-unix`, run `xhost +local:docker` on host |

---

## Key Topics and TF Frames

> All data in this section was captured live from a running `mapping.launch` simulation
> (commit `8e189ba`, `use_gazebo_target:=false`).

### Node Graph

#### Mapping Mode (`mapping.launch use_gazebo_target:=false`)

Seven nodes are active in this mode.

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  Node relationships — mapping.launch (use_gazebo_target:=false)                 │
│                                                                                 │
│  /joint_state_publisher ──/joint_states──► /robot_state_publisher               │
│                                                    │                            │
│                                               /tf, /tf_static                   │
│                                                    ▼                            │
│  /gazebo ──/scan──────────────────────────► /slam_gmapping ──/map──► /move_base │
│         ──/odom──────────────────────────────────────────────────────►          │
│         ──/tf (odom→base_footprint)──────────────────────────────────►          │
│         ◄─/cmd_vel─────────────────────────────────────── /move_base            │
│                                                                │                │
│                                        /move_base/NavfnROS/plan│                │
│                                     /move_base/DWAPlannerROS/* │                │
│                                                                ▼                │
│  /rviz ◄──/map, /scan, /tf, /move_base/NavfnROS/plan, /move_base/DWAPlannerROS/local_plan
└─────────────────────────────────────────────────────────────────────────────────┘
```

#### Target Following Mode (`mapping.launch` — default with `use_gazebo_target:=true`)

Three extra nodes appear when target following is enabled:

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Additional nodes for target following                                  │
│                                                                         │
│  /move_target‡ ──/gazebo/set_model_state (service)──► /gazebo (target)  │
│                                                                         │
│  /gazebo ──/gazebo/get_model_state (service)──► /gazebo_target_publisher │
│                                                         │               │
│                                                   /target_pose          │
│                                                         ▼               │
│                                               /target_follower          │
│           /tf (for robot position lookup) ◄───────────────│             │
│                                               /move_base/goal           │
│                                                         ▼               │
│                                                   /move_base            │
└─────────────────────────────────────────────────────────────────────────┘
```

‡ Active only when `move_target:=true`.

When `use_rviz_goal_relay:=true`, an extra `/goal_to_target_relay` node bridges
`/move_base_simple/goal` (set by RViz "2D Nav Goal") to `/target_pose`.

### Complete Node List (mapping mode, observed at runtime)

| Node | Package | Role |
|------|---------|------|
| `/gazebo` | `gazebo_ros` | Physics simulator; publishes `/scan`, `/odom`, `/tf (odom→base_footprint)` |
| `/joint_state_publisher` | `joint_state_publisher` | Publishes URDF joint states to `/joint_states` |
| `/robot_state_publisher` | `robot_state_publisher` | Computes and broadcasts static/dynamic `/tf` and `/tf_static` from URDF |
| `/slam_gmapping` | `slam_gmapping` | SLAM; subscribes `/scan`+`/tf`, publishes `/map` and `/tf (map→odom)` |
| `/move_base` | `move_base` | Global + local planner; subscribes `/map`, `/scan`, `/odom`, `/tf`; publishes `/cmd_vel` |
| `/rviz` | `rviz` | Visualization; subscribes `/map`, `/scan`, `/tf`, planner paths |
| `/rosout` | `rosout` | ROS logging aggregator |
| `/gazebo_target_publisher`* | `target_follower` | Calls Gazebo service to get target model pose; publishes `/target_pose` |
| `/move_target`‡ | `target_follower` | Moves the Gazebo target model along waypoints via `set_model_state` service |
| `/target_follower`* | `target_follower` | Subscribes `/target_pose`; sends `MoveBaseAction` goals to `/move_base` |
| `/goal_to_target_relay`† | `target_follower` | Relays `/move_base_simple/goal` → `/target_pose` |

\* Active only when `use_gazebo_target:=true` (default in `mapping.launch`).  
‡ Active only when `move_target:=true`.  
† Active only when `use_rviz_goal_relay:=true`.

### Topic Pub/Sub Reference

All data observed from `rosnode info` in live simulation.

| Topic | Message Type | Publisher(s) | Subscriber(s) |
|-------|-------------|--------------|---------------|
| `/clock` | `rosgraph_msgs/Clock` | `/gazebo` | `/joint_state_publisher`, `/robot_state_publisher`, `/slam_gmapping`, `/move_base`, `/rviz` |
| `/scan` | `sensor_msgs/LaserScan` | `/gazebo` (ray sensor plugin) | `/slam_gmapping`, `/move_base`, `/rviz` |
| `/odom` | `nav_msgs/Odometry` | `/gazebo` (skid-steer plugin) | `/move_base` |
| `/cmd_vel` | `geometry_msgs/Twist` | `/move_base` | `/gazebo` |
| `/joint_states` | `sensor_msgs/JointState` | `/joint_state_publisher` | `/robot_state_publisher` |
| `/tf` | `tf2_msgs/TFMessage` | `/gazebo`, `/robot_state_publisher`, `/slam_gmapping` | `/slam_gmapping`, `/move_base`, `/rviz` |
| `/tf_static` | `tf2_msgs/TFMessage` | `/robot_state_publisher` | `/slam_gmapping`, `/move_base`, `/rviz` |
| `/map` | `nav_msgs/OccupancyGrid` | `/slam_gmapping` | `/move_base`, `/rviz` |
| `/map_metadata` | `nav_msgs/MapMetaData` | `/slam_gmapping` | — |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | `/slam_gmapping` | `/move_base`, `/rviz` |
| `/slam_gmapping/entropy` | `std_msgs/Float64` | `/slam_gmapping` | — |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz / user | `/move_base` (direct), `/goal_to_target_relay`† |
| `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | `/target_follower`* | `/move_base` |
| `/move_base/feedback` | `move_base_msgs/MoveBaseActionFeedback` | `/move_base` | `/target_follower`* |
| `/move_base/result` | `move_base_msgs/MoveBaseActionResult` | `/move_base` | `/target_follower`* |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | `/move_base` | — |
| `/move_base/cancel` | `actionlib_msgs/GoalID` | `/target_follower`* | `/move_base` |
| `/move_base/current_goal` | `geometry_msgs/PoseStamped` | `/move_base` | — |
| `/move_base/NavfnROS/plan` | `nav_msgs/Path` | `/move_base` | `/rviz` |
| `/move_base/DWAPlannerROS/global_plan` | `nav_msgs/Path` | `/move_base` | — |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | `/move_base` | `/rviz` |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `/move_base` | `/rviz` |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `/move_base` | `/rviz` |
| `/target_pose` | `geometry_msgs/PoseStamped` | `/gazebo_target_publisher`* / YOLO node / `/goal_to_target_relay`† | `/target_follower`* |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | `/gazebo` | — |
| `/gazebo/link_states` | `gazebo_msgs/LinkStates` | `/gazebo` | — |

\* Active only when `use_gazebo_target:=true`.  
† Active only when `use_rviz_goal_relay:=true`.

### TF Tree

The full TF tree captured live via `rosrun tf view_frames`:

```
map
└── odom                       [/slam_gmapping  ~20 Hz]
    └── base_footprint         [/gazebo (skid-steer plugin)  ~50 Hz]
        └── base_link          [/robot_state_publisher  static]
            ├── laser          [/robot_state_publisher  static]
            ├── top_plate      [/robot_state_publisher  static]
            ├── front_sonar    [/robot_state_publisher  static]
            ├── back_sonar     [/robot_state_publisher  static]
            ├── p3at_front_left_axle   [/robot_state_publisher  static]
            │   └── p3at_front_left_hub
            │       └── p3at_front_left_wheel   [/robot_state_publisher  ~10 Hz]
            ├── p3at_front_right_axle  [/robot_state_publisher  static]
            │   └── p3at_front_right_hub
            │       └── p3at_front_right_wheel  [/robot_state_publisher  ~10 Hz]
            ├── p3at_back_left_axle    [/robot_state_publisher  static]
            │   └── p3at_back_left_hub
            │       └── p3at_back_left_wheel    [/robot_state_publisher  ~10 Hz]
            └── p3at_back_right_axle   [/robot_state_publisher  static]
                └── p3at_back_right_hub
                    └── p3at_back_right_wheel   [/robot_state_publisher  ~10 Hz]
```

**Frame broadcaster summary:**

| TF Edge | Broadcaster | Update Rate | Notes |
|---------|-------------|-------------|-------|
| `map` → `odom` | `/slam_gmapping` | ~20 Hz | During mapping; replaced by `/amcl` during navigation |
| `odom` → `base_footprint` | `/gazebo` (skid-steer plugin) | ~50 Hz | Real robot: published by RosAria |
| `base_footprint` → `base_link` | `/robot_state_publisher` | Static (10 kHz cached) | Identity transform from URDF |
| `base_link` → `laser` | `/robot_state_publisher` | Static | Laser mount offset from URDF |
| `base_link` → `top_plate` | `/robot_state_publisher` | Static | Deck plate reference |
| `base_link` → `front_sonar` / `back_sonar` | `/robot_state_publisher` | Static | Sonar frame references |
| `base_link` → `p3at_*_axle` (×4) | `/robot_state_publisher` | Static | Wheel axle pivots |
| `p3at_*_hub` → `p3at_*_wheel` (×4) | `/robot_state_publisher` | ~10 Hz | Driven by Gazebo joint states |

> During **AMCL navigation** (`nav.launch`), `/slam_gmapping` is replaced by `/map_server`
> (static map) + `/amcl` (particle-filter localization). The `map → odom` edge is then
> maintained by `/amcl` at the laser scan rate (~10 Hz in simulation).

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

3. Manual test without a detector:
   ```bash
   rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
     "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

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
- [depthimage_to_laserscan Package](http://wiki.ros.org/depthimage_to_laserscan) (used on `main` branch)
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

- [x] Gazebo simulation (gmapping + move_base + target following) -- verified
- [x] Simulation laser self-collision bug fixed -- verified (commit `5d9e4d2`)
- [x] Obstacle detection (obstacle_layer, inflation_layer in costmap) -- verified
- [x] 3-waypoint sequential navigation test -- all SUCCEEDED (errors < 0.2 m)
- [x] Map saving and loading (map_server + AMCL) -- verified
- [x] `standoff_distance` feature -- implemented and verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] `face_target` feature -- implemented and verified (commit `7d87bbb`, 21/21 unit tests pass)
- [x] Unit test suite for standoff + face_target logic -- 21/21 pass (commit `8e189ba`)
- [x] Dynamic target following (`move_target` node) -- implemented and verified in Gazebo
- [x] Collision-free target model (ghost marker) -- target no longer flips the robot
- [x] Target-lost goal cancellation -- `target_follower` cancels `move_base` goal on stale target
- [x] Autonomous frontier exploration (`autonomous_explorer.py`) -- verified in `complex_maze.world`
- [x] `auto_mapping.launch` — one-command autonomous mapping pipeline -- verified
- [x] AMCL accuracy verifier (`amcl_verifier.py`) -- verified; mean pos error 0.089 m, convergence 1.0 s
- [x] `auto_amcl_verify.launch` — automated AMCL verification pipeline -- verified
- [x] Complex maze world (`complex_maze.world`, 12 × 12 m) -- created and verified
- [x] `run_full_pipeline.sh` — one-click two-phase mapping + verification script -- created
- [x] Real robot launch files (mapping + navigation) -- created, pending hardware test
- [x] Raspberry Pi base driver package (p3at_base) -- created, pending hardware test
- [x] Multi-machine network setup (Jetson + Pi, direct Ethernet, Docker, static IPs) -- documented
- [x] Multi-machine ROS communication verified (cross-machine topic test)
- [ ] YOLO target detection node (Orbbec Femto Bolt + YOLO) -- not started
- [ ] Real hardware parameter tuning -- not started
- [ ] End-to-end real robot navigation test -- not started

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
- [ ] Static target: `roslaunch p3at_lms_navigation mapping.launch` → drag "target" in Gazebo; robot follows
- [ ] Dynamic target: `roslaunch p3at_lms_navigation mapping.launch move_target:=true target_speed:=0.3 target_pause:=2.0`
- [ ] Dynamic + standoff + face: add `standoff_distance:=1.0 face_target:=true` to the above
- [ ] Verify 11 nodes running: `rosnode list` (includes `/move_target`)
- [ ] Unit tests pass: `python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py` (21/21)

### AMCL Navigation (requires a saved map)
- [ ] AMCL launches: `roslaunch p3at_lms_navigation nav.launch map_file:=<path_to_map.yaml>`
- [ ] Set initial pose with "2D Pose Estimate" in RViz
- [ ] Particle cloud converges around robot
- [ ] Navigation goal succeeds with AMCL localization

### Autonomous Mapping + AMCL Verification
- [ ] `complex_maze.world` loads without errors: `roslaunch p3at_lms_gazebo sim.launch world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world`
- [ ] Autonomous mapping runs: `roslaunch p3at_lms_navigation auto_mapping.launch gui:=true exploration_timeout:=300`
- [ ] Terminal prints coverage percentage updates and goal outcomes during mapping
- [ ] Map saved automatically on completion: check `maps/complex_maze_map.pgm` exists
- [ ] AMCL verification runs: `roslaunch p3at_lms_navigation auto_amcl_verify.launch gui:=true`
- [ ] Verifier prints convergence time and per-waypoint error to terminal
- [ ] Report generated: check `maps/amcl_report.txt` and `maps/amcl_report.json` exist
- [ ] Mean position error < 0.30 m: `grep "Mean position" maps/amcl_report.txt`
- [ ] Full pipeline script runs end-to-end: `bash run_full_pipeline.sh`

### Real Robot Deployment
#### Network & Docker
- [ ] Jetson host: static IP `192.168.50.1` on Ethernet — `ip -br addr show eth0`
- [ ] Pi host: static IP `192.168.50.2` on Ethernet — `ip -br addr show eth0`
- [ ] Bidirectional ping: Jetson ↔ Pi via `192.168.50.x`
- [ ] Jetson Docker container `ros_noetic` uses `--net=host` — `docker inspect -f '{{.HostConfig.NetworkMode}}' ros_noetic`
- [ ] Pi Docker container `noetic_pi` uses `--net=host` — `docker inspect -f '{{.HostConfig.NetworkMode}}' noetic_pi`
- [ ] `roscore` running on Jetson container — `ss -lntp | grep 11311`
- [ ] Pi container can reach master — `/dev/tcp/192.168.50.1/11311` test returns `PORT_OK`
- [ ] `rosnode list` from Pi returns at least `/rosout`
- [ ] Cross-machine topic test passes: publish from Jetson, echo on Pi

#### Hardware Devices
- [ ] LMS200 detected on Jetson: `ls /dev/ttyUSB0` (or appropriate device)
- [ ] LMS200 serial permissions: `sudo chmod 666 /dev/ttyUSB0` or dialout group
- [ ] P3-AT detected on Pi: `ls /dev/ttyS0` or `ls /dev/ttyUSB0`
- [ ] P3-AT serial permissions: user in `dialout` group or `chmod 666`

#### Base Driver (Pi)
- [ ] `roslaunch p3at_base base.launch` starts without errors
- [ ] `/odom` publishes: `rostopic hz /odom`
- [ ] TF `odom→base_link` is active: `rosrun tf tf_echo odom base_link`
- [ ] `/cmd_vel` is accepted: short teleop test with `teleop_twist_keyboard`

#### Mapping (Jetson)
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping.launch` starts without errors
- [ ] `/scan` publishes from LMS200: `rostopic hz /scan` (expect ~75 Hz)
- [ ] TF `base_link→laser` is active: `rosrun tf tf_echo base_link laser`
- [ ] Map builds correctly: `rostopic hz /map`
- [ ] Navigation goal (manual) succeeds
- [ ] Map saved: `rosrun map_server map_saver -f ...`

#### Navigation (Jetson)
- [ ] `roslaunch p3at_lms_navigation real_robot_nav.launch map_file:=...` starts without errors
- [ ] AMCL particle cloud converges after "2D Pose Estimate"
- [ ] Navigation goal succeeds with AMCL localization
