# ELEC70015 Human-Centered Robotics 2026 — Imperial College London

Pioneer 3-AT autonomous navigation + trash-detection demo system.  
ROS1 Noetic · Ubuntu 20.04 · Gazebo 11 (Simulation) / Docker (Real Robot).

> **`main` branch** — full documentation for both simulation development and real-robot deployment.  
> Unitree 4D Lidar L1 is the **primary sensor**; SICK LMS200 is the backup.  
> Real-robot deployment runs in Docker; simulation uses Gazebo.  
> Operational runbook: [`doc.md`](doc.md)

---

## Table of Contents

1. [System Overview](#system-overview)
   - [Sensor Stack Comparison](#sensor-stack-comparison)
   - [Simulation vs Real Robot — Key Differences](#simulation-vs-real-robot--key-differences)
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
   - [Simulation Node & Topic Reference](#simulation-node--topic-reference)
   - [Simulation TF Tree](#simulation-tf-tree)
7. [Simulation Verification Tests](#simulation-verification-tests)
8. [Part B — Real Robot Deployment](#part-b--real-robot-deployment)
   - [Docker Environment Setup](#docker-environment-setup)
   - [Network Architecture](#network-architecture)
   - [Startup Order](#startup-order)
   - [Keyboard Teleoperation + Unitree Mapping Runbook](#keyboard-teleoperation--unitree-mapping-runbook)
   - [Autonomous Exploration SLAM Runbook](#autonomous-exploration-slam-runbook)
   - [Target Following Demo Runbook](#target-following-demo-runbook)
   - [Option A — Unitree L1 (Primary)](#option-a--unitree-l1-primary)
   - [Option B — SICK LMS200 (Backup)](#option-b--sick-lms200-backup)
   - [Real Robot Node & Topic Reference](#real-robot-node--topic-reference)
   - [Real Robot TF Tree](#real-robot-tf-tree)
9. [Target Follower State Machine](#target-follower-state-machine)
10. [Trash Detection Bridge](#trash-detection-bridge)
11. [Dialogue Integration](#dialogue-integration)
12. [Parameter Tuning Guide](#parameter-tuning-guide)
13. [Autonomous Explorer Algorithm](#autonomous-explorer-algorithm)
14. [Known Issues and Notes](#known-issues-and-notes)
15. [Git Workflow](#git-workflow)
16. [Resources](#resources)
17. [Status](#status)
18. [Post-Installation Checklist](#post-installation-checklist)

---

## System Overview

This workspace implements a complete autonomous mobile robot system for the Pioneer 3-AT platform, featuring:

- **SLAM mapping** (gmapping) with manual or autonomous exploration
- **AMCL localisation** on pre-built maps
- **Target following** with standoff distance and face-target orientation
- **Trash detection** via YOLO + depth camera (real robot)
- **Dialogue interaction** with speech-to-text and NLU intent recognition (real robot)

### Sensor Stack Comparison

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

### Simulation vs Real Robot — Key Differences

| Aspect | Simulation (Gazebo) | Real Robot (Docker) |
|--------|---------------------|---------------------|
| **Environment** | Single-machine, Gazebo physics | Multi-machine: Jetson + Raspberry Pi |
| **LiDAR driver** | Gazebo plugin (`/unitree/scan`) | `unitree_lidar_ros` node + `pointcloud_to_laserscan` |
| **Odometry** | Gazebo skid-steer plugin | RosAria on Pi (`/odom`) |
| **ROS Master** | Local | Jetson Docker (`192.168.50.1:11311`) |
| **Target source** | Gazebo model position | UDP JSON from YOLO detection |
| **Dialogue** | Not available | Full pipeline: STT → NLU → TTS |
| **Navigation frame** | `map` (with gmapping) | `map` (mapping/nav) or `odom` (standalone target-following) |
| **Camera TF** | Not used | `base_link → camera_link` static TF for depth detection |
| **Network** | N/A | Gigabit Ethernet `192.168.50.0/24` |

### Workspace Layout

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── catkin_ws/     ← Primary ROS workspace (navigation, target follower, URDF)
├── ros_ws/        ← Secondary workspace (AMR driver config)
├── dialogue/      ← Dialogue system (STT + NLU + TTS, runs on host)
├── handobj_detection/  ← YOLO detection (runs on Jetson host)
├── trash_detection/    ← Alternative detection scripts
├── scripts/       ← Deployment scripts (start_demo.sh, etc.)
├── tools/         ← Utility scripts (camera relay, depth inspector)
└── doc.md         ← Real-robot operational runbook
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
| Range | 0.05 – 30 m |
| Scan frequency | ~10 Hz |
| ROS topics | `/unitree/scan` (LaserScan), `/unilidar/cloud` (PointCloud2), `/unilidar/imu` (Imu) |
| TF frame | `unitree_lidar` |
| Interface | USB Type-C (serial) |
| Driver | `unitree_lidar_ros` (compiled from `unilidar_sdk`) |

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

### Depth Camera: Orbbec Femto Bolt (Real Robot Only)

| Property | Value |
|----------|-------|
| RGB resolution | 1920×1080 @ 30 fps |
| Depth resolution | 640×576 @ 30 fps |
| Range | 0.25 – 5.5 m |
| ROS topic | N/A (runs on native host, not ROS) |
| TF frame | `camera_link` |
| Interface | USB 3.0 |
| Driver | Orbbec SDK (native Ubuntu 22.04) |

### Compute Platform

| Node | Hardware | Role |
|------|----------|------|
| Jetson | NVIDIA Jetson Orin Nano | ROS master, SLAM, navigation, Unitree driver, YOLO inference |
| Pi | Raspberry Pi 4 | P3-AT base driver (RosAria) |
| (Simulation) | Developer laptop/workstation | All nodes in one process |

---

## Repository & Package Structure

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── catkin_ws/src/
│   ├── CMakeLists.txt                    # Catkin top-level
│   ├── p3at_base/                        # Pi-side P3-AT base driver (real robot)
│   │   ├── launch/base.launch
│   │   └── scripts/odom_republisher.py
│   ├── p3at_lms_description/             # URDF/Xacro robot models
│   │   ├── urdf/p3at_lms.urdf.xacro           (SICK model)
│   │   ├── urdf/p3at_unitree.urdf.xacro       (Unitree model)
│   │   └── urdf/unitree_lidar_l1.urdf.xacro   (Unitree sensor macro)
│   ├── p3at_lms_gazebo/                  # Gazebo worlds and sim launch files
│   │   ├── launch/
│   │   │   ├── sim.launch                (SICK simulation)
│   │   │   └── sim_unitree.launch        (Unitree simulation)
│   │   └── worlds/
│   │       └── complex_maze.world        (12.2×12.2 m test maze)
│   ├── p3at_lms_navigation/              # Navigation stack (both sensors)
│   │   ├── launch/
│   │   │   ├── mapping.launch                    # SICK: manual mapping (sim)
│   │   │   ├── mapping_unitree.launch            # Unitree: manual mapping (sim)
│   │   │   ├── nav.launch                        # SICK: AMCL navigation (sim)
│   │   │   ├── nav_unitree.launch                # Unitree: AMCL navigation (sim)
│   │   │   ├── auto_mapping.launch               # SICK: autonomous exploration (sim)
│   │   │   ├── auto_mapping_unitree.launch       # Unitree: autonomous exploration (sim)
│   │   │   ├── auto_amcl_verify.launch           # SICK: AMCL verifier (sim)
│   │   │   ├── auto_amcl_verify_unitree.launch   # Unitree: AMCL verifier (sim)
│   │   │   ├── real_robot_mapping.launch         # SICK: real-robot mapping
│   │   │   ├── real_robot_mapping_unitree.launch # Unitree: real-robot mapping
│   │   │   ├── real_robot_nav.launch             # SICK: real-robot nav
│   │   │   └── real_robot_nav_unitree.launch     # Unitree: real-robot nav
│   │   ├── param/                        # SICK / default parameters
│   │   │   ├── gmapping.yaml
│   │   │   ├── costmap_common.yaml
│   │   │   ├── global_costmap.yaml
│   │   │   ├── local_costmap.yaml
│   │   │   ├── move_base.yaml
│   │   │   └── amcl.yaml
│   │   ├── param/unitree/                # Unitree-specific parameters (tuned)
│   │   │   ├── gmapping.yaml             # maxUrange: 10.0
│   │   │   ├── costmap_common.yaml       # inflation 0.3 / scale 5.0
│   │   │   ├── global_costmap.yaml       # 360° obstacle source (unitree_lidar frame)
│   │   │   ├── global_costmap_local_only.yaml  # Standalone target-following (odom frame, no /map)
│   │   │   ├── local_costmap.yaml        # inflation 0.3 / scale 5.0
│   │   │   ├── move_base.yaml            # clearing_rotation_allowed: false
│   │   │   └── amcl.yaml
│   │   ├── rviz/
│   │   │   ├── nav.rviz                  # SICK RViz config
│   │   │   └── nav_unitree.rviz          # Unitree RViz config
│   │   ├── scripts/
│   │   │   ├── autonomous_explorer.py    # Frontier exploration node
│   │   │   ├── amcl_verifier.py          # AMCL accuracy verifier
│   │   │   └── waypoint_test.py          # 3-waypoint navigation test
│   │   └── maps/                         # Saved maps (git-ignored)
│   ├── target_follower/                  # Target following package
│   │   ├── launch/
│   │   │   └── target_follow_real.launch # Real-robot target following
│   │   └── scripts/
│   │       ├── target_follower.py              # /target_pose → MoveBaseGoal (state machine)
│   │       ├── gazebo_target_publisher.py      # Get target from Gazebo model (sim only)
│   │       ├── goal_to_target_relay.py         # RViz goal → /target_pose relay (sim only)
│   │       ├── move_target.py                  # Move Gazebo target along waypoints (sim only)
│   │       ├── udp_target_bridge.py            # UDP JSON → /trash_detection/target_point (real only)
│   │       ├── point_to_target_pose.py         # PointStamped → PoseStamped (real only)
│   │       ├── navigation_success_udp_bridge.py # /target_follower/result → UDP (dialogue trigger)
│   │       ├── udp_trash_action_bridge.py      # UDP → /trash_action (dialogue result)
│   │       ├── scan_body_filter.py             # Filter robot body from LiDAR scan (real only)
│   │       ├── mock_target_point_publisher.py  # Test target publisher
│   │       └── test_standoff_face.py           # Unit tests (21 tests)
│   ├── sicktoolbox/                      # SICK C++ library (source)
│   └── sicktoolbox_wrapper/              # SICK ROS wrapper (source)
├── dialogue/                             # Dialogue system (real robot only)
│   ├── dialogue_udp_runner.py            # UDP trigger/result bridge + dialogue loop
│   ├── src/
│   │   ├── dialogue_manager.py
│   │   └── utils/
│   │       ├── speech_to_text.py
│   │       ├── text_to_speech.py
│   │       ├── nlu_intent.py
│   │       └── generate_prompt.py
│   ├── models/nlu_intent.bin             # Pre-trained NLU model
│   └── voice_data/                       # Pre-recorded audio prompts
├── handobj_detection/                    # YOLO detection (real robot, runs on host)
│   └── handobj_detection_rgbd.py
├── trash_detection/                      # Alternative detection scripts
│   ├── predict_15cls_rgbd.py
│   └── weights/
├── scripts/                              # Deployment scripts
│   ├── start_demo.sh                     # One-command demo launcher
│   ├── stop_demo_all.sh                  # Stop all demo processes
│   ├── start_base.sh                     # Start Pi base driver
│   ├── start_master.sh                   # Start roscore
│   ├── start_real_mapping_unitree.sh     # Start mapping stack
│   ├── start_teleop.sh                   # Keyboard teleoperation
│   ├── start_dialogue_host.sh            # Start dialogue on host
│   ├── start_dialogue_docker_bridges.sh  # Start UDP bridges in Docker
│   ├── test_dialogue_chain.sh            # Test dialogue pipeline
│   ├── demo_dashboard.sh                 # Runtime status dashboard
│   ├── deploy.env                        # IP and port configuration
│   └── env.sh                            # ROS environment setup
├── tools/                                # Utility scripts
│   ├── source_ros.sh / source_ros.zsh
│   ├── camera_info_pub.py
│   ├── inspect_depth_once.py
│   └── relay_camera_info.py
├── setup_unitree_lidar.sh                # Unitree SDK install helper
├── run_full_pipeline_unitree.sh          # Sim: Unitree mapping → AMCL verify
├── run_full_pipeline.sh                  # Sim: SICK mapping → AMCL verify
├── build_and_hint.sh
└── doc.md                                # Real-robot operational runbook
```

---

## Prerequisites & Dependencies

### System (Simulation — Local Machine)

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
  ros-noetic-teleop-twist-keyboard \
  python3-catkin-tools \
  python3-rospy \
  python3-numpy
```

### Jetson Docker (Real Robot)

> **Pre-installed in `ros_noetic:nav_unitree` image.** No manual installation needed.

The image is based on `ghcr.io/sloretz/ros:noetic-desktop-full` (arm64) and contains:
- All navigation packages (gmapping, move_base, amcl, etc.)
- `unitree_lidar_ros` pre-compiled
- `pointcloud_to_laserscan`
- Build tools (cmake, git, build-essential)

#### Docker Image Info

| Image | Tag | Arch | Contents |
|-------|-----|------|----------|
| `ros_noetic` | `nav_unitree` **← use this** | arm64 | ROS Noetic + nav + Unitree driver |
| `ros_noetic` | `nav` | arm64 | ROS Noetic + nav (no Unitree driver) |

### Unitree L1 Driver Setup (Reference Only)

> **Already compiled in `ros_noetic:nav_unitree`.** These steps are for rebuilding from scratch.

```bash
# Inside Docker container
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src
git clone https://github.com/unitreerobotics/unilidar_sdk.git

# Prevent non-catkin subdirectories from breaking catkin_make
touch unilidar_sdk/unitree_lidar_ros2/CATKIN_IGNORE
touch unilidar_sdk/unitree_lidar_sdk/CATKIN_IGNORE

source /opt/ros/noetic/setup.bash
cd .. && catkin_make
```

Or use the helper script:

```bash
./setup_unitree_lidar.sh
```

### Unitree L1 USB — Udev Rule (Jetson Host)

> **Already created on Jetson host.**

File: `/etc/udev/rules.d/99-unitree-lidar.rules`
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="unitree_lidar"
```

### SICK LMS200 Driver (Backup)

```bash
# Already in catkin_ws/src/ as sicktoolbox + sicktoolbox_wrapper
cd catkin_ws && catkin_make
```

### Pi Docker (Real Robot)

```bash
sudo apt-get install -y ros-noetic-rosaria
# Or build from source: https://github.com/amor-ros-pkg/rosaria
```

### Jetson Native Ubuntu 22.04 (YOLO Detection + Dialogue)

```bash
# YOLO detection
pip3 install ultralytics opencv-python numpy scipy pillow

# Dialogue system
cd dialogue
pip install -r requirements.txt
sudo apt-get install -y libportaudio2 portaudio19-dev  # For microphone
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

### Docker Environment Setup

All ROS nodes run inside a Docker container on Jetson. The image `ros_noetic:nav_unitree` contains ROS Noetic + all navigation packages + Unitree driver pre-compiled.

#### Container Management

Use the `ros_noetic` management script (located at `~/.fishros/bin/ros_noetic`):

| Action | Command | Docker equivalent |
|--------|---------|-------------------|
| **Start / create** | `ros_noetic s` | `docker run ...` |
| **Enter (non-root)** | `ros_noetic e` | `docker exec -it --user $(id -u):$(id -g) ros_noetic bash` |
| **Restart** | `ros_noetic r` | `docker restart ros_noetic` |
| **Stop** | `ros_noetic c` | `docker stop ros_noetic` |
| **Delete** | `ros_noetic d` | `docker stop ros_noetic && docker rm ros_noetic` |
| Save changes | — | `docker commit ros_noetic ros_noetic:nav_unitree` |

> **⚠️ Never use** `docker exec -it ros_noetic bash` without `--user` — that enters as **root** and will cause file permission issues.

#### Inside Container — Source & Build

```bash
# Source ROS
source /opt/ros/noetic/setup.bash

# Go to workspace and build (first time only)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws
catkin_make
source devel/setup.bash

# Verify
rospack find p3at_lms_navigation   # should print the package path
```

> **Every new shell** inside the container needs both `source` commands.

---

### Network Architecture

#### Physical Topology

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│   JETSON ORIN NANO  (192.168.50.1)                                              │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐            │
│  │  ROS Noetic Docker  (--net=host)  ROS_MASTER_URI=:11311         │            │
│  │                                                                 │            │
│  │  roscore                                                        │            │
│  │  robot_state_publisher  ──►  /tf_static                         │            │
│  │  unitree_lidar_ros      ──►  /unilidar/cloud, /unitree/scan     │            │
│  │  pointcloud_to_laserscan  /unilidar/cloud → /unitree/scan       │            │
│  │  slam_gmapping*         ──►  /map, /tf(map→odom)                │  *mapping  │
│  │  amcl*                  ──►  /tf(map→odom)                      │  *nav      │
│  │  move_base              ──►  /cmd_vel                           │            │
│  │  udp_target_bridge      ──►  /trash_detection/target_point      │            │
│  │  point_to_target_pose   ──►  /target_pose                       │            │
│  │  target_follower        ──►  MoveBaseGoal, /target_follower/*   │            │
│  │  navigation_success_udp_bridge  /target_follower/result → UDP   │            │
│  │  udp_trash_action_bridge        UDP → /trash_action             │            │
│  └─────────────────────────────────────────────────────────────────┘            │
│        ▲  UDP JSON  127.0.0.1:16031 (detection)                                 │
│        ▲  UDP JSON  127.0.0.1:16041 / 16032 (dialogue)                          │
│  ┌─────┴───────────────────────────────────────────────────────────┐            │
│  │  Host — Native Ubuntu 22.04                                     │            │
│  │  handobj_detection_rgbd.py  (YOLO + depth camera, GPU)          │            │
│  │  dialogue_udp_runner.py  (audio + NLU)                          │            │
│  │  Orbbec Femto Bolt SDK                                          │            │
│  └─────────────────────────────────────────────────────────────────┘            │
└────────────────────────────┬────────────────────────────────────────────────────┘
                             │  Gigabit Ethernet  192.168.50.0/24
                             │  /cmd_vel → Pi  |  /odom, /tf ← Pi
┌────────────────────────────┴────────────────────────────────────────────────────┐
│   RASPBERRY PI 4  (192.168.50.2)                                                │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────┐            │
│  │  ROS Noetic Docker  (--net=host)  ROS_MASTER_URI=:11311         │            │
│  │  rosaria  ◄── /cmd_vel                                          │            │
│  │           ──► /odom, /tf(odom→base_link)                        │            │
│  └─────────────────────────────────────────────────────────────────┘            │
│   Serial USB → P3-AT chassis controller                                         │
└─────────────────────────────────────────────────────────────────────────────────┘
```

#### Key Topic Flow (Target Following Demo)

```
[Host] handobj_detection_rgbd.py  ──UDP 16031──►  udp_target_bridge
  → /trash_detection/target_point  (frame: camera_link)
  → point_to_target_pose  → /target_pose  (frame: camera_link)
  → target_follower  → /move_base (action)  → /cmd_vel
  → rosaria (Pi)  → P3-AT chassis

[Docker] /target_follower/result
  → navigation_success_udp_bridge  ──UDP 16041──► [Host] dialogue_udp_runner.py
  → speech + intent (yes/no)
  → dialogue_udp_runner.py ──UDP 16032──► udp_trash_action_bridge
  → /trash_action (Bool)
```

#### ROS Environment Variables

**Jetson Docker:**

```bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
```

**Pi Docker:**

```bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.2
```

Configure IPs in `scripts/deploy.env`:
```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
TRASH_UDP_PORT=16031
```

> Both Docker containers must use `--net=host`.

---

### Startup Order

**Option A — Target Following Demo (standalone, no map needed):**
```
1. Pi:              ./scripts/start_base.sh
2. Jetson Docker:   roscore
3. Jetson Docker:   roslaunch target_follower target_follow_real.launch launch_move_base:=true
4. Jetson Host:     python3 handobj_detection/handobj_detection_rgbd.py --udp-enable
   (or one-command: ./scripts/start_demo.sh)
```

**Option B — Mapping:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      ./scripts/start_base.sh
3. Jetson Docker:  roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch
4. Jetson Docker:  rosrun teleop_twist_keyboard teleop_twist_keyboard.py (or autonomous_explorer.py)
```

**Option C — Navigation on saved map:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      ./scripts/start_base.sh
3. Jetson Docker:  roslaunch p3at_lms_navigation real_robot_nav_unitree.launch map_file:=...
4. Jetson Docker:  roslaunch target_follower target_follow_real.launch launch_move_base:=false
5. Jetson Host:    python3 handobj_detection/handobj_detection_rgbd.py --udp-enable
```

---

### Keyboard Teleoperation + Unitree Mapping Runbook

> **Verified real-robot procedure** (tested 2026-02-24).

#### Hardware Checklist

| Item | Check |
|------|-------|
| Jetson Orin Nano powered on | ✓ |
| P3-AT powered on, serial cable connected to Pi (`/dev/ttyS0` or `/dev/ttyUSB0`) | ✓ |
| Pi connected to Jetson via Ethernet (`192.168.50.0/24`) | ✓ |
| Unitree L1 LiDAR connected to Jetson USB-C, **DO NOT power on yet** | ✓ |
| `docker ps` shows `ros_noetic` container running | ✓ |

> **⚠️ Important:** Power on the Unitree LiDAR **only after** `roslaunch` has already started (Step 3).

#### Step 1 — Raspberry Pi: Start Chassis Driver

```bash
# On Pi (ssh frank@192.168.50.2)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

#### Step 2 — Jetson: Ensure roscore is Running

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
# If NOT running:
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
exec roscore"
```

#### Step 3 — Jetson: Start Mapping Stack

```bash
./scripts/start_real_mapping_unitree.sh use_rviz:=false
```

Or manually:
```bash
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
exec roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch use_rviz:=false \
  > /tmp/mapping_unitree.log 2>&1"
```

#### Step 4 — Power on Unitree LiDAR

Plug in / switch on the Unitree L1. Wait **10–15 seconds**.

#### Step 5 — Keyboard Teleoperation

```bash
./scripts/start_teleop.sh jetson
```

Or manually:
```bash
docker exec -it --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel"
```

**Key bindings:** `i`=forward, `,`=backward, `j`=rotate left, `l`=rotate right, `k`=stop

#### Step 6 — Save the Map

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosrun map_server map_saver -f \
  /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/my_map_unitree"
```

---

### Autonomous Exploration SLAM Runbook

> **Verified real-robot procedure** for fully autonomous frontier-based SLAM mapping.

#### Tuned Parameters (Anti-Collision, Real Robot)

| Parameter | File | Value | Reason |
|-----------|------|-------|--------|
| `inflation_radius` | `costmap_common.yaml` | `0.3 m` | Reduced from 0.55 (caused spin-in-place near walls) |
| `cost_scaling_factor` | `costmap_common.yaml` | `5.0` | Faster cost decay |
| `range_min` | `pointcloud_to_laserscan` | `0.35 m` | Filters robot body from LiDAR scan |
| `max_vel_x` | `move_base.yaml` | `0.22 m/s` | Reduced speed for safety |
| `sim_time` | `move_base.yaml` | `2.5 s` | Longer DWA lookahead |
| `occdist_scale` | `move_base.yaml` | `0.08` | 4× higher obstacle weight |

#### Steps

1. **Start Pi base driver** (same as Step 1 above)
2. **Start roscore** (same as Step 2 above)
3. **Start mapping stack** (same as Step 3 above)
4. **Launch autonomous explorer:**

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
rosrun p3at_lms_navigation autonomous_explorer.py \
  _exploration_timeout:=600 \
  _min_frontier_size:=3 \
  _initial_wait:=12.0 \
  _save_map:=true \
  _map_save_path:=/home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/explored_map_unitree"
```

---

### Target Following Demo Runbook

> **Primary demo task.** Robot follows a detected target using YOLO + depth-camera detection.  
> **No pre-built map required** in standalone mode.

#### Workflow Update (2026-03-14)

- Startup behavior changed from passive waiting to **active search**: the robot now starts in auto-explore when no fresh target is available.
- Post-dialogue behavior is now unified: **both** `/trash_action=True` and `/trash_action=False` trigger the same retreat policy.
- Unified retreat policy:
  1. Record current dialogue location and traveled path.
  2. Rotate in place by a large angle (`retreat_turn_angle_deg`, default 180°).
  3. Drive away forward (`retreat_distance`, default 1.5 m).
  4. Resume auto-explore to find the next person/object.
- Anti-repeat policy:
  - Exploration avoids recently visited areas for `explore_revisit_window_s` (default 120 s).
  - Targets near recent dialogue locations are temporarily ignored (`target_reacquire_block_s`, default 120 s).

#### Quick Start — One Command

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_demo.sh
```

This starts the demo in this order:

1. `roscore`
2. `handobj_detection_rgbd.py`
3. `dialogue_udp_runner.py` + Docker dialogue bridges
4. `target_follow_real.launch`

The navigation stack is started last on purpose, so YOLO and dialogue are already healthy before the robot begins searching or triggering dialogue.

#### Partial Startup / Repair

When a single module dies during testing, you do not need to stop the whole demo.

Bring up only selected modules:

```bash
./scripts/start_demo.sh --only master
./scripts/start_demo.sh --only master,dialogue
./scripts/start_demo.sh --only master,yolo
./scripts/start_demo.sh --only master,nav
```

Restart one or more running modules using the current demo runtime parameters:

```bash
./scripts/restart_demo_modules.sh --only yolo
./scripts/restart_demo_modules.sh --only dialogue
./scripts/restart_demo_modules.sh --only nav
./scripts/restart_demo_modules.sh --only yolo,dialogue
```

Notes:

- `restart_demo_modules.sh` reads the runtime state written by `start_demo.sh`, so start the demo once before using it.
- This is intended for field repair when YOLO or dialogue occasionally fails to start cleanly.

#### Camera → Robot Coordinate Transform

Static TF `base_link → camera_link`: xyz=`(0.208, 0, 1.0)`, quat=`(-0.5, 0.5, -0.5, 0.5)` = RPY `[-90°, 0°, -90°]`

| Camera optical | → | Robot base_link |
|---|---|---|
| `+z` (depth / forward) | → | `+x` (forward) |
| `+x` (right) | → | `-y` (right) |
| `+y` (down) | → | `-z` (down) |

#### target_follow_real.launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `launch_move_base` | `true` | `true` = standalone (odom frame); `false` = overlay on existing nav |
| `standoff_distance` | `0.6` m | Stop distance from target |
| `face_target` | `true` | Orient robot toward target at REACHED |
| `target_timeout` | `5.0` s | Cancel goal if no detection for this long |
| `udp_port` | `16031` | Must match detection `--udp-port` |
| `retreat_distance` | `1.5` m | Forward retreat distance after dialogue |
| `retreat_turn_angle_deg` | `180.0` deg | In-place turn angle before retreat (large angle to avoid re-detection) |
| `action_wait_timeout` | `45.0` s | Timeout waiting for `/trash_action` |
| `enable_auto_explore` | `true` | Enable active exploration when no target is tracked |
| `explore_goal_distance` | `2.0` m | Step distance for each explore navigation goal |
| `explore_goal_timeout` | `30.0` s | Timeout for one explore goal |
| `explore_revisit_window` | `120.0` s | Time window for no-repeat exploration |
| `explore_revisit_radius` | `1.5` m | Spatial radius for no-repeat exploration |
| `target_reacquire_block_s` | `120.0` s | Block re-querying targets near recent dialogue points |
| `target_reacquire_radius` | `1.6` m | Radius for target reacquire suppression |

#### start_demo.sh Common Options (New)

```bash
./scripts/start_demo.sh \
  --retreat-turn-deg 180 \
  --explore-step 2.0 \
  --explore-no-repeat-sec 120
```

- `--retreat-turn-deg`: large in-place turn before leaving.
- `--explore-step`: exploration step distance.
- `--explore-no-repeat-sec`: region no-repeat time window.
- `--no-explore`: disable active exploration (debug only).

#### Stop Demo Cleanly

Use the stop script instead of manually killing a few processes:

```bash
./scripts/stop_demo_all.sh
```

Useful scoped stop modes:

```bash
./scripts/stop_demo_all.sh --dialogue
./scripts/stop_demo_all.sh --master
```

Current behavior:

- Stops host-side YOLO, dialogue runner, bridges, and dashboard processes.
- Stops Docker-side ROS demo processes without stopping the Docker container itself.
- Verifies that key demo processes are actually gone, instead of always printing success.

#### Manual Test (No Hardware)

```bash
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(30):
    s.sendto(json.dumps({'x': 0.0, 'y': 0.0, 'z': 2.0,
                         'frame_id': 'camera_link',
                         'stamp': time.time()}).encode(),
             ('127.0.0.1', 16031))
    time.sleep(0.2)"
```

---

### SICK LMS200 Stack (Backup)

> **Use only if Unitree hardware is unavailable.**

| Attribute | SICK | Unitree |
|-----------|------|---------|
| Topic | `/scan` | `/unitree/scan` |
| FOV | 180° | 360° |
| Frame ID | `laser` | `unitree_lidar` |

**Mapping:**
```bash
roslaunch p3at_lms_navigation real_robot_mapping.launch
```

**Navigation:**
```bash
roslaunch p3at_lms_navigation real_robot_nav.launch \
  map_file:=$HOME/maps/real_robot_map_sick.yaml
```

---

### Real Robot Parameter Tuning (Verified)

**costmap_common.yaml (Unitree):**

| Parameter | Sim | Real | Reason |
|-----------|-----|------|--------|
| `inflation_radius` | `0.55` | `0.30` | Reduced: robot was spinning in-place near walls |
| `cost_scaling_factor` | `5.0` | `5.0` | Unified |
| `footprint` | `0.28` radius | `0.28` radius | No change |

**move_base.yaml (Real Robot):**

| Parameter | Value | Notes |
|-----------|-------|-------|
| `max_vel_x` | `0.22` | Slower than sim (`0.4`) for safety |
| `min_vel_x` | `0.08` | - |
| `max_vel_theta` | `0.35` | Reduced rotation speed |
| `acc_lim_x` | `1.0` | - |
| `acc_lim_theta` | `1.5` | - |
| `sim_time` | `2.5` | Longer DWA lookahead |
| `occdist_scale` | `0.08` | 4× higher obstacle weighting |
| `xy_goal_tolerance` | `0.20` | Tighter for target following |
| `clearing_rotation_allowed` | `false` | Disable recovery in tight spaces |

**global_costmap_local_only.yaml (Target Following Standalone):**

For target following without a pre-built map, uses rolling window costmap:

```yaml
global_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  width: 10.0
  height: 10.0
  resolution: 0.05
  rolling_window: true
  static_map: false
```

---

## Target Follower State Machine

### State Diagram

```
                         ┌───────────────────────────────────────────────────────────┐
                         │                    EXPLORING                              │
                         │  No fresh target: send short move_base goals to search    │
                         │  Avoid areas visited in last 120s                          │
                         └───────────────────────┬───────────────────────────────────┘
                                                 │  fresh target detected
                                                 ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                      IDLE                                 │
                         │  Wait for /trash_detection/target_point                   │
                         └───────────────────────┬───────────────────────────────────┘
                                                 │  valid detection
                                                 ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                    TRACKING                               │
                         │  Send MoveBase goal (standoff_distance from target)       │
                         │  Continuously update goal as target moves                 │
                         └───────────────────────┬───────────────────────────────────┘
                                                 │  within standoff_distance
                                                 ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                 CLOSE_APPROACH                            │
                         │  Cancel MoveBase; drive forward slowly to final pose     │
                         │  (cmd_vel direct publish, bypass local planner)           │
                         └───────────────────────┬───────────────────────────────────┘
                                                 │  within close_enough_distance (0.3 m)
                                                 ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                    REACHED                                │
                         │  Stop robot; optionally face target; publish success     │
                         │  /target_follower/result (Bool: True)                     │
                         │  Trigger dialogue (UDP 16041 → dialogue system)           │
                         └───────────────────────┬───────────────────────────────────┘
                                                 │  wait for /trash_action
                                                 ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                 WAITING_ACTION                            │
                         │  Wait up to action_wait_timeout (45 s) for Bool message   │
                         │  on /trash_action (from dialogue UDP 16032)               │
                         └──────────────┬───────────────────────────┬────────────────┘
                          True (accept) │                           │ False (decline)
                                        ▼                           ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                    RETREATING                             │
                         │  Unified policy for both outcomes:                        │
                         │  1) record dialogue point/path                            │
                         │  2) in-place large-angle turn                             │
                         │  3) drive forward retreat_distance                         │
                         │  4) return to IDLE then resume EXPLORING                  │
                         └───────────────────────────────────────────────────────────┘
```

### ROS Interface

| Topic/Action | Type | Direction | Description |
|--------------|------|-----------|-------------|
| `/trash_detection/target_point` | `geometry_msgs/PointStamped` | ← | 3D target location from YOLO |
| `/target_pose` | `geometry_msgs/PoseStamped` | internal | Transformed pose for MoveBase |
| `/move_base` | `MoveBaseAction` | → | Navigation goal |
| `/cmd_vel` | `geometry_msgs/Twist` | → | Direct drive (CLOSE_APPROACH) |
| `/target_follower/status` | `std_msgs/String` | → | Current state: `IDLE|EXPLORING|TRACKING|...` |
| `/target_follower/result` | `std_msgs/Bool` | → | `True` = reached target, `False` = reset/failed/lost |
| `/trash_action` | `std_msgs/Bool` | ← | Human response (`true/false`, both trigger unified retreat) |
| `/target_follower/path_history` | `nav_msgs/Path` | → | Recorded recent robot path (for no-repeat exploration) |
| `/target_follower/dialogue_points` | `nav_msgs/Path` | → | Recorded dialogue positions (for anti-repeat filtering) |

### Key Parameters

```yaml
standoff_distance: 0.6       # Stop this far from target (MoveBase goal)
close_enough_distance: 0.3   # Switch to CLOSE_APPROACH when closer
approach_speed: 0.1          # m/s during CLOSE_APPROACH
face_target: true            # Rotate to face target at REACHED
target_timeout: 5.0          # Cancel goal if no detection (seconds)
action_wait_timeout: 45.0    # Max wait for dialogue result
retreat_distance: 1.5        # Forward retreat distance after dialogue
retreat_turn_angle_deg: 180  # In-place turn angle before retreat
enable_auto_explore: true    # Explore while no fresh target
explore_revisit_window_s: 120
explore_revisit_radius: 1.5
target_reacquire_block_s: 120
target_reacquire_radius: 1.6
```

---

## Trash Detection Bridge

The trash detection system runs on the Jetson **native host** (Ubuntu 22.04) and communicates to ROS via UDP.

### Detection Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│  handobj_detection_rgbd.py (Native Host)                             │
│                                                                      │
│  Orbbec SDK → RGB + Depth @ 15 fps                                  │
│  YOLOv8 inference → Bounding boxes                                  │
│  Depth lookup → 3D centroid (camera frame)                          │
│  UDP JSON at 5 Hz → 127.0.0.1:16031                                 │
└────────────────────────────┬─────────────────────────────────────────┘
                             │  JSON: {"x":0.5, "y":0.2, "z":2.3,
                             │         "frame_id":"camera_link",
                             │         "stamp":1708700000.123,
                             │         "class":"bottle"}
                             ▼
┌──────────────────────────────────────────────────────────────────────┐
│  udp_target_bridge.py (Docker)                                       │
│                                                                      │
│  UDP socket 16031 → /trash_detection/target_point (PointStamped)     │
└──────────────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌──────────────────────────────────────────────────────────────────────┐
│  point_to_target_pose.py (Docker)                                    │
│                                                                      │
│  PointStamped → tf2 transform → PoseStamped in base_link frame       │
│  pub /target_pose                                                    │
└──────────────────────────────────────────────────────────────────────┘
```

### Run Detection Standalone

```bash
# Native Jetson host (not Docker)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
source trash_detection/venv/bin/activate  # if using venv
python3 handobj_detection/handobj_detection_rgbd.py \
  --weights handobj_detection/weights/handobj.pt \
  --udp-enable \
  --udp-ip 127.0.0.1 \
  --udp-port 16031
```

---

## Dialogue Integration

The dialogue system provides voice-based human interaction when the robot reaches a target.

### Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│  [Docker] /target_follower/result = "REACHED"                         │
└───────────────────────────────┬───────────────────────────────────────┘
                                │  UDP 16041
                                ▼
┌───────────────────────────────────────────────────────────────────────┐
│  [Host] dialogue_udp_runner.py                                        │
│                                                                       │
│  1. Trigger audio recording (microphone)                              │
│  2. STT: Whisper-small                                                │
│  3. NLU: intent classification (yes / no / unclear)                   │
│  4. TTS: response audio                                               │
│  5. Send result UDP 16032                                             │
└───────────────────────────────┬───────────────────────────────────────┘
                                │  JSON: {"action": true/false}
                                ▼
┌───────────────────────────────────────────────────────────────────────┐
│  [Docker] udp_trash_action_bridge.py                                  │
│                                                                       │
│  UDP → /trash_action (std_msgs/Bool)                                  │
└───────────────────────────────────────────────────────────────────────┘
```

### Run Dialogue System

```bash
# Native Jetson host
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/dialogue
python3 dialogue_udp_runner.py \
  --trigger-port 16041 \
  --result-port 16032 \
  --stt-model whisper-small \
  --language en
```

### UDP Protocol

**Trigger (ROS→Host) — Port 16041:**
```json
{"trigger": "reached", "timestamp": 1708700000.123}
```

**Result (Host→ROS) — Port 16032:**
```json
{"action": true, "confidence": 0.92, "intent": "yes"}
```

---

## Demo Scripts Reference

All scripts are located in `scripts/`:

| Script | Purpose |
|--------|---------|
| `start_demo.sh` | One-command start: roscore + target_follow_real.launch + detection |
| `stop_demo_all.sh` | Kill all demo processes |
| `start_base.sh` | Start rosaria on Pi |
| `start_real_mapping_unitree.sh` | Start mapping with Unitree LiDAR |
| `start_teleop.sh` | Keyboard teleoperation |
| `test_dialogue_chain.sh` | Test full detection → dialogue → action pipeline |
| `demo_dashboard.sh` | tmux dashboard with all logs |
| `deploy.env.example` | Template for IP/port configuration |

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
| `/target_pose` | `geometry_msgs/PoseStamped` | `/gazebo_target_publisher` (sim) / `camera_json_bridge` (real robot) / goal relay | `/target_follower` |
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

#### Real Robot — Unitree + Orbbec Camera

```
odom
+-- base_link                     [rosaria  ~50 Hz]
    +-- unitree_lidar             [/robot_state_publisher  static]
    +-- camera_link               [static_transform_publisher  static]
    |   ├── xyz: (0.208, 0, 1.0)
    |   └── rpy: [-90°, 0°, -90°]  (camera optical axis → robot forward)
    +-- top_plate                 [static]

(with gmapping/amcl)
map
+-- odom                          [/slam_gmapping or /amcl  ~20 Hz]
    +-- base_link                 [rosaria  ~50 Hz]
        +-- ...
```

> **Note:** On real robot, `base_footprint` may be omitted; `rosaria` publishes `odom → base_link` directly.

#### TF Edge Summary

| TF Edge | Broadcaster | Rate | Notes |
|---------|-------------|------|-------|
| `map -> odom` | `/slam_gmapping` | ~20 Hz | Replaced by `/amcl` during navigation |
| `odom -> base_footprint` | `/gazebo` (sim) | ~50 Hz | — |
| `odom -> base_link` | `rosaria` (real robot) | ~50 Hz | Real robot skips `base_footprint` |
| `base_footprint -> base_link` | `/robot_state_publisher` | static | Identity from URDF (sim only) |
| `base_link -> unitree_lidar` | `/robot_state_publisher` | static | Unitree mount offset |
| `base_link -> camera_link` | `static_transform_publisher` | static | Orbbec camera mount (real robot) |
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

## YOLO Target Detection (Native Ubuntu 22.04 → Docker Bridge)

> **Architecture**: The entire YOLO detection pipeline runs **outside Docker** on Jetson's native Ubuntu 22.04, where the Orbbec Femto Bolt kernel driver is available. The 3D navigation target coordinates are sent to the ROS Docker container via a **JSON bridge over localhost**.

### System Flow

```
[Jetson — Native Ubuntu 22.04]
  Orbbec Femto Bolt SDK
      └─► RGB frame + aligned depth frame
  YOLO inference (e.g. ultralytics YOLOv8)
      └─► bounding box → pick centre pixel
  Depth lookup: depth_image[cy, cx] → Z metres
  Back-project to 3D (camera intrinsics K):
      X = (cx - K.ppx) * Z / K.fx
      Y = (cy - K.ppy) * Z / K.fy
  Transform to map frame (camera extrinsics + /tf)
  Publish JSON to localhost TCP socket (default port 9097):
      {"x": 1.23, "y": -0.45, "z": 0.0,
       "frame_id": "map",
       "stamp": 1708700000.123}

[Jetson — ROS Noetic Docker  (--net=host)]
  tools/camera_json_bridge.py
      └─► reads JSON stream from TCP 127.0.0.1:9097
      └─► publishes geometry_msgs/PoseStamped → /target_pose
  target_follower
      └─► sub /target_pose → send MoveBaseAction goal
```

### Running the Bridge Inside Docker

```bash
# Start the JSON bridge node (inside Jetson Docker)
rosrun p3at_lms_navigation camera_json_bridge.py \
  _port:=9097 \
  _frame_id:=map
```

Or launch it together with navigation:

```bash
roslaunch p3at_lms_navigation real_robot_nav_unitree.launch \
  use_target_follower:=true \
  use_json_bridge:=true
```

### Running YOLO Detection on Native Ubuntu 22.04

```bash
# Outside Docker, on Jetson native Ubuntu 22.04
python3 tools/yolo_target_detector.py \
  --model yolov8n.pt \
  --target-class person \
  --port 9097
```

The script:
1. Opens the Orbbec Femto Bolt colour + depth streams via the native SDK
2. Runs YOLO inference on each RGB frame
3. For detected targets, reads aligned depth at the bounding-box centre
4. Back-projects to 3D in the camera frame, then transforms to `map` frame
5. Serialises to JSON and sends over TCP to Docker on `127.0.0.1:9097`

### JSON Message Format

```json
{
  "x": 1.23,
  "y": -0.45,
  "z": 0.00,
  "frame_id": "map",
  "stamp": 1708700000.123,
  "confidence": 0.91,
  "label": "person"
}
```

### Manual Testing Without YOLO

Publish a fake target directly inside Docker to verify `target_follower` end-to-end:

```bash
rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Why Native Ubuntu 22.04 (not Docker)?

| Reason | Detail |
|--------|--------|
| Kernel USB driver | Orbbec SDK requires custom kernel modules; not available inside Docker |
| libusb access | USB 3.0 device-level access; `--privileged` + volume mounts are fragile |
| CUDA / TensorRT | YOLO GPU inference benefits from native CUDA without container overhead |
| Simplicity | JSON socket is a clean, version-agnostic interface between the two environments |

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

### Real Robot (Verified)
- [x] Real robot launch files — all 4 variants (mapping + nav, both stacks) — created
- [x] Raspberry Pi base driver package (`p3at_base`) — created and verified
- [x] Multi-machine network setup documented and verified (cross-machine topic test)
- [x] Docker image `ros_noetic:nav_unitree` — built and deployed
- [x] Unitree L1 driver integration — verified (`/unitree/scan` publishing)
- [x] Keyboard teleoperation + mapping Runbook — verified (2026-02-24)
- [x] Autonomous exploration SLAM — verified
- [x] Target following with YOLO + depth camera — verified
- [x] UDP bridges (detection 16031, dialogue 16041/16032) — verified
- [x] Target follower CLOSE_APPROACH state — verified
- [x] Dialogue integration (STT→NLU→TTS→action) — verified
- [x] Parameter tuning (`inflation_radius: 0.3`, `max_vel_x: 0.22`) — verified
- [x] `scan_body_filter.py` — robot body filtering verified
- [x] `target_follow_real.launch` — standalone mode verified
- [x] Demo scripts (`start_demo.sh`, `stop_demo_all.sh`) — verified

### Pending
- [ ] Bin motor driver integration — not started
- [ ] Full trash collection demo with bin motor — not started
- [ ] Hand-object detection combined mode — in progress
- [ ] Multi-target sequential pickup — not started

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

### Real Robot — Docker Setup
- [ ] Docker container running: `docker ps | grep ros_noetic`
- [ ] Enter container: `ros_noetic e` (or `~/.fishros/bin/ros_noetic e`)
- [ ] Source ROS: `source /opt/ros/noetic/setup.bash`
- [ ] Source workspace: `source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash`

### Real Robot — Network
- [ ] Jetson: `ip -br addr show eth0` shows `192.168.50.1`
- [ ] Pi: `ip -br addr show eth0` shows `192.168.50.2`
- [ ] Ping both directions succeed
- [ ] Jetson Docker `--net=host`: `docker inspect -f '{{.HostConfig.NetworkMode}}' ros_noetic`
- [ ] Pi Docker `--net=host`: `docker inspect -f '{{.HostConfig.NetworkMode}}' noetic_pi`
- [ ] `roscore` on Jetson: `ss -lntp | grep 11311`
- [ ] `rosnode list` from Pi returns at least `/rosout`
- [ ] Cross-machine topic test passes

### Real Robot — Keyboard Teleoperation + Mapping
- [ ] Pi base driver: `./scripts/start_base.sh`
- [ ] `rostopic hz /odom` — publishing
- [ ] Jetson roscore running
- [ ] Mapping launch: `./scripts/start_real_mapping_unitree.sh use_rviz:=false`
- [ ] Power on Unitree LiDAR (wait 10-15 s)
- [ ] `rostopic hz /unitree/scan` — publishing
- [ ] Teleop: `./scripts/start_teleop.sh jetson` — robot moves
- [ ] Map saved: `rosrun map_server map_saver -f ~/maps/unitree_map`

### Real Robot — Target Following Demo
- [ ] Quick start: `./scripts/start_demo.sh`
- [ ] Or manual:
  - [ ] roscore + `target_follow_real.launch launch_move_base:=true`
  - [ ] `handobj_detection_rgbd.py --udp-enable`
- [ ] Hold object in front of camera → robot follows
- [ ] Robot stops at standoff distance
- [ ] `/target_follower/result` publishes `True`
- [ ] After dialogue, robot performs large-angle turn + forward retreat
- [ ] Robot resumes auto-explore instead of waiting in place

### Real Robot — Dialogue Integration
- [ ] Dialogue system: `python3 dialogue/dialogue_udp_runner.py`
- [ ] Robot says prompt after reaching target
- [ ] Voice "yes" → `/trash_action` publishes True → unified retreat policy triggered
- [ ] Voice "no" → `/trash_action` publishes False → same unified retreat policy triggered

### Real Robot — SICK (Backup) — Only If Unitree Unavailable
- [ ] LMS200 detected: `ls /dev/ttyUSB0`
- [ ] Permissions: `sudo chmod 666 /dev/ttyUSB0`
- [ ] P3-AT base driver active
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping.launch` — no errors
- [ ] `rostopic hz /scan` — ~75 Hz from LMS200
- [ ] Map saved
- [ ] AMCL navigation goal SUCCEEDED
