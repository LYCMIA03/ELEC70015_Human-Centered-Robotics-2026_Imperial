# ELEC70015 Human-Centered Robotics 2026 — Imperial College London

Pioneer 3-AT autonomous navigation + trash-detection demo system.  
ROS1 Noetic · Ubuntu 20.04 · Gazebo 11 (Simulation) / Docker (Real Robot).

> **`main` branch** — full documentation for both simulation development and real-robot deployment.  
> To reproduce the full simulation navigation experiments (Exp1-Exp4), use branch **`navigation_sim_experiments`**.  
> Unitree 4D Lidar L1 is the **primary sensor**; RPLIDAR A2 is used as local-costmap supplement in dual-lidar runtime.  
> Real-robot deployment runs in Docker; simulation uses Gazebo.  
> Operational runbook: [`doc.md`](doc.md)
>
> Current real-robot target-follow runtime defaults to **dual-lidar**:
> Unitree L1 remains the main ranging sensor, and RPLIDAR A2 is added as a
> short-range local-obstacle supplement. Mapping/AMCL/global-costmap stays
> Unitree-only; dual fusion is only enabled on local costmap. You can still fall back to
> `--lidar unitree` or `--lidar rplidar` when debugging hardware.

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
8. [Simulation Experiments](#simulation-experiments)
9. [Part B — Real Robot Deployment](#part-b--real-robot-deployment)
   - [Docker Environment Setup](#docker-environment-setup)
   - [Network Architecture](#network-architecture)
  - [Technical Report — Real-Robot Navigation Stack](#technical-report--real-robot-navigation-stack)
   - [Startup Order](#startup-order)
   - [Keyboard Teleoperation + Unitree Mapping Runbook](#keyboard-teleoperation--unitree-mapping-runbook)
   - [Autonomous Exploration SLAM Runbook](#autonomous-exploration-slam-runbook)
   - [Target Following Demo Runbook](#target-following-demo-runbook)
   - [Option A — Unitree L1 (Primary)](#option-a--unitree-l1-primary)
   - [Real Robot Node & Topic Reference](#real-robot-node--topic-reference)
   - [Real Robot TF Tree](#real-robot-tf-tree)
10. [Target Follower State Machine](#target-follower-state-machine)
11. [Trash Detection Bridge](#trash-detection-bridge)
12. [Dialogue Integration](#dialogue-integration)
13. [Parameter Tuning Guide](#parameter-tuning-guide)
14. [Autonomous Explorer Algorithm](#autonomous-explorer-algorithm)
15. [Known Issues and Notes](#known-issues-and-notes)
16. [Git Workflow](#git-workflow)
17. [Resources](#resources)
18. [Status](#status)
19. [Post-Installation Checklist](#post-installation-checklist)

---

## System Overview

This workspace implements a complete autonomous mobile robot system for the Pioneer 3-AT platform, featuring:

- **SLAM mapping** (gmapping) with manual or autonomous exploration
- **AMCL localisation** on pre-built maps
- **Target following** with standoff distance and face-target orientation
- **Trash detection** via YOLO + depth camera (real robot)
- **Dialogue interaction** with speech-to-text and NLU intent recognition (real robot)
- **Switchable radar runtime**: default dual-lidar (`Unitree + RPLIDAR`), with single-lidar fallback modes

### Sensor Stack Comparison

| | Unitree L1 (Primary) | RPLIDAR A2 (Supplement) |
|---|---|---|
| **Primary use** | gmapping, AMCL, global costmap, baseline local costmap | local costmap short-range supplement only |
| **Scan topic** | `/unitree/scan` | `/rplidar/scan_filtered` |
| **Sensor frame** | `unitree_lidar` | `laser` |
| **Param directory** | `param/unitree/` | local obstacle layer only |
| **Priority** | **Primary — always enabled in current workflow** | enabled only for local avoidance support |

**Unitree L1** provides 360° coverage, enabling faster frontier discovery and better obstacle avoidance.  
**RPLIDAR A2** is used only to enhance short-range local obstacle response in dual-lidar runtime.

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

### Sensor B: RPLIDAR A2 (Local Supplement)

| Property | Value |
|----------|-------|
| FOV | 360° horizontal |
| Range | up to ~12 m |
| Scan frequency | ~10 Hz in current runtime |
| ROS topic | `/rplidar/scan_filtered` (`sensor_msgs/LaserScan`) |
| TF frame | `laser` |
| Driver package | `rplidar_ros` |
| Interface | USB serial |

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

Real-robot RGB-D orientation correction is handled in software. When the
camera is mounted upside down, the host-side RGB-D detectors rotate the
aligned RGB and depth frames together via `common_utils/rgbd_orientation.py`
before YOLO inference. This keeps the detection boxes aligned with the depth
pixels later used for XYZ recovery. `./scripts/start_demo.sh` enables this
correction by default for the current demo setup.

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
│   │   ├── urdf/p3at_unitree.urdf.xacro       (Unitree model)
│   │   └── urdf/unitree_lidar_l1.urdf.xacro   (Unitree sensor macro)
│   ├── p3at_lms_gazebo/                  # Gazebo worlds and sim launch files
│   │   ├── launch/
│   │   │   ├── sim.launch                (Unitree simulation)
│   │   │   └── sim_unitree.launch        (Unitree simulation)
│   │   └── worlds/
│   │       └── complex_maze.world        (12.2×12.2 m test maze)
│   ├── p3at_lms_navigation/              # Navigation stack (Unitree-first + dual-lidar local avoidance)
│   │   ├── launch/
│   │   │   ├── mapping_unitree.launch            # Unitree: manual mapping (sim)
│   │   │   ├── nav_unitree.launch                # Unitree: AMCL navigation (sim)
│   │   │   ├── auto_mapping_unitree.launch       # Unitree: autonomous exploration (sim)
│   │   │   ├── auto_amcl_verify_unitree.launch   # Unitree: AMCL verifier (sim)
│   │   │   ├── real_robot_lidar_bringup.launch   # Real robot: lidar-only bringup/validation
│   │   │   ├── real_robot_mapping_rplidar.launch # RPLIDAR: real-robot mapping
│   │   │   ├── real_robot_mapping_unitree.launch # Unitree: real-robot mapping
│   │   │   ├── real_robot_nav_rplidar.launch     # RPLIDAR: real-robot nav
│   │   │   └── real_robot_nav_unitree.launch     # Unitree: real-robot nav
│   │   ├── param/unitree/                # Unitree-specific parameters (tuned)
│   │   │   ├── gmapping.yaml             # maxUrange: 10.0
│   │   │   ├── costmap_common.yaml       # inflation 0.3 / scale 5.0
│   │   │   ├── global_costmap.yaml       # 360° obstacle source (unitree_lidar frame)
│   │   │   ├── global_costmap_local_only.yaml  # Standalone target-following (odom frame, no /map)
│   │   │   ├── local_costmap.yaml        # inflation 0.3 / scale 5.0
│   │   │   ├── move_base.yaml            # clearing_rotation_allowed: false
│   │   │   └── amcl.yaml
│   │   ├── rviz/
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
│   ├── rplidar_ros/                        # Local RPLIDAR overlay with motor pre-start workaround
│   └── unilidar_sdk/                     # Unitree lidar SDK source
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
│   ├── start_real_nav_unitree.sh         # Start AMCL nav stack
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

> **Core Unitree stack is pre-installed in `ros_noetic:nav_unitree`.**
> For dual-lidar runtime, also install/verify `rplidar_ros` (see below).

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
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<UNITREE_CP2104_SERIAL>", MODE:="0666", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="unitree_lidar"
```

> Use **serial-pinned** rules (or `/dev/serial/by-id/...`) to avoid VID/PID ambiguity
> when multiple CP210x devices are connected.

### RPLIDAR A2 Driver Setup (Dual-Lidar Runtime)

Required when using `--lidar dual` (default demo mode) or `enable_rplidar:=true`
in `real_robot_mapping_unitree.launch` / `real_robot_nav_unitree.launch`.

> `rplidar_ros` already bundles the Slamtec SDK backend, so no separate
> `rplidar_sdk` install step is needed for this repository workflow.

If startup logs show:
`Error, operation time out. RESULT_OPERATION_TIMEOUT!`
it usually means the serial endpoint opens but the lidar core does not reply.
Per Slamtec `rplidar_ros`/`rplidar_sdk` and A2M12 datasheet, prioritize checks:
1. stable 5V power/current margin,
2. accessory-board PWM/MOTOCTL path (A2/A3 motor control),
3. correct serial endpoint mapping.

```bash
# Jetson host helper (installs ros-noetic-rplidar-ros + dialout setup)
./setup_rplidar_a2.sh

# Quick verify inside ROS environment
rospack find rplidar_ros
```

Prefer stable serial paths (recommended):

```bash
ls -la /dev/serial/by-id
# then use:
#   rplidar_port:=/dev/serial/by-id/usb-... 
```

#### Current Working RPLIDAR A2M12 Bringup Recipe

The current robot build uses the official Slamtec accessory board and the
validated working serial configuration is:

- `rplidar_port:=/dev/rplidar_lidar`
- `rplidar_baud:=256000`
- `rplidar_pre_start_motor:=true`
- `rplidar_pre_start_motor_pwm:=600`
- `rplidar_pre_start_motor_warmup_s:=2.0`

Why this is needed on the current hardware:

- The A2M12 is stable only after the motor control path has been asserted first.
- Simply opening the serial port and immediately requesting scan data was
  observed to time out intermittently.
- The repository now carries a local `rplidar_ros` overlay that can
  optionally start the motor before the driver requests device info / scan.

The overlay adds three private ROS params on `rplidarNode`:

- `pre_start_motor`
- `pre_start_motor_pwm`
- `pre_start_motor_warmup_s`

This workaround applies only to the second lidar (`RPLIDAR A2`). The Unitree
L1 path remains unchanged and continues to use the stock `unitree_lidar_ros`
driver flow.

When `pre_start_motor:=true`, the repo-local overlay:

1. connects to the device,
2. starts the motor first,
3. waits for the configured warmup interval,
4. then continues with normal SDK bringup (`getDeviceInfo`, `startScan`, etc.).

This behaviour is opt-in at the launch layer so it stays easy to revert later:
remove the `pre_start_motor*` launch args and the driver falls back to the
upstream initialization order.

The params are already wired through:

- `target_follow_real.launch`
- `real_robot_mapping_rplidar.launch`
- `real_robot_nav_rplidar.launch`
- `start_demo.sh`
- `real_robot_lidar_bringup.launch`

Quick standalone validation:

```bash
./scripts/start_demo.sh --sensor-only --lidar dual
```

Expected current real-hardware rates after successful bringup:

- `/unitree/scan`: about `9.7 Hz`
- `/rplidar/scan_filtered`: about `11 Hz`

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

**Unitree Stack (recommended):**

```bash
roslaunch p3at_lms_gazebo sim_unitree.launch \
  world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world
```

This launches Gazebo physics simulator, the Unitree-first robot model, and `robot_state_publisher` for TF.

### A-2 Mapping with Manual Control

Unitree:

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch
```

Includes: Gazebo + slam_gmapping + move_base + RViz.  
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
```

Drag the `target` model in Gazebo — the robot follows.

**Dynamic moving target with standoff and face:**

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch \
  move_target:=true \
  target_speed:=0.3 \
  target_pause:=2.0 \
  standoff_distance:=1.0 \
  face_target:=true
```

**Manual target via RViz goal relay:**

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch \
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

Unitree:

```bash
roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true
```

The `amcl_verifier.py` node:

1. Loads the saved map via `map_server`
2. Teleports the robot to 5 predefined waypoints
3. Measures AMCL convergence time and position error at each waypoint
4. Saves `maps/amcl_report.txt` and `maps/amcl_report.json`

Reference result (Unitree stack): mean position error 0.089 m, convergence 1.0 s.

### A-7 Full Pipeline Script

Run both phases (autonomous mapping → AMCL verification) in sequence:

```bash
bash run_full_pipeline_unitree.sh
```

---

## Simulation Verification Tests

### Test 1 — gmapping + Waypoint Navigation

**Purpose:** Verify SLAM and navigation stack end-to-end.

```bash
# Terminal 1: launch simulation
roslaunch p3at_lms_navigation mapping_unitree.launch use_gazebo_target:=false

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
roslaunch p3at_lms_navigation mapping_unitree.launch \
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
```

Expected: terminal prints coverage updates; map file saved on completion.

### Test 6b — AMCL Accuracy Verification

**Purpose:** Verify AMCL localisation accuracy on the explored map.

```bash
roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true
```

Expected: mean position error < 0.30 m, convergence time < 5 s.

---

## Simulation Experiments

This section records the consolidated simulation work completed on **2026-03-17** for the Unitree-first Gazebo + RViz workflow, including configuration updates, experiment design, acceptance criteria, tuning iterations, and final results. The main implementation entry point is [`scripts/run_sim_four_experiments.sh`](scripts/run_sim_four_experiments.sh), with supporting metrics collectors in [`catkin_ws/src/p3at_lms_navigation/scripts/nav_experiment_runner.py`](catkin_ws/src/p3at_lms_navigation/scripts/nav_experiment_runner.py) and [`catkin_ws/src/p3at_lms_navigation/scripts/target_follow_metrics.py`](catkin_ws/src/p3at_lms_navigation/scripts/target_follow_metrics.py).

### Scope of the 2026-03-17 Update

- Added a one-click four-experiment simulation workflow for:
  - simple-obstacle fixed-point navigation
  - complex-obstacle fixed-point navigation
  - simple-obstacle target following
  - Task6/7-style autonomous mapping + AMCL verification
- Standardised the simulation robot to the **dual-lidar + RGB-D** URDF path already present in `cat_ws`, and re-verified the sensor geometry with generated visuals under `Log/urdf_visual/`.
- Shifted the simulated `RPLIDAR` further in the negative `x` direction by **3 cm**, resulting in `rplidar_tf_x = -0.232`, and kept the rest of the sensor layout unchanged.
- Added experiment-specific automation and metrics collection so each experiment now generates reusable logs and JSON summaries under `Log/sim_experiments/`.
- Updated `scripts/run_sim_four_experiments.sh` so **Exp3 defaults to the high-speed stable profile** (`target_speed=0.36 m/s`, high-speed DWA overrides, `enable_interaction_mode=false`), with conservative fallback used only if the high-speed run fails to launch or collect metrics.

### Sensor Configuration Policy Used in These Experiments

All experiments in this section follow the same sensor-role split:

- **Unitree 4D Lidar L1** is the primary scan source for:
  - `slam_gmapping`
  - `AMCL`
  - `global_costmap`
  - baseline `local_costmap`
- **RPLIDAR A2** is used only as a **short-range local obstacle supplement** on `local_costmap`.
- `AMCL` verification is explicitly **Unitree-only**. `RPLIDAR` does **not** participate in AMCL scan matching.

For the task4 AMCL launch, this policy is enforced directly in [`catkin_ws/src/p3at_lms_navigation/launch/auto_amcl_verify_unitree.launch`](catkin_ws/src/p3at_lms_navigation/launch/auto_amcl_verify_unitree.launch): `amcl` remaps `scan` to `/unitree/scan`, while `local_costmap` fuses `unitree_scan_sensor` and `rplidar_scan_sensor`.

### Experiment Design and Acceptance Standards

The four experiments were designed as follows:

| Experiment | Scenario | Goal | Main metric | Acceptance focus |
|----------|----------|------|-------------|------------------|
| Exp1 | Simple obstacles | Fixed-point navigation | waypoint success, final goal error, travel distance | stable end-to-end navigation |
| Exp2 | Complex maze obstacles | Fixed-point navigation | waypoint success, timeout behaviour, path efficiency | ability to detour around tighter clutter |
| Exp3 | Simple obstacles | Target following | mean robot-target distance, standoff tolerance rate, robot/target speed | robot must actually follow rather than remain static |
| Exp4 | Complex maze | Autonomous mapping + AMCL | exploration coverage within bounded time, AMCL convergence/error, nav success on verification waypoints | mapping feasibility in 20 min scale and localisation accuracy |

Acceptance standards used in this update:

- **Navigation experiments (Exp1/Exp2)**:
  - success is measured waypoint-by-waypoint from `move_base`
  - target final position error should stay below roughly `0.2 m` in normal success cases
- **Target following (Exp3)**:
  - the robot must show non-trivial motion
  - `result_false_count` should remain low
  - target motion must be physically followable; if the robot remains stationary, the target path/speed must be re-designed instead of accepting the failure
- **Autonomous mapping (Exp4 mapping)**:
  - full maze coverage is *not* required in 20 minutes
  - the experiment is considered useful if the robot can repeatedly discover frontiers, expand known space, avoid permanent wall-lock, and save a usable partial map
- **AMCL verification (Exp4 AMCL)**:
  - convergence time should be within a few seconds
  - mean waypoint position error should stay below `0.30 m`
  - maximum waypoint position error should stay below `0.50 m`
  - verification waypoint navigation success should exceed `80%`

### Auto-Explore Wall-Stuck Handling Strategy

The autonomous mapping stage was explicitly tuned against the common failure mode where the robot gets trapped against walls and repeatedly fails with little translational progress.

Two main mapping runs were used:

- **Run 1**: more conservative blacklist/progress settings, but the robot frequently triggered `stuck_no_progress` and coverage remained around `7.1%`.
- **Run 2**: tuned settings with:
  - `frontier_blacklist_radius = 0.25`
  - `stuck_min_progress = 0.06`
  - `stuck_progress_timeout = 10.0`

This second run achieved a much more acceptable short-horizon mapping result:

- map saved successfully as `exp4_task67_tuned_run2`
- final known-space coverage: **14.4%**
- runtime scale: about **12.5 minutes**

This is the recommended 20-minute feasibility protocol for the current maze: run the explorer long enough to validate frontier discovery, corridor penetration, repeated re-goaling, and partial map growth, rather than treating full-maze closure as the only success condition.

### Final Experiment Results

Primary artifact directory for the final consolidated run:

```text
Log/sim_experiments/20260317_0215_manual/
```

#### Exp1 — Simple Obstacle Fixed-Point Navigation

Source metric: [`Log/sim_experiments/20260317_0215_manual/metrics/exp1_simple_nav.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp1_simple_nav.json)

- Waypoints succeeded: **3 / 3**
- Success rate: **100%**
- Mean goal error: **0.1475 m**
- Mean navigation time: **76.26 s**
- Total traveled distance: **16.80 m**

Conclusion: the simple-scene fixed-point navigation pipeline is stable and fully passes.

#### Exp2 — Complex Obstacle Fixed-Point Navigation

Source metric: [`Log/sim_experiments/20260317_0215_manual/metrics/exp2_complex_nav.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp2_complex_nav.json)

- Waypoints succeeded: **4 / 5**
- Success rate: **80%**
- Mean goal error: **0.1530 m**
- Mean navigation time: **71.49 s**
- Total traveled distance: **17.91 m**

The failed waypoint was `WP3_west`, which remained near the target but did not get marked `SUCCEEDED` before timeout. This motivated the later task4 tuning work around final-goal acceptance behaviour.

#### Exp3 — Simple Obstacle Target Following

Accepted source metric: [`Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_tuned.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_tuned.json)

- Mean robot-target distance: **1.8037 m**
- Distance median: **1.5530 m**
- Standoff target: **0.8 m**
- Standoff RMSE: **1.2612 m**
- Within-tolerance rate: **6.65%**
- Mean robot speed: **0.0339 m/s**
- Mean target speed: **0.0376 m/s**
- `result_false_count`: **0**

Interpretation:

- the robot did move and continuously stayed in `TRACKING`
- aggressive moving-target setups were not reliable enough, so the accepted result uses a more conservative tuned target profile
- additional robustness work was added afterwards to detect fake Gazebo starts (`/clock` missing) and avoid hanging metrics runs

#### Exp3 — High-Speed Stable Profile (New One-Click Default)

High-speed default source metric: [`Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_hs_fix_attempt5.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_hs_fix_attempt5.json)

High-speed default launch log: [`Log/sim_experiments/20260317_0215_manual/logs/exp3_hs_fix_attempt5_launch.log`](Log/sim_experiments/20260317_0215_manual/logs/exp3_hs_fix_attempt5_launch.log)

- Standoff target: **1.2 m**
- Mean robot-target distance: **1.5046 m**
- Standoff RMSE: **0.9816 m**
- Within-tolerance rate (`±0.35 m`): **68.8%**
- `result_false_count`: **0**
- Tracking state ratio: **100% TRACKING**

Speed-note for this high-speed run:

- `move_target` waypoint-segment speed from launch logs: mean **0.3606 m/s**, median **0.3605 m/s**
- metrics JSON reports lower aggregated robot/target speeds due sampling-time effects in long headless Gazebo runs; the target command profile itself is the intended **0.36 m/s** high-speed condition.

#### Exp4 — Task6/7-Style Autonomous Mapping + AMCL Verification

Mapping artifact:

- [`Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.yaml`](Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.yaml)
- [`Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.pgm`](Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.pgm)

Final tuned AMCL report:

- [`Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.txt`](Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.txt)
- [`Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.json`](Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.json)

Final tuned AMCL result:

- Convergence time: **1.1 s**
- Tracking mean position error: **0.0610 m**
- Tracking max position error: **0.3340 m**
- Waypoint navigation success: **6 / 6**
- Mean waypoint position error: **0.0479 m**
- Max waypoint position error: **0.0964 m**
- Overall AMCL verdict: **PASS**

This was not the first AMCL attempt. Earlier verification runs only achieved **2 / 6** navigation success because the robot often reached the target position but still failed final acceptance under stricter goal/yaw conditions. A task4-specific tuning pass then changed:

- `DWAPlannerROS/yaw_goal_tolerance` to `pi`
- `DWAPlannerROS/xy_goal_tolerance` to `0.20`
- `DWAPlannerROS/latch_xy_goal_tolerance` to `true`
- `local_costmap/obstacle_layer/rplidar_scan_sensor/clearing` to `true`

After that, task4 improved from:

- navigation success `2 / 6` -> `6 / 6`
- mean waypoint position error `0.1052 m` -> `0.0479 m`
- max waypoint position error `0.3634 m` -> `0.0964 m`

### Local Costmap Participation Check

The final logs confirm that all four experiments used **dual participation** on `local_costmap`:

- Exp1: `local_costmap/observation_sources = unitree_scan_sensor rplidar_scan_sensor`
- Exp2: `local_costmap/observation_sources = unitree_scan_sensor rplidar_scan_sensor`
- Exp3: `local_costmap/observation_sources = unitree_scan_sensor rplidar_scan_sensor`
- Exp4: `local_costmap/observation_sources = unitree_scan_sensor rplidar_scan_sensor`

At the same time, `global_costmap`, `gmapping`, and `AMCL` stayed Unitree-led, which preserves the intended division of labour.

### Recommended Current Reading of the Results

As of the 2026-03-17 update, the simulation stack should be interpreted like this:

- **Exp1** is fully passed.
- **Exp2** is mostly passed but still exposes one difficult waypoint in the complex obstacle field.
- **Exp3** now has two valid references:
  - conservative slow-speed profile (historical baseline),
  - high-speed stable profile (`0.36 m/s`) now used as the default in one-click four-experiment runs.
  The high-speed profile reaches **68.8%** standoff tolerance with **0 false terminations**, but distance tightness can still be improved further.
- **Exp4** is now in an acceptable state: short-horizon autonomous exploration is feasible, and the tuned Unitree-only AMCL verification passes cleanly.

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

### Technical Report — Real-Robot Navigation Stack

This section is a code-grounded technical report for the current branch (`full_system_tmp`, commit `b379d0a`).
It explains how real-robot mapping, localization, navigation, target following, and dialogue/state-machine logic are integrated end-to-end.

#### 1) Runtime Architecture and Process Boundaries

The real stack is deliberately split into three runtime domains:

- **Jetson host (Ubuntu 22.04, non-ROS app layer)**
  - Runs RGB-D detection and dialogue services.
  - Main processes: [`handobj_detection/handobj_detection_rgbd.py`](handobj_detection/handobj_detection_rgbd.py), [`dialogue/dialogue_udp_runner.py`](dialogue/dialogue_udp_runner.py).
- **Jetson Docker (`ros_noetic`, ROS1 control layer)**
  - Runs ROS core, LiDAR drivers, SLAM/AMCL, move_base, target follower, and UDP bridge nodes.
  - Main orchestration entry: [`scripts/start_demo.sh`](scripts/start_demo.sh).
- **Raspberry Pi (base driver layer)**
  - Runs RosAria and publishes chassis odometry/TF.
  - Entry: [`catkin_ws/src/p3at_base/launch/base.launch`](catkin_ws/src/p3at_base/launch/base.launch).

This separation keeps camera/AI dependencies off ROS Docker while preserving deterministic ROS control/TF behavior.

#### 2) Control-Plane Entry and Operational Profiles

The top-level supervisor is [`scripts/start_demo.sh`](scripts/start_demo.sh), which selects one of three real-robot operational profiles:

1. **`--sensor-only`**
   - Launches LiDAR chain only via [`real_robot_lidar_bringup.launch`](catkin_ws/src/p3at_lms_navigation/launch/real_robot_lidar_bringup.launch).
   - No move_base, no target follower motion, no dialogue.

2. **Default task mode (`online_slam_task`)**
   - Launches [`target_follow_real.launch`](catkin_ws/src/target_follower/launch/target_follow_real.launch) with:
     - `launch_move_base:=true`
     - `use_online_slam:=true`
   - This creates a self-contained task stack: LiDAR + SLAM + move_base + follower + bridge chain.

3. **Map-assisted overlay mode (`--assist-map`)**
   - Starts nav backbone from [`real_robot_nav_unitree.launch`](catkin_ws/src/p3at_lms_navigation/launch/real_robot_nav_unitree.launch) (map_server + AMCL + move_base).
   - Starts follower overlay via [`target_follow_real.launch`](catkin_ws/src/target_follower/launch/target_follow_real.launch) with `launch_move_base:=false` and `global_frame:=map`.

In all profiles, `start_demo.sh` performs readiness checks on `/target_follower/status`, `/map`, TF freshness, and scan topics before continuing.

#### 3) Sensor and TF Ingestion Pipeline (Real Robot)

For Unitree-first runtime (default):

1. `unitree_lidar_ros_node` publishes `/unilidar/cloud`.
2. `pointcloud_to_laserscan` converts cloud to `/unitree/scan_raw`.
3. [`scan_body_filter.py`](catkin_ws/src/target_follower/scripts/scan_body_filter.py) removes chassis self-hits and republishes `/unitree/scan`.

For dual-lidar runtime:

- `rplidarNode` publishes `/scan`.
- `rplidar_health_monitor.py` supervises startup and stream freshness.
- Another `scan_body_filter.py` instance republishes `/rplidar/scan_filtered`.

Robot TF/description are unified by:

- URDF: [`p3at_unitree.urdf.xacro`](catkin_ws/src/p3at_lms_description/urdf/p3at_unitree.urdf.xacro)
- `robot_state_publisher` for static sensor extrinsics.

Base odometry chain:

- RosAria publishes `/RosAria/pose` and `odom -> base_link`.
- [`odom_republisher.py`](catkin_ws/src/p3at_base/scripts/odom_republisher.py) standardizes `/odom` topic semantics.

#### 4) Mapping Stack (SLAM)

Online SLAM uses `slam_gmapping` with Unitree parameters from:

- [`param/unitree/gmapping.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/gmapping.yaml)

Key technical points:

- `map_frame=map`, `odom_frame=odom`, `base_frame=base_link`.
- 360° LiDAR configuration with `maxRange=30.0`, `maxUrange=12.0`.
- Higher particle count (`particles=50`) and stricter `minimumScore=50` for robust loop closure.

Two SLAM map usages coexist in task workflow:

- **Primary online nav map**: `/map` (from `slam_gmapping_online` inside `target_follow_real.launch` when `use_online_slam=true`).
- **Work-map channel**: `/work_map` via [`passive_mapping_unitree.launch`](catkin_ws/src/p3at_lms_navigation/launch/passive_mapping_unitree.launch), used for background learning during tasks.

Persistent map saving/merging is handled by [`continuous_map_manager.py`](catkin_ws/src/p3at_lms_navigation/scripts/continuous_map_manager.py):

- Periodically serializes live map to `*.yaml/*.pgm`.
- Can merge live observations into an imported base map using TF-based frame alignment.

#### 5) Localization Stack (AMCL)

In map-assisted mode, localization is switched to AMCL via [`real_robot_nav_unitree.launch`](catkin_ws/src/p3at_lms_navigation/launch/real_robot_nav_unitree.launch):

- `map_server` provides static map.
- `amcl` consumes `/unitree/scan` and publishes `map -> odom`.
- AMCL parameters from [`param/unitree/amcl.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/amcl.yaml).

This creates a clear authority swap:

- **Online SLAM mode**: `slam_gmapping` owns `map -> odom`.
- **Map-assisted mode**: `amcl` owns `map -> odom`.

The follower remains compatible in both modes because its `global_frame` is explicitly configured (`map` in overlay mode).

#### 6) Navigation Stack (move_base + Costmaps)

Core planner/controller config is loaded from:

- [`param/unitree/move_base.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/move_base.yaml)

Current branch characteristics:

- Global planner: `NavfnROS`.
- Local planner: `DWAPlannerROS`.
- `clearing_rotation_allowed: true` in Unitree profile (note: RPLIDAR profile uses `false`).
- DWA tuned for real robot with moderate velocities and extended simulation horizon (`sim_time: 3.0`).

Costmap structure:

- Common inflation/footprint: [`param/unitree/costmap_common.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/costmap_common.yaml).
- Global costmap (map frame): [`param/unitree/global_costmap.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/global_costmap.yaml).
- Local costmap (odom rolling window): [`param/unitree/local_costmap.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/local_costmap.yaml).
- Standalone no-map global rolling window: [`param/unitree/global_costmap_local_only.yaml`](catkin_ws/src/p3at_lms_navigation/param/unitree/global_costmap_local_only.yaml).

Dual-lidar policy is explicitly local-costmap-biased:

- Global planning remains Unitree-centric.
- Local obstacle layer can be switched to `unitree_scan_sensor + rplidar_scan_sensor`.

#### 7) Perception-to-Goal Transformation Chain

Target input path is intentionally staged and frame-safe:

1. Host detection sends UDP JSON (`x,y,z,frame_id`) to Docker.
2. [`udp_target_bridge.py`](catkin_ws/src/target_follower/scripts/udp_target_bridge.py): UDP -> `/trash_detection/target_point` (`PointStamped`).
3. [`point_to_target_pose.py`](catkin_ws/src/target_follower/scripts/point_to_target_pose.py): `PointStamped` -> `/target_pose` (`PoseStamped`).
4. [`target_follower.py`](catkin_ws/src/target_follower/scripts/target_follower.py) transforms target into `global_frame` and produces move_base goals.

This decomposition isolates transport, message conversion, and control logic, simplifying diagnosis of frame or latency faults.

#### 8) Target Following Controller and Goal Policy

`target_follower.py` is the **single goal owner** for tracking behavior.

Core policies:

- Goal resend rate limit (`send_rate_hz`) and displacement gate (`min_update_dist`) reduce excessive preemption.
- Standoff control computes an offset goal rather than commanding exact target position.
- Optional `face_target` sets terminal orientation toward target.
- Target freshness guard (`target_timeout_s`) cancels stale pursuits.

Important branch-specific behavior:

- This branch includes **CLOSE_APPROACH** logic (direct `/cmd_vel` fallback near target) in addition to move_base tracking.
- Trigger condition is distance-based (`reach_dist < close_approach_threshold`), not a global enable switch in this revision.

#### 9) State Machine and Dialogue Coupling

Implemented states in [`target_follower.py`](catkin_ws/src/target_follower/scripts/target_follower.py):

- `STARTING`, `IDLE`, `EXPLORING`, `TRACKING`, `CLOSE_APPROACH`, `REACHED`,
  `WAITING_ACTION`, `POST_ACCEPT_COOLDOWN`, `RETREATING`, `REACQUIRE_TARGET`, `LOST`, `FAILED`.

High-level transition logic:

- `IDLE/EXPLORING -> TRACKING`: target stream becomes fresh and confirmed.
- `TRACKING -> REACHED`: camera depth reaches standoff threshold.
- `TRACKING -> CLOSE_APPROACH`: near-target zone where move_base may struggle.
- `REACHED -> WAITING_ACTION`: publish success and wait for dialogue result.
- `WAITING_ACTION -> POST_ACCEPT_COOLDOWN` on `trash_action=True`.
- `WAITING_ACTION -> RETREATING` on `trash_action=False`.
- `RETREATING -> IDLE`: retreat complete or interrupted by valid new target.

Dialogue closed-loop is bridged by:

- [`navigation_success_udp_bridge.py`](catkin_ws/src/target_follower/scripts/navigation_success_udp_bridge.py): `/target_follower/result` -> UDP trigger.
- [`udp_trash_action_bridge.py`](catkin_ws/src/target_follower/scripts/udp_trash_action_bridge.py): UDP decision -> `/trash_action`.

The success bridge supports rising-edge triggering and optional WAITING_ACTION retrigger for robustness against packet loss.

#### 10) Autonomous Exploration as Tracking Fallback

When stable targets are absent, the follower invokes map-driven exploration:

- Frontier detection and scoring are handled by [`frontier_planner.py`](catkin_ws/src/target_follower/scripts/frontier_planner.py).
- Candidate goals are filtered by map safety, costmap safety, blacklist memory, and optional `make_plan` reachability.
- Follower monitors stuck conditions using odometry progress and near-zero `/cmd_vel` windows, then replans/backs off.

This means exploration and tracking are not separate nodes competing for move_base; they are coordinated inside one behavior owner (`target_follower`).

#### 11) Why This Stack Is Operationally Cohesive

The current branch forms a layered but internally consistent navigation stack:

- **Transport layer**: UDP bridges isolate host AI services from ROS action/control.
- **State/control layer**: `target_follower` owns both tracking and exploration arbitration.
- **Navigation layer**: move_base remains the primary avoidance/planning engine.
- **Localization/mapping layer**: switchable SLAM or AMCL authority for `map -> odom`.
- **Persistence layer**: continuous map manager keeps runtime mapping artifacts usable.

This architecture is what enables one-command demo startup while still supporting mode fallback (`sensor-only`, `online_slam_task`, `map_assisted`, `rplidar-only`).

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

`./scripts/start_demo.sh` now defaults to `--lidar dual`, which keeps Unitree as
the primary scan source and adds RPLIDAR A2 local obstacle sensing. For quick
fallbacks:

```bash
./scripts/start_demo.sh --lidar unitree
./scripts/start_demo.sh --lidar rplidar --rplidar-port /dev/rplidar_lidar
```

If you only want to wake the sensors and verify both scan streams without
starting navigation / detection / dialogue, use:

```bash
./scripts/start_demo.sh --sensor-only --lidar dual
```

**Option B — Mapping:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      ./scripts/start_base.sh
3. Jetson Docker:  ./scripts/start_real_mapping_unitree.sh use_rviz:=false
4. Jetson Docker:  rosrun teleop_twist_keyboard teleop_twist_keyboard.py (or autonomous_explorer.py)
```

**Option C — Navigation on saved map:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      ./scripts/start_base.sh
3. Jetson Docker:  ./scripts/start_real_nav_unitree.sh map_file:=...
4. Jetson Docker:  roslaunch target_follower target_follow_real.launch launch_move_base:=false
5. Jetson Host:    python3 handobj_detection/handobj_detection_rgbd.py --udp-enable
```

For unitree-only fallback in mapping/nav: append `enable_rplidar:=false`.

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
| RPLIDAR A2 connected to Jetson (if using default dual mode) | ✓ |
| `docker ps` shows `ros_noetic` container running | ✓ |

> **⚠️ Important:** Power on the Unitree LiDAR **only after** `roslaunch` has already started (Step 3).

#### Step 1 — Raspberry Pi: Start Chassis Driver

```bash
# On Pi (ssh frank@192.168.50.2)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

`start_base.sh` now runs `scripts/sync_time_from_master.sh` first, so the Pi
pulls time from the Jetson ROS master before `RosAria` starts publishing
`/RosAria/pose`, `/odom`, and `odom -> base_link`.

For boot-time protection on the Pi itself, install the companion service once:

```bash
cd /home/pi/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/install_pi_time_sync_service.sh
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

`start_real_mapping_unitree.sh` now performs RPLIDAR preflight by default
(`enable_rplidar:=true`). If the second lidar is not connected, append:

```bash
./scripts/start_real_mapping_unitree.sh use_rviz:=false enable_rplidar:=false
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
- Post-dialogue behavior now distinguishes acceptance vs refusal:
  - `/trash_action=True` → `POST_ACCEPT_COOLDOWN` → retreat
  - `/trash_action=False` → immediate retreat
- Retreat policy:
  1. Record current dialogue location and traveled path.
  2. Rotate in place by a moderate angle (`retreat_turn_angle_deg`, default 100°).
  3. Drive away forward (`retreat_distance`, default 1.5 m).
  4. Resume auto-explore to find the next person/object.
- Anti-repeat policy:
  - Exploration avoids recently visited areas for `explore_revisit_window_s` (default 120 s).
  - Targets near recent dialogue locations are temporarily ignored (`target_reacquire_block_s`, default 120 s).
- Exploration safeguards:
  - Exploration is now slightly more conservative by default (`explore_goal_distance = 2.4 m`) and uses a forward-biased heading set instead of a full rearward sweep.
  - A target must remain stable for a short period before it can interrupt exploration, which reduces cancellations caused by flickering detections.
  - Exploration now replans early if odometry progress stays too small, or if `move_base` keeps publishing near-zero `cmd_vel` in front of an obstacle.
- Demo launcher readiness checks:
  - `start_demo.sh` now waits for a live `/target_follower/status` stream and fresh `/odom`, not just process names.
  - If startup blocks in Step 4, the script prints the exact blocker, such as missing `/move_base`, no `/target_follower/status`, or stale `/odom`.

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

For the current robot build, `start_demo.sh` launches
`handobj_detection_rgbd.py` with `--rotate-180` because the Orbbec camera is
mounted upside down. The same flag is also available in
`trash_detection/predict_15cls_rgbd.py` for the trash-detection branch.

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

Static TF `base_link → camera_link`: xyz=`(0.208, 0, 0.85)`, quat=`(-0.5, 0.5, -0.5, 0.5)` = RPY `[-90°, 0°, -90°]`

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
| `retreat_turn_angle_deg` | `100.0` deg | In-place turn angle before retreat (moderate turn to leave the area without over-rotating into obstacles) |
| `action_wait_timeout` | `45.0` s | Timeout waiting for `/trash_action` |
| `enable_auto_explore` | `true` | Enable active exploration when no target is tracked |
| `explore_goal_distance` | `2.4` m | Step distance for each explore navigation goal |
| `explore_goal_timeout` | `30.0` s | Timeout for one explore goal |
| `explore_target_confirm` | `0.8` s | Stable-target debounce before exploration can be interrupted |
| `explore_target_max_gap` | `1.0` s | Maximum gap allowed between target updates during the debounce |
| `explore_stuck_timeout` | `8.0` s | Replan explore if odometry progress remains too small for this long |
| `explore_min_progress` | `0.2` m | Minimum progress expected before an explore goal is considered stuck |
| `explore_zero_cmd_vel_timeout` | `2.0` s | Early replan if `move_base` keeps publishing near-zero `cmd_vel` in front of obstacles |
| `explore_revisit_window` | `120.0` s | Time window for no-repeat exploration |
| `explore_revisit_radius` | `1.5` m | Spatial radius for no-repeat exploration |
| `target_reacquire_block_s` | `120.0` s | Block re-querying targets near recent dialogue points |
| `target_reacquire_radius` | `1.6` m | Radius for target reacquire suppression |

#### start_demo.sh Common Options (New)

```bash
./scripts/start_demo.sh \
  --lidar dual \
  --unitree-port /dev/unitree_lidar \
  --rplidar-port /dev/rplidar_lidar \
  --rplidar-baud 256000 \
  --rplidar-pre-start-motor \
  --rplidar-pre-start-pwm 600 \
  --rplidar-pre-start-warmup 2.0 \
  --retreat-turn-deg 100 \
  --explore-step 2.4 \
  --explore-no-repeat-sec 120
```

- `--lidar`: `dual` (default), `unitree`, or `rplidar`.
- `--unitree-port`: Unitree serial device.
- `--rplidar-port`: RPLIDAR serial device.
- `--rplidar-baud`: RPLIDAR baud rate, default `256000` on current robot (override per hardware).
- `--rplidar-pre-start-motor`: pre-start the RPLIDAR motor before scan bringup.
- `--rplidar-pre-start-pwm`: PWM used for the pre-start workaround.
- `--rplidar-pre-start-warmup`: warmup time after motor start and before scan requests.
- `--sensor-only`: bring up only the lidar chain for validation; do not start navigation, YOLO, or dialogue.
- `--retreat-turn-deg`: moderate in-place turn before leaving.
- `--explore-step`: exploration step distance.
- `--explore-no-repeat-sec`: region no-repeat time window.
- `--no-explore`: disable active exploration (debug only).

Dual-lidar policy in current real-robot stack:
- `gmapping`, `amcl`, and map-based `global_costmap` use **Unitree only**.
- `local_costmap` fuses `unitree/scan` + `rplidar/scan_filtered` for local avoidance.

If you want the standalone RPLIDAR-only stack outside the demo launcher:

```bash
./setup_rplidar_a2.sh
./scripts/start_real_mapping_rplidar.sh rplidar_port:=/dev/rplidar_lidar
./scripts/start_real_nav_rplidar.sh map_file:=/path/to/map.yaml rplidar_port:=/dev/rplidar_lidar
```

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
                         ┌──────────────────────────────┐   ┌───────────────────────┐
                         │   POST_ACCEPT_COOLDOWN       │   │      RETREATING       │
                         │ Wait briefly so the user can │   │ Immediate retreat for  │
                         │ finish dropping trash        │   │ the refusal case       │
                         └──────────────┬───────────────┘   └───────────┬───────────┘
                                        │                               │
                                        └───────────────┬───────────────┘
                                                        ▼
                         ┌───────────────────────────────────────────────────────────┐
                         │                    RETREATING                             │
                         │  1) record dialogue point/path                            │
                         │  2) moderate in-place retreat turn                        │
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
| `/trash_action` | `std_msgs/Bool` | ← | Human response (`true` = cooldown then retreat, `false` = immediate retreat) |
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
retreat_turn_angle_deg: 100  # In-place turn angle before retreat
enable_auto_explore: true    # Explore while no fresh target
explore_goal_distance: 2.4
explore_target_confirm_s: 0.8
explore_target_max_gap_s: 1.0
explore_stuck_timeout_s: 8.0
explore_min_progress_dist: 0.2
explore_zero_cmd_vel_timeout_s: 2.0
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
│  UDP JSON at up to 10 Hz → 127.0.0.1:16031                          │
└────────────────────────────┬─────────────────────────────────────────┘
                             │  JSON: {"stamp":1708700000.123,
                             │         "frame_id":"camera_link",
                             │         "x":0.5, "y":0.2, "z":2.3,
                             │         "source":"handobj_detection_rgbd",
                             │         "kind":"holding"}
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
│  PointStamped → PoseStamped (preserve frame_id, default camera_link) │
│  pub /target_pose; TF into odom/map happens later in target_follower │
└──────────────────────────────────────────────────────────────────────┘
```

### Run Detection Standalone

```bash
# Native Jetson host (not Docker)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
python3 handobj_detection/handobj_detection_rgbd.py \
  --udp-enable \
  --udp-host 127.0.0.1 \
  --udp-port 16031 \
  --udp-frame-id camera_link \
  --udp-kind holding \
  --rotate-180 \
  --headless
```

Or use the wrapper script:

```bash
./scripts/start_trash_detection_rgbd.sh --detector handobj
```

---

## Dialogue Integration

The dialogue system provides voice-based human interaction when the robot reaches a target.

### Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│  [Docker] /target_follower/result = True                              │
└───────────────────────────────┬───────────────────────────────────────┘
                                │  UDP 16041
                                ▼
┌───────────────────────────────────────────────────────────────────────┐
│  [Host] dialogue_udp_runner.py                                        │
│                                                                       │
│  1. Receive UDP trigger: navigation_success=1                         │
│  2. STT: Vosk offline speech recognition                              │
│  3. NLU: DistilBERT ONNX or FastText intent classifier                │
│  4. Play prerecorded robot prompts / responses                        │
│  5. Send decision UDP 16032                                           │
└───────────────────────────────┬───────────────────────────────────────┘
                                │  JSON: {"trash_action":1|0,
                                │         "decision":"proceed|decline"}
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
  --listen-host 0.0.0.0 \
  --listen-port 16041 \
  --send-host 127.0.0.1 \
  --send-port 16032 \
  --device 24
```

Simulation mode:

```bash
python3 dialogue_udp_runner.py --sim \
  --first-user-wav voice_data/sim_user_answer_other_b.wav \
  --second-user-wav voice_data/sim_user_answer_negative_a.wav
```

Implementation note: the current dialogue stack uses Vosk STT plus either the
DistilBERT ONNX classifier in `dialogue/models/nlu_intent_bert/` or the
FastText fallback model `dialogue/models/nlu_intent.bin`.

### UDP Protocol

**Trigger (ROS→Host) — Port 16041:**
```json
{
  "stamp": 1708700000.123,
  "navigation_success": 1,
  "reason": "result_cb",
  "source": "navigation_success_udp_bridge"
}
```

The bridge may also resend the trigger while `target_follower` remains in
`WAITING_ACTION`, depending on `DIALOGUE_RETRIGGER_ENABLE`.

**Result (Host→ROS) — Port 16032:**
```json
{
  "stamp": 1708700002.456,
  "trash_action": 1,
  "decision": "proceed",
  "source": "dialogue_udp_runner"
}
```

---

## Demo Scripts Reference

All scripts are located in `scripts/`:

| Script | Purpose |
|--------|---------|
| `start_demo.sh` | One-command start: roscore + target_follow_real.launch + detection |
| `stop_demo_all.sh` | Kill all demo processes |
| `start_base.sh` | Start rosaria on Pi, with pre-launch Pi clock sync from Jetson |
| `sync_time_from_master.sh` | Sync local clock from the ROS master HTTP Date header |
| `install_pi_time_sync_service.sh` | Install a Pi boot-time time-sync service |
| `start_real_mapping_unitree.sh` | Start mapping with Unitree LiDAR |
| `start_real_nav_unitree.sh` | Start AMCL navigation with Unitree map + optional dual-lidar local avoidance |
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

¹ Active only in `auto_mapping_unitree.launch`.  
² Active only when `use_gazebo_target:=true`.  
³ Active only when `move_target:=true`.  
⁴ Active only when `use_rviz_goal_relay:=true`.

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
| `/target_pose` | `geometry_msgs/PoseStamped` | `/gazebo_target_publisher` (sim) / `/point_to_target_pose` (real robot) / goal relay | `/target_follower` |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | `/gazebo` | — |
| `/slam_gmapping/entropy` | `std_msgs/Float64` | `/slam_gmapping` | — |

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

#### Real Robot — Unitree + RPLIDAR + Orbbec Camera

```
odom
+-- base_link                     [rosaria  ~50 Hz]
    +-- unitree_lidar             [/robot_state_publisher  static]
    +-- laser                     [static_transform_publisher  static, dual-lidar mode]
    |   └── topic: /rplidar/scan_filtered
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
| `base_link -> laser` | `static_transform_publisher` | static | RPLIDAR rear mount in dual-lidar runtime |
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

### `gmapping.yaml` Key Parameters

| Parameter | Unitree value | Effect |
|-----------|---------------|--------|
| `particles` | 30 | Increase for larger/more complex environments |
| `delta` | 0.05 m | Map resolution |
| `linearUpdate` | 0.2 m | Min linear motion before scan processing |
| `angularUpdate` | 0.2 rad | Min angular motion before scan processing |
| `maxUrange` | 10.0 m | Max usable sensor range |

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

Maze theoretical maximum ~18%. Accepted baseline performance: 12.6% in 300 s.

---

## YOLO Target Detection (Native Ubuntu 22.04 -> Docker UDP Bridge)

> **Architecture**: The entire RGB-D detection pipeline runs **outside Docker** on Jetson's native Ubuntu 22.04, where the Orbbec Femto Bolt SDK is available. The host sends target XYZ to the ROS Docker container via **UDP JSON on localhost**.

### System Flow

```
[Jetson — Native Ubuntu 22.04]
  Orbbec Femto Bolt SDK
      └─► RGB frame + aligned depth frame
  YOLO inference (handobj_detection or trash_detection)
      └─► bounding box -> centre pixel -> depth lookup
  Back-project to 3D in camera_link:
      X = (u - cx) * Z / fx
      Y = (v - cy) * Z / fy
      Z = depth
  Publish UDP JSON to 127.0.0.1:16031:
      {"stamp":1708700000.123,
       "frame_id":"camera_link",
       "x":0.12,"y":-0.08,"z":2.35,
       "source":"handobj_detection_rgbd",
       "kind":"holding"}

[Jetson — ROS Noetic Docker  (--net=host)]
  udp_target_bridge.py
      └─► UDP 16031 -> /trash_detection/target_point (PointStamped)
  point_to_target_pose.py
      └─► /trash_detection/target_point -> /target_pose (PoseStamped)
  target_follower
      └─► TF into odom/map -> MoveBaseAction goal
```

### Running the Bridge Inside Docker

```bash
# Start the UDP bridge pair (inside Jetson Docker)
python3 catkin_ws/src/target_follower/scripts/udp_target_bridge.py \
  _bind_port:=16031

python3 catkin_ws/src/target_follower/scripts/point_to_target_pose.py
```

Or launch them together with navigation:

```bash
roslaunch target_follower target_follow_real.launch launch_move_base:=true
```

### Running Detection on Native Ubuntu 22.04

```bash
# Hand-object branch (current demo default)
python3 handobj_detection/handobj_detection_rgbd.py \
  --udp-enable --udp-host 127.0.0.1 --udp-port 16031 \
  --udp-frame-id camera_link --udp-kind holding \
  --rotate-180 --headless

# 15-class trash branch
python3 trash_detection/predict_15cls_rgbd.py \
  --nearest-person --udp-enable --udp-host 127.0.0.1 --udp-port 16031 \
  --udp-frame-id camera_link --udp-kind waste \
  --rotate-180 --headless
```

The host detector:
1. Opens the Orbbec Femto Bolt colour and aligned depth streams.
2. Runs YOLO inference on each RGB frame.
3. Samples depth at the detection centre and back-projects XYZ in `camera_link`.
4. Serialises the target as UDP JSON and sends it to Docker on `127.0.0.1:16031`.

### JSON Message Format

```json
{
  "stamp": 1708700000.123,
  "frame_id": "camera_link",
  "x": 0.12,
  "y": -0.08,
  "z": 2.35,
  "source": "handobj_detection_rgbd",
  "kind": "holding"
}
```

For the trash-detection branch, `source` becomes `trash_detection_rgbd` and
`kind` is typically `waste` or `person`.

### Manual Testing Without YOLO

Send a fake UDP target from the host:

```bash
python3 trash_detection/examples/send_target_udp.py --port 16031 --rate 2 --count 5
```

Or publish a fake target directly inside Docker to verify `target_follower`
end-to-end:

```bash
rostopic pub -r 5 /target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'camera_link'}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}"
```

### Why Native Ubuntu 22.04 (not Docker)?

| Reason | Detail |
|--------|--------|
| Kernel USB driver | Orbbec SDK and device access are maintained on the host, not in the ROS container |
| libusb access | USB 3.0 device-level access is simpler and more stable on the host |
| CUDA / TensorRT | YOLO GPU inference benefits from native CUDA without container friction |
| Simplicity | UDP JSON is a clean, process-agnostic interface between host detection and Docker ROS |
| Separation of concerns | Host owns camera/inference; Docker owns ROS navigation and behavior |

---

## Known Issues and Notes

- **Simulation fidelity**: URDF and Gazebo dynamics are approximate. Real P3-AT skid-steer turning differs notably from simulation.

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
- Helper scripts: `build_and_hint.sh`, `run_full_pipeline_unitree.sh`, `tools/`
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
- [x] `auto_mapping_unitree.launch` (Unitree stack) — verified  <- PRIMARY
- [x] AMCL accuracy verifier (`amcl_verifier.py`) — mean pos error 0.089 m, convergence 1.0 s
- [x] `auto_amcl_verify_unitree.launch` — verified
- [x] Complex maze world (`complex_maze.world`, 12×12 m) — created and verified
- [x] `run_full_pipeline_unitree.sh` — one-click two-phase script — created and verified
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
- [x] Real robot launch files — mapping/nav variants for Unitree and RPLIDAR modes — created
- [x] Raspberry Pi base driver package (`p3at_base`) — created and verified
- [x] Multi-machine network setup documented and verified (cross-machine topic test)
- [x] Docker image `ros_noetic:nav_unitree` — built and deployed
- [x] Unitree L1 driver integration — verified (`/unitree/scan` publishing)
- [x] Dual-lidar bringup (`Unitree + RPLIDAR`) — verified with `start_demo.sh --sensor-only --lidar dual`
- [x] RPLIDAR A2 workaround — verified (`256000` + pre-start motor + `2.0 s` warmup)
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
- [ ] Full pipeline: `bash run_full_pipeline_unitree.sh`

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
- [ ] If RPLIDAR absent: relaunch with `enable_rplidar:=false`
- [ ] Power on Unitree LiDAR (wait 10-15 s)
- [ ] `rostopic hz /unitree/scan` — publishing
- [ ] Teleop: `./scripts/start_teleop.sh jetson` — robot moves
- [ ] Map saved: `rosrun map_server map_saver -f ~/maps/unitree_map`

### Real Robot — Target Following Demo
- [ ] Sensor-only dual-lidar check: `./scripts/start_demo.sh --sensor-only --lidar dual`
  - [ ] `/unitree/scan` — about `9-10 Hz`
  - [ ] `/rplidar/scan_filtered` — about `10-11 Hz`
- [ ] Quick start: `./scripts/start_demo.sh`
- [ ] Default mode is dual lidar; verify `/unitree/scan` and `/rplidar/scan_filtered` if both sensors are connected
- [ ] Fallbacks available: `./scripts/start_demo.sh --lidar unitree` or `./scripts/start_demo.sh --lidar rplidar --rplidar-port /dev/rplidar_lidar`
- [ ] Or manual:
  - [ ] roscore + `target_follow_real.launch launch_move_base:=true`
  - [ ] `handobj_detection_rgbd.py --udp-enable`
- [ ] Hold object in front of camera → robot follows
- [ ] Robot stops at standoff distance
- [ ] `/target_follower/result` publishes `True`
- [ ] After dialogue, robot performs a moderate retreat turn + forward retreat
- [ ] Robot resumes auto-explore instead of waiting in place

### Real Robot — Dialogue Integration
- [ ] Dialogue system: `python3 dialogue/dialogue_udp_runner.py`
- [ ] Robot says prompt after reaching target
- [ ] Voice "yes" → `/trash_action` publishes True → cooldown then retreat
- [ ] Voice "no" → `/trash_action` publishes False → immediate retreat
