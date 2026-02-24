# ELEC70015 Human-Centered Robotics 2026 — Imperial College London

Pioneer 3-AT autonomous navigation + trash-detection demo system.  
ROS1 Noetic · Ubuntu 20.04 · Docker (Jetson) · Raspberry Pi 4.

> **Branch: `release/demo-v1`** — real-robot deployment branch.  
> Unitree 4D Lidar L1 is the **primary** sensor. SICK LMS200 is the backup.  
> No Gazebo in the production stack (RViz only). See [Simulation (Development Only)](#simulation-development-only) for local testing.  
> Practical runbook: [`doc.md`](doc.md)

---

## Quick Start — Docker Environment

All ROS nodes run inside a Docker container on Jetson. The image `ros_noetic:nav` contains ROS Noetic + all navigation packages pre-installed.

### Docker Image Info

| Image | Tag | Arch | Size | Contents |
|-------|-----|------|------|----------|
| `ros_noetic` | `nav` | arm64 (Jetson native) | ~4.4 GB | ROS Noetic desktop-full + gmapping, move_base, amcl, map_server, DWA, NavfnROS, RViz, Gazebo 11, xacro, tf2, actionlib |

### Start the Container

```bash
# First time — create from saved image
docker run -it --net=host --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/frank/work:/home/frank/work \
  -v /dev:/dev \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  --user $(id -u):$(id -g) \
  --name ros_noetic \
  ros_noetic:nav \
  bash
```

### Enter a Running Container

```bash
# Start if stopped
docker start ros_noetic

# Open a new shell in the running container
docker exec -it ros_noetic bash
```

### Inside the Container — Source & Build

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

> **Every new shell** inside the container needs both `source` commands:
> ```bash
> source /opt/ros/noetic/setup.bash
> source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
> ```

### Save Container Changes

After installing new packages or making persistent changes inside the container:

```bash
docker commit ros_noetic ros_noetic:nav
```

### Container Lifecycle Cheat Sheet

| Action | Command |
|--------|---------|
| Start stopped container | `docker start ros_noetic` |
| Enter running container | `docker exec -it ros_noetic bash` |
| Stop container | `docker stop ros_noetic` |
| Check status | `docker ps -a` |
| Save changes to image | `docker commit ros_noetic ros_noetic:nav` |
| Remove and recreate | `docker rm ros_noetic` then `docker run ...` (see above) |

---

## Table of Contents

1. [Quick Start — Docker Environment](#quick-start--docker-environment)
2. [Branch Architecture](#branch-architecture)
2. [Hardware Configuration](#hardware-configuration)
3. [Repository & Package Structure](#repository--package-structure)
4. [Prerequisites & Dependencies](#prerequisites--dependencies)
5. [Build](#build)
6. [Real Robot Deployment](#real-robot-deployment)
   - [Network Setup](#network-setup)
   - [Startup Order](#startup-order)
   - [Option A — Unitree L1 (Primary)](#option-a--unitree-l1-primary)
   - [Option B — SICK LMS200 (Backup)](#option-b--sick-lms200-backup)
7. [Trash Detection Bridge (demo-v1)](#trash-detection-bridge-demo-v1)
8. [Simulation (Development Only)](#simulation-development-only)
9. [Key Topics & TF Reference](#key-topics--tf-reference)
10. [Parameter Tuning Guide](#parameter-tuning-guide)
11. [Known Issues and Notes](#known-issues-and-notes)
12. [Git Workflow](#git-workflow)
13. [Resources](#resources)
14. [Status](#status)
15. [Post-Installation Checklist](#post-installation-checklist)

---

## Branch Architecture

### Runtime Split

```
┌─────────────────────────────────────────────────────┐
│   JETSON ORIN  (192.168.50.1)                       │
│                                                     │
│  ┌───────────────────────────────────────────────┐  │
│  │  ROS Noetic Docker  (--net=host)              │  │
│  │                                               │  │
│  │  roscore                                      │  │
│  │  robot_state_publisher  ──►  /tf_static       │  │
│  │  unitree_lidar_ros      ──►  /unitree/scan    │  │  ← Primary sensor
│  │  slam_gmapping          ──►  /map, /tf        │  │
│  │  move_base              ──►  /cmd_vel         │  │
│  │  amcl                   ──►  /tf(map→odom)    │  │
│  │  udp_target_bridge      ──►  /trash_detection/│  │
│  │                               target_point    │  │
│  │  point_to_target_pose   ──►  /target_pose     │  │
│  │  target_follower        ──►  MoveBaseGoal     │  │
│  └───────────────────────────────────────────────┘  │
│          ▲  UDP JSON  127.0.0.1:${TRASH_UDP_PORT}   │
│  ┌───────┴───────────────────────────────────────┐  │
│  │  Host — Native Ubuntu 22.04                   │  │
│  │  predict_15cls_rgbd.py  (non-ROS, GPU)        │  │
│  │  Orbbec Femto Bolt SDK                        │  │
│  └───────────────────────────────────────────────┘  │
└──────────────────┬──────────────────────────────────┘
                   │  Gigabit Ethernet  192.168.50.0/24
                   │  /cmd_vel → Pi  |  /odom, /tf ← Pi
┌──────────────────┴──────────────────────────────────┐
│   RASPBERRY PI 4  (192.168.50.2)                    │
│                                                     │
│  ┌───────────────────────────────────────────────┐  │
│  │  ROS Noetic Docker  (--net=host)              │  │
│  │  rosaria  ◄── /cmd_vel                        │  │
│  │           ──► /odom, /tf(odom→base_link)      │  │
│  └───────────────────────────────────────────────┘  │
│   Serial USB → P3-AT chassis controller             │
└─────────────────────────────────────────────────────┘
```

### Key Topic Flow

```
Host detector (UDP) → udp_target_bridge → /trash_detection/target_point
    → point_to_target_pose → /target_pose → target_follower → /move_base (action)
    → /cmd_vel → rosaria (Pi) → /RosAria/pose → /odom

unitree_lidar_ros → /unitree/scan → slam_gmapping → /map
                                  → move_base → /cmd_vel
```

### Sensor Stack Comparison

| | Stack A — Unitree (Primary) | Stack B — SICK (Backup) |
|---|---|---|
| **Sensor** | Unitree 4D Lidar L1 | SICK LMS200 |
| **FOV** | 360° | 180° |
| **Max range** | 30 m | 80 m |
| **Scan topic** | `/unitree/scan` | `/scan` |
| **Sensor frame** | `unitree_lidar` | `laser` |
| **Param dir** | `param/unitree/` | `param/` |
| **Priority** | **Use this first** | Fallback if Unitree unavailable |

---

## Hardware Configuration

### Robot: Pioneer 3-AT

| Property | Value |
|----------|-------|
| Drive | 4-wheel skid-steer |
| Footprint | `[[0.32, 0.27], [0.32, -0.27], [-0.32, -0.27], [-0.32, 0.27]]` m |
| Inscribed radius | 0.27 m |

### Sensor A: Unitree 4D Lidar L1 (Primary)

| Property | Value |
|----------|-------|
| FOV | 360° horizontal |
| Range | 0.05 – 30 m |
| Scan frequency | ~10 Hz |
| ROS topic | `/unitree/scan` (`sensor_msgs/LaserScan`) |
| Additional topics | `/unitree/cloud` (PointCloud2), `/unitree/imu` (Imu) |
| TF frame | `unitree_lidar` |
| Interface | USB Type-C (serial) |
| Driver | `unitree_lidar_ros` (compiled from `unilidar_sdk`) |

### Sensor B: SICK LMS200 (Backup)

| Property | Value |
|----------|-------|
| FOV | 180° horizontal |
| Range | 0.1 – 80 m |
| Scan frequency | ~75 Hz |
| ROS topic | `/scan` (`sensor_msgs/LaserScan`) |
| TF frame | `laser` |
| Interface | RS-232/RS-422 serial |
| Driver | `sicktoolbox_wrapper` (built from source in `catkin_ws/src/`) |

### Compute

| Node | Hardware | Role |
|------|----------|------|
| Jetson | NVIDIA Jetson Orin Nano | ROS master, SLAM, navigation, Unitree driver |
| Pi | Raspberry Pi 4 | P3-AT base driver (RosAria) |

---

## Repository & Package Structure

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── catkin_ws/src/
│   ├── p3at_base/                    # Pi-side P3-AT base driver
│   │   ├── launch/base.launch
│   │   └── scripts/odom_republisher.py
│   ├── p3at_lms_description/         # URDF/Xacro robot models
│   │   ├── urdf/p3at_lms.urdf.xacro          (SICK model)
│   │   ├── urdf/p3at_unitree.urdf.xacro       (Unitree model)
│   │   └── urdf/unitree_lidar_l1.urdf.xacro   (Unitree sensor macro)
│   ├── p3at_lms_gazebo/              # Gazebo worlds and launch (dev only)
│   │   ├── launch/sim.launch
│   │   ├── launch/sim_unitree.launch
│   │   └── worlds/complex_maze.world
│   ├── p3at_lms_navigation/          # Navigation stack (both sensors)
│   │   ├── launch/
│   │   │   ├── real_robot_mapping_unitree.launch  ← PRIMARY (real robot)
│   │   │   ├── real_robot_nav_unitree.launch       ← PRIMARY (real robot)
│   │   │   ├── real_robot_mapping.launch           (SICK backup)
│   │   │   ├── real_robot_nav.launch               (SICK backup)
│   │   │   ├── auto_mapping_unitree.launch         (sim dev)
│   │   │   ├── auto_mapping.launch                 (sim dev)
│   │   │   ├── auto_amcl_verify_unitree.launch     (sim dev)
│   │   │   ├── auto_amcl_verify.launch             (sim dev)
│   │   │   ├── mapping_unitree.launch              (sim dev)
│   │   │   ├── mapping.launch                      (sim dev)
│   │   │   ├── nav_unitree.launch                  (sim dev)
│   │   │   └── nav.launch                          (sim dev)
│   │   ├── param/                    # SICK default parameters
│   │   ├── param/unitree/            # Unitree-tuned parameters
│   │   │   ├── gmapping.yaml         # maxUrange: 10.0
│   │   │   ├── costmap_common.yaml   # inflation 0.45 / scale 5.0
│   │   │   ├── global_costmap.yaml   # source: /unitree/scan
│   │   │   ├── local_costmap.yaml    # inflation 0.35 (split from global)
│   │   │   ├── move_base.yaml        # clearing_rotation_allowed: false
│   │   │   └── amcl.yaml
│   │   ├── rviz/nav_unitree.rviz     # Unitree RViz config
│   │   └── scripts/
│   │       ├── autonomous_explorer.py
│   │       ├── amcl_verifier.py
│   │       └── waypoint_test.py
│   ├── target_follower/
│   │   └── scripts/
│   │       ├── target_follower.py          # /target_pose → MoveBaseGoal
│   │       ├── udp_target_bridge.py        # UDP JSON → /trash_detection/target_point
│   │       ├── point_to_target_pose.py     # PointStamped → PoseStamped
│   │       ├── mock_target_point_publisher.py
│   │       ├── gazebo_target_publisher.py  # sim dev only
│   │       ├── move_target.py              # sim dev only
│   │       └── test_standoff_face.py       # unit tests
│   ├── sicktoolbox/                  # SICK C++ library (source)
│   └── sicktoolbox_wrapper/          # SICK ROS wrapper (source)
├── tools/
│   ├── source_ros.sh / source_ros.zsh
│   ├── camera_info_pub.py
│   ├── inspect_depth_once.py
│   └── relay_camera_info.py
├── setup_unitree_lidar.sh            # Unitree SDK install helper
├── run_full_pipeline_unitree.sh      # Sim: Unitree mapping → AMCL verify
├── run_full_pipeline.sh              # Sim: SICK mapping → AMCL verify
├── build_and_hint.sh
└── doc.md                            # Demo v1 runbook
```

---

## Prerequisites & Dependencies

### Jetson Docker — ROS Navigation Packages

The base image (`ros:noetic-ros-base`) does **not** include navigation packages. Install inside the container:

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-noetic-slam-gmapping \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-map-msgs \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-xacro \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-actionlib \
  ros-noetic-rviz \
  ros-noetic-teleop-twist-keyboard \
  build-essential cmake libboost-dev
```

> No Gazebo packages needed for real-robot deployment.

### Unitree L1 Driver (compile from source, inside Jetson Docker)

```bash
cd catkin_ws/src
git clone https://github.com/unitreerobotics/unilidar_sdk.git
ln -sf unilidar_sdk/unitree_lidar_ros unitree_lidar_ros
cd ../.. && catkin_make
```

Or use the provided helper script:

```bash
./setup_unitree_lidar.sh
```

> ⚠️ The Unitree SDK needs **kernel modules compiled for your Jetson kernel**.  
> Allow ~30 min on first setup. Jetson must be running its actual kernel (not a cross-compiled one).

### Pi Docker

```bash
sudo apt-get install -y ros-noetic-rosaria
# Or build from source: https://github.com/amor-ros-pkg/rosaria
```

### Jetson Native Ubuntu 22.04 (Trash Detection)

```bash
pip3 install ultralytics opencv-python numpy scipy pillow
# Orbbec Femto Bolt SDK: https://github.com/orbbec/OrbbecSDK_ROS1
```

---

## Build

```bash
cd catkin_ws
catkin_make
source devel/setup.zsh   # or devel/setup.bash
```

Verify:

```bash
rospack find p3at_lms_navigation   # should print path
rospack find unitree_lidar_ros     # must exist before real-robot use
```

Source helper (every new terminal):

```bash
source tools/source_ros.zsh   # zsh
source tools/source_ros.sh    # bash
```

---

## Real Robot Deployment

### Network Setup

Direct Gigabit Ethernet link between Jetson and Pi on `192.168.50.0/24`.

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

Configure IPs in `scripts/deploy.env` (see `doc.md`):
```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
TRASH_UDP_PORT=16031
```

Verify connectivity:
```bash
ping 192.168.50.2          # from Jetson
rosnode list               # from Pi — should return /rosout
```

> Both Docker containers **must** use `--net=host`.

### Startup Order

```
1. Jetson Docker:  roscore
2. Pi Docker:      roslaunch p3at_base base.launch
3. Jetson Docker:  roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch
                   (or real_robot_nav_unitree.launch for navigation on saved map)
4. Jetson Docker:  [optional] start udp_target_bridge for trash detection
```

---

### Option A — Unitree L1 (Primary)

> **Use for all real-robot deployments unless Unitree hardware is unavailable.**

#### Pi — Start Base Driver

```bash
roslaunch p3at_base base.launch
```

Publishes: `/odom`, `/tf (odom→base_link)`, `/battery_voltage`  
Subscribes: `/cmd_vel`

#### Jetson — Mapping

```bash
roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch
```

| Argument | Default | Description |
|----------|---------|-------------|
| `unitree_port` | `/dev/ttyUSB0` | Unitree L1 USB port |
| `use_rviz` | `true` | Open RViz |
| `use_target_follower` | `false` | Enable target following during mapping |

Starts: `unitree_lidar_ros` + `robot_state_publisher` + `slam_gmapping` + `move_base` + RViz.

**Optional — autonomous frontier exploration:**

```bash
rosrun p3at_lms_navigation autonomous_explorer.py \
  _exploration_timeout:=300 \
  _robot_radius:=0.25 \
  _goal_timeout:=30.0
```

**Save map:**

```bash
rosrun map_server map_saver -f $HOME/maps/real_robot_map
```

#### Jetson — Navigation

```bash
roslaunch p3at_lms_navigation real_robot_nav_unitree.launch \
  map_file:=$HOME/maps/real_robot_map.yaml
```

1. RViz → **"2D Pose Estimate"** → set initial robot pose
2. Wait for AMCL particle cloud to converge
3. RViz → **"2D Nav Goal"**, or:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

#### Unitree Parameter Files

| File | Key Values |
|------|-----------|
| `param/unitree/costmap_common.yaml` | `inflation_radius: 0.45`, `cost_scaling_factor: 5.0` |
| `param/unitree/global_costmap.yaml` | Obstacle source: `/unitree/scan`, frame `unitree_lidar` |
| `param/unitree/local_costmap.yaml` | `inflation_radius: 0.35` (smaller than global) |
| `param/unitree/move_base.yaml` | `clearing_rotation_allowed: false` (**anti-tip — do not change**) |
| `param/unitree/gmapping.yaml` | `maxUrange: 10.0` (30 m sensor) |

---

### Option B — SICK LMS200 (Backup)

> **Use only if Unitree hardware is unavailable.**

```bash
sudo chmod 666 /dev/ttyUSB0
roslaunch p3at_lms_navigation real_robot_mapping.launch    # mapping
roslaunch p3at_lms_navigation real_robot_nav.launch \     # navigation
  map_file:=$HOME/maps/sick_map.yaml
```

LMS200 firmware settings to verify: baud 38400–500000 bps, measuring units cm, resolution 0.5°, FOV 180°.

---

## Trash Detection Bridge (demo-v1)

The detection pipeline runs on **Jetson native Ubuntu 22.04** (not inside Docker) because the Orbbec SDK requires kernel-level USB access.

### Pipeline

```
[Jetson — Native Ubuntu 22.04]
  predict_15cls_rgbd.py
  → UDP JSON → 127.0.0.1:${TRASH_UDP_PORT}   (default 16031)
      {"x": 1.2, "y": -0.3, "z": 0.0, "frame_id": "camera_link", "stamp": ...}

[Jetson — ROS Docker]
  udp_target_bridge.py
  ← UDP JSON on 0.0.0.0:16031
  → /trash_detection/target_point  (geometry_msgs/PointStamped)

  point_to_target_pose.py
  ← /trash_detection/target_point
  → /target_pose  (geometry_msgs/PoseStamped)

  target_follower.py
  ← /target_pose
  → MoveBaseAction goal → move_base → /cmd_vel
```

### Starting the Bridge (Jetson Docker)

```bash
rosrun target_follower udp_target_bridge.py \
  _bind_port:=16031 \
  _default_frame:=camera_link

rosrun target_follower point_to_target_pose.py \
  _in_point_topic:=/trash_detection/target_point \
  _out_target_topic:=/target_pose
```

### Manual Test (no hardware)

```bash
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(json.dumps({'x':2.0,'y':0.0,'z':0.0,
                     'frame_id':'map','stamp':time.time()}).encode(),
         ('127.0.0.1', 16031))"
# Then check: rostopic echo /target_pose
```

---

## Simulation (Development Only)

> Production deployment does **not** use Gazebo. Use simulation for algorithm development and parameter tuning on a developer machine.

### Autonomous Exploration

```bash
# Unitree stack (primary)
roslaunch p3at_lms_navigation auto_mapping_unitree.launch exploration_timeout:=300 gui:=true

# SICK stack (backup)
roslaunch p3at_lms_navigation auto_mapping.launch exploration_timeout:=300 gui:=true
```

### AMCL Verification

```bash
roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true
```

### Full Pipeline Script

```bash
bash run_full_pipeline_unitree.sh   # Unitree: map → AMCL verify
bash run_full_pipeline.sh           # SICK: map → AMCL verify
```

### Unit Tests (no ROS required)

```bash
python3 catkin_ws/src/target_follower/scripts/test_standoff_face.py
# Expected: 21/21 PASS
```

---

## Key Topics & TF Reference

### Core Topics — Unitree Stack (Real Robot)

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/unitree/scan` | `sensor_msgs/LaserScan` | `unitree_lidar_ros` | `slam_gmapping`, `move_base`, RViz |
| `/unitree/cloud` | `sensor_msgs/PointCloud2` | `unitree_lidar_ros` | (optional) |
| `/unitree/imu` | `sensor_msgs/Imu` | `unitree_lidar_ros` | (optional) |
| `/odom` | `nav_msgs/Odometry` | `rosaria` (Pi) | `move_base` |
| `/cmd_vel` | `geometry_msgs/Twist` | `move_base` | `rosaria` (Pi) |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_gmapping` / `map_server` | `move_base`, RViz |
| `/trash_detection/target_point` | `geometry_msgs/PointStamped` | `udp_target_bridge` | `point_to_target_pose` |
| `/target_pose` | `geometry_msgs/PoseStamped` | `point_to_target_pose` | `target_follower` |
| `/move_base/goal` | `MoveBaseActionGoal` | `target_follower` | `move_base` |

### TF Tree (Real Robot — Unitree)

```
map
└── odom               [slam_gmapping (mapping) or amcl (nav)  ~20 Hz]
    └── base_footprint [rosaria on Pi  ~50 Hz]
        └── base_link  [robot_state_publisher  static]
            └── unitree_lidar  [robot_state_publisher  static]
```

| Edge | Broadcaster | Notes |
|------|-------------|-------|
| `map → odom` | `slam_gmapping` or `amcl` | Switches between mapping / nav phases |
| `odom → base_footprint` | `rosaria` (Pi) | Cross-machine via TCPROS |
| `base_footprint → base_link` | `robot_state_publisher` | Static, identity |
| `base_link → unitree_lidar` | `robot_state_publisher` | Static, mount offset from URDF |

**SICK stack:** identical with `laser` replacing `unitree_lidar`, `/scan` replacing `/unitree/scan`.

---

## Parameter Tuning Guide

### Why Split Global/Local Costmap Inflation

Global costmap (`inflation_radius: 0.45 m`) → NavFn plans a conservative path away from walls.  
Local costmap (`inflation_radius: 0.35 m`) → DWA has more room to find trajectories in tight corridors.  
Without this split, DWA frequently aborts in corridors even when the global plan is valid.

### `move_base.yaml` — Critical Settings (Unitree)

```yaml
clearing_rotation_allowed: false   # SAFETY: prevents P3-AT tipping with Unitree on top
DWAPlannerROS:
  max_vel_x: 0.4
  max_vel_theta: 0.4
  vx_samples: 20            # better trajectory search in corridors
  vtheta_samples: 40        # richer turning decisions
  xy_goal_tolerance: 0.55   # generous for large robot
  occdist_scale: 0.02       # allows close approach in tight spaces
NavfnROS:
  allow_unknown: true        # plan through unknown cells (essential for exploration)
  default_tolerance: 0.3
```

### `gmapping.yaml` — Unitree vs SICK

| Parameter | SICK | Unitree | Reason |
|-----------|------|---------|--------|
| `maxUrange` | 8.0 m | 10.0 m | Unitree L1 range is 30 m |
| `particles` | 30 | 30 | Same |
| `delta` | 0.05 m | 0.05 m | Same map resolution |

### `target_follower` — Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~standoff_distance` | `0.0` m | Stop this far short of target |
| `~face_target` | `false` | Orient robot toward target at goal |
| `~send_rate_hz` | `2.0` Hz | Max rate for new goals |
| `~min_update_dist` | `0.3` m | Min target movement before re-sending goal |
| `~target_timeout` | `2.0` s | Stale target timeout — cancels move_base goal |

---

## Known Issues and Notes

- **`clearing_rotation_allowed: false` is safety-critical**: In-place rotation recovery has caused the P3-AT to tip over with Unitree L1 on top (high CoM). Do not re-enable without anti-tip analysis.

- **Unitree kernel module on Jetson**: The SDK requires kernel modules compiled for your specific Jetson kernel. Allow ~30 min on first setup (`./setup_unitree_lidar.sh`).

- **Narrow corridor margins**: P3-AT is ~0.54 m wide; after inflation (0.35–0.45 m), margins in 1 m corridors are very tight. Reduce `inflation_radius` to 0.30 m if the robot gets stuck frequently, but expect more wall contacts.

- **Pi `/odom` latency**: Cross-machine TCPROS `/odom` has ~1–5 ms latency. If TF lookup fails, increase `transform_tolerance` in costmap YAMLs (currently 0.4 s).

- **`gazebo_target_publisher.py` and `move_target.py`** are simulation-only and are **not** started by real-robot launch files.

- **Conda environments**: Avoid nodes requiring `PyKDL`. All scripts in this branch (`udp_target_bridge.py`, `target_follower.py`, `autonomous_explorer.py`) use pure-Python math only.

---

## Git Workflow

### Branch Strategy

| Branch | Purpose |
|--------|---------|
| `main` | Simulation development + algorithm validation |
| `release/demo-v1` | Real-robot deployment (this branch) |

### What's Tracked

- `catkin_ws/src/` — all package source
- `param/` and `param/unitree/` YAML files
- `tools/` helper scripts
- `doc.md`, `README.md`

### What's Ignored

- `catkin_ws/build/`, `catkin_ws/devel/`, `ros_ws/build/`, `ros_ws/devel/`
- Maps: `*.pgm`, `*.yaml` in `maps/`
- `Log/`, `__pycache__/`, `.catkin_workspace`

### Setup After Clone

```bash
git clone <repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial
git checkout release/demo-v1
git submodule update --init --recursive
cd catkin_ws && catkin_make
source devel/setup.zsh
```

---

## Resources

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [GMapping SLAM](http://wiki.ros.org/gmapping)
- [AMCL Localization](http://wiki.ros.org/amcl)
- [move_base Navigation](http://wiki.ros.org/move_base)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)
- [sicktoolbox_wrapper (LMS200)](http://wiki.ros.org/sicktoolbox_wrapper)
- [RosAria (P3-AT driver)](http://wiki.ros.org/ROSARIA)
- [Orbbec Femto Bolt SDK](https://github.com/orbbec/OrbbecSDK_ROS1)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [Docker Host Networking](https://docs.docker.com/network/host/)

---

## Status

### Simulation (verified on `main`, available in this branch via merge)
- [x] Gazebo simulation (gmapping + move_base + target following) — verified
- [x] Autonomous frontier exploration — best 12.6% coverage / 300 s in `complex_maze.world`
- [x] AMCL accuracy verifier — mean pos error 0.089 m, convergence 1.0 s
- [x] `standoff_distance` + `face_target` — 21/21 unit tests pass
- [x] Dynamic moving target (`move_target.py`) — verified in Gazebo
- [x] Unitree costmap split inflation (0.45 vs 0.35 m) — done
- [x] `clearing_rotation_allowed: false` — anti-tip, done
- [x] `param/unitree/` parameter directory — created and tuned

### Real Robot (demo-v1)
- [x] Real robot launch files — all 4 variants (mapping + nav, Unitree + SICK) — created
- [x] Raspberry Pi base driver (`p3at_base`) — created
- [x] UDP trash detection bridge (`udp_target_bridge.py` + `point_to_target_pose.py`) — created
- [x] Multi-machine network architecture — documented
- [ ] `unitree_lidar_ros` compiled in Jetson Docker — **pending**
- [ ] Docker navigation ROS packages installed — **pending**
- [ ] `tools/yolo_target_detector.py` — not started
- [ ] End-to-end YOLO → `/target_pose` → `target_follower` real-robot test — not started
- [ ] Real hardware parameter tuning — not started
- [ ] End-to-end real robot navigation test — Unitree (primary) — not started
- [ ] End-to-end real robot navigation test — SICK (backup) — not started

---

## Post-Installation Checklist

### Build
- [ ] `cd catkin_ws && catkin_make` — no errors
- [ ] `source devel/setup.zsh && rospack find p3at_lms_navigation` — path returned
- [ ] `rospack find unitree_lidar_ros` — found (requires SDK clone + build)

### Docker Environment (Jetson)
- [ ] Navigation ROS packages installed (`gmapping`, `move_base`, `amcl`, `rviz` …)
- [ ] `--net=host` confirmed: `docker inspect -f '{{.HostConfig.NetworkMode}}' <container>`
- [ ] `roscore` running: `ss -lntp | grep 11311`

### Network
- [ ] Jetson `eth0`: `192.168.50.1`
- [ ] Pi `eth0`: `192.168.50.2`
- [ ] Ping both directions succeed
- [ ] `rosnode list` from Pi returns `/rosout`

### Unitree Hardware
- [ ] `ls /dev/ttyUSB*` shows device after connecting Unitree L1
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch` — no errors
- [ ] `rostopic hz /unitree/scan` — ~10 Hz
- [ ] RViz shows `unitree_lidar` frame and scan data

### Trash Detection Bridge
- [ ] `python3 tools/inspect_depth_once.py` — Orbbec frame received (native host)
- [ ] UDP mock send → `rostopic echo /trash_detection/target_point` — message received
- [ ] `rostopic echo /target_pose` — publishing when UDP messages arrive

### Navigation (Unitree)
- [ ] `rostopic hz /map` — map building during mapping launch
- [ ] Map saved: `rosrun map_server map_saver -f ~/maps/unitree_map`
- [ ] AMCL particle cloud converges after "2D Pose Estimate"
- [ ] Navigation goal SUCCEEDED

### Navigation (SICK — backup only)
- [ ] `roslaunch p3at_lms_navigation real_robot_mapping.launch` — no errors
- [ ] `rostopic hz /scan` — ~75 Hz
- [ ] Map saved and navigation goal SUCCEEDED
