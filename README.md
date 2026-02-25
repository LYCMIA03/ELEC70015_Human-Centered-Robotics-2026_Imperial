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
| `ros_noetic` | `nav_unitree` **← use this** | arm64 (Jetson native) | ~4.45 GB | ROS Noetic desktop-full + all nav packages + **`unitree_lidar_ros` pre-compiled** + git/build-essential |
| `ros_noetic` | `nav` | arm64 (Jetson native) | ~4.41 GB | ROS Noetic desktop-full + all nav packages (no Unitree driver) |

### Start the Container

Use the `ros_noetic` management script (located at `~/.fishros/bin/ros_noetic`).
It creates the container with the **correct non-root user** (`--user $(id -u):$(id -g)`), mounts `/dev`, `/home/frank/work`, and sets `--privileged --net=host` automatically.

```bash
# Auto-create (if not exists) or start (if stopped)
ros_noetic s
```

> **Manual fallback** — only if the script is unavailable:
> ```bash
> docker run -d --net=host --privileged \
>   --user $(id -u):$(id -g) \
>   --runtime nvidia \
>   -e HOME=$HOME \
>   -v /tmp/.X11-unix:/tmp/.X11-unix \
>   -v /home/frank/work:/home/frank/work \
>   -v /dev:/dev \
>   -v /etc/passwd:/etc/passwd:ro \
>   -v /etc/group:/etc/group:ro \
>   --name ros_noetic \
>   ros_noetic:nav_unitree bash
> ```

### Enter a Running Container

```bash
# Enter as current (non-root) user — starts container first if stopped
ros_noetic e
```

> `ros_noetic e` = `docker exec -it --user $(id -u):$(id -g) ros_noetic /bin/bash`  
> Always use `ros_noetic e` (not bare `docker exec`) to avoid entering as root.

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
docker commit ros_noetic ros_noetic:nav_unitree
```

### Container Lifecycle Cheat Sheet

| Action | `ros_noetic` script | Direct docker equivalent |
|--------|--------------------|--------------------------|
| **Start / create** | `ros_noetic s` | `docker run ...` (see above) |
| **Enter (non-root)** | `ros_noetic e` | `docker exec -it --user $(id -u):$(id -g) ros_noetic bash` |
| **Restart** | `ros_noetic r` | `docker restart ros_noetic` |
| **Stop** | `ros_noetic c` | `docker stop ros_noetic` |
| **Delete** | `ros_noetic d` | `docker stop ros_noetic && docker rm ros_noetic` |
| Check status | — | `docker ps -a` |
| Save changes to image | — | `docker commit ros_noetic ros_noetic:nav_unitree` |

> **⚠️ Never use** `docker exec -it ros_noetic bash` without `--user` — that enters as **root** and will cause file permission issues.

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
   - [**Keyboard Teleoperation + Unitree Mapping — Full Runbook**](#keyboard-teleoperation--unitree-mapping--full-runbook)
   - [**Autonomous Exploration SLAM — Full Runbook**](#autonomous-exploration-slam--full-runbook)
   - [**Target Following (Real Robot Demo) — Full Runbook**](#target-following-real-robot-demo--full-runbook)
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
│  │  slam_gmapping*         ──►  /map, /tf        │  │  * mapping mode only
│  │  move_base              ──►  /cmd_vel         │  │
│  │  amcl*                  ──►  /tf(map→odom)    │  │  * nav-on-map mode only
│  │  udp_target_bridge      ──►  /trash_detection/│  │
│  │                               target_point    │  │
│  │  point_to_target_pose   ──►  /target_pose     │  │
│  │  target_follower        ──►  MoveBaseGoal     │  │
│  └───────────────────────────────────────────────┘  │
│          ▲  UDP JSON  127.0.0.1:${TRASH_UDP_PORT}   │
│  ┌───────┴───────────────────────────────────────┐  │
│  │  Host — Native Ubuntu 22.04                   │  │
│  │  handobj_detection_rgbd.py  (non-ROS, GPU)    │  │
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
[Host] handobj_detection_rgbd.py  ──UDP 16031──►  udp_target_bridge
  → /trash_detection/target_point  (frame: camera_link, optical coords)
  → point_to_target_pose  → /target_pose  (frame: camera_link)
  → target_follower  → /move_base (action)  → /cmd_vel
  → rosaria (Pi)  → P3-AT chassis

unitree_lidar_ros → /unitree/scan
  → slam_gmapping* → /map + map→odom TF    (* mapping mode)
  → move_base → /cmd_vel                   (costmap obstacle source)

Static TF:  base_link → camera_link  (quat: -0.5 0.5 -0.5 0.5)
            optical z(depth/fwd) → base +x(fwd), optical x(right) → base -y, optical y(down) → base -z
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
│   ├── sicktoolbox_wrapper/          # SICK ROS wrapper (source)
│   └── unilidar_sdk/                 # Unitree SDK (git-cloned on Jetson, not tracked in this repo)
│       ├── unitree_lidar_ros/src/unitree_lidar_ros/  ← ROS1 package (catkin finds automatically)
│       ├── unitree_lidar_ros2/       (CATKIN_IGNORE)
│       └── unitree_lidar_sdk/        (CATKIN_IGNORE — aarch64 libunitree_lidar_sdk.a inside)
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

> **Already included in `ros_noetic:nav_unitree`.** No manual installation needed.

The image is based on `ghcr.io/sloretz/ros:noetic-desktop-full` (arm64) and contains all required packages:
`ros-noetic-slam-gmapping`, `ros-noetic-move-base`, `ros-noetic-dwa-local-planner`, `ros-noetic-navfn`,
`ros-noetic-amcl`, `ros-noetic-map-server`, `ros-noetic-robot-state-publisher`, `ros-noetic-xacro`,
`ros-noetic-tf2-ros`, `ros-noetic-actionlib`, `ros-noetic-rviz`, `ros-noetic-pcl-ros`,
`build-essential`, `cmake`, `git`.

> If you need to rebuild the image from scratch, base it on `ghcr.io/sloretz/ros:noetic-desktop-full` (arm64, **not** `osrf/ros:noetic-desktop-full` which is amd64).

### Unitree L1 Driver (pre-compiled in `nav_unitree` image)

> **Already compiled in `ros_noetic:nav_unitree`.** The steps below are for reference or if you need to rebuild.

The SDK ships a pre-built `libunitree_lidar_sdk.a` for `aarch64` — no kernel module compilation needed.

```bash
# Inside Docker container (one-time setup if rebuilding from scratch)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src
git clone https://github.com/unitreerobotics/unilidar_sdk.git

# Prevent non-catkin subdirectories from breaking catkin_make
touch unilidar_sdk/unitree_lidar_ros2/CATKIN_IGNORE
touch unilidar_sdk/unitree_lidar_sdk/CATKIN_IGNORE
# catkin finds unitree_lidar_ros automatically via recursive scan (no symlink needed)

source /opt/ros/noetic/setup.bash
cd .. && catkin_make
```

The SDK directory layout after clone:
```
unilidar_sdk/
├── unitree_lidar_ros/src/unitree_lidar_ros/   ← ROS1 package (catkin discovers automatically)
├── unitree_lidar_ros2/                         ← CATKIN_IGNORE applied
├── unitree_lidar_sdk/                          ← CATKIN_IGNORE applied
│   └── lib/aarch64/libunitree_lidar_sdk.a      ← pre-built static library
└── docs/
```

Or use the provided helper (runs the above steps plus udev setup):

```bash
./setup_unitree_lidar.sh
```

### Unitree L1 USB — Udev Rule (Jetson Host — already installed)

> **Already created on Jetson host.** Listed here for reference.

File: `/etc/udev/rules.d/99-unitree-lidar.rules`
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="unitree_lidar"
```

After connecting Unitree L1 via USB, the device appears as both `/dev/ttyUSB0` and `/dev/unitree_lidar`.

```bash
# Verify on host
ls -la /dev/unitree_lidar    # should be a symlink to /dev/ttyUSBx
```

The `frank` user is already in the `dialout` group (no logout required).

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

**Option A — Target Following Demo (standalone, no map needed):**
```
1. Pi:              roslaunch p3at_base base.launch
2. Jetson Docker:   roscore
3. Jetson Docker:   roslaunch target_follower target_follow_real.launch launch_move_base:=true
4. Jetson Host:     python3 handobj_detection/handobj_detection_rgbd.py --udp-enable ...
   (or one-command: ./scripts/start_demo.sh)
```

**Option B — Mapping:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      roslaunch p3at_base base.launch
3. Jetson Docker:  roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch
4. Jetson Docker:  [optional] start udp_target_bridge for trash detection
```

**Option C — Navigation on saved map:**
```
1. Jetson Docker:  roscore
2. Pi Docker:      roslaunch p3at_base base.launch
3. Jetson Docker:  roslaunch p3at_lms_navigation real_robot_nav_unitree.launch map_file:=...
4. Jetson Docker:  roslaunch target_follower target_follow_real.launch launch_move_base:=false
5. Jetson Host:    python3 handobj_detection/handobj_detection_rgbd.py --udp-enable ...
```

---

### Module Startup Flows (Merged from `doc.md`)

> This section is the module-by-module startup entry.
> Docker operation is unified to `ros_noetic` helper commands.

#### Docker convention (Jetson)

```bash
ros_noetic s   # start/create container
ros_noetic e   # enter container shell (non-root)
ros_noetic r   # restart container
ros_noetic c   # stop container
```

Inside `ros_noetic e`, source as needed:

```bash
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
```

#### Flow 1: Navigation + RasPi communication

```bash
# Jetson
./scripts/start_master.sh jetson

# Raspberry Pi
./scripts/start_base.sh

# Jetson (mapping or nav)
./scripts/start_real_mapping.sh
# or
./scripts/start_real_nav.sh map_file:=/absolute/path/to/map.yaml
```

#### Flow 2: Trash detection + UDP bridge

ROS side in container:

```bash
ros_noetic e
source /opt/ros/noetic/setup.bash
roscore
```

Open another terminal:

```bash
ros_noetic e
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/udp_target_bridge.py _bind_port:=16031
```

Optional relay terminal:

```bash
ros_noetic e
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/point_to_target_pose.py
```

Host detector:

```bash
./scripts/start_trash_detection_rgbd.sh
# or hand-object mode
./scripts/start_trash_detection_rgbd.sh --detector handobj
```

#### Flow 3: Dialogue action bridge

The dialogue integration uses two UDP bridge scripts that **run on the Jetson host** (not inside Docker):
- **`navigation_success_udp_bridge`** — ROS subscriber: watches `/target_follower/result`, on `True` → UDP:16041 → dialogue
- **`udp_trash_action_bridge`** — UDP listener on port 16032 → publishes `/trash_action` (Bool) to ROS

The `target_follower` node subscribes to `/trash_action` directly and handles the result:
- `True` → human accepted → `IDLE` (resume following)
- `False` → human refused → `RETREATING` (navigate `retreat_distance` m away) → `IDLE`

Jetson host (non-Docker terminal):

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_docker_bridges.sh
```

Dialogue device:

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_host.sh --device 24
```

Trigger test (simulate REACHED → dialogue → refuse → retreat):

```bash
ros_noetic e
source /opt/ros/noetic/setup.bash
# Simulate REACHED (triggers dialogue)
rostopic pub -1 /target_follower/result std_msgs/Bool "data: true"
# Simulate human refusing (triggers retreat)
rostopic pub -1 /trash_action std_msgs/Bool "data: false"
# Simulate human accepting (resume)
rostopic pub -1 /trash_action std_msgs/Bool "data: true"
```

#### Flow 4: Lidar-only

Lidar-only workflow is intentionally TBD in this release branch.

---

### Keyboard Teleoperation + Unitree Mapping — Full Runbook

> **This is the verified real-robot procedure** (tested 2026-02-24).  
> Goal: drive the robot manually with keyboard while Unitree L1 LiDAR builds a `gmapping` occupancy map in real time.  
> Everything runs inside the `ros_noetic` Docker container on Jetson unless stated otherwise.

#### Hardware checklist before starting

| Item | Check |
|------|-------|
| Jetson Orin Nano powered on | ✓ |
| P3-AT powered on, serial cable connected to Raspberry Pi (`/dev/ttyS0` or `/dev/ttyUSB0`) | ✓ |
| Raspberry Pi powered on, connected to Jetson via Ethernet (`192.168.50.0/24`) | ✓ |
| Unitree L1 LiDAR connected to Jetson via USB-C, **but DO NOT power it on yet** | ✓ |
| `docker ps` shows `ros_noetic` container running on Jetson | ✓ |

> **⚠️ Important:** Power on the Unitree LiDAR **only after** `roslaunch` has already started (Step 3). If the LiDAR is powered on before the driver node initialises, the nodes may register with rosmaster on startup and then lose connection when the LiDAR re-enumerates on power-up, causing all nodes to silently drop off `/rosnode list`. If this happens, see [Recovery: nodes disappeared from rosnode list](#recovery-nodes-disappeared-from-rosnode-list) below.

---

#### Step 1 — Raspberry Pi: start the chassis driver

**Device:** Raspberry Pi 4  
**Terminal:** SSH into the Pi (`ssh frank@192.168.50.2`) or open a terminal directly on it.

```bash
# On the Raspberry Pi
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

What this does:
- Sources `scripts/env.sh` with `role=raspi`, `master=jetson`  
- Sets `ROS_MASTER_URI=http://192.168.50.1:11311`, `ROS_IP=192.168.50.2`  
- Runs `roslaunch p3at_base base.launch`  
- Starts `RosAria` (P3-AT serial driver) and `odom_republisher`  

✅ **Verify:** From Jetson, run:
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rostopic hz /odom"
# Expected: ~10 Hz
```

---

#### Step 2 — Jetson: ensure roscore is running

**Device:** Jetson Orin Nano  
**Terminal:** Any terminal on Jetson (inside or outside Docker).

```bash
# Check if roscore is already running (it should be if the container started it)
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
# If output shows a PID → already running, skip to Step 3

# If NOT running, start it:
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
exec roscore"

sleep 3
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
# Should now show a rosmaster PID
```

> The `ros_noetic` container uses `--net=host`, so `roscore` on port 11311 is reachable at `192.168.50.1:11311` directly.

---

#### Step 3 — Jetson: start Unitree mapping (in background)

**Device:** Jetson Orin Nano  
**Terminal:** Any bash terminal on Jetson.

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_real_mapping_unitree.sh use_rviz:=false
```

Alternatively, run it in the background and stream logs:
```bash
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
exec roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch use_rviz:=false \
  > /tmp/mapping_unitree.log 2>&1"

# Monitor log
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "tail -f /tmp/mapping_unitree.log"
```

What this launches:
| Node | Package | Role |
|------|---------|------|
| `unitree_lidar` | `unitree_lidar_ros` | Reads `/dev/ttyUSB0`, publishes `/unilidar/cloud` + `/unilidar/imu` |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan` | Converts `/unilidar/cloud` → `/unitree/scan` |
| `slam_gmapping` | `gmapping` | Subscribes `/unitree/scan` + `/odom`, publishes `/map` + `map→odom` TF |
| `move_base` | `move_base` | Navigation costmaps (not used during keyboard teleop, but active) |
| `robot_state_publisher` | `robot_state_publisher` | Publishes static TF `base_link → unitree_lidar` |

✅ **Verify nodes are up:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosnode list"
# Expected:
# /RosAria
# /move_base
# /odom_republisher
# /pointcloud_to_laserscan
# /robot_state_publisher
# /rosout
# /slam_gmapping
# /unitree_lidar
```

---

#### Step 4 — Power on the Unitree LiDAR

**Device:** Unitree L1 LiDAR (physical hardware)  

Now plug in / switch on the Unitree LiDAR. The driver node (`unitree_lidar`, PID already running) will open `/dev/ttyUSB0` and begin receiving data.

The LiDAR takes **10–15 seconds** to initialise after power-on.

✅ **Verify data is flowing (~15 seconds after power-on):**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 8 rostopic hz /unilidar/cloud"
# Expected: average rate: ~9.7 Hz

docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 8 rostopic hz /unitree/scan"
# Expected: average rate: ~9.8 Hz
```

---

#### Step 5 — Keyboard teleoperation

**Device:** Jetson Orin Nano (run this in an **interactive** terminal — not `-d` / background)  
**Terminal:** Open a new `docker exec -it` terminal.

```bash
docker exec -it --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel"
```

Or use the helper script:
```bash
# Run on Jetson (role=jetson, no LAPTOP_IP needed)
./scripts/start_teleop.sh jetson
```

**The terminal will show:**
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
   u    i    o
   j    k    l
   m    ,    .
CTRL-C to quit
currently:      speed 0.5       turn 1.0
```

**Key bindings:**

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `u` | Forward-left arc |
| `o` | Forward-right arc |
| `k` or any other key | **STOP** |
| `q` / `z` | Increase / decrease linear speed by 10% |
| `w` / `x` | Increase / decrease linear speed only |
| `e` / `c` | Increase / decrease angular speed only |
| `Ctrl+C` | Quit teleop |

> **Click the terminal window first** to ensure it has keyboard focus before pressing keys.

While you drive, `slam_gmapping` builds the occupancy map in real time from `/unitree/scan` + `/odom`.

---

#### Step 6 — Save the map

When you have explored enough area, **save the map before stopping** the mapping launch:

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosrun map_server map_saver -f \
  /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/my_map_unitree"
# Produces:  my_map_unitree.pgm  +  my_map_unitree.yaml
```

Or with a timestamped filename:
```bash
MAP_PATH="/home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/session_$(date +%Y%m%d_%H%M%S)"
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosrun map_server map_saver -f ${MAP_PATH}"
echo "Saved: ${MAP_PATH}.pgm"
```

✅ **Preview the map in VS Code (no display needed):**
```bash
# Convert PGM → PNG and open in VS Code
python3 -c "
from PIL import Image
img = Image.open('${MAP_PATH}.pgm')
img.save('${MAP_PATH}.png')"
code "${MAP_PATH}.png"
```

Map colour key:
- ⬜ **White** — free / traversable space  
- ⬛ **Black** — obstacle (wall)  
- 🔲 **Grey** — unknown (not yet scanned)  

---

#### Full system health check (at any point)

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash

echo '--- Nodes ---'
rosnode list

echo '--- Key topic rates ---'
for t in /unilidar/cloud /unitree/scan /odom /map /tf; do
  rate=\$(timeout 4 rostopic hz \$t 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}')
  [ -n \"\$rate\" ] && echo \"  OK  \$t: \${rate} Hz\" || echo \"  --  \$t: no data\"
done

echo '--- TF: map -> base_link ---'
timeout 4 rosrun tf tf_echo map base_link 2>/dev/null | head -4"
```

Expected output:
```
--- Nodes ---
/RosAria
/move_base
/odom_republisher
/pointcloud_to_laserscan
/robot_state_publisher
/rosout
/slam_gmapping
/unitree_lidar
--- Key topic rates ---
  OK  /unilidar/cloud: 9.7 Hz
  OK  /unitree/scan: 9.8 Hz
  OK  /odom: 10.0 Hz
  OK  /map: 0.6 Hz
  OK  /tf: 30.1 Hz
--- TF: map -> base_link ---
At time ...
- Translation: [x, y, 0.000]
- Rotation: in Quaternion [...]
```

---

#### Recovery: nodes disappeared from `rosnode list`

**Symptom:** `rosnode list` only shows `/RosAria`, `/odom_republisher`, `/rosout` — mapping nodes have vanished, but `pgrep unitree_lidar_ros_node` still shows a PID.

**Cause:** The node processes are running but lost their rosmaster registration (TCP keepalive timeout). This happens most often when the LiDAR was powered on **before** the driver initialised, causing a re-enumeration of `/dev/ttyUSB0` that blocked the node's network thread long enough for the master to drop the registration.

**Fix:**
```bash
# 1. Kill the dangling processes inside Docker
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "kill \$(pgrep -d' ' -f 'unitree_lidar_ros_node|slam_gmapping|pointcloud_to_laserscan|roslaunch.*mapping') 2>/dev/null; sleep 2"

# 2. Clean up stale rosmaster registrations
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosnode cleanup 2>/dev/null || true"

# 3. Restart mapping (LiDAR already on this time → no re-enumeration, no race)
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
> /tmp/mapping_unitree.log
exec roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch use_rviz:=false \
  > /tmp/mapping_unitree.log 2>&1"

# 4. Wait 12 s then check nodes
sleep 12
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode list"
```

---

#### Scripts reference

| Script | Runs on | What it does |
|--------|---------|-------------|
| `scripts/start_base.sh` | Raspberry Pi | Sources env (role=raspi), runs `p3at_base/base.launch` |
| `scripts/start_master.sh` | Jetson or Laptop | Sources env, runs `roscore` |
| `scripts/start_real_mapping_unitree.sh` | Jetson | Sources env (role=jetson), runs `real_robot_mapping_unitree.launch` |
| `scripts/start_real_mapping.sh` | Jetson | Same but for SICK LMS200 backup |
| `scripts/start_real_nav.sh` | Jetson | Sources env, runs `real_robot_nav_unitree.launch` |
| `scripts/start_teleop.sh` | Jetson or Laptop | Sources env, runs `teleop_twist_keyboard` → `/cmd_vel` |
| `scripts/start_target_follow.sh` | Jetson (Docker) | Sources env, runs `target_follow_real.launch` (default follows launch default; use `launch_move_base:=false` for overlay mode) |
| `scripts/start_demo.sh` | Jetson Host | **One-command demo launcher** — starts roscore (if needed) + `target_follow_real.launch launch_move_base:=true` + `handobj_detection_rgbd.py` |

All scripts accept extra `key:=value` args that are forwarded to `roslaunch` / `rosrun`:
```bash
./scripts/start_real_mapping_unitree.sh unitree_port:=/dev/ttyUSB1 use_rviz:=true
./scripts/start_teleop.sh jetson                # run on Jetson (no LAPTOP_IP needed)
```

---

### Autonomous Exploration SLAM — Full Runbook

> **Verified real-robot procedure** for fully autonomous frontier-based SLAM mapping.  
> The robot drives itself — no keyboard required.  
> `autonomous_explorer.py` detects frontiers (boundaries between free and unknown space), clusters them, and sends the nearest large frontier centroid to `move_base` as a navigation goal. The loop repeats until no reachable frontiers remain or the timeout is reached, then the map is saved automatically.

#### Prerequisites

| Requirement | Check |
|-------------|-------|
| `scripts/deploy.env` configured (`JETSON_IP=192.168.50.1`, `RASPI_IP=192.168.50.2`) | ✓ |
| Raspberry Pi reachable (`ping 192.168.50.2`) | ✓ |
| `ros_noetic` Docker container running on Jetson (`docker ps`) | ✓ |
| Workspace built inside container (`catkin_make` completed) | ✓ |
| Unitree L1 LiDAR USB-C plugged into Jetson | ✓ |
| P3-AT chassis serial cable connected to Raspberry Pi | ✓ |

#### Tuned parameters (anti-collision, real robot)

The parameters in `param/unitree/` have been tuned for real-robot safety. Key changes vs simulation defaults:

| Parameter | File | Value | Reason |
|-----------|------|-------|--------|
| `footprint_padding` | `costmap_common.yaml` | `0.05 m` | Extra safety margin around footprint |
| `inflation_radius` | `costmap_common.yaml` | `0.3 m` | Clearance around walls (reduced from 0.55 — 0.55 caused spin-in-place near walls) |
| `cost_scaling_factor` | `costmap_common.yaml` | `5.0` | Faster cost decay — allows closer approach without excessive rotation |
| `inflation_radius` | `local_costmap.yaml` | `0.3 m` | Matches common, prevents DWA cutting corners |
| `range_min` | `pointcloud_to_laserscan` | `0.35 m` | Filters robot body from LiDAR scan (was 0.05 — caused self-detection) |
| `max_vel_x` / `max_vel_trans` | `move_base.yaml` | `0.22 m/s` | Reduced speed for safer real-robot operation |
| `sim_time` | `move_base.yaml` | `2.5 s` | Longer DWA lookahead — detects obstacles earlier |
| `occdist_scale` | `move_base.yaml` | `0.08` | 4× higher obstacle weight — actively avoids walls |

> `clearing_rotation_allowed: false` remains **mandatory** — in-place rotation recovery can tip the P3-AT with Unitree L1 on top (high CoM).

---

#### Step 1 — Raspberry Pi: start chassis driver

**Device:** Raspberry Pi 4  
**Terminal:** SSH into the Pi or use a terminal directly on it.

```bash
# On the Raspberry Pi
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

This sets `ROS_MASTER_URI=http://192.168.50.1:11311`, `ROS_IP=192.168.50.2` and runs `roslaunch p3at_base base.launch` (RosAria + odom_republisher).

✅ **Verify from Jetson:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 6 rostopic hz /odom"
# Expected: ~10 Hz
```

---

#### Step 2 — Jetson: ensure roscore is running

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
# If a PID is shown → already running, skip to Step 3

# If NOT running:
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
exec roscore"
sleep 3
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
```

---

#### Step 3 — Jetson: start the mapping stack (background)

```bash
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
exec roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch use_rviz:=false \
  > /tmp/mapping_unitree.log 2>&1"
```

This starts: `unitree_lidar` · `pointcloud_to_laserscan` · `slam_gmapping` · `move_base` · `robot_state_publisher`.

✅ **Verify all nodes are up (wait ~12 s):**
```bash
sleep 12 && docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosnode list"
# Expected:
# /RosAria
# /move_base
# /odom_republisher
# /pointcloud_to_laserscan
# /robot_state_publisher
# /rosout
# /slam_gmapping
# /unitree_lidar
```

✅ **Verify LiDAR data is flowing:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
for t in /unilidar/cloud /unitree/scan /odom /map; do
  rate=\$(timeout 5 rostopic hz \$t 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}')
  [ -n \"\$rate\" ] && echo \"  OK  \$t: \${rate} Hz\" || echo \"  !!  \$t: NO DATA\"
done"
# Expected:
#   OK  /unilidar/cloud: ~9.7 Hz
#   OK  /unitree/scan:   ~9.8 Hz
#   OK  /odom:           ~10.0 Hz
#   OK  /map:            ~0.6 Hz
```

---

#### Step 4 — Jetson: launch the autonomous explorer

**Terminal:** Open a new `docker exec` shell (not `-d` if you want to watch logs live).

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
  _map_save_path:=/home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/explored_map_unitree \
  _spin_in_place_first:=true \
  _robot_radius:=0.25 \
  _frontier_blacklist_radius:=0.2 \
  _goal_timeout:=30.0"
```

**Key parameters explained:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `_exploration_timeout` | `600` | Stop after 600 s (10 min) even if frontiers remain |
| `_min_frontier_size` | `3` | Ignore tiny frontier clusters (noise rejection) |
| `_initial_wait` | `12.0` | Wait 12 s for map + odom to stabilise before first goal |
| `_save_map` | `true` | Auto-save map on exit |
| `_map_save_path` | (path) | Output path without extension — produces `.pgm` + `.yaml` |
| `_spin_in_place_first` | `true` | Rotate 360° at start to seed the initial map |
| `_robot_radius` | `0.25` | Used to pad frontier distance from walls |
| `_frontier_blacklist_radius` | `0.2` | Goals within this radius of a failed goal are blacklisted |
| `_goal_timeout` | `30.0` | Abort a navigation goal after 30 s and try next frontier |

> **To run in the background** and stream the log:
> ```bash
> docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "... rosrun ... > /tmp/explorer.log 2>&1"
> docker exec --user "$(id -u):$(id -g)" ros_noetic tail -f /tmp/explorer.log
> ```

---

#### Step 5 — Monitor exploration progress

**Coverage and goal count** (from `/rosout` log):
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 15 rostopic echo /rosout -p 2>/dev/null \
  | grep -o '\"\[Explorer\].*\"' | tr -d '\"'"
# Sample output:
# [Explorer] >>> Goal #10: (0.48, -2.00) | Coverage: 4.7%
# [Explorer] >>> Goal #11: (1.20, 0.50) | Coverage: 6.1%
```

**Map update rate:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 8 rostopic hz /map"
# Expected: ~0.5–1.0 Hz while actively building
```

**Check explorer node is alive:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode list | grep autonomous_explorer"
# Returns: /autonomous_explorer  →  still running
# No output → exploration complete or crashed
```

**Check active navigation goal:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 5 rostopic echo /exploration_goal -n 1"
# Shows the current frontier goal point
```

---

#### Step 6 — Stop and save the map

**Option A — Let it finish automatically:**  
The explorer saves the map to `_map_save_path` when the timeout expires or no more frontiers are found. Watch for the log line:
```
[Explorer] No reachable frontiers. Exploration complete.
[Explorer] Map saved to: .../maps/explored_map_unitree
```

**Option B — Stop early and save manually:**

```bash
# Stop the explorer (triggers auto-save via ~save_map:=true)
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode kill /autonomous_explorer"

# Or save with a custom name at any time (map keeps building until you kill it)
MAP_PATH="/home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/p3at_lms_navigation/maps/session_$(date +%Y%m%d_%H%M%S)"
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
rosrun map_server map_saver -f ${MAP_PATH}"
echo "Saved: ${MAP_PATH}.pgm"
```

**Preview the saved map:**
```bash
python3 -c "
from PIL import Image
img = Image.open('${MAP_PATH}.pgm')
img.save('${MAP_PATH}.png')"
code "${MAP_PATH}.png"   # open in VS Code
```

Map colour key: ⬜ white = free · ⬛ black = obstacle · 🔲 grey = unknown

---

#### Step 7 — Shut down

```bash
# Kill all mapping nodes on Jetson (does NOT affect Pi base driver)
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode kill /autonomous_explorer /move_base /slam_gmapping \
  /pointcloud_to_laserscan /unitree_lidar /robot_state_publisher 2>/dev/null || true"

# Or kill the roslaunch process directly:
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c \
  "kill \$(pgrep -f 'roslaunch.*mapping') 2>/dev/null; sleep 2"
```

---

#### Troubleshooting — autonomous exploration

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot hits walls repeatedly | `occdist_scale` too low or `inflation_radius` too small | Check `param/unitree/costmap_common.yaml`; current tuned values: `inflation_radius: 0.3`, `occdist_scale: 0.08` |
| Explorer sends goals but robot doesn't move | `move_base` not running or DWA planning fails | Check `/tmp/mapping_unitree.log`; run `rosnode list` to confirm `/move_base` is up |
| `[Explorer] No frontiers found` immediately | `/map` not being published yet | Increase `_initial_wait` to `15.0`; verify `rostopic hz /map` > 0 |
| Goal always aborted (status 4) | Costmap inflation is blocking all paths | Temporarily reduce `inflation_radius` to `0.45` in `costmap_common.yaml` and restart |
| Explorer crashes with `TF lookup error` | `odom → base_link` TF broken | Check Pi base driver is running; verify `rostopic hz /odom` |
| Coverage stays at 0% after spin | gmapping hasn't received enough scans | Wait 20–30 s; check `rostopic hz /unitree/scan` ≥ 8 Hz |

---

### Target Following (Real Robot Demo) — Full Runbook

> **Primary demo task.** Robot follows a detected target (trash / person) using YOLO + depth-camera detection → `move_base` local navigation.
> The detection runs on **Jetson host** (native Ubuntu, GPU); the ROS pipeline runs inside the **Docker container**.
> **No pre-built map required.** The standalone mode uses a rolling-window costmap in the `odom` frame.

---

#### Architecture

```
[Jetson — Native Host]
  handobj_detection_rgbd.py --udp-enable
  ──► UDP JSON {x, y, z, frame_id="camera_link", stamp}  ──► 127.0.0.1:16031

[Jetson — ROS Docker]
  udp_target_bridge       ←── UDP ──► /trash_detection/target_point  (PointStamped, frame=camera_link)
  point_to_target_pose    ←── PointStamped ──► /target_pose  (PoseStamped, frame=camera_link)
  target_follower         ←── /target_pose
                          ──► MoveBaseGoal → move_base (odom frame) → /cmd_vel
                          ──► /target_follower/result  (Bool, latched: True=REACHED)
                          ──► /target_follower/status  (String, 2 Hz: IDLE|TRACKING|REACHED|
                                                          WAITING_ACTION|RETREATING|LOST|FAILED)
                          ←── /trash_action  (Bool: True=接受 / False=拒绝→后退)
  navigation_success_udp_bridge  ←── /target_follower/result ──► UDP:16041
  udp_trash_action_bridge        ←── UDP:16032 ──► /trash_action
  static_transform_publisher  ──► TF: base_link → camera_link  (quat: -0.5 0.5 -0.5 0.5)
  move_base               ←── rolling-window costmap (odom frame, NO /map needed)
  unitree_lidar_ros       ──► /unitree/scan  → move_base obstacle layer

[Jetson — Native Host (bridges, via start_dialogue_docker_bridges.sh)]
  navigation_success_udp_bridge  → UDP:16041 → dialogue_udp_runner (device)
  udp_trash_action_bridge        ← UDP:16032 ← dialogue_udp_runner (device)

[Dialogue Device]
  dialogue_udp_runner  ← UDP:16041 (trigger: REACHED)
                       → UDP:16032 (result: True=接受 / False=拒绝)
```

**Dialogue state flow:**
```
REACHED → publish result=True → UDP:16041 → dialogue
   │ /trash_action=True  → IDLE (接受, 重新开始追踪)
   └ /trash_action=False → RETREATING (拒绝, 后退 retreat_distance m) → IDLE
   └ 超时 (action_wait_timeout_s) → IDLE
```

> **Two launch modes** (controlled by `launch_move_base` arg):
>
> | Mode | `launch_move_base` | Nav frame | Use case |
> |------|--------------------|-----------|----------|
> | **Standalone** (default) | `true` | `odom` | Demo — no pre-built map, starts LiDAR + move_base automatically |
> | **Overlay** | `false` | `map` | Overlay on existing mapping/nav stack — requires `/map` + `move_base` already running |

---

#### Camera → Robot Coordinate Transform

Orbbec Femto Bolt depth camera outputs XYZ in **optical frame** convention:
`+z` = forward (depth), `+x` = right, `+y` = down

ROS `base_link` uses: `+x` = forward, `+y` = left, `+z` = up

The static TF `base_link → camera_link` is published with quaternion **`(-0.5, 0.5, -0.5, 0.5)`** — equivalent to RPY `[-90°, 0°, -90°]`.

| Camera optical | → | Robot base_link |
|---|---|---|
| `+z` (depth / forward) | → | `+x` (forward) ✓ |
| `+x` (right) | → | `-y` (right) ✓ |
| `+y` (down) | → | `-z` (down) ✓ |

> ⚠️ **Previous bug (fixed):** The quaternion was `(0.5, -0.5, 0.5, 0.5)` (conjugate/inverse), which caused all direction axes to be inverted. The correct value is `(-0.5, 0.5, -0.5, 0.5)`.
>
> **Verified:** detection at camera `(-0.1, 0.1, 0.65)` → base `(+0.65, +0.1, -0.1)` ✓ (tested 2026)

---

#### Target Follower State Machine

```
                  ┌──────────┐
                  │   IDLE   │ ← no /target_pose or stale > target_timeout
                  └────┬─────┘
                       │ valid /target_pose received
                       ▼
                  ┌──────────┐
          ┌──────►│ TRACKING │─────────────────────┐
          │       └─────┬────┘                     │
          │ REACHED     │ dist ≤ standoff_dist    │ move_base ABORTED
          │ & target    ▼                         ▼
          │ moved  ┌──────────┐             ┌──────────┐
          │        │ REACHED  │             │  FAILED  │
          └────────│  (True)  │             │  (False) │
                   └──────────┘             └──────────┘
                       ▲                        ▲
                  target stale             target stale
                  > target_timeout         > target_timeout
                  
                       └────────► LOST (False) ◄┘
```

**Key behaviour:**
- When `dist > standoff_distance`: goal placed `standoff_distance` m back along robot→target vector
- When `dist ≤ standoff_distance`: REACHED immediately (robot already close enough)
- `min_update_dist` (0.3 m): blocks re-sending goal if target didn't move, **EXCEPT** when state is FAILED/LOST/REACHED — prevents goal spam while tracking stationary target, allows re-engagement when person moves away
- `face_target:=true`: final orientation aligns robot heading toward last known target pose

---

#### Prerequisites

| Requirement | How to verify |
|-------------|---------------|
| Jetson Orin Nano running, Docker container `ros_noetic` active | `docker ps` |
| Raspberry Pi reachable, P3-AT chassis connected (`/dev/ttyS0`) | `ping 192.168.50.2` |
| Unitree L1 LiDAR USB-C connected to Jetson (initially powered OFF) | `ls /dev/unitree_lidar` |
| Orbbec Femto Bolt camera connected to Jetson USB | `python3 tools/inspect_depth_once.py` |
| `handobj_detection/handobj_detection_rgbd.py` host deps | `python3 -c "import ultralytics"` |
| Workspace built | `rospack find target_follower` (inside Docker) |

---

#### Quick Start — One Command

> Pi base driver (`scripts/start_base.sh`) must already be running.

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_demo.sh
```

`start_demo.sh` does:
1. Checks / starts `roscore` in Docker
2. Launches `target_follow_real.launch launch_move_base:=true` in background
3. Waits up to 30 s for key nodes (`move_base` + `target_follower`)
4. Starts `handobj_detection_rgbd.py` on Jetson host (with `--udp-enable`)

---

#### Step-by-Step Runbook

##### Step 1 — Raspberry Pi: start chassis driver

```bash
# On Pi (ssh frank@192.168.50.2)
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

Sets `ROS_MASTER_URI=http://192.168.50.1:11311`, `ROS_IP=192.168.50.2`, runs `p3at_base/base.launch` → `RosAria` + `odom_republisher`.

✅ **Verify from Jetson:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
timeout 6 rostopic hz /odom"
# Expected: ~10 Hz
```

---

##### Step 2 — Jetson: ensure roscore is running

```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
# PID shown → already running, skip to Step 3

# If NOT running:
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
exec roscore"
sleep 3
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "pgrep -la rosmaster"
```

---

##### Step 3 — Power on Unitree LiDAR

Plug in / switch on the Unitree L1. Wait **10–15 seconds** — the driver node will detect the enumeration automatically once the launch is running.

---

##### Step 4 — Jetson Docker: launch standalone target following

```bash
docker exec -d --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
exec roslaunch target_follower target_follow_real.launch \
  launch_move_base:=true \
  standoff_distance:=0.8 \
  face_target:=true \
  target_timeout:=5.0 \
  retreat_distance:=1.5 \
  action_wait_timeout:=45.0 \
  > /tmp/target_follow.log 2>&1'
```

**What this launches (standalone mode):**

| Node | Package | Role |
|------|---------|------|
| `unitree_lidar` | `unitree_lidar_ros` | Reads `/dev/ttyUSB0`, publishes `/unilidar/cloud` |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan` | Cloud → `/unitree/scan` (`range_min=0.35 m` — filters robot body) |
| `robot_state_publisher` | `robot_state_publisher` | TF `base_link → unitree_lidar` |
| `static_transform_publisher` | `tf` | TF `base_link → camera_link` (quat: `-0.5 0.5 -0.5 0.5`) |
| `move_base` | `move_base` | Rolling-window costmap (`global_frame=odom`, no `/map` needed) |
| `udp_target_bridge` | `target_follower` | UDP 16031 → `/trash_detection/target_point` |
| `point_to_target_pose` | `target_follower` | PointStamped → PoseStamped |
| `target_follower` | `target_follower` | State machine, sends goals, publishes result/status, handles dialogue response |

**Key launch parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `launch_move_base` | `true` | `true` = standalone; `false` = overlay on existing nav stack |
| `standoff_distance` | `0.8` m | Stop distance from target |
| `face_target` | `true` | Orient robot toward target at REACHED |
| `target_timeout` | `5.0` s | Cancel goal if no detection for this long |
| `udp_port` | `16031` | Must match detection `--udp-port` |
| `retreat_distance` | `1.5` m | How far to retreat when human refuses (RETREATING state) |
| `action_wait_timeout` | `45.0` s | Timeout waiting for `/trash_action` before resetting to IDLE |
| `retreat_timeout` | `20.0` s | Max time allowed for retreat navigation goal |

Wait ~12 s, then verify:
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode list'
# Expected (standalone — 10 nodes):
# /RosAria
# /move_base
# /odom_republisher
# /point_to_target_pose
# /pointcloud_to_laserscan
# /robot_state_publisher
# /rosout
# /target_follower
# /udp_target_bridge
# /unitree_lidar
```

✅ **Verify TF quaternion is correct:**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 4 rosrun tf tf_echo base_link camera_link 2>/dev/null | head -8'
# Expected:
# - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
# - Equivalent RPY: [-1.571, 0.000, -1.571]
```

✅ **Verify move_base is using odom frame (no /map dependency):**
```bash
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 4 rostopic echo /move_base/global_costmap/costmap/info -n 1 2>/dev/null | grep header'
# header.frame_id should be "odom" not "map"
```

---

##### Step 5 — Jetson Host (Native): start YOLO detection

**Terminal:** Non-Docker terminal on Jetson.

```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
python3 handobj_detection/handobj_detection_rgbd.py \
  --udp-enable \
  --udp-port 16031 \
  --udp-frame-id camera_link \
  --headless
```

**Key flags:**

| Flag | Purpose |
|------|---------|
| `--udp-enable` | Enable UDP JSON sending to Docker |
| `--udp-port 16031` | Must match `udp_target_bridge` port |
| `--udp-frame-id camera_link` | Detection XYZ is in camera optical frame |
| `--headless` | No GUI windows (for SSH; remove for display) |

> **Internal model stack:** `yolov8n.pt` (full-frame, ~13 FPS) + `weights/last.pt` (fine-tuned classifier). Both use GPU via PyTorch/CUDA on Jetson.

---

##### Step 6 — Start Dialogue UDP bridges

**Run on Jetson host** (non-Docker). This starts two Python bridge processes:
- `navigation_success_udp_bridge.py` — subscribes `/target_follower/result`, on `True` → sends UDP to dialogue (port 16041)
- `udp_trash_action_bridge.py` — listens UDP port 16032, publishes `/trash_action` (Bool) back to ROS

```bash
# Terminal A — Docker ROS bridges (must be started AFTER roscore is running)
./scripts/start_dialogue_docker_bridges.sh
```

```bash
# Terminal B — Dialogue runner on the dialogue device (e.g. Jetson or laptop)
./scripts/start_dialogue_host.sh --device 24
```

> If you don’t have the dialogue module yet, you can simulate the full flow manually:
> ```bash
> # Simulate REACHED trigger to dialogue
> rostopic pub -1 /target_follower/result std_msgs/Bool "data: true"
>
> # Simulate human ACCEPTING (robot stays)
> rostopic pub -1 /trash_action std_msgs/Bool "data: true"
>
> # Simulate human REFUSING (robot retreats)
> rostopic pub -1 /trash_action std_msgs/Bool "data: false"
> ```

---

##### Step 7 — Verify the full pipeline

```bash
# Check UDP → target_point
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 8 rostopic hz /trash_detection/target_point'
# Expected: ~5–13 Hz

# Check live status
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 15 rostopic echo /target_follower/status'
# "IDLE"           → no target
# "TRACKING"       → driving to target
# "REACHED"        → within standoff_distance (triggers dialogue)
# "WAITING_ACTION" → waiting for /trash_action from dialogue
# "RETREATING"     → human refused, executing retreat navigation
# "LOST"           → target stale > timeout
# "FAILED"         → move_base could not reach goal

# Watch result topic (dialogue trigger)
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rostopic echo /target_follower/result'
# data: True  → robot reached target (triggers dialogue)
# data: False → lost/failed

# Watch dialogue result (incoming from dialogue module)
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rostopic echo /trash_action'
# data: True  → human accepted (robot stays, returns to IDLE)
# data: False → human refused (robot retreats)
```

---

##### Step 8 — Stop

```bash
# Kill all target-following nodes
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c "
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
rosnode kill /target_follower /udp_target_bridge /point_to_target_pose \
  /move_base /pointcloud_to_laserscan /unitree_lidar /robot_state_publisher 2>/dev/null || true"

# Kill detection on host
pkill -f handobj_detection_rgbd.py

# Kill dialogue bridges on host
pkill -f navigation_success_udp_bridge.py || true
pkill -f udp_trash_action_bridge.py || true
```

---

#### Manual Test (no camera/hardware)

```bash
# Simulate target 2 m in front (optical z=2.0 → base x=2.0 m forward)
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(30):
    s.sendto(json.dumps({'x': 0.0, 'y': 0.0, 'z': 2.0,
                         'frame_id': 'camera_link',
                         'stamp': time.time()}).encode(),
             ('127.0.0.1', 16031))
    time.sleep(0.2)"

# Watch status
docker exec --user "$(id -u):$(id -g)" ros_noetic bash -c '
export ROS_MASTER_URI=http://192.168.50.1:11311
source /opt/ros/noetic/setup.bash
timeout 15 rostopic echo /target_follower/status'
# TRACKING → REACHED (once robot gets within 0.8 m of the 2 m goal)
```

---

#### Troubleshooting — Target Following

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `/trash_detection/target_point` no data | YOLO not running / wrong port | Check `--udp-enable --udp-port 16031`; `ss -lnup \| grep 16031` inside Docker |
| Status stuck on `IDLE` | No `/target_pose` or wrong frame_id | `rostopic echo /target_pose`; `camera_link` must exist in TF tree |
| Robot moves in **wrong direction** | Old inverted TF quaternion | Verify TF echo shows `(-0.5, 0.5, -0.5, 0.5)` |
| `TF lookup error camera_link → odom` | `camera_link` frame missing | Check `static_transform_publisher` is running (`rosnode list`) |
| Status stuck on `REACHED` forever | **(Fixed)** — old code didn't re-check standoff | `target_follower.py` now has `"REACHED"` in `min_update_dist` bypass list |
| Robot rotates in place near wall | Old `inflation_radius: 0.55 m` too large | Now `0.3 m` in all costmap YAMLs — rebuild costmap if still happening |
| LiDAR sees robot body as obstacle | Old `range_min: 0.05 m` | Now `range_min: 0.35 m` in all `pointcloud_to_laserscan` configs |
| `move_base` waiting for `/map` indefinitely | Wrong costmap config loaded | Standalone uses `global_costmap_local_only.yaml` (`static_map: false`, `rolling_window: true`) |
| Robot accepted goal but doesn't move | DWA no valid trajectory | `rosservice call /move_base/clear_costmaps`; verify `rostopic hz /unitree/scan` |


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
| `unitree_port` | `/dev/ttyUSB0` | Unitree L1 USB port — use `/dev/unitree_lidar` if udev rule is active |
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
| `param/unitree/costmap_common.yaml` | `inflation_radius: 0.3`, `cost_scaling_factor: 5.0` |
| `param/unitree/global_costmap.yaml` | Obstacle source: `/unitree/scan`, frame `unitree_lidar` |
| `param/unitree/global_costmap_local_only.yaml` | **Standalone target-following** — `global_frame: odom`, `rolling_window: true`, `static_map: false` |
| `param/unitree/local_costmap.yaml` | `inflation_radius: 0.3`, `cost_scaling_factor: 5.0` |
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
| `/target_follower/result` | `std_msgs/Bool` (latched) | `target_follower` | dialogue model |
| `/target_follower/status` | `std_msgs/String` (~2 Hz) | `target_follower` | monitoring |

### TF Tree (Real Robot — Unitree)

**Mapping mode** (`slam_gmapping` running) or **Nav mode** (`amcl` running):
```
map
└── odom               [slam_gmapping or amcl  ~20 Hz]
    └── base_footprint [rosaria on Pi  ~50 Hz]
        └── base_link  [robot_state_publisher  static]
            ├── unitree_lidar  [robot_state_publisher  static]
            └── camera_link    [static_transform_publisher  static — target following only]
```

**Standalone target-following mode** (`target_follow_real.launch launch_move_base:=true`):
```
odom                   [rosaria on Pi  ~50 Hz — odom is the nav root, no /map needed]
└── base_footprint     [rosaria on Pi  ~50 Hz]
    └── base_link      [robot_state_publisher  static]
        ├── unitree_lidar  [robot_state_publisher  static]
        └── camera_link    [static_transform_publisher  static — quat: -0.5 0.5 -0.5 0.5]
```

| Edge | Broadcaster | Notes |
|------|-------------|-------|
| `map → odom` | `slam_gmapping` or `amcl` | **Mapping/nav mode only** — not present in standalone target-following |
| `odom → base_footprint` | `rosaria` (Pi) | Cross-machine via TCPROS |
| `base_footprint → base_link` | `robot_state_publisher` | Static, identity |
| `base_link → unitree_lidar` | `robot_state_publisher` | Static, mount offset from URDF |
| `base_link → camera_link` | `static_transform_publisher` | Static, quat `(-0.5, 0.5, -0.5, 0.5)` = RPY `[-90°, 0°, -90°]` |

**SICK stack:** identical with `laser` replacing `unitree_lidar`, `/scan` replacing `/unitree/scan`.

---

## Parameter Tuning Guide

### Why Split Global/Local Costmap Inflation

For **mapping mode** (`global_costmap.yaml` + `local_costmap.yaml`):
- Global costmap (`inflation_radius: 0.3 m`) → NavFn plans a conservative path away from walls.
- Local costmap (`inflation_radius: 0.3 m`) → DWA has room to find trajectories in tight corridors.

For **standalone target-following** (`global_costmap_local_only.yaml`):
- `global_frame: odom`, `rolling_window: true`, `static_map: false` — no `/map` topic needed.
- `inflation_radius: 0.3 m`, `cost_scaling_factor: 5.0`.

> **Why 0.3 m, not 0.55 m?** At 0.55 m, the robot frequently entered costmap-blocked rotation loops when approaching a person. Reducing to 0.3 m eliminated this while keeping wall clearance adequate for the P3-AT's 0.54 m width.

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
| `~standoff_distance` | `0.8` m | Stop this far short of target (set in launch file) |
| `~face_target` | `true` | Orient robot toward target at goal (set in launch file) |
| `~send_rate_hz` | `2.0` Hz | Max rate for new goals |
| `~min_update_dist` | `0.3` m | Min target movement before re-sending goal — bypassed when state is FAILED/LOST/REACHED |
| `~target_timeout` | `5.0` s | Stale target timeout — cancels move_base goal (set in launch file) |
| `~global_frame` | `odom` | Navigation frame — `odom` in standalone mode, `map` in overlay mode |
| `~trash_action_topic` | `/trash_action` | Topic to receive dialogue result (Bool: True=accept, False=refuse) |
| `~action_wait_timeout_s` | `45.0` s | Timeout in WAITING_ACTION before resetting to IDLE |
| `~retreat_distance` | `1.5` m | Distance to retreat when human refuses (away from target) |
| `~retreat_timeout_s` | `20.0` s | Max time allowed for retreat navigation goal |

---

## Known Issues and Notes

- **`clearing_rotation_allowed: false` is safety-critical**: In-place rotation recovery has caused the P3-AT to tip over with Unitree L1 on top (high CoM). Do not re-enable without anti-tip analysis.

- **Camera TF quaternion is `(-0.5, 0.5, -0.5, 0.5)` — do not change**: This corrects optical→robot frame for the Orbbec Femto Bolt. The previous value `(0.5, -0.5, 0.5, 0.5)` was incorrect (inverse quaternion) and caused all direction axes to be inverted. Verified correct in 2026 session.

- **REACHED → WAITING_ACTION → RETREATING state flow**: After reaching standoff distance, the robot enters `WAITING_ACTION` (freezes target-following) and waits for `/trash_action` (Bool) from the dialogue module. If `False` (human refuses), the robot enters `RETREATING` — a new `move_base` goal is computed `retreat_distance` metres directly away from the target and dispatched. After the retreat completes (or times out), the robot returns to `IDLE`. This entire flow is handled inside `target_follower.py` without any separate node. If no `/trash_action` is received within `action_wait_timeout_s`, the robot also resets to `IDLE`.

- **LiDAR `range_min: 0.35 m`**: The P3-AT chassis and Unitree L1 mount are within 0.35 m of the sensor origin. Setting `range_min < 0.35` in `pointcloud_to_laserscan` causes the robot body to appear as an obstacle in the costmap. Current value: `0.35 m` in both `target_follow_real.launch` and `real_robot_nav_unitree.launch`.

- **Standalone target-following uses `odom` frame**: `global_costmap_local_only.yaml` uses `global_frame: odom` with `rolling_window: true` so no `/map` topic is needed. The TF chain `odom → base_footprint → base_link → camera_link` is sufficient.

- **Unitree SDK — no kernel modules needed**: The SDK uses a pre-built `libunitree_lidar_sdk.a` static library for `aarch64`. No kernel module compilation is required on Jetson Orin Nano.

- **Narrow corridor margins**: P3-AT is ~0.54 m wide; after inflation (0.3 m), margins in 1 m corridors are tight. If the robot gets stuck frequently, reduce `inflation_radius` to `0.25 m`, but expect more wall contacts.

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
- [x] Docker navigation ROS packages installed — `ros_noetic:nav_unitree` image
- [x] `unitree_lidar_ros` compiled in Jetson Docker — compiled in `ros_noetic:nav_unitree`
- [x] Udev rule for Unitree L1 — `/etc/udev/rules.d/99-unitree-lidar.rules` on Jetson host
- [x] `deploy.env` IPs configured — Jetson `192.168.50.1`, Pi `192.168.50.2`, UDP `16031`
- [x] `start_real_mapping_unitree.sh` + `start_teleop.sh` helper scripts — created
- [x] `start_demo.sh` one-command launcher — created (standalone target-following, no map needed)
- [x] **Keyboard teleoperation + Unitree mapping — end-to-end real robot test PASSED** (2026-02-24)
  - `/unilidar/cloud` ~9.7 Hz ✓, `/unitree/scan` ~9.8 Hz ✓, `/odom` ~10 Hz ✓
  - `slam_gmapping` processed **1008 scan frames**, map 576×576 px @ 0.05 m/px
  - `map→base_link` TF chain complete ✓
  - Map saved: `maps/session_20260224_223938.pgm` (325 KB)
- [x] **Target following pipeline — end-to-end real robot test PASSED** (2026 session)
  - Standalone mode (`launch_move_base:=true`) — no pre-built map needed ✓
  - Camera TF quaternion corrected to `(-0.5, 0.5, -0.5, 0.5)` (was inverted) ✓
  - Detection at camera `(-0.1, 0.1, 0.65)` → base `(+0.65, +0.1, -0.1)` ✓
  - REACHED state deadlock fixed (re-engages when target moves away) ✓
  - LiDAR self-detection fixed (`range_min=0.35 m`) ✓
  - Rolling-window costmap in `odom` frame — no `/map` dependency ✓
  - `handobj_detection_rgbd.py --udp-enable` → UDP → Docker → `move_base` goal — full pipeline verified ✓
- [x] Target following result topic (`/target_follower/result` Bool) — implemented for dialogue trigger
- [x] Real-robot target following launch (`target_follow_real.launch`) — rewritten with full pipeline + standalone mode
- [x] Camera optical frame → base_link static TF — auto-published by launch (corrected quaternion)
- [x] `global_costmap_local_only.yaml` — created for standalone target-following (no `/map` needed)
- [x] `inflation_radius` tuned: `0.55 m → 0.3 m` across all costmap YAMLs
- [x] `cost_scaling_factor` tuned: `3.0 → 5.0` across all costmap YAMLs
- [ ] End-to-end real robot navigation test (AMCL on saved map) — Unitree (primary) — not started
- [ ] End-to-end real robot navigation test — SICK (backup) — not started

---

## Post-Installation Checklist

### Build
- [ ] `cd catkin_ws && catkin_make` — no errors
- [ ] `source devel/setup.zsh && rospack find p3at_lms_navigation` — path returned
- [ ] `rospack find unitree_lidar_ros` — found (requires SDK clone + build)

### Docker Environment (Jetson)
- [x] Navigation ROS packages installed — included in `ros_noetic:nav_unitree`
- [x] `unitree_lidar_ros_node` binary present — `devel/lib/unitree_lidar_ros/unitree_lidar_ros_node`
- [ ] `--net=host` confirmed: `docker inspect -f '{{.HostConfig.NetworkMode}}' ros_noetic`
- [ ] `roscore` running: `ss -lntp | grep 11311`

### Network
- [ ] Jetson `eth0`: `192.168.50.1`
- [ ] Pi `eth0`: `192.168.50.2`
- [ ] Ping both directions succeed
- [ ] `rosnode list` from Pi returns `/rosout`

### Unitree Hardware
- [x] Udev rule installed — `/etc/udev/rules.d/99-unitree-lidar.rules` (symlink: `/dev/unitree_lidar`)
- [x] `frank` user in `dialout` group
- [ ] `ls /dev/unitree_lidar` shows symlink after connecting Unitree L1
- [x] `roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch use_rviz:=false` — no errors (verified 2026-02-24)
- [x] `rostopic hz /unitree/scan` — ~9.8 Hz (verified 2026-02-24)
- [x] `rostopic hz /unilidar/cloud` — ~9.7 Hz (verified 2026-02-24)
- [x] `slam_gmapping` builds `/map` at ~0.6 Hz during teleoperation (verified 2026-02-24)
- [ ] RViz shows `unitree_lidar` frame and scan data (requires display / X forwarding)

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
