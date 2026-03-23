# ELEC70015 Human-Centered Robotics 2026 — Imperial College London

Pioneer 3-AT autonomous navigation + trash-detection demo system.
ROS1 Noetic · Ubuntu 20.04 · Gazebo 11 (simulation) / Docker + multi-machine ROS (real robot).

This README is aligned with the current `main` branch implementation.
The default real-robot runtime is **dual lidar**: Unitree L1 is the primary sensor, and RPLIDAR A2 is used as a short-range local-costmap supplement. Historical simulation-only detail has been trimmed here; this README focuses on the current runnable system and the latest retained dual-lidar simulation results.

The final research paper (report) of this project can be found in ELEC70015_Human-Centered-Robotics-2026_Imperial/HCR_Final_Research_Paper_Crazy_Thursday.pdf.

## Table of Contents

1. [Branch Guide](#branch-guide)
2. [Runbook](#runbook)
3. [Deployment](#deployment)
4. [Interface Documentation](#interface-documentation)
5. [Technical Implementation](#technical-implementation)
6. [Experiment Report](#experiment-report)
7. [Team Contribution](#team-contribution)

<a id="branch-guide"></a>
## Branch Guide

Remote branch status below reflects the current fetched state (updated on 2026-03-20).
`origin/main` is the baseline for the full-system workflow.

| Remote branch | Status vs `origin/main` | Last commit (date) | Primary role |
|--------------|--------------------------|--------------------|--------------|
| `origin/main` | Baseline (`0 behind / 0 ahead`) | `2661057` (2026-03-20) | **Full-system workflow branch**: integrated real-robot deployment + retained simulation results + unified runbook. Use this as the default branch for end-to-end operation. |
| `origin/navigation_sim_experiments` | Diverged (`18 behind / 6 ahead`) | `1b8ee69` (2026-03-18) | **Gazebo + RViz simulation and experiment validation** branch (Exp1-Exp4 style simulation workflows and reproducibility notes). |
| `origin/esp32` | Diverged (`9 behind / 3 ahead`) | `8635c86` (2026-03-20) | **Smart sorting bin control and deployment** branch (ESP32 firmware, MQTT control, and host-side debug tooling for the bin actuator path). |
| `origin/dialogue` | Diverged (`103 behind / 12 ahead`) | `b87c512` (2026-03-18) | Dialogue subsystem development branch (NLU/STT/TTS runtime, dialogue models/assets, and dialogue-specific iteration). |
| `origin/trash_detection` | Diverged (`97 behind / 8 ahead`) | `a636eb1` (2026-03-10) | Trash-detection focused branch (detector models, scripts, and perception-side experiments/integration updates). |
| `origin/chore/deploy-prep-multimachine` | Behind only (`81 behind / 0 ahead`) | `04f9ab7` (2026-02-21) | Multi-machine deployment preparation branch (launcher/deployment hardening and environment bringup adjustments). |
| `origin/full_system_tmp` | Behind only (`9 behind / 0 ahead`) | `e34c0cc` (2026-03-18) | Full-system staging branch used for temporary integration/tuning snapshots before consolidation into `main`. |
| `origin/toppi` | Diverged (`26 behind / 4 ahead`) | `2c3c905` (2026-03-19) | Experimental integration branch for calibration and perception/navigation variants (e.g., ArUco calibration gating and alternative detection pipelines). |

<a id="runbook"></a>
## Runbook

### Quick Start

| Goal | Command(s) |
|------|------------|
| One-command target-following demo | Pi: `./scripts/start_base.sh`  •  Jetson host: `./scripts/start_demo.sh` |
| Sensor-only dual-lidar validation | Jetson host: `./scripts/start_demo.sh --sensor-only --lidar dual` |
| Real-robot mapping | Pi: `./scripts/start_base.sh`  •  Jetson: `./scripts/start_real_mapping_unitree.sh use_rviz:=false` |
| Real-robot navigation on saved map | Pi: `./scripts/start_base.sh`  •  Jetson: `./scripts/start_real_nav_unitree.sh map_file:=/path/to/map.yaml` |
| Standalone host dialogue | Jetson host: `./scripts/start_dialogue_host.sh --device 24` |
| Standalone host detection | Jetson host: `./scripts/start_trash_detection_rgbd.sh --detector handobj` |

### Recommended Workflows

#### A. One-Command Demo (Recommended)

1. On the Raspberry Pi, start the base driver:

```bash
cd /path/to/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_base.sh
```

2. On the Jetson host, start the full demo:

```bash
cd /path/to/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_demo.sh
```

This path starts:
- Docker ROS core + navigation / target follower
- Host-side YOLO detector (`handobj_detection_rgbd_remote_15cls.py`)
- Host-side dialogue runner
- Docker-side ROS↔UDP dialogue bridges

Current demo defaults:
- `--lidar dual`
- `--target holding`
- `online_slam_task` mode
- `explore_step = 2.4 m`
- RPLIDAR demo profile: `256000`, pre-start motor enabled, `2.0 s` warmup

Useful `start_demo.sh` options:

| Option | Purpose |
|--------|---------|
| `--sensor-only` | Only validate lidar bringup; do not start nav / detection / dialogue |
| `--lidar dual|unitree|rplidar` | Select lidar mode |
| `--assist-map /path/to/map.yaml` | Use saved map + AMCL instead of pure online SLAM |
| `--target holding|person|waste` | Detector metadata label |
| `--no-dialogue` | Start detection + navigation only |
| `--no-yolo` | Start navigation + dialogue only |
| `--only master,nav,yolo,dialogue,dashboard` | Partial startup / repair |
| `--nav-readiness strict|relaxed` | Control startup readiness checks |

For the full option list, run:

```bash
./scripts/start_demo.sh --help
```

#### B. Mapping Runbook

Use this when you want to create a map with the real robot.

1. Start the Pi base driver:

```bash
./scripts/start_base.sh
```

2. On the Jetson side, launch mapping:

```bash
./scripts/start_real_mapping_unitree.sh use_rviz:=false
```

3. Only after the mapping launch is up, power on the Unitree lidar and wait 10-15 seconds.

4. Teleoperate the robot:

```bash
./scripts/start_teleop.sh jetson
```

5. Save the map:

```bash
# Run this in the same ROS environment where mapping is running
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map_unitree
```

If the second lidar is not connected, disable it explicitly:

```bash
./scripts/start_real_mapping_unitree.sh use_rviz:=false enable_rplidar:=false
```

#### C. Navigation on a Saved Map

```bash
./scripts/start_base.sh
./scripts/start_real_nav_unitree.sh map_file:=/path/to/map.yaml
```

To overlay target following on top of an already running map-based stack:

```bash
# Run this in the same ROS environment as the existing map-based navigation stack
roslaunch target_follower target_follow_real.launch launch_move_base:=false
```

#### D. Standalone Host Services

Current demo detector:

```bash
python3 handobj_detection/handobj_detection_rgbd_remote_15cls.py \
  --udp-enable \
  --udp-host 127.0.0.1 \
  --udp-port 16031 \
  --udp-frame-id camera_link \
  --udp-kind holding \
  --rotate-180 \
  --headless \
  --print-xyz
```

Dialogue runner:

```bash
./scripts/start_dialogue_host.sh --device 24
```

### Operational Checks

Use these while bringing the system up:

```bash
rostopic hz /odom
rostopic hz /unitree/scan
rostopic hz /rplidar/scan_filtered
rostopic echo -n 1 /target_follower/status
rostopic echo -n 1 /target_follower/result
rosrun tf tf_echo map base_link
```

Expected current real-hardware rates after successful dual-lidar bringup:
- `/unitree/scan`: about `9-10 Hz`
- `/rplidar/scan_filtered`: about `10-11 Hz`

### Key Scripts

| Script | Role |
|--------|------|
| `scripts/start_demo.sh` | One-command supervisor for the full real-robot demo |
| `scripts/stop_demo_all.sh` | Stop demo processes |
| `scripts/start_base.sh` | Start Pi base driver and sync Pi time to Jetson master |
| `scripts/start_real_mapping_unitree.sh` | Real-robot mapping |
| `scripts/start_real_nav_unitree.sh` | Saved-map navigation |
| `scripts/start_dialogue_host.sh` | Host dialogue wrapper |
| `scripts/start_dialogue_docker_bridges.sh` | Docker ROS↔UDP dialogue bridges |
| `scripts/start_trash_detection_rgbd.sh` | Host-side detector wrapper |
| `scripts/start_teleop.sh` | Keyboard teleop |

### Known Issues

- `start_demo.sh` currently auto-launches `demo_dashboard.sh` with a `--modules` flag, but `demo_dashboard.sh` only accepts `--interval`. If the dashboard exits immediately, inspect `/tmp/demo_dashboard.log` and run `./scripts/demo_dashboard.sh --interval 2` manually.
- `maps/*.pgm` and `maps/*.yaml` are tracked by default in this repo. Only commit newly generated maps when you intend to keep them.

### Further Detail

For deeper operational detail beyond this README:
- legacy runbook: `doc.md`
- full demo options: `./scripts/start_demo.sh --help`
- full launch args: `catkin_ws/src/target_follower/launch/target_follow_real.launch`

<a id="deployment"></a>
## Deployment

### System Layout

| Runtime domain | Machine | Main responsibilities |
|---------------|---------|-----------------------|
| Jetson host | Ubuntu 22.04 | RGB-D detection, dialogue runner, Orbbec SDK |
| Jetson Docker (`ros_noetic`) | ROS Noetic | ROS core, LiDAR drivers, SLAM / AMCL, move_base, target follower, UDP bridges |
| Raspberry Pi | ROS Noetic native | `RosAria`, `odom_republisher`, chassis odometry / TF |

### Hardware and Compute

| Component | Current role |
|----------|---------------|
| Pioneer 3-AT | Base platform |
| Unitree 4D LiDAR L1 | Primary scan source for SLAM, AMCL, global costmap |
| RPLIDAR A2 | Short-range supplement for local costmap |
| Orbbec Femto Bolt | RGB-D target detection on host |
| Jetson Orin Nano | Host apps + ROS Docker |
| Raspberry Pi 4 | Base driver |

### Repository Layout

```text
catkin_ws/           ROS workspace (navigation, target follower, launch files)
dialogue/            STT / NLU / TTS runner and assets
handobj_detection/   Current host-side demo detector
trash_detection/     Alternative detector scripts
scripts/             Operational entry points and wrappers
doc.md               Older, more detailed runbook
```

### Install and Build

Clone and build:

```bash
git clone <repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial
git submodule update --init --recursive
git checkout main
cd catkin_ws && catkin_make
source devel/setup.bash
```

Verify:

```bash
rospack find p3at_lms_navigation
```

### Runtime Dependencies

#### Jetson Docker

The scripts expect a running Docker container named `ros_noetic` (image/tag can vary) with ROS Noetic and these packages available:
- navigation packages (`move_base`, `gmapping`, `amcl`, `map_server`, `dwa_local_planner`)
- `unitree_lidar_ros`
- `pointcloud_to_laserscan`

Common management commands:

| Action | Command |
|--------|---------|
| Start existing container | `docker start ros_noetic` |
| Enter as user | `docker exec -it --user "$(id -u):$(id -g)" ros_noetic bash` |
| Restart | `docker restart ros_noetic` |
| Stop | `docker stop ros_noetic` |

When entering the container manually, do **not** use a root shell unless you really intend to.

#### Raspberry Pi

```bash
sudo apt-get install -y ros-noetic-rosaria
```

`./scripts/start_base.sh` uses native ROS on the Pi; the current repo does **not** use a Pi-side Docker container for `RosAria`.

#### Jetson Host

Typical host-side dependencies:

```bash
pip3 install ultralytics opencv-python numpy scipy pillow
cd dialogue && pip install -r requirements.txt
sudo apt-get install -y libportaudio2 portaudio19-dev
```

#### Optional Helper Setup

```bash
./setup_rplidar_a2.sh
./setup_unitree_lidar.sh
```

### Network and ROS Environment

Default real-robot network:
- Jetson: `192.168.50.1`
- Raspberry Pi: `192.168.50.2`
- ROS master: `http://192.168.50.1:11311`

Environment helpers:

```bash
source scripts/env.sh jetson jetson
source scripts/env.sh raspi jetson
```

Typical environment values:

```bash
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=192.168.50.1   # Jetson
export ROS_IP=192.168.50.2   # Pi
```

### RPLIDAR Bringup Profiles

The current branch intentionally uses two different default RPLIDAR profiles:

| Entry point | Defaults |
|------------|----------|
| `start_demo.sh`, `target_follow_real.launch`, RPLIDAR-only flows | `256000`, pre-start motor `true`, warmup `2.0 s` |
| `start_real_mapping_unitree.sh`, `start_real_nav_unitree.sh` | `115200`, pre-start motor `false`, warmup `0.0 s` |

This split reflects current hardware behaviour: the demo / target-follow path uses a more aggressive A2 workaround, while Unitree-led mapping / nav keeps a more conservative profile unless you override it.

### Git Notes

- Current working branch for this README baseline: `main`
- Generated runtime maps `maps/*.pgm` and `maps/*.yaml` are tracked by default
- Runtime log files and temporary sidecars are partially ignored, but generated maps are **not** globally ignored

<a id="interface-documentation"></a>
## Interface Documentation

### UDP Ports

| Port | Direction | Purpose |
|------|-----------|---------|
| `16031` | Host detector -> Docker ROS | Target XYZ UDP bridge |
| `16041` | Docker ROS -> Host dialogue | Navigation success trigger |
| `16032` | Host dialogue -> Docker ROS | Dialogue action result |

### Key ROS Topics and Actions

| Name | Type | Direction | Meaning |
|------|------|-----------|---------|
| `/unitree/scan` | `sensor_msgs/LaserScan` | pub | Primary lidar scan |
| `/rplidar/scan_filtered` | `sensor_msgs/LaserScan` | pub | Supplementary local-costmap scan |
| `/odom` | `nav_msgs/Odometry` | pub | Chassis odometry from Pi |
| `/trash_detection/target_point` | `geometry_msgs/PointStamped` | sub | 3D target from UDP bridge |
| `/target_pose` | `geometry_msgs/PoseStamped` | internal | Target pose consumed by follower |
| `/move_base` | `MoveBaseAction` | action | Global exploration / overlay navigation |
| `/move_base_tracking` | `MoveBaseAction` | action | Optional local-only tracking action server |
| `/target_follower/status` | `std_msgs/String` | pub | State-machine status |
| `/target_follower/result` | `std_msgs/Bool` | pub | Target reached / reset state |
| `/trash_action` | `std_msgs/Bool` | sub | Dialogue decision (`true` accept, `false` decline) |
| `/cmd_vel` | `geometry_msgs/Twist` | pub | Final chassis motion command |

### Key Frames

- `map`: authoritative frame in online SLAM or AMCL modes
- `odom`: chassis odometry frame from `RosAria`
- `base_link`: robot body frame
- `unitree_lidar`: Unitree scan frame
- `laser`: RPLIDAR frame
- `camera_link`: Orbbec target frame

Real-robot TF ownership:
- `map -> odom`: `slam_gmapping` or `amcl`
- `odom -> base_link`: `RosAria`
- `base_link -> unitree_lidar`: `robot_state_publisher`
- `base_link -> laser`: `robot_state_publisher` (from `p3at_unitree.urdf.xacro`)
- `base_link -> camera_link`: `robot_state_publisher` (from `p3at_unitree.urdf.xacro`)

### Target Follower State Summary

| State | Meaning |
|-------|---------|
| `IDLE` | Waiting for a target or explore trigger |
| `EXPLORING` | No fresh target; sends short exploration goals |
| `TRACKING` | Sends / updates follow goals through `move_base` or `move_base_tracking` |
| `REACHED` | Target is within standoff distance; publishes success |
| `WAITING_ACTION` | Waiting for dialogue result on `/trash_action` |
| `POST_ACCEPT_COOLDOWN` | Pause after an accepted interaction |
| `RETREATING` | Reverse-first if safe, then turn and leave |
| `CLOSE_APPROACH` | Optional direct-`/cmd_vel` final approach |

Current default behaviour:
- `target_follow_real.launch` default keeps `enable_close_approach = false`
- `start_demo.sh` overrides this to `enable_close_approach = true` with `close_approach_threshold = 1.2`
- after interaction, retreat is reverse-first-if-safe, then moderate turn, then forward leave

### High-Value Parameters

`target_follow_real.launch` highlights:

| Parameter | Current default | Notes |
|----------|------------------|-------|
| `standoff_distance` | `0.6` | Launch default; `start_demo.sh` overrides to `0.8` |
| `enable_close_approach` | `false` | Launch default; `start_demo.sh` overrides to `true` |
| `close_approach_threshold` | `1.5` | Launch default; `start_demo.sh` overrides to `1.2` |
| `tracking_frame` | `odom` | Standalone tracking goals are sent in `odom` |
| `tracking_action_name` | `move_base_tracking` | Tracking uses local-only action server in standalone mode |
| `explore_goal_distance` | `2.2` | `start_demo.sh` overrides to `2.4` |
| `action_wait_timeout` | `45.0` | Dialogue wait timeout |
| `retreat_distance` | `1.5` | Forward leave distance |
| `retreat_reverse_distance` | `0.50` | Reverse-first stage |
| `retreat_turn_angle_deg` | `100.0` | Moderate leave turn |

`start_demo.sh` highlights:

| Option | Default | Notes |
|--------|---------|-------|
| `--lidar` | `dual` | `dual`, `unitree`, or `rplidar` |
| `--target` | `holding` | Detector metadata label |
| `--assist-map` | unset | Enables map-assisted mode |
| `--sensor-only` | off | Lidar-only validation |
| `--dialogue-device` | `24` | Host microphone index |
| `--nav-readiness` | `strict` | Requires live nav status before continue |

For the full parameter surface, inspect:
- `catkin_ws/src/target_follower/launch/target_follow_real.launch`
- `./scripts/start_demo.sh --help`

### JSON Interfaces

Detector UDP payload (host -> ROS):

```json
{
  "stamp": 1708700000.123,
  "frame_id": "camera_link",
  "x": 0.12,
  "y": -0.08,
  "z": 2.35,
  "source": "handobj_detection_rgbd_remote_15cls",
  "kind": "holding"
}
```

Dialogue result payload (host -> ROS):

```json
{
  "stamp": 1708700002.456,
  "trash_action": 1,
  "decision": "proceed",
  "source": "dialogue_udp_runner"
}
```

<a id="technical-implementation"></a>
## Technical Implementation

### Runtime Architecture

The current stack is split into three layers:
- **Jetson host**: camera / detector / dialogue
- **Jetson Docker**: ROS graph, navigation, target follower, UDP bridges
- **Raspberry Pi**: base driver and odometry

This keeps camera and GPU dependencies off the ROS container while preserving a stable ROS1 control plane.

### Operating Modes

| Mode | How it is entered | Use case |
|------|-------------------|----------|
| `online_slam_task` | default `./scripts/start_demo.sh` | No prior map; task execution while building a live SLAM map |
| `map_assisted` | `./scripts/start_demo.sh --assist-map /path/to/map.yaml` | Use saved map + AMCL while keeping the full demo pipeline |
| Overlay mode | `target_follow_real.launch launch_move_base:=false` | Add target following on top of an already running navigation backbone |

### Sensor Policy

Current dual-lidar policy is intentionally asymmetric:
- Unitree L1 owns `gmapping`, `amcl`, and the global costmap
- local obstacle avoidance may fuse Unitree + RPLIDAR
- RPLIDAR is a supplement, not the authoritative global mapping sensor

### Perception-to-Action Flow

1. The host detector publishes target XYZ over UDP `16031` in `camera_link`.
2. `udp_target_bridge.py` republishes it as `/trash_detection/target_point`.
3. `point_to_target_pose.py` converts it to `/target_pose`.
4. `target_follower.py` transforms the pose into the active planning frame and owns follow / explore goals.
5. On success, `navigation_success_udp_bridge.py` sends a UDP trigger to the host dialogue runner on `16041`.
6. The dialogue runner returns a decision on `16032`, and `udp_trash_action_bridge.py` republishes it to `/trash_action`.
7. The robot either pauses briefly for accepted trash drop or retreats immediately on refusal.

### Current Behavioural Defaults

Important current defaults in this branch:
- `start_demo.sh` launches the host detector `handobj_detection_rgbd_remote_15cls.py`
- default detector `kind` is `holding`
- `start_demo.sh` uses `--rotate-180`, `--headless`, and `--print-xyz` by default
- standalone tracking uses `tracking_frame=odom` + `tracking_action_name=move_base_tracking`
- `start_demo.sh` enables `enable_close_approach=true` by default (launch default remains `false`)
- dialogue retrigger support exists in code, but `scripts/start_dialogue_docker_bridges.sh` defaults `DIALOGUE_RETRIGGER_ENABLE=false`

### Navigation / Planner Defaults Worth Knowing

Selected current values from the Unitree profile:

| Parameter | Current value |
|----------|----------------|
| `inflation_radius` | `0.30` |
| `max_vel_x` | `0.22` |
| `min_vel_x` | `-0.05` |
| `max_vel_theta` | `0.55` |
| `acc_lim_x` | `0.30` |
| `acc_lim_theta` | `0.90` |
| `sim_time` | `1.5` |
| `occdist_scale` | `0.08` |
| `xy_goal_tolerance` | `0.15` |
| `clearing_rotation_allowed` | `true` |

For the full parameter set, inspect:
- `catkin_ws/src/p3at_lms_navigation/param/unitree/`
- `catkin_ws/src/target_follower/launch/target_follow_real.launch`

### Main Entry Points

| File | Responsibility |
|------|----------------|
| `scripts/start_demo.sh` | Top-level supervisor for the real-robot demo |
| `scripts/start_base.sh` | Raspberry Pi base bringup |
| `scripts/start_real_mapping_unitree.sh` | Real-robot mapping |
| `scripts/start_real_nav_unitree.sh` | Saved-map navigation |
| `catkin_ws/src/target_follower/launch/target_follow_real.launch` | Main target-follow launch entry |
| `catkin_ws/src/target_follower/scripts/target_follower.py` | Tracking / exploration / dialogue state machine |
| `handobj_detection/handobj_detection_rgbd_remote_15cls.py` | Current default host detector |
| `dialogue/dialogue_udp_runner.py` | Host dialogue runtime |

<a id="experiment-report"></a>
## Experiment Report

Only the latest retained **dual-lidar simulation** summary is kept in this README. Historical per-run detail has been intentionally removed from the main document.
The full-system experiment data record sheet is stored at `ELEC70015_Human-Centered-Robotics-2026_Imperial/WasteMate_experiment_record_updated(Record_Sheet).csv`.

### Retained Dual-Lidar Simulation Summary

| Experiment | Scenario | Key result |
|-----------|----------|------------|
| Exp1 | Simple obstacle fixed-point navigation | `3/3` waypoints succeeded; mean goal error `0.1475 m` |
| Exp2 | Complex obstacle fixed-point navigation | `4/5` waypoints succeeded; mean goal error `0.1530 m` |
| Exp3 | Target following | stable tracking retained; high-speed profile (`0.36 m/s`) reached `68.8%` standoff tolerance with `0` false terminations |
| Exp4 | Autonomous mapping + AMCL verification | short-horizon exploration judged feasible; tuned AMCL verification passed `6/6` waypoints |

Current reading of those results:
- fixed-point navigation is solid in simple scenes and mostly solid in clutter
- target following is usable, but distance tightness can still improve
- autonomous exploration and subsequent AMCL verification are both viable in the retained dual-lidar simulation setup

### Current Real-Robot Status

Verified on the current branch:
- Unitree lidar integration and dual-lidar bringup
- Raspberry Pi base driver workflow
- mapping and saved-map navigation scripts
- target following with YOLO + depth camera
- dialogue trigger / result bridge chain
- optional `CLOSE_APPROACH` logic present in code, and enabled by default in `start_demo.sh` (launch default remains disabled)

Still pending:
- bin motor driver integration
- full trash-collection demo with the bin motor
- combined multi-target pickup workflow

<a id="team-contribution"></a>
## Team Contribution

Contributors are listed in alphabetical order by surname initial.

- `Haocheng Fan` — computer vision and target detection, report contributor
- `Yaohan Huang` — computer vision and target detection, report contributor
- `Quincy Li` — multi-system integration, experiments and testing, report contributor
- `Zian Lin` — hardware system, robot body / platform design, report contributor
- `Yuchen Liu` — robot infrastructure (URDF, TF relationships, ROS node communication), ROS simulation, navigation algorithms, real-robot navigation deployment, group management, report editor
- `Guanxi Lu` — dialogue system (STT, TTS, NLU Classifier, Decision Manager Interface, Hardware I/O), report contributor
- `Wenxin Tang` — hardware system, rotary distributor motor control, report contributor
- `Tony Zeng` — hardware system, rotary distributor design, report contributor
