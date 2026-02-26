# Demo v1 Runbook (release/demo-v1)

This document defines startup flows by module and uses one Docker convention:

- Always use `ros_noetic` helper commands on Jetson.
- Enter container with `ros_noetic e` (non-root), not bare `docker exec`.

The same flows are merged into `README.md` under "Module Startup Flows".

## Docker Operation Convention (Unified)

On Jetson:

```bash
# Start/create container
ros_noetic s

# Enter container shell (non-root)
ros_noetic e

# Restart container
ros_noetic r

# Stop container
ros_noetic c
```

Inside `ros_noetic e`, source ROS/catkin when needed:

```bash
source /opt/ros/noetic/setup.bash
source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash
```

## 1. Navigation + RasPi Communication (Startup Flow)

### 1.1 Configure shared network settings

Edit `scripts/deploy.env`:

```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
LAPTOP_IP=<your_lan_ip>
TRASH_UDP_PORT=16031
DIALOGUE_TRIGGER_UDP_PORT=16041
DIALOGUE_ACTION_UDP_PORT=16032
```

### 1.2 Recommended startup order

Terminal A (Jetson):
```bash
scripts/start_master.sh jetson
```

Terminal B (Raspberry Pi):
```bash
scripts/start_base.sh
```

Terminal C (Jetson):
```bash
# Mapping mode
scripts/start_real_mapping.sh

# OR nav mode with an existing map
scripts/start_real_nav.sh map_file:=/absolute/path/to/map.yaml
```

## 2. Trash Detection + UDP Bridge (Startup Flow)

Architecture:
- Host detector (non-ROS):
  - `trash_detection/predict_15cls_rgbd.py` (default)
  - `handobj_detection/handobj_detection_rgbd.py` (`--detector handobj`)
- Docker ROS chain:
  - `udp_target_bridge.py` -> `/trash_detection/target_point`
  - `point_to_target_pose.py` -> `/target_pose`
  - `target_follower.py` consumes `/target_pose`

### 2.1 Start ROS side (inside container via `ros_noetic e`)

Terminal A:
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
roscore
```

Terminal B:
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/udp_target_bridge.py _bind_port:=16031
```

Terminal C (optional transform relay):
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/point_to_target_pose.py
```

Terminal D (observe):
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
rostopic echo /trash_detection/target_point
```

### 2.2 Start host detection sender

Default:
```bash
./scripts/start_trash_detection_rgbd.sh
```

Hand-object:
```bash
./scripts/start_trash_detection_rgbd.sh --detector handobj
```

Override target kind:
```bash
UDP_KIND=waste ./scripts/start_trash_detection_rgbd.sh
```

### 2.3 Smoke test (no camera/inference)

```bash
python3 trash_detection/examples/send_target_udp.py --port 16031 --rate 2 --count 5
```

## 3. Dialogue Action Bridge (Startup Flow)

Goal:
- ROS publishes `/target_follower/result`.
- ROS->UDP bridge triggers host dialogue.
- Host sends decision back by UDP.
- UDP->ROS bridge publishes `/trash_action`.

Ports:
- Trigger: `16041` (`DIALOGUE_TRIGGER_UDP_PORT`)
- Action: `16032` (`DIALOGUE_ACTION_UDP_PORT`)

### 3.1 One-command startup (recommended)

Docker side:
```bash
ros_noetic e
# inside container:
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_docker_bridges.sh
```

Host side:
```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_host.sh --device 24
```

Host simulation:
```bash
./scripts/start_dialogue_host.sh --sim \
  --first-user-wav voice_data/sim_user_answer_other_b.wav \
  --second-user-wav voice_data/sim_user_answer_negative_a.wav
```

### 3.2 Start ROS bridges manually (inside container via `ros_noetic e`)

Terminal E (`/target_follower/result` -> UDP trigger):
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/navigation_success_udp_bridge.py \
  _in_topic:=/target_follower/result _out_host:=127.0.0.1 _out_port:=16041
```

Terminal F (UDP action -> `/trash_action`):
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/udp_trash_action_bridge.py \
  _bind_port:=16032 _out_topic:=/trash_action
```

Terminal G (observe output):
```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
rostopic echo /trash_action
```

### 3.3 Trigger test

```bash
ros_noetic e
# inside container:
source /opt/ros/noetic/setup.bash
rostopic pub -1 /target_follower/result std_msgs/Bool "data: true"
```

Expected:
- Host prints `[Result] outcome=proceed|decline -> trash_action=1|0`
- Docker topic `/trash_action` receives `True|False`

## 4. Lidar Part (TBD)

Lidar-only workflow remains TBD in this document.
