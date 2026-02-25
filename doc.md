# Demo v1 Runbook (release/demo-v1)

This branch is organized around three practical parts:

1. Navigation + Raspberry Pi communication (ROS1 core stack)
2. Trash detection (non-ROS on host) + UDP bridge to ROS1 in Docker
3. Dialogue action bridge (navigation_success -> voice decision -> trash_action)
4. Lidar-only workflow (TBD)

## 1. Navigation + RasPi Communication

### 1.1 Configure shared network settings

Edit `scripts/deploy.env` on your machine:

```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
LAPTOP_IP=<your_lan_ip>
TRASH_UDP_PORT=16031
DIALOGUE_TRIGGER_UDP_PORT=16041
DIALOGUE_ACTION_UDP_PORT=16032
```

Notes:
- `11311` is ROS master port.
- `TRASH_UDP_PORT` is detector->ROS bridge UDP port (separate from 11311).

### 1.2 Recommended real-robot startup order

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

Useful check:
```bash
source scripts/env.sh jetson jetson
rostopic list | egrep "/odom|/scan|/cmd_vel"
rostopic hz /odom
rostopic hz /scan
```

## 2. Trash Detection + UDP Bridge

Architecture:
- Host runs one detector on host (non-ROS, GPU inference):
  - `trash_detection/predict_15cls_rgbd.py` (default)
  - `handobj_detection/handobj_detection_rgbd.py` (`--detector handobj`)
- All ROS chain runs inside Docker:
  - `udp_target_bridge.py` publishes `/trash_detection/target_point`
  - `point_to_target_pose.py` converts to `/target_pose`
  - `target_follower.py` consumes `/target_pose`

### 2.1 Start ROS side (inside Docker)

Terminal A:
```bash
docker exec -it ros_noetic bash -lc 'source /opt/ros/noetic/setup.bash && roscore'
```

Terminal B:
```bash
docker exec -it ros_noetic bash -lc '
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/udp_target_bridge.py _bind_port:=16031
'
```

Terminal C (optional transform relay):
```bash
docker exec -it ros_noetic bash -lc '
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/point_to_target_pose.py
'
```

Terminal D (observe input topic):
```bash
docker exec -it ros_noetic bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo /trash_detection/target_point'
```

### 2.2 Start host detection sender

Default launcher (trash detector):
```bash
./scripts/start_trash_detection_rgbd.sh
```

Switch to hand-object detector:
```bash
./scripts/start_trash_detection_rgbd.sh --detector handobj
```

Equivalent explicit command (trash):
```bash
python3 trash_detection/predict_15cls_rgbd.py \
  --nearest-person --print-xyz --headless \
  --udp-enable --udp-host 127.0.0.1 --udp-port 16031
```

Equivalent explicit command (handobj):
```bash
python3 handobj_detection/handobj_detection_rgbd.py \
  --print-xyz --headless \
  --udp-enable --udp-host 127.0.0.1 --udp-port 16031
```

Current default behavior:
- Launcher default is `--detector trash`.
- UDP sends `person` target by default.
- Override when needed:
```bash
UDP_KIND=waste ./scripts/start_trash_detection_rgbd.sh
```

### 2.3 Smoke test without camera/inference

```bash
python3 trash_detection/examples/send_target_udp.py --port 16031 --rate 2 --count 5
```

If Terminal D prints `PointStamped`, bridge chain is healthy.

## 3. Dialogue Action Bridge

Goal:
- ROS in Docker publishes `/navigation_success` (`std_msgs/Bool`).
- A ROS->UDP bridge sends this trigger to host.
- Host runs dialogue module when trigger is `1`, gets `decline/proceed`.
- Host sends back `decline=0`, `proceed=1` via UDP.
- A UDP->ROS bridge in Docker publishes `/trash_action` (`std_msgs/Bool`).

Topics and ports used in this chain:
- ROS input trigger: `/navigation_success` (`std_msgs/Bool`)
- ROS output action: `/trash_action` (`std_msgs/Bool`)
- UDP trigger port: `16041` (`DIALOGUE_TRIGGER_UDP_PORT`)
- UDP action port: `16032` (`DIALOGUE_ACTION_UDP_PORT`)

### 3.1 One-command startup (recommended)

Docker side (inside `ros_noetic` container):
```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_docker_bridges.sh
```

Host side:
```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_host.sh --device 24
```

Host simulation mode (no microphone):
```bash
cd /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial
./scripts/start_dialogue_host.sh --sim \
  --first-user-wav dialogue/voice_data/sim_user_answer_other_b.wav \
  --second-user-wav dialogue/voice_data/sim_user_answer_negative_a.wav
```

Trigger once from Docker (for debugging):
```bash
rostopic pub -1 /navigation_success std_msgs/Bool "data: true"
```

Observe result (for debugging):
```bash
rostopic echo /trash_action
```

### 3.2 Start ROS bridges manually (inside Docker)

Terminal E (`/navigation_success` -> UDP trigger):
```bash
docker exec -it ros_noetic bash -lc '
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/navigation_success_udp_bridge.py \
  _in_topic:=/navigation_success _out_host:=127.0.0.1 _out_port:=16041
'
```

Terminal F (UDP action -> `/trash_action`):
```bash
docker exec -it ros_noetic bash -lc '
source /opt/ros/noetic/setup.bash
python3 /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/src/target_follower/scripts/udp_trash_action_bridge.py \
  _bind_port:=16032 _out_topic:=/trash_action
'
```

Terminal G (observe output):
```bash
docker exec -it ros_noetic bash -lc 'source /opt/ros/noetic/setup.bash && rostopic echo /trash_action'
```

### 3.3 Start host dialogue runner manually

Microphone mode:
```bash
./scripts/start_dialogue_udp_runner.sh --device 24
```

Simulation mode:
```bash
./scripts/start_dialogue_udp_runner.sh --sim \
  --first-user-wav dialogue/voice_data/sim_user_answer_other_b.wav \
  --second-user-wav dialogue/voice_data/sim_user_answer_negative_a.wav
```

### 3.4 Trigger test

In Docker, publish trigger:
```bash
docker exec -it ros_noetic bash -lc '
source /opt/ros/noetic/setup.bash
rostopic pub -1 /navigation_success std_msgs/Bool "data: true"
'
```

Expected:
- Host prints `[Result] outcome=proceed|decline -> trash_action=1|0`
- Docker topic `/trash_action` receives `True|False`

## 4. Lidar Part (TBD)

Lidar-only workflow is intentionally left as TBD in this demo branch.
Current priority is the integrated navigation + target-follow pipeline above.
