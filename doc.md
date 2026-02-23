# Demo v1 Runbook (release/demo-v1)

This branch is organized around three practical parts:

1. Navigation + Raspberry Pi communication (ROS1 core stack)
2. Trash detection (non-ROS on host) + UDP bridge to ROS1 in Docker
3. Lidar-only workflow (TBD)

## 1. Navigation + RasPi Communication

### 1.1 Configure shared network settings

Edit `scripts/deploy.env` on your machine:

```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
LAPTOP_IP=<your_lan_ip>
TRASH_UDP_PORT=16031
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
- Host runs only `trash_detection/predict_15cls_rgbd.py` (non-ROS, GPU inference).
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

Default launcher:
```bash
./scripts/start_trash_detection_rgbd.sh
```

Equivalent explicit command:
```bash
python3 trash_detection/predict_15cls_rgbd.py \
  --nearest-person --print-xyz --headless \
  --udp-enable --udp-host 127.0.0.1 --udp-port 16031
```

Current default behavior:
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

## 3. Lidar Part (TBD)

Lidar-only workflow is intentionally left as TBD in this demo branch.
Current priority is the integrated navigation + target-follow pipeline above.
