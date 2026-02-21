# Launcher 使用说明（Jetson + RasPi）

## 1. 前置要求

1. 已安装 ROS Noetic，且 `catkin_ws` 已编译通过：
```bash
cd ...<path>.../ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

2. 已配置 `scripts/deploy.env`（每台机器都建议配置一份本地副本）：
```bash
JETSON_IP=192.168.50.1
RASPI_IP=192.168.50.2
LAPTOP_IP=172.26.183.130
```

3. Jetson 与 RasPi 网络互通（例如 `ping 192.168.50.2` 可通）。

## 2. 脚本职责

- `scripts/start_master.sh [jetson|laptop]`：设置网络环境并启动 `roscore`。
- `scripts/start_base.sh`：在 `raspi` 角色下启动 `p3at_base/base.launch`（底盘驱动）。
- `scripts/start_real_mapping.sh`：在 `jetson` 角色下启动真机建图。
- `scripts/start_real_nav.sh`：在 `jetson` 角色下启动真机导航（AMCL + map）。
- `scripts/start_sim_mapping.sh`：本地仿真建图。
- `scripts/start_sim_nav.sh`：本地仿真导航。
- `scripts/env.sh <role> [master_host]`：统一设置 `ROS_MASTER_URI` 与 `ROS_IP`，并自动 `source` ROS/catkin 环境。

## 3. 真机启动顺序（推荐）

### 3.1 Jetson 终端 1：启动 ROS Master
```bash
cd /home/frank/quincy_workspace/ELEC70015_Human-Centered-Robotics-2026_Imperial
scripts/start_master.sh jetson
```
作用：启动 `roscore`，作为全系统主控。

### 3.2 RasPi 终端：启动底盘
```bash
cd /home/frank/quincy_workspace/ELEC70015_Human-Centered-Robotics-2026_Imperial
scripts/start_base.sh
```
作用：启动 RosAria，发布 `/odom` 并接收 `/cmd_vel`。

### 3.3 Jetson 终端 2：启动导航主栈（二选一）

1. 建图模式：
```bash
scripts/start_real_mapping.sh
```

2. 导航模式（用已有地图）：
```bash
scripts/start_real_nav.sh map_file:=/absolute/path/to/your_map.yaml
```

## 4. 仿真启动顺序

### 4.1 启动本地 master
```bash
cd /home/frank/quincy_workspace/ELEC70015_Human-Centered-Robotics-2026_Imperial
scripts/start_master.sh laptop
```

### 4.2 启动仿真（二选一）
```bash
scripts/start_sim_mapping.sh
scripts/start_sim_nav.sh
```

## 5. 常用参数说明（真机 launch）

适用：`start_real_mapping.sh` / `start_real_nav.sh`

- `lms200_port:=/dev/ttyUSB0`：LMS200 串口设备名。
- `lms200_baud:=38400`：LMS200 波特率。
- `use_rviz:=true|false`：是否启动 RViz。
- `map_file:=...`：仅 `real_nav` 需要，指定地图 yaml。
- `use_target_follower:=true|false`：是否启用目标跟随。
- `use_point_target_bridge:=true|false`：是否启用点到位姿桥接。
- `point_target_topic:=/trash_detection/target_point`：目标点输入话题（YOLO 常用）。
- `point_target_frame:=base_link|map|odom`：桥接输出 frame 覆盖。
- `use_mock_detector:=true|false`：是否启用 mock 检测器。
- `standoff_distance:=1.0`：与目标保持距离（米）。
- `face_target:=true|false`：到达目标后是否朝向目标。

示例（YOLO 点目标）：
```bash
scripts/start_real_nav.sh \
  use_target_follower:=true \
  use_point_target_bridge:=true \
  point_target_topic:=/trash_detection/target_point \
  point_target_frame:=base_link \
  standoff_distance:=1.0 \
  face_target:=true
```

示例（mock 测试）：
```bash
scripts/start_real_nav.sh \
  use_target_follower:=true \
  use_mock_detector:=true \
  use_point_target_bridge:=true \
  mock_mode:=circle
```

## 6. `MASTER_HOST` 覆盖

默认拓扑下：
- `start_base.sh` 默认连接 Jetson master。
- `start_real_*` 默认 Jetson 作为 master。
- `start_sim_*` 默认 laptop 作为 master。

如需覆盖，可这样传：
```bash
MASTER_HOST=laptop scripts/start_base.sh
```

## 7. 快速联通检查

在 Jetson 上执行：
```bash
source scripts/env.sh jetson jetson
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
rostopic list | egrep "/odom|/scan|/cmd_vel"
rostopic hz /odom
rostopic hz /scan
```

期望：
- `/odom` 有频率：说明 RasPi 底盘链路正常。
- `/scan` 有频率：说明 Jetson 雷达链路正常。
- `/cmd_vel` 可见：说明导航控制输出正常。