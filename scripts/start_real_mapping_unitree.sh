#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# ============================================================================
# start_real_mapping_unitree.sh
# 真机建图 — Unitree 4D Lidar L1
#
# 运行机器（Jetson Orin Nano）：
#   ./scripts/start_real_mapping_unitree.sh
#
# 可选参数（透传给 roslaunch）：
#   unitree_port:=/dev/ttyUSB0   Unitree LiDAR USB 串口（默认 /dev/ttyUSB0）
#   use_rviz:=true               是否同时启动 RViz（默认 true）
#   enable_sick:=false           是否同时启用 SICK LMS200（默认 false）
#   sick_port:=/dev/ttyUSB1      SICK USB 串口（默认 /dev/ttyUSB1）
#
# 建图完成后（Ctrl+C 停止建图前）保存地图：
#   rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map_unitree
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"

exec roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch "$@"
