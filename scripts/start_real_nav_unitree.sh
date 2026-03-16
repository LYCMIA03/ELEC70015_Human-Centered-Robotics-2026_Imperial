#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# ============================================================================
# start_real_nav_unitree.sh
# 真机导航 — Unitree 4D Lidar L1 + AMCL (支持双雷达局部避障)
#
# 用法：
#   ./scripts/start_real_nav_unitree.sh map_file:=/path/to/map.yaml
#
# 常用参数（透传给 roslaunch）：
#   map_file:=...                  地图 yaml 路径
#   unitree_port:=/dev/unitree_lidar     Unitree 串口
#   enable_rplidar:=true           启用第二个雷达（默认 true）
#   rplidar_port:=/dev/rplidar_lidar     RPLIDAR 串口
#   rplidar_baud:=115200           RPLIDAR 波特率
#   rplidar_pre_start_motor:=true  在初始化前先拉起第二个雷达电机
#   rplidar_pre_start_motor_pwm:=600
#   rplidar_pre_start_motor_warmup_s:=2.0
#   use_rviz:=true                 是否启动 RViz
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"

ENABLE_RPLIDAR="true"
RPLIDAR_PORT="/dev/rplidar_lidar"
RPLIDAR_BAUD="115200"
RPLIDAR_PRE_START_MOTOR="false"
RPLIDAR_PRE_START_PWM="600"
RPLIDAR_PRE_START_WARMUP_S="0.0"
for arg in "$@"; do
  case "${arg}" in
    enable_rplidar:=*) ENABLE_RPLIDAR="${arg#*=}" ;;
    rplidar_port:=*)   RPLIDAR_PORT="${arg#*=}" ;;
    rplidar_baud:=*)   RPLIDAR_BAUD="${arg#*=}" ;;
    rplidar_pre_start_motor:=*) RPLIDAR_PRE_START_MOTOR="${arg#*=}" ;;
    rplidar_pre_start_motor_pwm:=*) RPLIDAR_PRE_START_PWM="${arg#*=}" ;;
    rplidar_pre_start_motor_warmup_s:=*) RPLIDAR_PRE_START_WARMUP_S="${arg#*=}" ;;
  esac
done

case "${ENABLE_RPLIDAR,,}" in
  true|1|yes|on)
    if ! rospack find rplidar_ros >/dev/null 2>&1; then
      echo "[start_real_nav_unitree] ERROR: rplidar_ros not found. Run ./setup_rplidar_a2.sh first." >&2
      exit 1
    fi
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/lib/rplidar_preflight.sh"
    rplidar_preflight::run \
      "rplidar_port:=${RPLIDAR_PORT}" \
      "rplidar_baud:=${RPLIDAR_BAUD}" \
      "rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR}" \
      "rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM}" \
      "rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S}"
    ;;
esac

exec roslaunch p3at_lms_navigation real_robot_nav_unitree.launch "$@"
