#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# ============================================================================
# start_real_nav_rplidar.sh
# 真机导航 — RPLIDAR A2M12
#
# 用法：
#   ./scripts/start_real_nav_rplidar.sh map_file:=/path/to/map.yaml
#   ./scripts/start_real_nav_rplidar.sh rplidar_port:=/dev/rplidar_lidar use_rviz:=false
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/rplidar_preflight.sh"

# Keep the workaround at the entrypoint so it can be reverted by removing these args.
rplidar_preflight::run "$@" \
  "rplidar_pre_start_motor:=true" \
  "rplidar_pre_start_motor_pwm:=600" \
  "rplidar_pre_start_motor_warmup_s:=2.0"

exec roslaunch p3at_lms_navigation real_robot_nav_rplidar.launch "$@"
