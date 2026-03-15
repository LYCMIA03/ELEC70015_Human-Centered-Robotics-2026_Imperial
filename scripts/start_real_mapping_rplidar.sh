#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# ============================================================================
# start_real_mapping_rplidar.sh
# 真机建图 — RPLIDAR A2M12
#
# 用法：
#   ./scripts/start_real_mapping_rplidar.sh
#   ./scripts/start_real_mapping_rplidar.sh rplidar_port:=/dev/ttyUSB0 use_rviz:=false
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/rplidar_preflight.sh"

rplidar_preflight::run "$@"

exec roslaunch p3at_lms_navigation real_robot_mapping_rplidar.launch "$@"
