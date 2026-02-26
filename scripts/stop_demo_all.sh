#!/usr/bin/env bash
# =============================================================================
# stop_demo_all.sh — Stop all demo-related components (host + docker),
#                    but DO NOT stop the Docker container itself.
#
# Usage:
#   ./scripts/stop_demo_all.sh
#   DOCKER_NAME=ros_noetic ./scripts/stop_demo_all.sh
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; BLU='\033[0;34m'; NC='\033[0m'
info() { echo -e "${BLU}[INFO]${NC}  $*"; }
ok()   { echo -e "${GRN}[OK]${NC}    $*"; }
err()  { echo -e "${RED}[FATAL]${NC} $*" >&2; exit 1; }

DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
DOCKER_BIN="$(command -v docker || true)"
if [[ -z "${DOCKER_BIN}" && -x "/usr/bin/docker" ]]; then
  DOCKER_BIN="/usr/bin/docker"
fi
[[ -n "${DOCKER_BIN}" ]] || err "docker command not found"

if ! "${DOCKER_BIN}" ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
  err "Docker container '${DOCKER_NAME}' is not running"
fi

info "Stopping host-side demo processes..."
pkill -f "start_demo.sh" 2>/dev/null || true
pkill -f "start_dialogue_docker_bridges.sh" 2>/dev/null || true
pkill -f "dialogue/dialogue_udp_runner.py" 2>/dev/null || true
pkill -f "handobj_detection_rgbd.py" 2>/dev/null || true
pkill -f "navigation_success_udp_bridge.py" 2>/dev/null || true
pkill -f "udp_trash_action_bridge.py" 2>/dev/null || true

info "Stopping docker-side ROS components in '${DOCKER_NAME}'..."
"${DOCKER_BIN}" exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc '
  pkill -f "roslaunch.*target_follow_real.launch" 2>/dev/null || true
  pkill -f "roslaunch.*target_follow.launch" 2>/dev/null || true
  pkill -f "roslaunch.*real_robot" 2>/dev/null || true
  pkill -f "move_base" 2>/dev/null || true
  pkill -f "unitree_lidar_ros_node" 2>/dev/null || true
  pkill -f "pointcloud_to_laserscan_node" 2>/dev/null || true
  pkill -f "target_follower.py" 2>/dev/null || true
  pkill -f "udp_target_bridge.py" 2>/dev/null || true
  pkill -f "point_to_target_pose.py" 2>/dev/null || true
  pkill -f "navigation_success_udp_bridge.py" 2>/dev/null || true
  pkill -f "udp_trash_action_bridge.py" 2>/dev/null || true
  pkill -f "odom_republisher.py" 2>/dev/null || true
  pkill -f "RosAria" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
  pkill -f "rosout" 2>/dev/null || true
  pkill -f "roscore" 2>/dev/null || true
  pkill -f "rosmaster" 2>/dev/null || true
'

sleep 1

info "Verification (container still running, ROS components stopped)..."
"${DOCKER_BIN}" ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$" \
  && ok "Docker container '${DOCKER_NAME}' is still running"

"${DOCKER_BIN}" exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc '
  ps -ef | grep -E "roslaunch|rosmaster|roscore|move_base|unitree_lidar_ros_node|target_follower.py|udp_target_bridge.py|point_to_target_pose.py|navigation_success_udp_bridge.py|udp_trash_action_bridge.py" | grep -v grep || true
'

ok "Demo components stopped."

