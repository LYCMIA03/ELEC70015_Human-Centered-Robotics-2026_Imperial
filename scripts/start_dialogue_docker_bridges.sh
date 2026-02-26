#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# =============================================================================
# start_dialogue_docker_bridges.sh — Run ROS↔UDP bridge nodes inside Docker
#
# These bridges require rospy and therefore must run inside the ros_noetic
# container.  Since the container uses --net=host, UDP traffic on 127.0.0.1
# reaches host-side processes (e.g. dialogue_udp_runner.py) transparently.
#
# Bridges:
#   navigation_success_udp_bridge  /target_follower/result → UDP:16041
#   udp_trash_action_bridge        UDP:16032 → /trash_action
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${REPO_ROOT}/catkin_ws"

# ---------- deploy.env for IPs / ports ----------
if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

JETSON_IP="${JETSON_IP:-192.168.50.1}"
DIALOGUE_TRIGGER_UDP_PORT="${DIALOGUE_TRIGGER_UDP_PORT:-16041}"
DIALOGUE_ACTION_UDP_PORT="${DIALOGUE_ACTION_UDP_PORT:-16032}"
NAV_SUCCESS_TOPIC="${NAV_SUCCESS_TOPIC:-/target_follower/result}"
TRASH_ACTION_TOPIC="${TRASH_ACTION_TOPIC:-/trash_action}"
DIALOGUE_TRIGGER_HOST="${DIALOGUE_TRIGGER_HOST:-127.0.0.1}"
# Retry policy for WAITING_ACTION:
# - false (default): single trigger per False->True edge, no periodic retrigger
# - true: periodic retrigger while WAITING_ACTION and no /trash_action received
DIALOGUE_RETRIGGER_ENABLE="${DIALOGUE_RETRIGGER_ENABLE:-false}"
DIALOGUE_RETRIGGER_INTERVAL_S="${DIALOGUE_RETRIGGER_INTERVAL_S:-2.0}"

DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
DOCKER_USER="$(id -u):$(id -g)"
ROS_MASTER="http://${JETSON_IP}:11311"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
DOCKER_BIN="$(command -v docker || true)"
if [[ -z "${DOCKER_BIN}" && -x "/usr/bin/docker" ]]; then
  DOCKER_BIN="/usr/bin/docker"
fi
if [[ -z "${DOCKER_BIN}" ]]; then
  echo "[dialogue-bridges] FATAL: docker command not found."
  echo "  Use host terminal (not container) and ensure Docker is installed."
  echo "  Try: export PATH=/usr/bin:\$PATH"
  exit 1
fi

# ---------- cleanup ----------
cleanup() {
  "${DOCKER_BIN}" exec --user "${DOCKER_USER}" "${DOCKER_NAME}" bash -c \
    "pkill -f 'navigation_success_udp_bridge' 2>/dev/null; \
     pkill -f 'udp_trash_action_bridge' 2>/dev/null" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[dialogue-bridges] Starting inside Docker '${DOCKER_NAME}'..."

# Ensure idempotent startup: clear any stale bridge instances first.
"${DOCKER_BIN}" exec --user "${DOCKER_USER}" "${DOCKER_NAME}" bash -c \
  "pkill -f 'navigation_success_udp_bridge' 2>/dev/null || true; \
   pkill -f 'udp_trash_action_bridge' 2>/dev/null || true; \
   sleep 0.5" 2>/dev/null || true

# --- Bridge 1: nav_success → UDP (background docker exec) ---
"${DOCKER_BIN}" exec --user "${DOCKER_USER}" "${DOCKER_NAME}" bash -c \
  "${ROS_ENV} && ${ROS_SETUP} && \
   exec python3 ${CATKIN_WS}/src/target_follower/scripts/navigation_success_udp_bridge.py \
     _in_topic:=${NAV_SUCCESS_TOPIC} \
     _out_host:=${DIALOGUE_TRIGGER_HOST} \
     _out_port:=${DIALOGUE_TRIGGER_UDP_PORT} \
     _enable_waiting_action_retrigger:=${DIALOGUE_RETRIGGER_ENABLE} \
     _resend_interval_s:=${DIALOGUE_RETRIGGER_INTERVAL_S}" &
PID_NAV2UDP=$!

# --- Bridge 2: UDP → trash_action (background docker exec) ---
"${DOCKER_BIN}" exec --user "${DOCKER_USER}" "${DOCKER_NAME}" bash -c \
  "${ROS_ENV} && ${ROS_SETUP} && \
   exec python3 ${CATKIN_WS}/src/target_follower/scripts/udp_trash_action_bridge.py \
     _bind_port:=${DIALOGUE_ACTION_UDP_PORT} \
     _out_topic:=${TRASH_ACTION_TOPIC}" &
PID_UDP2ROS=$!

echo "[dialogue-bridges] started:"
echo "  nav_success -> UDP : ${NAV_SUCCESS_TOPIC} -> ${DIALOGUE_TRIGGER_HOST}:${DIALOGUE_TRIGGER_UDP_PORT}"
echo "  retrigger policy  : enable=${DIALOGUE_RETRIGGER_ENABLE}, interval=${DIALOGUE_RETRIGGER_INTERVAL_S}s"
echo "  UDP -> trash_action: 0.0.0.0:${DIALOGUE_ACTION_UDP_PORT} -> ${TRASH_ACTION_TOPIC}"

wait -n "${PID_NAV2UDP}" "${PID_UDP2ROS}"
