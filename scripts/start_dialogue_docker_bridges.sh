#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MASTER_HOST="${MASTER_HOST:-jetson}"
NAV_SUCCESS_TOPIC="${NAV_SUCCESS_TOPIC:-/navigation_success}"
TRASH_ACTION_TOPIC="${TRASH_ACTION_TOPIC:-/trash_action}"
DIALOGUE_TRIGGER_HOST="${DIALOGUE_TRIGGER_HOST:-127.0.0.1}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"

cleanup() {
  for pid in "${PID_NAV2UDP:-}" "${PID_UDP2ROS:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

cd "${REPO_ROOT}"

python3 catkin_ws/src/target_follower/scripts/navigation_success_udp_bridge.py \
  _in_topic:="${NAV_SUCCESS_TOPIC}" \
  _out_host:="${DIALOGUE_TRIGGER_HOST}" \
  _out_port:="${DIALOGUE_TRIGGER_UDP_PORT}" &
PID_NAV2UDP=$!

python3 catkin_ws/src/target_follower/scripts/udp_trash_action_bridge.py \
  _bind_port:="${DIALOGUE_ACTION_UDP_PORT}" \
  _out_topic:="${TRASH_ACTION_TOPIC}" &
PID_UDP2ROS=$!

echo "[dialogue-docker] started:"
echo "  nav_success -> UDP : ${NAV_SUCCESS_TOPIC} -> ${DIALOGUE_TRIGGER_HOST}:${DIALOGUE_TRIGGER_UDP_PORT}"
echo "  UDP -> trash_action: 0.0.0.0:${DIALOGUE_ACTION_UDP_PORT} -> ${TRASH_ACTION_TOPIC}"

wait -n "${PID_NAV2UDP}" "${PID_UDP2ROS}"
