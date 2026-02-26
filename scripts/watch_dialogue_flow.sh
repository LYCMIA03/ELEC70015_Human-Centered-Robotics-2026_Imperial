#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
JETSON_IP="${JETSON_IP:-192.168.50.1}"
INTERVAL="1.0"
ONCE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval) INTERVAL="$2"; shift 2 ;;
    --once) ONCE="true"; shift ;;
    -h|--help)
      cat <<'HLP'
Usage:
  ./scripts/watch_dialogue_flow.sh [--interval SEC] [--once]

Monitors:
  1) Dialogue trigger (/target_follower/result + WAITING_ACTION)
  2) Dialogue return True/False (/trash_action)
  3) Node health (target_follower + dialogue bridges + runner)
HLP
      exit 0
      ;;
    *) echo "[watch-dialogue] Unknown argument: $1" >&2; exit 1 ;;
  esac
done

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
  echo "[watch-dialogue] Docker container '${DOCKER_NAME}' is not running." >&2
  echo "Start it with: docker start ${DOCKER_NAME}" >&2
  exit 1
fi

RUNNER_UP="down"
if pgrep -af "dialogue/dialogue_udp_runner.py" >/dev/null 2>&1; then
  RUNNER_UP="up"
fi

ONCE_FLAG=""
if [[ "${ONCE}" == "true" ]]; then
  ONCE_FLAG="--once"
fi

docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP} && python3 -u ${REPO_ROOT}/scripts/watch_dialogue_flow_ros.py --interval ${INTERVAL} ${ONCE_FLAG} --runner ${RUNNER_UP}"
