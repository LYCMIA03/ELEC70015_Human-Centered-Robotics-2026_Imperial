#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
JETSON_IP="${JETSON_IP:-192.168.50.1}"
INTERVAL="2"
ONCE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval)
      INTERVAL="$2"
      shift 2
      ;;
    --once)
      ONCE="true"
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/watch_demo_topics.sh [--interval SEC] [--once]

Examples:
  ./scripts/watch_demo_topics.sh
  ./scripts/watch_demo_topics.sh --interval 1
  ./scripts/watch_demo_topics.sh --once
EOF
      exit 0
      ;;
    *)
      echo "[watch] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
  echo "[watch] Docker container '${DOCKER_NAME}' is not running." >&2
  echo "        Start it with: docker start ${DOCKER_NAME}" >&2
  exit 1
fi

DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -lc"
ROS_ENV="source /opt/ros/noetic/setup.bash && source /home/frank/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://${JETSON_IP}:11311 && export ROS_IP=${JETSON_IP}"

topic_snapshot() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && timeout 2 rostopic echo -n 1 ${topic} 2>/dev/null" \
    | sed '/^[[:space:]]*$/d' \
    | head -n 8 \
    | tr '\n' ' ' \
    | sed 's/[[:space:]]\+/ /g; s/[[:space:]]$//' || true
}

topic_rate() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && timeout 4 rostopic hz ${topic} 2>/dev/null | awk '/average rate/{print \$3}' | tail -n1" || true
}

node_exists() {
  local node="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && rosnode list 2>/dev/null | grep -q '^/${node}\$'" && echo "up" || echo "down"
}

print_once() {
  local now scan_hz odom_hz
  now="$(date '+%F %T')"

  scan_hz="$(topic_rate /unitree/scan)"
  odom_hz="$(topic_rate /odom)"
  [[ -z "${scan_hz}" ]] && scan_hz="n/a"
  [[ -z "${odom_hz}" ]] && odom_hz="n/a"

  echo ""
  echo "[$now] demo topic monitor"
  echo "  nodes: move_base=$(node_exists move_base) target_follower=$(node_exists target_follower) udp_target_bridge=$(node_exists udp_target_bridge) point_to_target_pose=$(node_exists point_to_target_pose) nav_success_udp_bridge=$(node_exists navigation_success_udp_bridge) udp_trash_action_bridge=$(node_exists udp_trash_action_bridge)"
  echo "  rate : /unitree/scan=${scan_hz}Hz  /odom=${odom_hz}Hz"
  echo "  /trash_detection/target_point : $(topic_snapshot /trash_detection/target_point)"
  echo "  /target_pose                  : $(topic_snapshot /target_pose)"
  echo "  /target_follower/status       : $(topic_snapshot /target_follower/status)"
  echo "  /target_follower/result       : $(topic_snapshot /target_follower/result)"
  echo "  /trash_action                 : $(topic_snapshot /trash_action)"
  echo "  /cmd_vel                      : $(topic_snapshot /cmd_vel)"
  echo "  /odom                         : $(topic_snapshot /odom)"
}

while true; do
  print_once
  if [[ "${ONCE}" == "true" ]]; then
    exit 0
  fi
  sleep "${INTERVAL}"
done

