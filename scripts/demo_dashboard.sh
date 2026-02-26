#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
JETSON_IP="${JETSON_IP:-192.168.50.1}"
INTERVAL="${INTERVAL:-2}"
MODULES="${MODULES:-master,dialogue,yolo,base}"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
  JETSON_IP="${JETSON_IP:-192.168.50.1}"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval) INTERVAL="$2"; shift 2 ;;
    --modules) MODULES="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/demo_dashboard.sh [--interval 2] [--modules master,dialogue,yolo,base]
EOF
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

has_mod() {
  local m="$1"
  [[ ",${MODULES}," == *",${m},"* ]]
}

DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -lc"
ROS_ENV="source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP}"

topic_val() {
  local t="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && timeout 2 rostopic echo -n 1 ${t} 2>/dev/null | awk '/data:/{print \$2}'" 2>/dev/null || true
}

topic_has_pub() {
  local t="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && rostopic info ${t} 2>/dev/null | awk '/Publishers:/{f=1;next}/Subscribers:/{f=0}f' | grep -q '\\*'" 2>/dev/null && echo yes || echo no
}

node_up() {
  local n="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && rosnode list 2>/dev/null | grep -q '^/${n}$'" 2>/dev/null && echo up || echo down
}

while true; do
  now="$(date '+%F %T')"
  line="[$now]"

  if has_mod master; then
    if ${DOCKER_EXEC} "pgrep -x rosmaster >/dev/null" 2>/dev/null; then
      line="${line} master=up"
    else
      line="${line} master=down"
    fi
  fi

  if has_mod dialogue; then
    runner="down"
    pgrep -af "dialogue/dialogue_udp_runner.py" >/dev/null 2>&1 && runner="up"
    nav2udp="$(node_up navigation_success_udp_bridge)"
    udp2ros="$(node_up udp_trash_action_bridge)"
    result="$(topic_val /target_follower/result)"; [[ -z "${result}" ]] && result="?"
    action="$(topic_val /trash_action)"; [[ -z "${action}" ]] && action="?"
    line="${line} dialogue(runner=${runner},nav2udp=${nav2udp},udp2ros=${udp2ros},result=${result},action=${action})"
  fi

  if has_mod yolo; then
    yolo_proc="down"
    pgrep -af "handobj_detection_rgbd.py" >/dev/null 2>&1 && yolo_proc="up"
    tgt_pub="$(topic_has_pub /trash_detection/target_point)"
    line="${line} yolo(proc=${yolo_proc},target_point_pub=${tgt_pub})"
  fi

  if has_mod base; then
    cmd_pub="$(topic_has_pub /cmd_vel)"
    line="${line} base(cmd_vel_pub=${cmd_pub})"
  fi

  echo "${line}"
  sleep "${INTERVAL}"
done
