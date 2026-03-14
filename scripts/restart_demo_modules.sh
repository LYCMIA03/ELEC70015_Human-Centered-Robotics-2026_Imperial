#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${SCRIPT_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"

set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; BLU='\033[0;34m'; YLW='\033[0;33m'; NC='\033[0m'
info() { echo -e "${BLU}[INFO]${NC}  $*"; }
ok()   { echo -e "${GRN}[OK]${NC}    $*"; }
warn() { echo -e "${YLW}[WARN]${NC}  $*"; }
die()  { echo -e "${RED}[FATAL]${NC} $*" >&2; exit 1; }

REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_STATE_FILE="${XDG_RUNTIME_DIR:-/tmp}/hcr_demo_runtime.env"
ONLY_MODULES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --only)
      ONLY_MODULES="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/restart_demo_modules.sh --only master
  ./scripts/restart_demo_modules.sh --only yolo
  ./scripts/restart_demo_modules.sh --only dialogue
  ./scripts/restart_demo_modules.sh --only nav
  ./scripts/restart_demo_modules.sh --only yolo,dialogue
EOF
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ -n "${ONLY_MODULES}" ]] || die "--only is required"
[[ -f "${RUNTIME_STATE_FILE}" ]] || die "Runtime state file not found: ${RUNTIME_STATE_FILE}. Start demo once first."

# shellcheck disable=SC1090
source "${RUNTIME_STATE_FILE}"

ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -c"

ensure_master() {
  if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
    info "Starting docker container '${DOCKER_NAME}'..."
    docker start "${DOCKER_NAME}" >/dev/null
    sleep 2
  fi

  if ${DOCKER_EXEC} "pgrep -x rosmaster" >/dev/null 2>&1; then
    ok "roscore already running"
  else
    info "Starting roscore..."
    ${DOCKER_EXEC} "( ${ROS_ENV} && source /opt/ros/noetic/setup.bash && exec roscore ) > /tmp/roscore.log 2>&1 &"
    sleep 4
    ${DOCKER_EXEC} "pgrep -x rosmaster" >/dev/null 2>&1 || die "Failed to start roscore"
    ok "roscore started"
  fi
}

restart_yolo() {
  pkill -f "[h]andobj_detection_rgbd.py" 2>/dev/null || true
  info "Restarting yolo on UDP ${TRASH_UDP_PORT}..."
  (
    cd "${HANDOBJ_DIR}"
    exec python3 handobj_detection_rgbd.py \
      --udp-enable \
      --udp-host "127.0.0.1" \
      --udp-port "${TRASH_UDP_PORT}" \
      --udp-frame-id "camera_link" \
      --udp-kind "${TARGET_KIND}" \
      --headless \
      --print-xyz
  ) > /tmp/handobj.log 2>&1 &
  sleep 4
  pgrep -af "handobj_detection_rgbd.py" >/dev/null 2>&1 || die "YOLO failed to start"
  ok "YOLO restarted"
}

restart_dialogue() {
  pkill -f "[d]ialogue/dialogue_udp_runner.py" 2>/dev/null || true
  pkill -f "[s]tart_dialogue_docker_bridges.sh" 2>/dev/null || true
  ${DOCKER_EXEC} "pkill -f '[n]avigation_success_udp_bridge.py' 2>/dev/null || true; pkill -f '[u]dp_trash_action_bridge.py' 2>/dev/null || true" 2>/dev/null || true

  info "Restarting dialogue runner..."
  python3 "${REPO_ROOT}/dialogue/dialogue_udp_runner.py" \
    --listen-host "0.0.0.0" \
    --listen-port "${DIALOGUE_TRIGGER_UDP_PORT}" \
    --send-host "127.0.0.1" \
    --send-port "${DIALOGUE_ACTION_UDP_PORT}" \
    --device "${DIALOGUE_DEVICE}" \
    > /tmp/dialogue_runner.log 2>&1 &

  local runner_ready=0
  local wait_start now
  wait_start="$(date +%s)"
  while true; do
    if ss -lunp 2>/dev/null | grep -q "0.0.0.0:${DIALOGUE_TRIGGER_UDP_PORT}"; then
      runner_ready=1
      break
    fi
    now="$(date +%s)"
    if (( now - wait_start >= RUNNER_READY_TIMEOUT )); then
      break
    fi
    sleep 0.5
  done
  [[ "${runner_ready}" -eq 1 ]] || die "Dialogue runner failed to become ready"

  info "Restarting dialogue docker bridges..."
  MASTER_HOST=jetson \
  DIALOGUE_TRIGGER_UDP_PORT="${DIALOGUE_TRIGGER_UDP_PORT}" \
  DIALOGUE_ACTION_UDP_PORT="${DIALOGUE_ACTION_UDP_PORT}" \
  "${SCRIPT_DIR}/start_dialogue_docker_bridges.sh" \
    > /tmp/dialogue_bridge.log 2>&1 &
  sleep 3

  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null | grep -Eq '^/(navigation_success_udp_bridge|udp_trash_action_bridge)$'" \
    >/dev/null 2>&1 || die "Dialogue bridges failed to restart"
  ok "Dialogue restarted"
}

restart_nav() {
  ${DOCKER_EXEC} "pkill -f 'roslaunch.*real_robot' 2>/dev/null || true; pkill -f 'roslaunch.*target_follow' 2>/dev/null || true; pkill -f 'move_base' 2>/dev/null || true; sleep 1"
  info "Restarting navigation..."
  ${DOCKER_EXEC} "( ${ROS_ENV} && ${ROS_SETUP} && \
    exec roslaunch target_follower target_follow_real.launch \
      launch_move_base:=true \
      standoff_distance:=${STANDOFF} \
      face_target:=true \
      target_timeout:=5.0 \
      udp_port:=${TRASH_UDP_PORT} \
      retreat_distance:=${RETREAT_DIST} \
      retreat_turn_angle_deg:=${RETREAT_TURN_DEG} \
      action_wait_timeout:=${ACTION_WAIT} \
      enable_auto_explore:=${ENABLE_AUTO_EXPLORE} \
      explore_goal_distance:=${EXPLORE_STEP} \
      explore_revisit_window:=${EXPLORE_NO_REPEAT_SEC} \
      target_reacquire_block_s:=${EXPLORE_NO_REPEAT_SEC} \
      post_accept_cooldown:=${POST_ACCEPT_COOLDOWN} \
    > /tmp/target_follow.log 2>&1 ) &"

  for _ in $(seq 1 30); do
    sleep 1
    if ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null | grep -q '/target_follower'" >/dev/null 2>&1; then
      ok "Navigation restarted"
      return 0
    fi
  done
  die "Navigation failed to restart"
}

IFS=',' read -r -a MODULES <<< "${ONLY_MODULES}"

for module in "${MODULES[@]}"; do
  case "${module}" in
    master)
      info "Ensuring master is up..."
      ensure_master
      ;;
    yolo)
      ensure_master
      restart_yolo
      ;;
    dialogue)
      ensure_master
      restart_dialogue
      ;;
    nav)
      ensure_master
      restart_nav
      ;;
    *)
      die "Unknown module in --only: ${module}"
      ;;
  esac
done

ok "Requested module restart flow completed."
