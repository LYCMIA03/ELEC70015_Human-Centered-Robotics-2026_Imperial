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
NAV_READINESS_MODE="${NAV_READINESS_MODE:-relaxed}"

ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -c"

_docker_exec_detached() {
  local cmd="$1"
  docker exec --user "$(id -u):$(id -g)" -d "${DOCKER_NAME}" bash -lc "${cmd}" >/dev/null
}

_ros_node_exists() {
  local node="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rosnode list 2>/dev/null | grep -q \"^/${node}$\"" \
    >/dev/null 2>&1
}

_topic_has_publisher() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info ${topic} 2>/dev/null | awk '/Publishers:/{flag=1;next}/Subscribers:/{flag=0}flag' | grep -q '\\*'" \
    >/dev/null 2>&1
}

_topic_has_message() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic echo -n 1 ${topic} 2>/dev/null >/dev/null" \
    >/dev/null 2>&1
}

_set_target_follower_auto_explore() {
  local enabled="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosparam set /target_follower/enable_auto_explore ${enabled}" \
    >/dev/null 2>&1
}

_target_follower_ready() {
  _ros_node_exists "target_follower" \
    && _topic_has_publisher "/target_follower/status" \
    && _topic_has_message "/target_follower/status"
}

_nav_restart_ready() {
  case "${NAV_READINESS_MODE}" in
    strict)
      _ros_node_exists "move_base" \
        && _target_follower_ready
      ;;
    relaxed)
      _ros_node_exists "move_base" \
        && _ros_node_exists "target_follower"
      ;;
  esac
}

_nav_restart_blockers() {
  local blockers=()

  if ! _ros_node_exists "move_base"; then
    blockers+=("/move_base node missing")
  fi

  if ! _ros_node_exists "target_follower"; then
    blockers+=("/target_follower node missing")
  elif [[ "${NAV_READINESS_MODE}" == "strict" ]]; then
    if ! _topic_has_publisher "/target_follower/status"; then
      blockers+=("/target_follower/status has no publisher")
    elif ! _topic_has_message "/target_follower/status"; then
      blockers+=("/target_follower/status has not published a message yet")
    fi
  fi

  if [[ ${#blockers[@]} -eq 0 ]]; then
    return 0
  fi

  printf '%s\n' "${blockers[@]}"
  return 1
}

_ros_node_pid() {
  local node="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rosnode info /${node} 2>/dev/null | awk '/Pid:/{print \$2; exit}'" \
    2>/dev/null || true
}

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
    _docker_exec_detached "${ROS_ENV} && source /opt/ros/noetic/setup.bash && exec roscore > /tmp/roscore.log 2>&1"
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
      --rotate-180 \
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
  local old_move_base_pid old_target_follower_pid new_move_base_pid new_target_follower_pid
  local blockers
  old_move_base_pid="$(_ros_node_pid "move_base")"
  old_target_follower_pid="$(_ros_node_pid "target_follower")"

  info "Stopping existing navigation nodes..."
  ${DOCKER_EXEC} "pkill -f 'roslaunch.*real_robot' 2>/dev/null || true; \
                   pkill -f 'roslaunch.*target_follow' 2>/dev/null || true; \
                   pkill -f '[m]ove_base' 2>/dev/null || true; \
                   pkill -f '[t]arget_follower.py' 2>/dev/null || true; \
                   pkill -f '[u]dp_target_bridge.py' 2>/dev/null || true; \
                   pkill -f '[p]oint_to_target_pose.py' 2>/dev/null || true; \
                   sleep 1"

  for _ in $(seq 1 15); do
    if ! _ros_node_exists "move_base" && ! _ros_node_exists "target_follower"; then
      break
    fi
    sleep 1
  done

  info "Restarting navigation..."
  _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
    exec roslaunch target_follower target_follow_real.launch \
      launch_move_base:=true \
      lidar_mode:=${LIDAR_MODE:-dual} \
      unitree_port:=${UNITREE_PORT:-/dev/unitree_lidar} \
      rplidar_port:=${RPLIDAR_PORT:-/dev/rplidar_lidar} \
      rplidar_baud:=${RPLIDAR_BAUD:-256000} \
      rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR:-true} \
      rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM:-600} \
      rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S:-2.0} \
      standoff_distance:=${STANDOFF} \
      face_target:=true \
      target_timeout:=5.0 \
      udp_port:=${TRASH_UDP_PORT} \
      retreat_distance:=${RETREAT_DIST} \
      retreat_turn_angle_deg:=${RETREAT_TURN_DEG} \
      action_wait_timeout:=${ACTION_WAIT} \
      enable_auto_explore:=false \
      explore_goal_distance:=${EXPLORE_STEP} \
      explore_revisit_window:=${EXPLORE_NO_REPEAT_SEC} \
      target_reacquire_block_s:=${EXPLORE_NO_REPEAT_SEC} \
      post_accept_cooldown:=${POST_ACCEPT_COOLDOWN} \
    > /tmp/target_follow.log 2>&1"

  for _ in $(seq 1 45); do
    sleep 1
    new_move_base_pid="$(_ros_node_pid "move_base")"
    new_target_follower_pid="$(_ros_node_pid "target_follower")"

    if _nav_restart_ready \
      && [[ -n "${new_move_base_pid}" ]] \
      && [[ -n "${new_target_follower_pid}" ]] \
      && [[ "${new_move_base_pid}" != "${old_move_base_pid}" || -z "${old_move_base_pid}" ]] \
      && [[ "${new_target_follower_pid}" != "${old_target_follower_pid}" || -z "${old_target_follower_pid}" ]]; then
      if ${ENABLE_AUTO_EXPLORE}; then
        if _set_target_follower_auto_explore true; then
          ok "Auto-explore armed after navigation restart readiness"
        else
          warn "Navigation restarted, but failed to re-enable auto-explore on /target_follower"
        fi
      fi
      if ! _target_follower_ready; then
        warn "Navigation restart proceeding before /target_follower/status became live; target_follower is still finishing startup"
      fi
      ok "Navigation restarted"
      return 0
    fi
  done

  warn "Navigation restart verification failed."
  blockers="$(_nav_restart_blockers 2>/dev/null || true)"
  if [[ -n "${blockers}" ]]; then
    while IFS= read -r blocker; do
      [[ -n "${blocker}" ]] && warn "  blocker: ${blocker}"
    done <<< "${blockers}"
  fi
  warn "  old move_base pid: ${old_move_base_pid:-none}"
  warn "  old target_follower pid: ${old_target_follower_pid:-none}"
  warn "  new move_base pid: ${new_move_base_pid:-none}"
  warn "  new target_follower pid: ${new_target_follower_pid:-none}"
  if ${ENABLE_AUTO_EXPLORE}; then
    warn "Auto-explore remains disabled because navigation restart never became ready."
  fi
  die "Navigation failed to restart cleanly"
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
