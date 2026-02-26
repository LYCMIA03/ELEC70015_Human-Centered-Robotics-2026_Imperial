#!/usr/bin/env bash
# =============================================================================
# start_demo.sh — 一键启动完整 Target-Following + Dialogue Demo (无需先验地图)
#
# 运行机器：Jetson Orin Nano (host 终端, 非 Docker)
# 启动内容：
#   1. Jetson Docker — roscore (if not running)
#   2. Jetson Docker — target_follow_real.launch (独立模式: LiDAR + move_base + 目标追踪)
#   3. Jetson Host   — Hand-Object 检测 (handobj_detection_rgbd.py)
#   4. Jetson Host   — Dialogue UDP bridges (nav_success → UDP:16041 → dialogue → /trash_action)
#
# 导航方式：纯局部规划 (rolling-window costmap in odom frame)，不依赖全局地图。
#
# 对话流程：
#   REACHED → result=True → UDP:16041 → dialogue_udp_runner → UDP:16032 → /trash_action
#   /trash_action=True  → IDLE (接受, 继续)
#   /trash_action=False → RETREATING (拒绝, 后退 ${RETREAT_DIST}m) → IDLE
#
# 前提（需手动完成）：
#   - Raspberry Pi 已启动: ./scripts/start_base.sh  (on Pi)
#   - Unitree LiDAR 已接 USB
#   - Orbbec 摄像头已接 USB
#   - dialogue/dialogue_udp_runner.py 已安装依赖 (start_dialogue_host.sh)
#
# 用法：
#   ./scripts/start_demo.sh
#   ./scripts/start_demo.sh --standoff 0.8 --retreat 1.5 --target waste
#
# 参数：
#   --standoff M      停在目标前多远 (m), 默认 0.8
#   --retreat M       人类拒绝后后退距离 (m), 默认 1.5
#   --action-timeout S 等待对话结果超时 (s), 默认 45
#   --target TYPE     检测目标类型 holding|person|waste, 默认 holding
#   --dialogue-device N 对话麦克风设备号, 默认 24
#   --only LIST       仅启动模块(逗号分隔): master,nav,yolo,dialogue,dashboard
#   --no-nav          不启动 target_follow_real (仅联调 dialogue 等)
#   --no-yolo         不启动检测（手动测试用，可用 send_target_udp.py 模拟）
#   --no-dialogue     不启动对话桥接（纲连测试）
#   --no-dashboard    不启动 dashboard
#   --dashboard-interval S dashboard 刷新周期(秒), 默认 2
# =============================================================================

set -uo pipefail

# ---------- 颜色 ----------
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'
BLU='\033[0;34m'; CYN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
info()  { echo -e "${BLU}[INFO]${NC}  $*"; }
ok()    { echo -e "${GRN}[OK]${NC}    $*"; }
warn()  { echo -e "${YLW}[WARN]${NC}  $*"; }
die()   { echo -e "${RED}[FATAL]${NC} $*" >&2; exit 1; }
step()  { echo -e "\n${BOLD}${CYN}── $* ${NC}"; }

# ---------- 路径 ----------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${REPO_ROOT}/catkin_ws"
HANDOBJ_DIR="${REPO_ROOT}/handobj_detection"

# ---------- 默认参数 ----------
STANDOFF="0.8"
RETREAT_DIST="1.5"
ACTION_WAIT="45.0"
TARGET_KIND="holding"
DIALOGUE_DEVICE="24"
LAUNCH_YOLO=true
LAUNCH_DIALOGUE=true
LAUNCH_NAV=true
LAUNCH_DASHBOARD=true
DASHBOARD_INTERVAL="2"
AUTO_START_DOCKER=true
ONLY_MODULES=""
NEED_MASTER=true

# ---------- 解析参数 ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --standoff)        STANDOFF="$2";       shift 2 ;;
    --retreat)         RETREAT_DIST="$2";   shift 2 ;;
    --action-timeout)  ACTION_WAIT="$2";    shift 2 ;;
    --target)          TARGET_KIND="$2";    shift 2 ;;
    --dialogue-device) DIALOGUE_DEVICE="$2"; shift 2 ;;
    --only)            ONLY_MODULES="$2";   shift 2 ;;
    --no-nav)          LAUNCH_NAV=false;    shift   ;;
    --no-yolo)         LAUNCH_YOLO=false;   shift   ;;
    --no-dialogue)     LAUNCH_DIALOGUE=false; shift  ;;
    --no-dashboard)    LAUNCH_DASHBOARD=false; shift ;;
    --dashboard-interval) DASHBOARD_INTERVAL="$2"; shift 2 ;;
    --no-auto-start-docker) AUTO_START_DOCKER=false; shift ;;
    -h|--help)
      grep '^#' "$0" | head -40 | sed 's/^# \{0,2\}//'
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

if [[ -n "${ONLY_MODULES}" ]]; then
  LAUNCH_NAV=false
  LAUNCH_YOLO=false
  LAUNCH_DIALOGUE=false
  LAUNCH_DASHBOARD=false
  NEED_MASTER=false
  IFS=',' read -r -a _mods <<< "${ONLY_MODULES}"
  for m in "${_mods[@]}"; do
    case "${m}" in
      master) NEED_MASTER=true ;;
      nav) LAUNCH_NAV=true; NEED_MASTER=true ;;
      yolo) LAUNCH_YOLO=true; NEED_MASTER=true ;;
      dialogue) LAUNCH_DIALOGUE=true; NEED_MASTER=true ;;
      dashboard) LAUNCH_DASHBOARD=true; NEED_MASTER=true ;;
      *) die "Unknown module in --only: ${m}" ;;
    esac
  done
fi

if ${LAUNCH_NAV} || ${LAUNCH_YOLO} || ${LAUNCH_DIALOGUE} || ${LAUNCH_DASHBOARD}; then
  NEED_MASTER=true
fi

# ---------- 加载 deploy.env ----------
if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
else
  die "scripts/deploy.env not found. Copy from deploy.env.example and set IPs."
fi
JETSON_IP="${JETSON_IP:-192.168.50.1}"
RASPI_IP="${RASPI_IP:-192.168.50.2}"
TRASH_UDP_PORT="${TRASH_UDP_PORT:-16031}"
DIALOGUE_TRIGGER_UDP_PORT="${DIALOGUE_TRIGGER_UDP_PORT:-16041}"
DIALOGUE_ACTION_UDP_PORT="${DIALOGUE_ACTION_UDP_PORT:-16032}"
DIALOGUE_DEVICE="${DIALOGUE_DEVICE:-${DEVICE:-24}}"
UDP_PORT_WINDOW="${UDP_PORT_WINDOW:-500}"
DOCKER_NAME="ros_noetic"
ROS_MASTER="http://${JETSON_IP}:11311"
ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -c"

# ---------- UDP 端口工具 ----------
_udp_port_in_use() {
  local port="$1"
  ss -lunH 2>/dev/null | awk '{print $5}' | grep -Eq "(^|:)$port$"
}

_udp_port_reserved() {
  local port="$1"
  local p
  for p in "${USED_UDP_PORTS[@]:-}"; do
    if [[ "${p}" == "${port}" ]]; then
      return 0
    fi
  done
  return 1
}

_next_free_udp_port() {
  local start="$1"
  local p="$start"
  local max=$((start + UDP_PORT_WINDOW))
  while (( p <= max )); do
    if ! _udp_port_in_use "${p}" && ! _udp_port_reserved "${p}"; then
      echo "${p}"
      return 0
    fi
    ((p++))
  done
  return 1
}

_select_udp_port() {
  local label="$1"
  local requested="$2"
  local selected
  selected="$(_next_free_udp_port "${requested}")" || \
    die "No free UDP port found in [${requested}, $((requested + UDP_PORT_WINDOW))] for ${label}"
  if [[ "${selected}" != "${requested}" ]]; then
    warn "${label} UDP port ${requested} is in use; switched to ${selected}"
  fi
  USED_UDP_PORTS+=("${selected}")
  echo "${selected}"
}

# ---------- 动态端口分配 ----------
USED_UDP_PORTS=()
TRASH_UDP_PORT="$(_select_udp_port "trash_detection->ROS" "${TRASH_UDP_PORT}")"
DIALOGUE_TRIGGER_UDP_PORT="$(_select_udp_port "nav_success->dialogue" "${DIALOGUE_TRIGGER_UDP_PORT}")"
DIALOGUE_ACTION_UDP_PORT="$(_select_udp_port "dialogue->trash_action" "${DIALOGUE_ACTION_UDP_PORT}")"

# ---------- Bridge 健康检查 ----------
_ros_node_exists() {
  local node="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null | grep -q \"^/${node}$\"" \
    2>/dev/null
}

_topic_has_publisher() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rostopic info ${topic} 2>/dev/null | awk '/Publishers:/{flag=1;next}/Subscribers:/{flag=0}flag' | grep -q '\\*'" \
    2>/dev/null
}

# ---------- 后台进程 PID ----------
YOLO_PID=""
BRIDGE_PID=""
DIALOGUE_PID=""
DASHBOARD_PID=""

# ---------- 清理函数 ----------
cleanup() {
  echo ""
  step "Shutting down..."

  # 杀掉 host 上的 YOLO
  if [[ -n "${YOLO_PID}" ]] && kill -0 "${YOLO_PID}" 2>/dev/null; then
    info "Stopping YOLO (pid ${YOLO_PID})..."
    kill "${YOLO_PID}" 2>/dev/null || true
  fi

  # 杀掉 host 上的对话 bridge
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    info "Stopping dialogue bridges (pid ${BRIDGE_PID})..."
    kill "${BRIDGE_PID}" 2>/dev/null || true
  fi

  # 杀掉 host 上的 dialogue runner
  if [[ -n "${DIALOGUE_PID}" ]] && kill -0 "${DIALOGUE_PID}" 2>/dev/null; then
    info "Stopping dialogue runner (pid ${DIALOGUE_PID})..."
    kill "${DIALOGUE_PID}" 2>/dev/null || true
  fi

  if [[ -n "${DASHBOARD_PID}" ]] && kill -0 "${DASHBOARD_PID}" 2>/dev/null; then
    info "Stopping dashboard (pid ${DASHBOARD_PID})..."
    kill "${DASHBOARD_PID}" 2>/dev/null || true
  fi

  # 停止 Docker 内所有 target follow 相关节点 (包括 move_base)
  if ${LAUNCH_NAV}; then
    info "Stopping target following + navigation nodes..."
    ${DOCKER_EXEC} "pkill -f 'roslaunch.*target_follow' 2>/dev/null; sleep 1" \
      2>/dev/null || true
  fi

  ok "Demo stopped. Pi base driver still running — stop it manually on the Pi."
  exit 0
}
trap cleanup INT TERM

# =============================================================================
echo ""
echo -e "${BOLD}${CYN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYN}║  P3-AT Target Following Demo — Local Planning    ║${NC}"
echo -e "${BOLD}${CYN}╚══════════════════════════════════════════════════╝${NC}"
echo -e "  Mode:       standalone (no global map)"
echo -e "  Standoff:   ${STANDOFF} m"
echo -e "  Retreat:    ${RETREAT_DIST} m (on refusal)"
echo -e "  Act.timeout:${ACTION_WAIT} s"
echo -e "  Target:     ${TARGET_KIND}"
echo -e "  UDP detect: ${TRASH_UDP_PORT}  (trash_detection -> ROS)"
echo -e "  UDP trig:   ${DIALOGUE_TRIGGER_UDP_PORT}  (nav_success -> dialogue)"
echo -e "  UDP action: ${DIALOGUE_ACTION_UDP_PORT}  (dialogue -> /trash_action)"
echo -e "  YOLO:       $(${LAUNCH_YOLO} && echo enabled || echo DISABLED)"
echo -e "  Dialogue:   $(${LAUNCH_DIALOGUE} && echo enabled || echo DISABLED)"
echo -e "  Navigation: $(${LAUNCH_NAV} && echo enabled || echo DISABLED)"
echo -e "  Dashboard:  $(${LAUNCH_DASHBOARD} && echo enabled || echo DISABLED)"
echo ""

# =============================================================================
step "STEP 0 — Preflight checks"

# Docker running?
if ${NEED_MASTER} && ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${DOCKER_NAME}$"; then
  if ${AUTO_START_DOCKER}; then
    info "Docker container '${DOCKER_NAME}' not running, starting it..."
    docker start "${DOCKER_NAME}" >/dev/null || die "Failed to start docker container ${DOCKER_NAME}"
    sleep 2
  else
    die "Docker container '${DOCKER_NAME}' is not running. Start it first:\n  docker start ${DOCKER_NAME}"
  fi
fi
if ${NEED_MASTER}; then
  ok "Docker container '${DOCKER_NAME}' is running"
fi

# Workspace built?
if [[ ! -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  die "catkin_ws not built. Run: cd catkin_ws && catkin_make"
fi
ok "catkin_ws/devel/setup.bash exists"

# Pi reachable?
if ping -c1 -W2 "${RASPI_IP}" &>/dev/null; then
  ok "Raspberry Pi (${RASPI_IP}) is reachable"
else
  warn "Cannot reach Raspberry Pi (${RASPI_IP}). Make sure start_base.sh is running on Pi!"
  echo -e "    ${YLW}SSH into Pi and run:${NC}  ./scripts/start_base.sh"
  echo "    Continuing anyway — /odom will be missing until Pi is up."
fi

# Detection script available?
if ${LAUNCH_YOLO}; then
  if ! command -v python3 &>/dev/null; then
    warn "python3 not found on host — detection will be skipped"
    LAUNCH_YOLO=false
  elif [[ ! -f "${HANDOBJ_DIR}/handobj_detection_rgbd.py" ]]; then
    warn "handobj_detection_rgbd.py not found — detection will be skipped"
    LAUNCH_YOLO=false
  else
    ok "Hand-Object detection script found"
  fi
fi

# =============================================================================
step "STEP 1 — Ensure roscore is running"

if ${NEED_MASTER}; then
  if ${DOCKER_EXEC} "pgrep -x rosmaster" &>/dev/null; then
    ok "roscore already running"
  else
    info "Starting roscore..."
    ${DOCKER_EXEC} "( ${ROS_ENV} && source /opt/ros/noetic/setup.bash && exec roscore ) \
      > /tmp/roscore.log 2>&1 &" 2>/dev/null || true
    sleep 4
    if ${DOCKER_EXEC} "pgrep -x rosmaster" &>/dev/null; then
      ok "roscore started"
    else
      die "roscore failed to start. Check: docker exec ${DOCKER_NAME} tail /tmp/roscore.log"
    fi
  fi
else
  warn "Master disabled by module selection; skip roscore startup."
fi

# =============================================================================
step "STEP 2 — Start target following (standalone: LiDAR + move_base + follower)"

if ! ${LAUNCH_NAV}; then
  warn "Navigation disabled (--no-nav or --only without nav); skipping STEP 2"
else

# Kill any existing mapping/nav/target-follow launch
${DOCKER_EXEC} "pkill -f 'roslaunch.*real_robot' 2>/dev/null; \
                 pkill -f 'roslaunch.*target_follow' 2>/dev/null; sleep 1" 2>/dev/null || true
sleep 1

info "Launching target_follow_real.launch (standalone mode — no global map)..."
${DOCKER_EXEC} "( ${ROS_ENV} && ${ROS_SETUP} && \
  exec roslaunch target_follower target_follow_real.launch \
    launch_move_base:=true \
    standoff_distance:=${STANDOFF} \
    face_target:=true \
    target_timeout:=5.0 \
    udp_port:=${TRASH_UDP_PORT} \
    retreat_distance:=${RETREAT_DIST} \
    action_wait_timeout:=${ACTION_WAIT} \
  > /tmp/target_follow.log 2>&1 ) &" 2>/dev/null

info "Waiting for move_base + target_follower to come up (up to 30 s)..."
for i in $(seq 1 30); do
  sleep 1
  NODES=$(${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null" 2>/dev/null || true)
  HAS_MB=$(echo "${NODES}" | grep -c move_base || true)
  HAS_TF=$(echo "${NODES}" | grep -c target_follower || true)
  if [[ "${HAS_MB}" -ge 1 && "${HAS_TF}" -ge 1 ]]; then
    ok "move_base + target_follower are up (${i}s)"
    break
  fi
  if [[ $i -eq 30 ]]; then
    die "Nodes not ready after 30 s.\n  Check: docker exec ${DOCKER_NAME} tail -30 /tmp/target_follow.log"
  fi
done

# Check LiDAR data
info "Checking LiDAR data..."
SCAN_RATE=$(${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
  timeout 6 rostopic hz /unitree/scan 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}'" 2>/dev/null || true)
if [[ -n "${SCAN_RATE}" ]]; then
  ok "/unitree/scan: ${SCAN_RATE} Hz"
else
  warn "/unitree/scan: no data — LiDAR may not be fully up yet (OK to proceed)"
fi

# Check core in-Docker bridge chain
info "Validating in-Docker bridge nodes (/udp_target_bridge, /point_to_target_pose)..."
CORE_BRIDGE_OK=0
for i in $(seq 1 20); do
  if _ros_node_exists "udp_target_bridge" && _ros_node_exists "point_to_target_pose"; then
    CORE_BRIDGE_OK=1
    break
  fi
  sleep 1
done
if [[ "${CORE_BRIDGE_OK}" -ne 1 ]]; then
  die "Core bridge nodes not ready. Check: docker exec ${DOCKER_NAME} tail -60 /tmp/target_follow.log"
fi
ok "Core bridge chain is up (udp_target_bridge + point_to_target_pose)"
fi

# =============================================================================
step "STEP 3 — Start Hand-Object detection (host)"

if ${LAUNCH_YOLO}; then
  pkill -f "handobj_detection_rgbd.py" 2>/dev/null || true
  info "Starting handobj_detection_rgbd.py (UDP → 127.0.0.1:${TRASH_UDP_PORT})..."
  cd "${HANDOBJ_DIR}"
  python3 handobj_detection_rgbd.py \
    --udp-enable \
    --udp-host "127.0.0.1" \
    --udp-port "${TRASH_UDP_PORT}" \
    --udp-frame-id "camera_link" \
    --headless \
    --print-xyz \
    > /tmp/handobj.log 2>&1 &
  YOLO_PID=$!
  sleep 4
  if kill -0 "${YOLO_PID}" 2>/dev/null; then
    ok "Hand-Object detection started (pid ${YOLO_PID}), log: /tmp/handobj.log"
  else
    warn "Detection exited early. Check: tail -30 /tmp/handobj.log"
    YOLO_PID=""
  fi
else
  warn "Detection disabled. To simulate a target manually:"
  echo -e "    ${CYN}python3 ${REPO_ROOT}/trash_detection/examples/send_target_udp.py --z 2.0 --port ${TRASH_UDP_PORT}${NC}"
fi

# =============================================================================
step "STEP 4 — Start Dialogue (runner + Docker bridges)"

if ${LAUNCH_DIALOGUE}; then
  if [[ ! -f "${REPO_ROOT}/dialogue/dialogue_udp_runner.py" ]]; then
    warn "dialogue/dialogue_udp_runner.py not found — dialogue disabled"
    LAUNCH_DIALOGUE=false
  else
    info "Starting dialogue runner (device=${DIALOGUE_DEVICE})..."
    pkill -f "python3 .*dialogue/dialogue_udp_runner.py" 2>/dev/null || true
    python3 "${REPO_ROOT}/dialogue/dialogue_udp_runner.py" \
      --listen-host "0.0.0.0" \
      --listen-port "${DIALOGUE_TRIGGER_UDP_PORT}" \
      --send-host "127.0.0.1" \
      --send-port "${DIALOGUE_ACTION_UDP_PORT}" \
      --device "${DIALOGUE_DEVICE}" \
      > /tmp/dialogue_runner.log 2>&1 &
    DIALOGUE_PID=$!

    info "Waiting for dialogue runner to listen on UDP:${DIALOGUE_TRIGGER_UDP_PORT}..."
    RUNNER_READY=0
    for i in $(seq 1 20); do
      sleep 1
      if ! kill -0 "${DIALOGUE_PID}" 2>/dev/null; then
        break
      fi
      if ss -lunp 2>/dev/null | grep -q "0.0.0.0:${DIALOGUE_TRIGGER_UDP_PORT}"; then
        RUNNER_READY=1
        break
      fi
    done
    if [[ "${RUNNER_READY}" -ne 1 ]]; then
      die "Dialogue runner failed to become ready. Check: tail -60 /tmp/dialogue_runner.log"
    fi
    ok "Dialogue runner started (pid ${DIALOGUE_PID}), log: /tmp/dialogue_runner.log"

    info "Starting dialogue docker bridges..."
    MASTER_HOST=jetson \
    DIALOGUE_TRIGGER_UDP_PORT=${DIALOGUE_TRIGGER_UDP_PORT} \
    DIALOGUE_ACTION_UDP_PORT=${DIALOGUE_ACTION_UDP_PORT} \
    "${SCRIPT_DIR}/start_dialogue_docker_bridges.sh" \
      > /tmp/dialogue_bridge.log 2>&1 &
    BRIDGE_PID=$!
    sleep 2
    if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      ok "Dialogue bridges started (pid ${BRIDGE_PID}), log: /tmp/dialogue_bridge.log"
      ok "  nav_success → UDP:${DIALOGUE_TRIGGER_UDP_PORT} → dialogue"
      ok "  dialogue   → UDP:${DIALOGUE_ACTION_UDP_PORT}  → /trash_action"
    else
      die "Dialogue bridge exited early. Check: tail -40 /tmp/dialogue_bridge.log"
    fi

    info "Validating dialogue bridge mesh (runner + ROS bridge nodes + /trash_action publisher)..."
    DIALOGUE_OK=0
    for i in $(seq 1 15); do
      if kill -0 "${DIALOGUE_PID}" 2>/dev/null \
         && _ros_node_exists "navigation_success_udp_bridge" \
         && _ros_node_exists "udp_trash_action_bridge" \
         && _topic_has_publisher "/trash_action"; then
        DIALOGUE_OK=1
        break
      fi
      sleep 1
    done
    if [[ "${DIALOGUE_OK}" -ne 1 ]]; then
      warn "Dialogue bridge mesh check failed; retrying bridge startup once..."
      if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
        sleep 1
      fi
      MASTER_HOST=jetson \
      DIALOGUE_TRIGGER_UDP_PORT=${DIALOGUE_TRIGGER_UDP_PORT} \
      DIALOGUE_ACTION_UDP_PORT=${DIALOGUE_ACTION_UDP_PORT} \
      "${SCRIPT_DIR}/start_dialogue_docker_bridges.sh" \
        > /tmp/dialogue_bridge.log 2>&1 &
      BRIDGE_PID=$!
      sleep 2

      DIALOGUE_OK=0
      for i in $(seq 1 10); do
        if kill -0 "${DIALOGUE_PID}" 2>/dev/null \
           && _ros_node_exists "navigation_success_udp_bridge" \
           && _ros_node_exists "udp_trash_action_bridge" \
           && _topic_has_publisher "/trash_action"; then
          DIALOGUE_OK=1
          break
        fi
        sleep 1
      done
      if [[ "${DIALOGUE_OK}" -ne 1 ]]; then
        die "Dialogue bridge mesh still unhealthy after retry. Check: tail -80 /tmp/dialogue_bridge.log and /tmp/dialogue_runner.log"
      fi
    fi
    ok "Dialogue bridge mesh is healthy"
  fi
else
  warn "Dialogue bridge disabled (--no-dialogue). /trash_action will not be published."
  warn "  Simulate with: rostopic pub /trash_action std_msgs/Bool 'data: false'"
fi

# =============================================================================
step "STEP 5 — System ready! Live status monitor"

if ${LAUNCH_DASHBOARD} && [[ -x "${SCRIPT_DIR}/demo_dashboard.sh" ]]; then
  pkill -f "demo_dashboard.sh" 2>/dev/null || true
  MODULES="master"
  ${LAUNCH_DIALOGUE} && MODULES="${MODULES},dialogue"
  ${LAUNCH_YOLO} && MODULES="${MODULES},yolo"
  MODULES="${MODULES},base"
  nohup "${SCRIPT_DIR}/demo_dashboard.sh" --interval "${DASHBOARD_INTERVAL}" --modules "${MODULES}" \
    > /tmp/demo_dashboard.log 2>&1 &
  DASHBOARD_PID=$!
  ok "Dashboard started (pid ${DASHBOARD_PID}), log: /tmp/demo_dashboard.log"
fi

echo ""
echo -e "  ${GRN}All components launched.${NC}  Press ${BOLD}Ctrl+C${NC} to shut everything down."
echo ""
echo -e "  Useful topics:"
echo -e "    ${CYN}rostopic echo /target_follower/status${NC}   — IDLE|TRACKING|REACHED|WAITING_ACTION|RETREATING|LOST|FAILED"
echo -e "    ${CYN}rostopic echo /target_follower/result${NC}   — True/False (dialogue trigger)"
echo -e "    ${CYN}rostopic echo /trash_action${NC}             — True=接受 / False=拒绝 (对话结果)"
echo -e "    ${CYN}rostopic echo /target_pose${NC}              — current target pose"
echo -e "    ${CYN}tail -f /tmp/handobj.log${NC}                  — detection log"
echo -e "    ${CYN}tail -f /tmp/dialogue_bridge.log${NC}           — dialogue bridge log"
echo -e "    ${CYN}tail -f /tmp/dialogue_runner.log${NC}           — dialogue runner log"
echo -e "    ${CYN}tail -f /tmp/demo_dashboard.log${NC}            — dashboard log"
echo ""

_ros_topic_val() {
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
    timeout 3 rostopic echo $1 -n 1 2>/dev/null | grep 'data:' | awk '{print \$2}'" 2>/dev/null || echo "?"
}

_ros_node_check() {
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
    rosnode list 2>/dev/null | grep -c $1" 2>/dev/null || echo "0"
}

# Monitor loop
while true; do
  STATUS=$(_ros_topic_val /target_follower/status)
  RESULT=$(_ros_topic_val /target_follower/result)

  # Color status
  case "${STATUS}" in
    TRACKING)         S_COLOR="${YLW}${STATUS}${NC}" ;;
    REACHED)          S_COLOR="${GRN}${STATUS}${NC}" ;;
    WAITING_ACTION)   S_COLOR="${CYN}${STATUS}${NC}" ;;
    RETREATING)       S_COLOR="${BLU}${STATUS}${NC}" ;;
    LOST|FAILED)      S_COLOR="${RED}${STATUS}${NC}" ;;
    *)                S_COLOR="${STATUS}" ;;
  esac

  case "${RESULT}" in
    True)  R_COLOR="${GRN}True (REACHED)${NC}" ;;
    False) R_COLOR="${RED}False (FAILED)${NC}" ;;
    *)     R_COLOR="${RESULT}" ;;
  esac

  YOLO_ALIVE="stopped"
  if [[ -n "${YOLO_PID}" ]] && kill -0 "${YOLO_PID}" 2>/dev/null; then
    YOLO_ALIVE="${GRN}running (${YOLO_PID})${NC}"
  else
    YOLO_ALIVE="${RED}stopped${NC}"
  fi

  BRIDGE_ALIVE="disabled"
  if ${LAUNCH_DIALOGUE}; then
    if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
      BRIDGE_ALIVE="${GRN}running${NC}"
    else
      BRIDGE_ALIVE="${RED}stopped${NC}"
    fi
  fi

  printf "\r  [%s]  status=%-16b  result=%-20b  YOLO=%b  dialogue=%b     " \
    "$(date +%H:%M:%S)" "${S_COLOR}" "${R_COLOR}" "${YOLO_ALIVE}" "${BRIDGE_ALIVE}"

  sleep 2
done
