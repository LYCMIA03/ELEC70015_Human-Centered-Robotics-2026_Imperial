#!/usr/bin/env bash
# =============================================================================
# start_demo.sh — 一键启动完整 Target-Following Demo (无需先验地图)
#
# 运行机器：Jetson Orin Nano (host 终端, 非 Docker)
# 启动内容：
#   1. Jetson Docker — roscore (if not running)
#   2. Jetson Docker — target_follow_real.launch (独立模式: LiDAR + move_base + 目标追踪)
#   3. Jetson Host   — Hand-Object 检测 (handobj_detection_rgbd.py)
#
# 导航方式：纯局部规划 (rolling-window costmap in odom frame)，不依赖全局地图。
#
# 前提（需手动完成）：
#   - Raspberry Pi 已启动: ./scripts/start_base.sh  (on Pi)
#   - Unitree LiDAR 已接 USB
#   - Orbbec 摄像头已接 USB
#
# 用法：
#   ./scripts/start_demo.sh
#   ./scripts/start_demo.sh --standoff 0.8 --target waste
#
# 参数：
#   --standoff M      停在目标前多远 (m), 默认 0.8
#   --target TYPE     检测目标类型 holding|person|waste, 默认 holding
#   --no-yolo         不启动检测（手动测试用，可用 send_target_udp.py 模拟）
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
TARGET_KIND="holding"
LAUNCH_YOLO=true

# ---------- 解析参数 ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --standoff) STANDOFF="$2";            shift 2 ;;
    --target)   TARGET_KIND="$2";         shift 2 ;;
    --no-yolo)  LAUNCH_YOLO=false;        shift   ;;
    -h|--help)
      grep '^#' "$0" | head -30 | sed 's/^# \{0,2\}//'
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

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
DOCKER_NAME="ros_noetic"
ROS_MASTER="http://${JETSON_IP}:11311"
ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
DOCKER_EXEC="docker exec ${DOCKER_NAME} bash -c"

# ---------- 后台进程 PID ----------
YOLO_PID=""

# ---------- 清理函数 ----------
cleanup() {
  echo ""
  step "Shutting down..."

  # 杀掉 host 上的 YOLO
  if [[ -n "${YOLO_PID}" ]] && kill -0 "${YOLO_PID}" 2>/dev/null; then
    info "Stopping YOLO (pid ${YOLO_PID})..."
    kill "${YOLO_PID}" 2>/dev/null || true
  fi

  # 停止 Docker 内所有 target follow 相关节点 (包括 move_base)
  info "Stopping target following + navigation nodes..."
  ${DOCKER_EXEC} "pkill -f 'roslaunch.*target_follow' 2>/dev/null; sleep 1" \
    2>/dev/null || true

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
echo -e "  Target:     ${TARGET_KIND}"
echo -e "  UDP port:   ${TRASH_UDP_PORT}"
echo -e "  YOLO:       $(${LAUNCH_YOLO} && echo enabled || echo DISABLED)"
echo ""

# =============================================================================
step "STEP 0 — Preflight checks"

# Docker running?
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${DOCKER_NAME}$"; then
  die "Docker container '${DOCKER_NAME}' is not running. Start it first:\n  docker start ${DOCKER_NAME}"
fi
ok "Docker container '${DOCKER_NAME}' is running"

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

# =============================================================================
step "STEP 2 — Start target following (standalone: LiDAR + move_base + follower)"

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

# =============================================================================
step "STEP 3 — Start Hand-Object detection (host)"

if ${LAUNCH_YOLO}; then
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
step "STEP 4 — System ready! Live status monitor"

echo ""
echo -e "  ${GRN}All components launched.${NC}  Press ${BOLD}Ctrl+C${NC} to shut everything down."
echo ""
echo -e "  Useful topics:"
echo -e "    ${CYN}rostopic echo /target_follower/status${NC}   — IDLE|TRACKING|REACHED|LOST|FAILED"
echo -e "    ${CYN}rostopic echo /target_follower/result${NC}   — True/False (dialogue trigger)"
echo -e "    ${CYN}rostopic echo /target_pose${NC}              — current target pose"
  echo -e "    ${CYN}tail -f /tmp/handobj.log${NC}                  — detection log"
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
    TRACKING) S_COLOR="${YLW}${STATUS}${NC}" ;;
    REACHED)  S_COLOR="${GRN}${STATUS}${NC}" ;;
    LOST|FAILED) S_COLOR="${RED}${STATUS}${NC}" ;;
    *)        S_COLOR="${STATUS}" ;;
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

  printf "\r  [%s]  status=%-10b  result=%-20b  YOLO=%b     " \
    "$(date +%H:%M:%S)" "${S_COLOR}" "${R_COLOR}" "${YOLO_ALIVE}"

  sleep 2
done
