#!/usr/bin/env bash
# =============================================================================
# start_demo.sh — 一键启动完整 Target-Following Demo
#
# 运行机器：Jetson Orin Nano (host 终端, 非 Docker)
# 启动内容：
#   1. Jetson Docker — roscore (if not running)
#   2. Jetson Docker — AMCL 导航栈 (real_robot_nav_unitree.launch)
#   3. Jetson Docker — Target Following Overlay (target_follow_real.launch)
#   4. Jetson Host   — YOLO 深度摄像头检测 (predict_15cls_rgbd.py)
#
# 前提（需手动完成）：
#   - Raspberry Pi 已启动: ./scripts/start_base.sh  (on Pi)
#   - Unitree LiDAR 已接 USB
#   - Orbbec 摄像头已接 USB
#
# 用法：
#   ./scripts/start_demo.sh
#   ./scripts/start_demo.sh --map maps/session_20260224_223938.yaml
#   ./scripts/start_demo.sh --map maps/demo_map.yaml --standoff 0.6 --target waste
#
# 参数：
#   --map PATH        地图 yaml 文件（相对于 catkin_ws/src/p3at_lms_navigation/），
#                     默认 maps/demo_map.yaml
#   --standoff M      停在目标前多远 (m), 默认 0.5
#   --target TYPE     检测目标类型 waste|person|auto, 默认 person
#   --no-yolo         不启动 YOLO（手动测试用，可用 send_target_udp.py 模拟）
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
MAPS_DIR="${CATKIN_WS}/src/p3at_lms_navigation/maps"
TRASH_DIR="${REPO_ROOT}/trash_detection"

# ---------- 默认参数 ----------
MAP_FILE="${MAPS_DIR}/demo_map.yaml"
STANDOFF="0.5"
TARGET_KIND="person"
LAUNCH_YOLO=true

# ---------- 解析参数 ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)      MAP_FILE="${MAPS_DIR}/$2"; shift 2 ;;
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

  # 停止 Docker 内的 target follow overlay
  info "Stopping target following nodes..."
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
    rosnode kill /target_follower /udp_target_bridge /point_to_target_pose /camera_link_tf 2>/dev/null || true" \
    2>/dev/null || true

  # 停止 Docker 内的导航栈
  info "Stopping navigation stack..."
  ${DOCKER_EXEC} "kill \$(pgrep -f 'roslaunch.*real_robot_nav') 2>/dev/null; sleep 1" \
    2>/dev/null || true

  ok "Demo stopped. Pi base driver still running — stop it manually on the Pi."
  exit 0
}
trap cleanup INT TERM

# =============================================================================
echo ""
echo -e "${BOLD}${CYN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYN}║     P3-AT Target Following Demo — Full Launch    ║${NC}"
echo -e "${BOLD}${CYN}╚══════════════════════════════════════════════════╝${NC}"
echo -e "  Map:        ${MAP_FILE##*/}"
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

# Map file exists?
if [[ ! -f "${MAP_FILE}" ]]; then
  warn "Map file not found: ${MAP_FILE}"
  echo "    Available maps:"
  ls "${MAPS_DIR}"/*.yaml 2>/dev/null | while read -r f; do echo "      ${f##*/}"; done || true
  die "Specify a valid map with --map <filename>"
fi
ok "Map file found: ${MAP_FILE##*/}"

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

# YOLO available?
if ${LAUNCH_YOLO}; then
  if ! command -v python3 &>/dev/null; then
    warn "python3 not found on host — YOLO will be skipped"
    LAUNCH_YOLO=false
  elif [[ ! -f "${TRASH_DIR}/predict_15cls_rgbd.py" ]]; then
    warn "predict_15cls_rgbd.py not found — YOLO will be skipped"
    LAUNCH_YOLO=false
  else
    ok "YOLO detection script found"
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
step "STEP 2 — Start AMCL navigation stack"

# Kill any existing mapping/nav launch before starting nav
${DOCKER_EXEC} "pkill -f 'roslaunch.*real_robot' 2>/dev/null; sleep 1" 2>/dev/null || true
sleep 1

info "Launching real_robot_nav_unitree.launch (map: ${MAP_FILE##*/})..."
${DOCKER_EXEC} "( ${ROS_ENV} && ${ROS_SETUP} && \
  exec roslaunch p3at_lms_navigation real_robot_nav_unitree.launch \
    map_file:=${MAP_FILE} \
    use_rviz:=false \
    use_target_follower:=false \
  > /tmp/nav_unitree.log 2>&1 ) &" 2>/dev/null

info "Waiting for move_base to come up (up to 30 s)..."
for i in $(seq 1 30); do
  sleep 1
  if ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null | grep -q move_base" 2>/dev/null; then
    ok "move_base is up (${i}s)"
    break
  fi
  if [[ $i -eq 30 ]]; then
    die "move_base did not appear after 30 s.\n  Check: docker exec ${DOCKER_NAME} tail -30 /tmp/nav_unitree.log"
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

echo ""
echo -e "  ${YLW}ACTION REQUIRED:${NC} Open RViz and set the initial pose estimate!"
echo -e "  Run in another terminal:  ${CYN}docker exec -it ${DOCKER_NAME} bash${NC}"
echo -e "  Then:  ${CYN}source /opt/ros/noetic/setup.bash${NC}"
echo -e "         ${CYN}export ROS_MASTER_URI=${ROS_MASTER}${NC}"
echo -e "         ${CYN}rviz -d ${CATKIN_WS}/src/p3at_lms_navigation/rviz/nav_unitree.rviz${NC}"
echo ""
echo -n "  Press ENTER when initial pose is set (or wait 10s to skip)..."
read -t 10 _ || true

# =============================================================================
step "STEP 3 — Start target following overlay"

${DOCKER_EXEC} "pkill -f 'roslaunch.*target_follow' 2>/dev/null; sleep 1" 2>/dev/null || true

info "Launching target_follow_real.launch..."
${DOCKER_EXEC} "( ${ROS_ENV} && ${ROS_SETUP} && \
  exec roslaunch target_follower target_follow_real.launch \
    standoff_distance:=${STANDOFF} \
    face_target:=true \
    target_timeout:=5.0 \
    udp_port:=${TRASH_UDP_PORT} \
  > /tmp/target_follow.log 2>&1 ) &" 2>/dev/null

info "Waiting for target_follower node (up to 20 s)..."
for i in $(seq 1 20); do
  sleep 1
  if ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
    rosnode list 2>/dev/null | grep -q target_follower" 2>/dev/null; then
    ok "target_follower is up (${i}s)"
    break
  fi
  if [[ $i -eq 20 ]]; then
    warn "target_follower not found after 20 s. Check: docker exec ${DOCKER_NAME} tail /tmp/target_follow.log"
  fi
done

# =============================================================================
step "STEP 4 — Start YOLO detection (host)"

if ${LAUNCH_YOLO}; then
  info "Starting predict_15cls_rgbd.py (UDP → ${JETSON_IP}:${TRASH_UDP_PORT})..."
  cd "${TRASH_DIR}"
  python3 predict_15cls_rgbd.py \
    --udp-enable \
    --udp-host "127.0.0.1" \
    --udp-port "${TRASH_UDP_PORT}" \
    --udp-frame-id "camera_link" \
    --udp-kind "${TARGET_KIND}" \
    --nearest-person \
    --headless \
    --print-xyz \
    > /tmp/yolo.log 2>&1 &
  YOLO_PID=$!
  sleep 2
  if kill -0 "${YOLO_PID}" 2>/dev/null; then
    ok "YOLO started (pid ${YOLO_PID}), log: /tmp/yolo.log"
  else
    warn "YOLO exited early. Check: tail /tmp/yolo.log"
    YOLO_PID=""
  fi
else
  warn "YOLO disabled. To simulate a target manually:"
  echo -e "    ${CYN}python3 ${TRASH_DIR}/examples/send_target_udp.py --z 2.0 --port ${TRASH_UDP_PORT}${NC}"
fi

# =============================================================================
step "STEP 5 — System ready! Live status monitor"

echo ""
echo -e "  ${GRN}All components launched.${NC}  Press ${BOLD}Ctrl+C${NC} to shut everything down."
echo ""
echo -e "  Useful topics:"
echo -e "    ${CYN}rostopic echo /target_follower/status${NC}   — IDLE|TRACKING|REACHED|LOST|FAILED"
echo -e "    ${CYN}rostopic echo /target_follower/result${NC}   — True/False (dialogue trigger)"
echo -e "    ${CYN}rostopic echo /target_pose${NC}              — current target pose"
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
