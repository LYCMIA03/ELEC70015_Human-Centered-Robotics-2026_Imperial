#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${SCRIPT_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# =============================================================================
# start_demo.sh — 一键启动完整 Target-Following + Dialogue Demo
#
# 运行机器：Jetson Orin Nano (host 终端, 非 Docker)
# 启动内容：
#   1. Jetson Docker — roscore (if not running)
#   2. Jetson Docker — navigation + target_follow_real
#   3. Jetson Host   — Hand-Object 检测 (handobj_detection_rgbd_remote_15cls.py)
#   4. Jetson Host   — Dialogue UDP bridges (nav_success → UDP:16041 → dialogue → /trash_action)
#
# 任务主逻辑：
#   默认进入 online_slam_task：实时SLAM地图作为权威地图，探索与去重基于 map frame。
#   仍由 target_follower 作为 move_base 的唯一 goal owner；frontier 只提供探索候选点。
#
# 导航方式（可切换）：
#   A) 默认：online_slam_task（无先验地图，边建图边任务）
#   B) 导入已有地图：map_server + amcl + move_base（map-assisted）
#
# 地图保存（可切换）：
#   --work-mapping 开启后，任务执行时持续保存在线地图。
#   若使用 --assist-map，则可同时保存融合后的工作地图。
#
# 对话流程：
#   REACHED → result=True → UDP:16041 → dialogue_udp_runner → UDP:16032 → /trash_action
#   /trash_action=True  → cooldown to ensure trash drop is complete → retreat
#   /trash_action=False → immediate retreat
#   retreat             → reverse-first (if safe) + turn-away → auto explore resumes
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
#   --lidar MODE     雷达模式 dual|unitree|rplidar, 默认 dual
#   --unitree-port DEV Unitree 串口设备, 默认 /dev/unitree_lidar
#   --rplidar-port DEV RPLIDAR 串口设备, 默认 /dev/rplidar_lidar
#   --rplidar-baud N RPLIDAR 串口波特率, 默认 256000
#   --rplidar-pre-start-motor     在初始化前先拉起第二个雷达电机（默认开启）
#   --no-rplidar-pre-start-motor  关闭第二个雷达电机预启动
#   --rplidar-pre-start-pwm N     第二个雷达预启动 PWM，默认 600
#   --rplidar-pre-start-warmup S  第二个雷达预热时间(秒)，默认 2.0
#   --sensor-only      只唤醒雷达链并验数，不启动导航/检测/对话/底盘相关逻辑
#   --assist-map YAML 导入已有地图辅助任务（启用 map_server+amcl）
#   --work-mapping     开启“工作中被动建图”（默认开启）
#   --no-work-mapping  关闭“工作中被动建图”
#   --map-save-prefix PFX 地图保存前缀（不含后缀 .yaml/.pgm）
#   --map-save-interval S 地图保存周期（秒），默认 45
#   --standoff M      停在目标前多远 (m), 默认 0.8
#   --retreat M       对话后前进撤离距离 (m), 默认 1.5
#   --retreat-turn-deg DEG 对话后原地转向角度(度), 默认 100
#   --action-timeout S 等待对话结果超时 (s), 默认 45
#   --explore-step M  auto explore 单步距离 (m), 默认 2.4
#   --explore-no-repeat-sec S 2分钟区域去重窗口(秒), 默认 120
#   --no-explore      关闭 auto explore（仅调试用）
#   --target TYPE     检测目标类型标签 holding|person|waste, 默认 holding
#   --waste-server URL 启用远程 15cls server (例如 http://172.26.183.130:8765)
#   --waste-call-every N 远程 15cls 调用频率；>=1 按每 N 帧，<1 按每 N 秒
#   --waste-async     远程 15cls 异步调用，避免阻塞主检测循环
#   --stream-enable   开启 detector MJPEG 流
#   --stream-port N   detector MJPEG 端口，默认 8765
#   --dialogue-device N 对话麦克风设备号, 默认 24
#   --only LIST       仅启动模块(逗号分隔): master,nav,yolo,dialogue,dashboard
#   --no-nav          不启动 target_follow_real (仅联调 dialogue 等)
#   --no-yolo         不启动检测（手动测试用，可用 send_target_udp.py 模拟）
#   --no-dialogue     不启动对话桥接（纲连测试）
#   --no-dashboard    不启动 dashboard
#   --dashboard-interval S dashboard 刷新周期(秒), 默认 2
#   --nav-readiness MODE 导航 ready 判定 strict|relaxed, 默认 strict
#   --strict-nav-readiness   等待 live /target_follower/status 后再视为 ready
#   --relaxed-nav-readiness  只要 move_base + target_follower 节点起来就继续
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
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${REPO_ROOT}/catkin_ws"
HANDOBJ_DIR="${REPO_ROOT}/handobj_detection"
RUNTIME_STATE_FILE="${XDG_RUNTIME_DIR:-/tmp}/hcr_demo_runtime.env"
HANDOBJ_DETECTOR_SCRIPT="handobj_detection_rgbd_remote_15cls.py"

# ---------- 默认参数 ----------
STANDOFF="0.8"
LIDAR_MODE="dual"
UNITREE_PORT="/dev/unitree_lidar"
RPLIDAR_PORT="/dev/rplidar_lidar"
RPLIDAR_BAUD="256000"
RPLIDAR_PRE_START_MOTOR=true
RPLIDAR_PRE_START_PWM="600"
RPLIDAR_PRE_START_WARMUP_S="2.0"
RETREAT_DIST="1.5"
ACTION_WAIT="45.0"
RUNNER_READY_TIMEOUT="45"
POST_ACCEPT_COOLDOWN="15.0"
POST_ACCEPT_COOLDOWN_SET=false
# A moderate retreat turn helps the robot leave the interaction area without
# rotating so aggressively that it traps itself near nearby obstacles.
RETREAT_TURN_DEG="100.0"
RETREAT_REVERSE_ENABLED=true
RETREAT_REVERSE_DIST="0.50"
ENABLE_AUTO_EXPLORE=true
# Keep exploration active, but slightly reduce step size so the robot probes
# nearby free space more cautiously when the environment is cluttered.
EXPLORE_STEP="2.4"
EXPLORE_NO_REPEAT_SEC="120.0"
EXPLORE_SCAN_TOPIC="/unitree/scan"
WORK_MAPPING=true
MAP_PERSISTENCE_RUNNING=false
ASSIST_MAP_FILE=""
MAP_SAVE_INTERVAL="45.0"
TARGET_KIND="holding"
WASTE_SERVER_URL=""
WASTE_CALL_EVERY="1.0"
WASTE_ASYNC=false
HANDOBJ_STREAM_ENABLE=false
HANDOBJ_STREAM_PORT="8765"
DIALOGUE_DEVICE="24"
LAUNCH_YOLO=true
LAUNCH_DIALOGUE=true
LAUNCH_NAV=true
LAUNCH_DASHBOARD=true
SENSOR_ONLY=false
DASHBOARD_INTERVAL="2"
AUTO_START_DOCKER=true
ONLY_MODULES=""
NEED_MASTER=true
SESSION_TAG="$(date +%Y%m%d_%H%M%S)"
MAP_SAVE_PREFIX="${CATKIN_WS}/src/p3at_lms_navigation/maps/task_session_${SESSION_TAG}"
NAV_PROFILE="online_slam_task"
ENABLE_RPLIDAR_IN_NAV="true"
NAV_READINESS_MODE="strict"

# ---------- 解析参数 ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --lidar)           LIDAR_MODE="$2"; shift 2 ;;
    --unitree-port)    UNITREE_PORT="$2"; shift 2 ;;
    --rplidar-port)    RPLIDAR_PORT="$2"; shift 2 ;;
    --rplidar-baud)    RPLIDAR_BAUD="$2"; shift 2 ;;
    --rplidar-pre-start-motor) RPLIDAR_PRE_START_MOTOR=true; shift ;;
    --no-rplidar-pre-start-motor) RPLIDAR_PRE_START_MOTOR=false; shift ;;
    --rplidar-pre-start-pwm) RPLIDAR_PRE_START_PWM="$2"; shift 2 ;;
    --rplidar-pre-start-warmup) RPLIDAR_PRE_START_WARMUP_S="$2"; shift 2 ;;
    --sensor-only)     SENSOR_ONLY=true; shift ;;
    --assist-map)      ASSIST_MAP_FILE="$2"; shift 2 ;;
    --work-mapping)    WORK_MAPPING=true; shift ;;
    --no-work-mapping) WORK_MAPPING=false; shift ;;
    --map-save-prefix) MAP_SAVE_PREFIX="$2"; shift 2 ;;
    --map-save-interval) MAP_SAVE_INTERVAL="$2"; shift 2 ;;
    --standoff)        STANDOFF="$2";       shift 2 ;;
    --retreat)         RETREAT_DIST="$2";   shift 2 ;;
    --retreat-turn-deg) RETREAT_TURN_DEG="$2"; shift 2 ;;
    --action-timeout)  ACTION_WAIT="$2";    shift 2 ;;
    --post-accept-cooldown) POST_ACCEPT_COOLDOWN="$2"; POST_ACCEPT_COOLDOWN_SET=true; shift 2 ;;
    --explore-step)    EXPLORE_STEP="$2"; shift 2 ;;
    --explore-no-repeat-sec) EXPLORE_NO_REPEAT_SEC="$2"; shift 2 ;;
    --no-explore)      ENABLE_AUTO_EXPLORE=false; shift ;;
    --target)          TARGET_KIND="$2";    shift 2 ;;
    --waste-server)    WASTE_SERVER_URL="$2"; shift 2 ;;
    --waste-call-every) WASTE_CALL_EVERY="$2"; shift 2 ;;
    --waste-async)     WASTE_ASYNC=true; shift ;;
    --stream-enable)   HANDOBJ_STREAM_ENABLE=true; shift ;;
    --stream-port)     HANDOBJ_STREAM_PORT="$2"; shift 2 ;;
    --dialogue-device) DIALOGUE_DEVICE="$2"; shift 2 ;;
    --only)            ONLY_MODULES="$2";   shift 2 ;;
    --no-nav)          LAUNCH_NAV=false;    shift   ;;
    --no-yolo)         LAUNCH_YOLO=false;   shift   ;;
    --no-dialogue)     LAUNCH_DIALOGUE=false; shift  ;;
    --no-dashboard)    LAUNCH_DASHBOARD=false; shift ;;
    --dashboard-interval) DASHBOARD_INTERVAL="$2"; shift 2 ;;
    --nav-readiness)   NAV_READINESS_MODE="$2"; shift 2 ;;
    --strict-nav-readiness) NAV_READINESS_MODE="strict"; shift ;;
    --relaxed-nav-readiness) NAV_READINESS_MODE="relaxed"; shift ;;
    --no-auto-start-docker) AUTO_START_DOCKER=false; shift ;;
    -h|--help)
      awk '
        /^# =============================================================================$/ {sep_count++; if (sep_count == 2) exit}
        /^#/ {sub(/^# ?/, ""); print}
      ' "$0" | sed '/^!/d;/^shellcheck /d'
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

if [[ "${LIDAR_MODE}" != "dual" && "${LIDAR_MODE}" != "unitree" && "${LIDAR_MODE}" != "rplidar" ]]; then
  die "Invalid --lidar: ${LIDAR_MODE}. Expected dual|unitree|rplidar"
fi

if [[ "${NAV_READINESS_MODE}" != "strict" && "${NAV_READINESS_MODE}" != "relaxed" ]]; then
  die "Invalid --nav-readiness: ${NAV_READINESS_MODE}. Expected strict|relaxed"
fi

if [[ -n "${ASSIST_MAP_FILE}" ]]; then
  if [[ ! -f "${ASSIST_MAP_FILE}" ]]; then
    die "--assist-map not found: ${ASSIST_MAP_FILE}"
  fi
  ASSIST_MAP_FILE="$(realpath "${ASSIST_MAP_FILE}")"
  NAV_PROFILE="map_assisted"
  if [[ "${LIDAR_MODE}" == "rplidar" ]]; then
    die "--assist-map currently supports lidar mode unitree|dual (rplidar-only map-assisted nav is not supported)"
  fi
fi

if [[ "${LIDAR_MODE}" == "unitree" ]]; then
  ENABLE_RPLIDAR_IN_NAV="false"
  EXPLORE_SCAN_TOPIC="/unitree/scan"
elif [[ "${LIDAR_MODE}" == "dual" ]]; then
  ENABLE_RPLIDAR_IN_NAV="true"
  EXPLORE_SCAN_TOPIC="/unitree/scan"
elif [[ "${LIDAR_MODE}" == "rplidar" ]]; then
  EXPLORE_SCAN_TOPIC="/scan_filtered"
fi

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

if ${SENSOR_ONLY}; then
  LAUNCH_NAV=false
  LAUNCH_YOLO=false
  LAUNCH_DIALOGUE=false
  LAUNCH_DASHBOARD=false
  WORK_MAPPING=false
  ENABLE_AUTO_EXPLORE=false
  NEED_MASTER=true
  NAV_PROFILE="lidar_only"
  if [[ -n "${ASSIST_MAP_FILE}" ]]; then
    die "--assist-map cannot be used with --sensor-only"
  fi
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
if ! ${POST_ACCEPT_COOLDOWN_SET}; then
  POST_ACCEPT_COOLDOWN="${POST_ACCEPT_COOLDOWN_S:-${POST_ACCEPT_COOLDOWN}}"
fi
RUNNER_READY_TIMEOUT="${RUNNER_READY_TIMEOUT:-45}"
UDP_PORT_WINDOW="${UDP_PORT_WINDOW:-500}"
DOCKER_NAME="ros_noetic"
ROS_MASTER="http://${JETSON_IP}:11311"
ROS_SETUP="source /opt/ros/noetic/setup.bash && source ${CATKIN_WS}/devel/setup.bash"
ROS_ENV="export ROS_MASTER_URI=${ROS_MASTER} && export ROS_IP=${JETSON_IP}"
DOCKER_EXEC="docker exec --user $(id -u):$(id -g) ${DOCKER_NAME} bash -c"

_docker_exec_detached() {
  local cmd="$1"
  docker exec --user "$(id -u):$(id -g)" -d "${DOCKER_NAME}" bash -lc "${cmd}" >/dev/null
}

_stop_docker_nav_and_lidar_processes() {
  ${DOCKER_EXEC} "pkill -f 'roslaunch.*target_follow' 2>/dev/null || true; \
                  pkill -f 'roslaunch.*real_robot' 2>/dev/null || true; \
                  pkill -f 'roslaunch.*passive_mapping_unitree' 2>/dev/null || true; \
                  pkill -f '[m]ove_base' 2>/dev/null || true; \
                  pkill -f '[a]mcl' 2>/dev/null || true; \
                  pkill -f '[m]ap_server' 2>/dev/null || true; \
                  pkill -f '[s]lam_gmapping' 2>/dev/null || true; \
                  pkill -f '[u]nitree_lidar_ros_node' 2>/dev/null || true; \
                  pkill -f '[p]ointcloud_to_laserscan_node' 2>/dev/null || true; \
                  pkill -f '[r]plidarNode' 2>/dev/null || true; \
                  pkill -f '[r]plidar_health_monitor.py' 2>/dev/null || true; \
                  pkill -f '[s]can_body_filter.py' 2>/dev/null || true; \
                  pkill -f '[t]arget_follower.py' 2>/dev/null || true; \
                  pkill -f '[u]dp_target_bridge.py' 2>/dev/null || true; \
                  pkill -f '[p]oint_to_target_pose.py' 2>/dev/null || true; \
                  pkill -f '[o]dom_republisher.py' 2>/dev/null || true; \
                  pkill -f '[r]obot_state_publisher' 2>/dev/null || true; \
                  pkill -f '[s]tatic_transform_publisher' 2>/dev/null || true; \
                  pkill -f '[c]ontinuous_map_manager.py' 2>/dev/null || true; \
                  if pgrep -x rosmaster >/dev/null 2>&1; then \
                    ${ROS_ENV} && ${ROS_SETUP} && printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true; \
                  fi; \
                  sleep 1" 2>/dev/null || true
}

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

_write_runtime_state() {
  cat > "${RUNTIME_STATE_FILE}" <<EOF
REPO_ROOT='${REPO_ROOT}'
CATKIN_WS='${CATKIN_WS}'
HANDOBJ_DIR='${HANDOBJ_DIR}'
HANDOBJ_DETECTOR_SCRIPT='${HANDOBJ_DETECTOR_SCRIPT}'
DOCKER_NAME='${DOCKER_NAME}'
JETSON_IP='${JETSON_IP}'
ROS_MASTER='${ROS_MASTER}'
TRASH_UDP_PORT='${TRASH_UDP_PORT}'
DIALOGUE_TRIGGER_UDP_PORT='${DIALOGUE_TRIGGER_UDP_PORT}'
DIALOGUE_ACTION_UDP_PORT='${DIALOGUE_ACTION_UDP_PORT}'
LIDAR_MODE='${LIDAR_MODE}'
UNITREE_PORT='${UNITREE_PORT}'
RPLIDAR_PORT='${RPLIDAR_PORT}'
RPLIDAR_BAUD='${RPLIDAR_BAUD}'
RPLIDAR_PRE_START_MOTOR='${RPLIDAR_PRE_START_MOTOR}'
RPLIDAR_PRE_START_PWM='${RPLIDAR_PRE_START_PWM}'
RPLIDAR_PRE_START_WARMUP_S='${RPLIDAR_PRE_START_WARMUP_S}'
SENSOR_ONLY='${SENSOR_ONLY}'
NAV_PROFILE='${NAV_PROFILE}'
WORK_MAPPING='${WORK_MAPPING}'
ASSIST_MAP_FILE='${ASSIST_MAP_FILE}'
MAP_SAVE_PREFIX='${MAP_SAVE_PREFIX}'
MAP_SAVE_INTERVAL='${MAP_SAVE_INTERVAL}'
STANDOFF='${STANDOFF}'
RETREAT_DIST='${RETREAT_DIST}'
RETREAT_TURN_DEG='${RETREAT_TURN_DEG}'
ACTION_WAIT='${ACTION_WAIT}'
POST_ACCEPT_COOLDOWN='${POST_ACCEPT_COOLDOWN}'
ENABLE_AUTO_EXPLORE='${ENABLE_AUTO_EXPLORE}'
EXPLORE_STEP='${EXPLORE_STEP}'
EXPLORE_NO_REPEAT_SEC='${EXPLORE_NO_REPEAT_SEC}'
EXPLORE_SCAN_TOPIC='${EXPLORE_SCAN_TOPIC}'
NAV_READINESS_MODE='${NAV_READINESS_MODE}'
TARGET_KIND='${TARGET_KIND}'
WASTE_SERVER_URL='${WASTE_SERVER_URL}'
WASTE_CALL_EVERY='${WASTE_CALL_EVERY}'
WASTE_ASYNC='${WASTE_ASYNC}'
HANDOBJ_STREAM_ENABLE='${HANDOBJ_STREAM_ENABLE}'
HANDOBJ_STREAM_PORT='${HANDOBJ_STREAM_PORT}'
DIALOGUE_DEVICE='${DIALOGUE_DEVICE}'
RUNNER_READY_TIMEOUT='${RUNNER_READY_TIMEOUT}'
EOF
}

# ---------- 动态端口分配 ----------
USED_UDP_PORTS=()
TRASH_UDP_PORT="$(_select_udp_port "trash_detection->ROS" "${TRASH_UDP_PORT}")"
DIALOGUE_TRIGGER_UDP_PORT="$(_select_udp_port "nav_success->dialogue" "${DIALOGUE_TRIGGER_UDP_PORT}")"
DIALOGUE_ACTION_UDP_PORT="$(_select_udp_port "dialogue->trash_action" "${DIALOGUE_ACTION_UDP_PORT}")"
if [[ "${MAP_SAVE_PREFIX}" != /* ]]; then
  MAP_SAVE_PREFIX="${REPO_ROOT}/${MAP_SAVE_PREFIX}"
fi
_write_runtime_state

# ---------- Bridge 健康检查 ----------
_ros_node_exists() {
  local node="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rosnode list 2>/dev/null | grep -q \"^/${node}$\"" \
    2>/dev/null
}

_topic_has_publisher() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info ${topic} 2>/dev/null | awk '/Publishers:/{flag=1;next}/Subscribers:/{flag=0}flag' | grep -q '\\*'" \
    2>/dev/null
}

_topic_has_message() {
  local topic="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic echo -n 1 ${topic} 2>/dev/null >/dev/null" \
    2>/dev/null
}

_ros_package_exists() {
  local pkg="$1"
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rospack find ${pkg} >/dev/null 2>&1" \
    2>/dev/null
}

_check_serial_device_rw() {
  local label="$1"
  local device="$2"

  if [[ ! -e "${device}" ]]; then
    die "${label} device not found: ${device}. Reconnect the sensor or pass the correct port."
  fi
  if [[ ! -r "${device}" || ! -w "${device}" ]]; then
    ls -l "${device}" 2>/dev/null || true
    die "${label} device is not readable/writable: ${device}. Check dialout group / udev rules."
  fi
}

_ros_odom_fresh() {
  local max_age_s="${1:-3.0}"
  ${DOCKER_EXEC} "export MAX_AGE_S='${max_age_s}' && ${ROS_ENV} && ${ROS_SETUP} && python3 - <<'PY'
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry

rospy.init_node('odom_fresh_check', anonymous=True, disable_signals=True)
max_age_s = float(os.environ['MAX_AGE_S'])

try:
    msg = rospy.wait_for_message('/odom', Odometry, timeout=4.0)
except Exception:
    sys.exit(1)

stamp = msg.header.stamp.to_sec()
age = float('inf') if stamp <= 0.0 else abs(time.time() - stamp)
sys.exit(0 if age <= max_age_s else 2)
PY" 2>/dev/null
}

_ros_odom_alive() {
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic echo -n 1 /odom 2>/dev/null >/dev/null" \
    2>/dev/null
}

_ros_odom_stale_but_alive() {
  _ros_odom_alive && ! _ros_odom_fresh 3.0
}

_ros_tf_fresh() {
  local parent_frame="$1"
  local child_frame="$2"
  local max_age_s="${3:-3.0}"
  ${DOCKER_EXEC} "export TF_PARENT='${parent_frame}' TF_CHILD='${child_frame}' MAX_AGE_S='${max_age_s}' && ${ROS_ENV} && ${ROS_SETUP} && python3 - <<'PY'
import os
import sys
import time

import rospy
import tf2_ros

parent = os.environ['TF_PARENT']
child = os.environ['TF_CHILD']
max_age_s = float(os.environ['MAX_AGE_S'])

rospy.init_node('tf_fresh_check', anonymous=True, disable_signals=True)
buf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
listener = tf2_ros.TransformListener(buf)
deadline = time.time() + 4.0

while time.time() < deadline and not rospy.is_shutdown():
    try:
        if buf.can_transform(parent, child, rospy.Time(0), rospy.Duration(0.3)):
            tfm = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.3))
            stamp = tfm.header.stamp.to_sec()
            # Some base drivers expose a valid latest transform with stamp=0.
            # Treat that as usable once the transform is available, and rely on
            # the separate /odom freshness check to guard against stale motion data.
            if stamp <= 0.0:
                sys.exit(0)
            age = abs(time.time() - stamp)
            sys.exit(0 if age <= max_age_s else 2)
    except Exception:
        pass
    time.sleep(0.1)

sys.exit(1)
PY" 2>/dev/null
}

_target_follower_ready() {
  _ros_node_exists "target_follower" \
    && _topic_has_publisher "/target_follower/status" \
    && _topic_has_message "/target_follower/status"
}

_nav_stack_ready() {
  local core_ok=1
  case "${NAV_READINESS_MODE}" in
    strict)
      _ros_node_exists "move_base" \
        && _target_follower_ready \
        || core_ok=0
      ;;
    relaxed)
      _ros_node_exists "move_base" \
        && _ros_node_exists "target_follower" \
        || core_ok=0
      ;;
  esac
  [[ "${core_ok}" -eq 1 ]] || return 1

  if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
    _topic_has_message "/map" || return 1
    _ros_tf_fresh map base_link 3.0 || return 1
  fi

  return 0
}

_lidar_stack_ready() {
  case "${LIDAR_MODE}" in
    dual)
      _ros_node_exists "unitree_lidar" \
        && _topic_has_message "/unitree/scan" \
        && _ros_node_exists "rplidarNode" \
        && _topic_has_message "/rplidar/scan_filtered"
      ;;
    unitree)
      _ros_node_exists "unitree_lidar" \
        && _topic_has_message "/unitree/scan"
      ;;
    rplidar)
      _ros_node_exists "rplidarNode" \
        && _topic_has_message "/scan_filtered"
      ;;
  esac
}

_lidar_stack_blockers() {
  local blockers=()

  if [[ "${LIDAR_MODE}" != "rplidar" ]]; then
    if ! _ros_node_exists "unitree_lidar"; then
      blockers+=("/unitree_lidar node missing")
    elif ! _topic_has_message "/unitree/scan"; then
      blockers+=("/unitree/scan has no messages")
    fi
  fi

  if [[ "${LIDAR_MODE}" != "unitree" ]]; then
    if ! _ros_node_exists "rplidarNode"; then
      blockers+=("/rplidarNode missing")
    elif [[ "${LIDAR_MODE}" == "dual" ]]; then
      _topic_has_message "/rplidar/scan_filtered" || blockers+=("/rplidar/scan_filtered has no messages")
    else
      _topic_has_message "/scan_filtered" || blockers+=("/scan_filtered has no messages")
    fi
  fi

  if [[ ${#blockers[@]} -eq 0 ]]; then
    return 0
  fi

  printf '%s\n' "${blockers[@]}"
  return 1
}

_nav_stack_blockers() {
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

  if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
    if ! _topic_has_message "/map"; then
      blockers+=("/map has no messages")
    fi
    if ! _ros_tf_fresh map base_link 3.0; then
      blockers+=("TF map->base_link unavailable/stale")
    fi
  fi

  if [[ ${#blockers[@]} -eq 0 ]]; then
    return 0
  fi

  printf '%s\n' "${blockers[@]}"
  return 1
}

_dump_nav_readiness_debug() {
  warn "Navigation readiness failed. Capturing quick diagnostics..."
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null" \
    2>/dev/null | sed 's/^/  node: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /move_base/status 2>/dev/null" \
    2>/dev/null | sed 's/^/  move_base_status: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /target_follower/status 2>/dev/null" \
    2>/dev/null | sed 's/^/  follower_status: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic echo -n 1 /target_follower/status 2>/dev/null" \
    2>/dev/null | sed 's/^/  follower_status_msg: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic echo -n 1 /odom/header 2>/dev/null" \
    2>/dev/null | sed 's/^/  odom_header: /' || true
  ${DOCKER_EXEC} "tail -n 60 /tmp/target_follow.log 2>/dev/null" \
    2>/dev/null | sed 's/^/  nav_log: /' || true
  ${DOCKER_EXEC} "tail -n 60 /tmp/task_nav_backbone.log 2>/dev/null" \
    2>/dev/null | sed 's/^/  nav_backbone_log: /' || true
}

_dump_lidar_bringup_debug() {
  warn "LiDAR bringup failed. Capturing quick diagnostics..."
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && rosnode list 2>/dev/null" \
    2>/dev/null | sed 's/^/  node: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /unilidar/cloud 2>/dev/null" \
    2>/dev/null | sed 's/^/  unitree_cloud: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /unitree/scan 2>/dev/null" \
    2>/dev/null | sed 's/^/  unitree_scan: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /rplidar/scan_filtered 2>/dev/null" \
    2>/dev/null | sed 's/^/  rplidar_scan: /' || true
  ${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && timeout 5 rostopic info /scan_filtered 2>/dev/null" \
    2>/dev/null | sed 's/^/  rplidar_scan_single: /' || true
  ${DOCKER_EXEC} "tail -n 80 /tmp/lidar_bringup.log 2>/dev/null" \
    2>/dev/null | sed 's/^/  lidar_log: /' || true
}

# ---------- 后台进程 PID ----------
YOLO_PID=""
BRIDGE_PID=""
DIALOGUE_PID=""
DASHBOARD_PID=""

_kill_tree() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then return; fi
  kill -0 "${pid}" 2>/dev/null || return 0
  local child
  for child in $(pgrep -P "${pid}" 2>/dev/null || true); do
    _kill_tree "${child}"
  done
  kill "${pid}" 2>/dev/null || true
}

# ---------- 清理函数 ----------
cleanup() {
  echo ""
  step "Shutting down..."

  if [[ -n "${YOLO_PID}" ]] && kill -0 "${YOLO_PID}" 2>/dev/null; then
    info "Stopping YOLO (pid ${YOLO_PID})..."
    _kill_tree "${YOLO_PID}"
  fi

  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    info "Stopping dialogue bridges (pid ${BRIDGE_PID})..."
    _kill_tree "${BRIDGE_PID}"
  fi

  if [[ -n "${DIALOGUE_PID}" ]] && kill -0 "${DIALOGUE_PID}" 2>/dev/null; then
    info "Stopping dialogue runner (pid ${DIALOGUE_PID})..."
    _kill_tree "${DIALOGUE_PID}"
  fi

  if [[ -n "${DASHBOARD_PID}" ]] && kill -0 "${DASHBOARD_PID}" 2>/dev/null; then
    info "Stopping dashboard (pid ${DASHBOARD_PID})..."
    _kill_tree "${DASHBOARD_PID}"
  fi

  rm -f "${RUNTIME_STATE_FILE}"

  # 停止 Docker 内所有 target follow 相关节点 (包括 move_base)
  if ${LAUNCH_NAV} || ${SENSOR_ONLY}; then
    info "Stopping target-following + navigation + passive-mapping nodes..."
    _stop_docker_nav_and_lidar_processes
  fi

  ok "Demo stopped. Pi base driver still running — stop it manually on the Pi."
  exit 0
}
trap cleanup INT TERM

# =============================================================================
echo ""
echo -e "${BOLD}${CYN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYN}║  P3-AT Target Following Demo — Task-First Mode   ║${NC}"
echo -e "${BOLD}${CYN}╚══════════════════════════════════════════════════╝${NC}"
echo -e "  Nav mode:   ${NAV_PROFILE}"
if [[ -n "${ASSIST_MAP_FILE}" ]]; then
  echo -e "  Assist map: ${ASSIST_MAP_FILE}"
fi
echo -e "  Work map:   $(${WORK_MAPPING} && echo enabled || echo DISABLED)"
echo -e "  Map save:   ${MAP_SAVE_PREFIX}.yaml/.pgm (interval ${MAP_SAVE_INTERVAL}s)"
echo -e "  Standoff:   ${STANDOFF} m"
echo -e "  Retreat:    ${RETREAT_DIST} m (after dialogue)"
echo -e "  Turn angle: ${RETREAT_TURN_DEG} deg"
echo -e "  Act.timeout:${ACTION_WAIT} s"
echo -e "  Auto explore: $(${ENABLE_AUTO_EXPLORE} && echo enabled || echo DISABLED)"
echo -e "  Explore step: ${EXPLORE_STEP} m"
echo -e "  Explore no-repeat: ${EXPLORE_NO_REPEAT_SEC} s"
echo -e "  Nav ready:  ${NAV_READINESS_MODE}"
echo -e "  Sensor only:$(${SENSOR_ONLY} && echo enabled || echo disabled)"
echo -e "  Target:     ${TARGET_KIND}"
echo -e "  Detector:   ${HANDOBJ_DETECTOR_SCRIPT}"
if [[ -n "${WASTE_SERVER_URL}" ]]; then
  echo -e "  Waste srv:  ${WASTE_SERVER_URL}  (every ${WASTE_CALL_EVERY}, async=$(${WASTE_ASYNC} && echo on || echo off))"
fi
if ${HANDOBJ_STREAM_ENABLE}; then
  echo -e "  MJPEG:      http://<jetson-ip>:${HANDOBJ_STREAM_PORT}/"
fi
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

if ! [[ "${MAP_SAVE_INTERVAL}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  die "Invalid --map-save-interval: ${MAP_SAVE_INTERVAL}"
fi

if [[ "${MAP_SAVE_PREFIX}" != /* ]]; then
  MAP_SAVE_PREFIX="${REPO_ROOT}/${MAP_SAVE_PREFIX}"
fi

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

if [[ -n "${ASSIST_MAP_FILE}" ]]; then
  [[ -f "${ASSIST_MAP_FILE}" ]] || die "Assist map missing: ${ASSIST_MAP_FILE}"
  if ! ${LAUNCH_NAV}; then
    die "--assist-map requires navigation to be enabled"
  fi
  ok "Assist map found"
fi

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
  elif [[ ! -f "${HANDOBJ_DIR}/${HANDOBJ_DETECTOR_SCRIPT}" ]]; then
    warn "${HANDOBJ_DETECTOR_SCRIPT} not found — detection will be skipped"
    LAUNCH_YOLO=false
  else
    ok "Hand-Object detection script found"
  fi
fi

if ${NEED_MASTER}; then
  if ${LAUNCH_NAV} || ${SENSOR_ONLY}; then
    _ros_package_exists "target_follower" || die "target_follower package missing in ROS environment"
    if [[ "${LIDAR_MODE}" != "rplidar" ]]; then
      _ros_package_exists "unitree_lidar_ros" || die "unitree_lidar_ros not found in ROS environment. Install/build Unitree driver first."
      _ros_package_exists "pointcloud_to_laserscan" || die "pointcloud_to_laserscan package missing in ROS environment."
    fi
    if [[ "${LIDAR_MODE}" != "unitree" ]]; then
      _ros_package_exists "rplidar_ros" || die "rplidar_ros not found in ROS environment. Install it first with ./setup_rplidar_a2.sh or apt."
    fi
  fi
  if ${LAUNCH_NAV}; then
    if [[ "${NAV_PROFILE}" == "online_slam_task" ]] && ! _ros_package_exists "gmapping"; then
      die "gmapping package missing in ROS environment (required by online_slam_task)"
    fi
    if [[ "${NAV_PROFILE}" != "online_slam_task" ]] && ${WORK_MAPPING} && ! _ros_package_exists "gmapping"; then
      die "gmapping package missing in ROS environment (required by --work-mapping)"
    fi
  fi
  if ${LAUNCH_NAV} && [[ -n "${ASSIST_MAP_FILE}" ]]; then
    _ros_package_exists "map_server" || die "map_server package missing in ROS environment"
    _ros_package_exists "amcl" || die "amcl package missing in ROS environment"
  fi
fi

# =============================================================================
step "STEP 1 — Ensure roscore is running"

if ${NEED_MASTER}; then
  if ${DOCKER_EXEC} "pgrep -x rosmaster" &>/dev/null; then
    ok "roscore already running"
  else
    info "Starting roscore..."
    _docker_exec_detached "${ROS_ENV} && source /opt/ros/noetic/setup.bash && exec roscore > /tmp/roscore.log 2>&1" || true
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
step "STEP 2 — Start Hand-Object detection (host)"

if ${LAUNCH_YOLO}; then
  pkill -f "handobj_detection_rgbd.py|handobj_detection_rgbd_remote_15cls.py" 2>/dev/null || true
  info "Starting ${HANDOBJ_DETECTOR_SCRIPT} (UDP → 127.0.0.1:${TRASH_UDP_PORT})..."
  (
    cd "${HANDOBJ_DIR}"
    HANDOBJ_CMD=(
      python3 "${HANDOBJ_DETECTOR_SCRIPT}"
      --udp-enable
      --udp-host "127.0.0.1"
      --udp-port "${TRASH_UDP_PORT}"
      --udp-frame-id "camera_link"
      --udp-kind "${TARGET_KIND}"
      --rotate-180
      --headless
      --print-xyz
    )
    if [[ -n "${WASTE_SERVER_URL}" ]]; then
      HANDOBJ_CMD+=(
        --waste-server "${WASTE_SERVER_URL}"
        --waste-call-every "${WASTE_CALL_EVERY}"
      )
      if ${WASTE_ASYNC}; then
        HANDOBJ_CMD+=(--waste-async)
      fi
    fi
    if ${HANDOBJ_STREAM_ENABLE}; then
      HANDOBJ_CMD+=(
        --stream-enable
        --stream-port "${HANDOBJ_STREAM_PORT}"
      )
    fi
    exec "${HANDOBJ_CMD[@]}"
  ) > /tmp/handobj.log 2>&1 &
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
step "STEP 3 — Start Dialogue (runner + Docker bridges)"

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
    _runner_wait_start_ts="$(date +%s)"
    while true; do
      if ! kill -0 "${DIALOGUE_PID}" 2>/dev/null; then
        break
      fi
      if ss -lunp 2>/dev/null | grep -q "0.0.0.0:${DIALOGUE_TRIGGER_UDP_PORT}"; then
        RUNNER_READY=1
        break
      fi
      _runner_now_ts="$(date +%s)"
      if (( _runner_now_ts - _runner_wait_start_ts >= RUNNER_READY_TIMEOUT )); then
        break
      fi
      sleep 0.5
    done
    if [[ "${RUNNER_READY}" -ne 1 ]]; then
      die "Dialogue runner failed to become ready within ${RUNNER_READY_TIMEOUT}s. Check: tail -60 /tmp/dialogue_runner.log"
    fi
    ok "Dialogue runner started (pid ${DIALOGUE_PID}), log: /tmp/dialogue_runner.log"

    _launch_dialogue_bridges() {
      if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
        sleep 1
      fi
      (
        cd "${REPO_ROOT}"
        exec env \
          MASTER_HOST=jetson \
          DIALOGUE_TRIGGER_UDP_PORT="${DIALOGUE_TRIGGER_UDP_PORT}" \
          DIALOGUE_ACTION_UDP_PORT="${DIALOGUE_ACTION_UDP_PORT}" \
          bash "${SCRIPT_DIR}/start_dialogue_docker_bridges.sh"
      ) > /tmp/dialogue_bridge.log 2>&1 &
      BRIDGE_PID=$!
      sleep 2
      kill -0 "${BRIDGE_PID}" 2>/dev/null
    }

    _dialogue_processes_healthy() {
      local max_wait="$1"
      for i in $(seq 1 "${max_wait}"); do
        if kill -0 "${DIALOGUE_PID}" 2>/dev/null \
           && _ros_node_exists "navigation_success_udp_bridge" \
           && _ros_node_exists "udp_trash_action_bridge"; then
          return 0
        fi
        sleep 1
      done
      return 1
    }

    _dialogue_mesh_healthy() {
      local max_wait="$1"
      for i in $(seq 1 "${max_wait}"); do
        if _dialogue_processes_healthy 1 && _topic_has_publisher "/trash_action"; then
          return 0
        fi
        sleep 1
      done
      return 1
    }

    info "Starting dialogue docker bridges..."
    _launch_dialogue_bridges || die "Dialogue bridge exited early. Check: tail -40 /tmp/dialogue_bridge.log"
    ok "Dialogue bridges started (pid ${BRIDGE_PID}), log: /tmp/dialogue_bridge.log"
    ok "  nav_success → UDP:${DIALOGUE_TRIGGER_UDP_PORT} → dialogue"
    ok "  dialogue   → UDP:${DIALOGUE_ACTION_UDP_PORT}  → /trash_action"

    info "Validating dialogue startup (runner + ROS bridge nodes)..."
    if ! _dialogue_processes_healthy 15; then
      warn "Dialogue startup check failed; retrying bridge startup once..."
      _launch_dialogue_bridges || die "Dialogue bridge retry failed. Check: tail -40 /tmp/dialogue_bridge.log"
      if ! _dialogue_processes_healthy 10; then
        die "Dialogue startup still unhealthy after retry. Check: tail -80 /tmp/dialogue_bridge.log and /tmp/dialogue_runner.log"
      fi
    fi
    ok "Dialogue runner and bridge nodes are healthy"
  fi
else
  warn "Dialogue bridge disabled (--no-dialogue). /trash_action will not be published."
  warn "  Simulate with: rostopic pub /trash_action std_msgs/Bool 'data: false'"
fi

# =============================================================================
step "STEP 4 — Start runtime stack"

if ! ${LAUNCH_NAV} && ! ${SENSOR_ONLY}; then
  warn "Navigation disabled (--no-nav or --only without nav); skipping STEP 4"
else
  # Clear stale manual test nodes so both lidar serial devices stay single-owner.
  _stop_docker_nav_and_lidar_processes
  sleep 1

  if ${SENSOR_ONLY}; then
    info "Launching lidar-only bringup (no move_base, no target_follower motion logic)..."
  elif [[ "${NAV_PROFILE}" == "map_assisted" ]]; then
    info "Launching map-assisted task flow (map_server + amcl + move_base + target overlay)..."
  else
    info "Launching online_slam_task flow (LiDAR + SLAM + move_base + frontier target_follower)..."
  fi

  if [[ "${LIDAR_MODE}" == "dual" && "${UNITREE_PORT}" == "${RPLIDAR_PORT}" ]]; then
    die "Dual-lidar mode requires two distinct serial ports. Current value: ${UNITREE_PORT}"
  fi

  if [[ "${LIDAR_MODE}" != "rplidar" ]]; then
    _check_serial_device_rw "Unitree LiDAR" "${UNITREE_PORT}"
  fi

  if [[ "${LIDAR_MODE}" != "unitree" ]]; then
    _check_serial_device_rw "RPLIDAR" "${RPLIDAR_PORT}"
  fi

  if ${SENSOR_ONLY}; then
    _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
      exec roslaunch p3at_lms_navigation real_robot_lidar_bringup.launch \
        lidar_mode:=${LIDAR_MODE} \
        unitree_port:=${UNITREE_PORT} \
        rplidar_port:=${RPLIDAR_PORT} \
        rplidar_baud:=${RPLIDAR_BAUD} \
        rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR} \
        rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM} \
        rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S} \
      > /tmp/lidar_bringup.log 2>&1"
  elif [[ "${NAV_PROFILE}" == "map_assisted" ]]; then
    _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
      exec roslaunch p3at_lms_navigation real_robot_nav_unitree.launch \
        use_rviz:=false \
        use_target_follower:=false \
        map_file:=${ASSIST_MAP_FILE} \
        unitree_port:=${UNITREE_PORT} \
        enable_rplidar:=${ENABLE_RPLIDAR_IN_NAV} \
        rplidar_port:=${RPLIDAR_PORT} \
        rplidar_baud:=${RPLIDAR_BAUD} \
        rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR} \
        rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM} \
        rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S} \
      > /tmp/task_nav_backbone.log 2>&1"
    sleep 2

    _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
      exec roslaunch target_follower target_follow_real.launch \
        launch_move_base:=false \
        global_frame:=map \
        lidar_mode:=${LIDAR_MODE} \
        unitree_port:=${UNITREE_PORT} \
        rplidar_port:=${RPLIDAR_PORT} \
        rplidar_baud:=${RPLIDAR_BAUD} \
        rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR} \
        rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM} \
        rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S} \
        standoff_distance:=${STANDOFF} \
        face_target:=true \
        target_timeout:=5.0 \
        send_rate_hz:=4.0 \
        min_update_dist:=0.08 \
        enable_close_approach:=true \
        close_approach_threshold:=1.2 \
        tracking_near_target_window_m:=2.2 \
        tracking_near_send_rate_hz:=6.0 \
        tracking_near_min_update_dist:=0.04 \
        tracking_near_goal_to_target:=true \
        udp_port:=${TRASH_UDP_PORT} \
        retreat_distance:=${RETREAT_DIST} \
        retreat_turn_angle_deg:=${RETREAT_TURN_DEG} \
        retreat_reverse_enabled:=${RETREAT_REVERSE_ENABLED} \
        retreat_reverse_distance:=${RETREAT_REVERSE_DIST} \
        action_wait_timeout:=${ACTION_WAIT} \
        enable_auto_explore:=${ENABLE_AUTO_EXPLORE} \
        explore_goal_distance:=${EXPLORE_STEP} \
        explore_short_horizon:=${EXPLORE_STEP} \
        explore_map_topic:=/map \
        explore_costmap_topic:=/move_base/global_costmap/costmap \
        explore_scan_topic:=${EXPLORE_SCAN_TOPIC} \
        explore_revisit_window:=${EXPLORE_NO_REPEAT_SEC} \
        target_reacquire_block_s:=${EXPLORE_NO_REPEAT_SEC} \
        post_accept_cooldown:=${POST_ACCEPT_COOLDOWN} \
      > /tmp/target_follow.log 2>&1"
  else
    _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
      exec roslaunch target_follower target_follow_real.launch \
        launch_move_base:=true \
        use_online_slam:=true \
        lidar_mode:=${LIDAR_MODE} \
        unitree_port:=${UNITREE_PORT} \
        rplidar_port:=${RPLIDAR_PORT} \
        rplidar_baud:=${RPLIDAR_BAUD} \
        rplidar_pre_start_motor:=${RPLIDAR_PRE_START_MOTOR} \
        rplidar_pre_start_motor_pwm:=${RPLIDAR_PRE_START_PWM} \
        rplidar_pre_start_motor_warmup_s:=${RPLIDAR_PRE_START_WARMUP_S} \
        standoff_distance:=${STANDOFF} \
        face_target:=true \
        target_timeout:=5.0 \
        send_rate_hz:=4.0 \
        min_update_dist:=0.08 \
        enable_close_approach:=true \
        close_approach_threshold:=1.2 \
        tracking_near_target_window_m:=2.2 \
        tracking_near_send_rate_hz:=6.0 \
        tracking_near_min_update_dist:=0.04 \
        tracking_near_goal_to_target:=true \
        udp_port:=${TRASH_UDP_PORT} \
        retreat_distance:=${RETREAT_DIST} \
        retreat_turn_angle_deg:=${RETREAT_TURN_DEG} \
        retreat_reverse_enabled:=${RETREAT_REVERSE_ENABLED} \
        retreat_reverse_distance:=${RETREAT_REVERSE_DIST} \
        action_wait_timeout:=${ACTION_WAIT} \
        enable_auto_explore:=${ENABLE_AUTO_EXPLORE} \
        explore_goal_distance:=${EXPLORE_STEP} \
        explore_short_horizon:=${EXPLORE_STEP} \
        explore_map_topic:=/map \
        explore_costmap_topic:=/move_base/global_costmap/costmap \
        explore_scan_topic:=${EXPLORE_SCAN_TOPIC} \
        slam_map_topic:=/map \
        slam_map_updates_topic:=/map_updates \
        slam_map_metadata_topic:=/map_metadata \
        slam_map_frame:=map \
        explore_revisit_window:=${EXPLORE_NO_REPEAT_SEC} \
        target_reacquire_block_s:=${EXPLORE_NO_REPEAT_SEC} \
        post_accept_cooldown:=${POST_ACCEPT_COOLDOWN} \
      > /tmp/target_follow.log 2>&1"
  fi

  if ${SENSOR_ONLY}; then
    info "Waiting for lidar-only readiness (scan topics must publish live data)..."
    LIDAR_READY=0
    LAST_LIDAR_BLOCKERS=""
    for i in $(seq 1 35); do
      sleep 1
      if _lidar_stack_ready; then
        LIDAR_READY=1
        ok "LiDAR stack is ready (${i}s)"
        break
      fi
      LIDAR_BLOCKERS="$(_lidar_stack_blockers 2>/dev/null || true)"
      if [[ -n "${LIDAR_BLOCKERS}" ]]; then
        LIDAR_BLOCKERS_INLINE="$(printf '%s' "${LIDAR_BLOCKERS}" | tr '\n' ';' | sed 's/;$/ /; s/;/; /g')"
        if [[ "${LIDAR_BLOCKERS_INLINE}" != "${LAST_LIDAR_BLOCKERS}" || $((i % 5)) -eq 0 ]]; then
          warn "STEP 4 still waiting on: ${LIDAR_BLOCKERS_INLINE}"
          LAST_LIDAR_BLOCKERS="${LIDAR_BLOCKERS_INLINE}"
        fi
      fi
    done
    if [[ "${LIDAR_READY}" -ne 1 ]]; then
      _dump_lidar_bringup_debug
      die "LiDAR-only bringup failed. Check: docker exec ${DOCKER_NAME} tail -80 /tmp/lidar_bringup.log"
    fi
  else
    if [[ "${NAV_READINESS_MODE}" == "strict" ]]; then
      info "Waiting for navigation stack readiness (move_base + live target_follower status; online_slam 还需 /map + TF map->base_link)..."
    else
      info "Waiting for navigation control readiness (move_base + target_follower nodes; online_slam 还需 /map + TF map->base_link)..."
    fi
    NAV_READY=0
    LAST_NAV_BLOCKERS=""
    ODOM_STALE_WARNED=0
    for i in $(seq 1 45); do
      sleep 1
      if _nav_stack_ready; then
        NAV_READY=1
        if [[ "${ODOM_STALE_WARNED}" -ne 1 ]] && _ros_odom_stale_but_alive; then
          warn "STEP 4 proceeding with active /odom stream even though odom timestamps are stale"
          ODOM_STALE_WARNED=1
        fi
        if ! _ros_odom_alive; then
          warn "STEP 4 proceeding without /odom stream (base not started or unreachable)"
        fi
        if ! _target_follower_ready; then
          warn "STEP 4 proceeding before /target_follower/status became live; target_follower is still finishing startup"
        fi
        ok "Navigation stack is ready (${i}s)"
        break
      fi
      NAV_BLOCKERS="$(_nav_stack_blockers 2>/dev/null || true)"
      if [[ -n "${NAV_BLOCKERS}" ]]; then
        NAV_BLOCKERS_INLINE="$(printf '%s' "${NAV_BLOCKERS}" | tr '\n' ';' | sed 's/;$/ /; s/;/; /g')"
        if [[ "${NAV_BLOCKERS_INLINE}" != "${LAST_NAV_BLOCKERS}" || $((i % 5)) -eq 0 ]]; then
          warn "STEP 4 still waiting on: ${NAV_BLOCKERS_INLINE}"
          LAST_NAV_BLOCKERS="${NAV_BLOCKERS_INLINE}"
        fi
      fi
      if [[ $i -eq 45 ]]; then
        _dump_nav_readiness_debug
        warn "Navigation stack readiness timed out after 45 s. Continuing anyway."
      fi
    done
  fi

  info "Checking LiDAR data..."
  if [[ "${LIDAR_MODE}" == "dual" || "${LIDAR_MODE}" == "unitree" ]]; then
    SCAN_RATE=$(${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
      timeout 6 rostopic hz /unitree/scan 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}'" 2>/dev/null || true)
    if [[ -n "${SCAN_RATE}" ]]; then
      ok "/unitree/scan: ${SCAN_RATE} Hz"
    elif ${SENSOR_ONLY}; then
      die "/unitree/scan: no data in lidar-only mode"
    else
      warn "/unitree/scan: no data — Unitree may not be fully up yet (OK to proceed)"
    fi
  fi
  if [[ "${LIDAR_MODE}" == "dual" ]]; then
    RPLIDAR_RATE=$(${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
      timeout 6 rostopic hz /rplidar/scan_filtered 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}'" 2>/dev/null || true)
    if [[ -n "${RPLIDAR_RATE}" ]]; then
      ok "/rplidar/scan_filtered: ${RPLIDAR_RATE} Hz"
    elif ${SENSOR_ONLY}; then
      die "/rplidar/scan_filtered: no data in lidar-only mode"
    else
      warn "/rplidar/scan_filtered: no data — continuing with Unitree-only obstacle sensing if needed"
    fi
  elif [[ "${LIDAR_MODE}" == "rplidar" ]]; then
    RPLIDAR_RATE=$(${DOCKER_EXEC} "${ROS_ENV} && ${ROS_SETUP} && \
      timeout 6 rostopic hz /scan_filtered 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}'" 2>/dev/null || true)
    if [[ -n "${RPLIDAR_RATE}" ]]; then
      ok "/scan_filtered: ${RPLIDAR_RATE} Hz"
    elif ${SENSOR_ONLY}; then
      die "/scan_filtered: no data in lidar-only mode"
    else
      warn "/scan_filtered: no data — RPLIDAR may not be fully up yet (OK to proceed)"
    fi
  fi

  if ! ${SENSOR_ONLY}; then
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

    if ${LAUNCH_DIALOGUE}; then
      info "Validating full dialogue mesh after navigation startup..."
      if ! _dialogue_mesh_healthy 15; then
        die "Dialogue mesh did not become healthy after navigation startup. Check: tail -80 /tmp/dialogue_bridge.log and /tmp/dialogue_runner.log"
      fi
      ok "Dialogue mesh is healthy"
    fi

    if ${WORK_MAPPING}; then
      step "STEP 4.5 — Start map persistence"

      MAP_MANAGER_BASE_ARG="_base_map_yaml:="
      MAP_MANAGER_BASE_DESC="<none>"
      if [[ -n "${ASSIST_MAP_FILE}" ]]; then
        if [[ -f "${ASSIST_MAP_FILE}" ]]; then
          MAP_MANAGER_BASE_ARG="_base_map_yaml:=${ASSIST_MAP_FILE}"
          MAP_MANAGER_BASE_DESC="${ASSIST_MAP_FILE}"
        else
          warn "Assist map for map persistence is missing: ${ASSIST_MAP_FILE}"
          warn "Starting map persistence without base-map merge so current navigation stack can keep running."
        fi
      fi

      if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
        info "Using online SLAM map (/map) as navigation + exploration authority."
        ${DOCKER_EXEC} "pkill -f '[c]ontinuous_map_manager.py' 2>/dev/null; sleep 1" 2>/dev/null || true
        info "Starting continuous map manager on /map (save every ${MAP_SAVE_INTERVAL}s)..."
        if [[ "${MAP_MANAGER_BASE_DESC}" != "<none>" ]]; then
          info "Map persistence base map: ${MAP_MANAGER_BASE_DESC}"
        fi
        _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
          exec rosrun p3at_lms_navigation continuous_map_manager.py \
            _map_topic:=/map \
            _save_interval_s:=${MAP_SAVE_INTERVAL} \
            _output_map_prefix:=${MAP_SAVE_PREFIX} \
            _base_frame:=map \
            ${MAP_MANAGER_BASE_ARG} \
          > /tmp/map_manager.log 2>&1"

        ONLINE_SLAM_READY=0
        MAP_MANAGER_READY=0
        for i in $(seq 1 20); do
          _ros_node_exists "slam_gmapping_online" && ONLINE_SLAM_READY=1
          _ros_node_exists "continuous_map_manager" && MAP_MANAGER_READY=1
          if [[ "${ONLINE_SLAM_READY}" -eq 1 && "${MAP_MANAGER_READY}" -eq 1 ]]; then
            break
          fi
          sleep 1
        done
        if [[ "${ONLINE_SLAM_READY}" -ne 1 ]]; then
          die "Online SLAM stack failed to start. Check: tail -80 /tmp/target_follow.log"
        fi
        if [[ "${MAP_MANAGER_READY}" -ne 1 ]]; then
          warn "Map persistence did not start, but online SLAM/navigation is alive. Continuing with live /map only."
          warn "Check: tail -80 /tmp/map_manager.log"
        else
          MAP_PERSISTENCE_RUNNING=true
          ok "Online SLAM + map persistence is running"
        fi
      else
        MAP_LEARN_SCAN_TOPIC="/unitree/scan"
        if [[ "${LIDAR_MODE}" == "rplidar" ]]; then
          MAP_LEARN_SCAN_TOPIC="/scan_filtered"
        fi

        info "Starting passive SLAM on ${MAP_LEARN_SCAN_TOPIC}..."
        ${DOCKER_EXEC} "pkill -f 'roslaunch.*passive_mapping_unitree' 2>/dev/null; \
                        pkill -f '[c]ontinuous_map_manager.py' 2>/dev/null; sleep 1" 2>/dev/null || true

        _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
          exec roslaunch p3at_lms_navigation passive_mapping_unitree.launch \
            scan_topic:=${MAP_LEARN_SCAN_TOPIC} \
            map_topic:=/work_map \
            map_updates_topic:=/work_map_updates \
            map_metadata_topic:=/work_map_metadata \
            map_frame:=work_map \
            odom_frame:=odom \
            base_frame:=base_link \
          > /tmp/passive_mapping.log 2>&1"

        info "Starting continuous map manager (save every ${MAP_SAVE_INTERVAL}s)..."
        if [[ "${MAP_MANAGER_BASE_DESC}" != "<none>" ]]; then
          info "Map persistence base map: ${MAP_MANAGER_BASE_DESC}"
        fi
        _docker_exec_detached "${ROS_ENV} && ${ROS_SETUP} && \
          exec rosrun p3at_lms_navigation continuous_map_manager.py \
            _map_topic:=/work_map \
            _save_interval_s:=${MAP_SAVE_INTERVAL} \
            _output_map_prefix:=${MAP_SAVE_PREFIX} \
            _base_frame:=map \
            ${MAP_MANAGER_BASE_ARG} \
          > /tmp/map_manager.log 2>&1"

        PASSIVE_MAPPING_READY=0
        MAP_MANAGER_READY=0
        for i in $(seq 1 20); do
          _ros_node_exists "slam_gmapping_task" && PASSIVE_MAPPING_READY=1
          _ros_node_exists "continuous_map_manager" && MAP_MANAGER_READY=1
          if [[ "${PASSIVE_MAPPING_READY}" -eq 1 && "${MAP_MANAGER_READY}" -eq 1 ]]; then
            break
          fi
          sleep 1
        done
        if [[ "${PASSIVE_MAPPING_READY}" -ne 1 ]]; then
          die "Passive mapping stack failed to start. Check: tail -80 /tmp/passive_mapping.log"
        fi
        if [[ "${MAP_MANAGER_READY}" -ne 1 ]]; then
          warn "Map persistence did not start, but passive mapping is alive. Continuing without periodic map saves."
          warn "Check: tail -80 /tmp/map_manager.log"
        else
          MAP_PERSISTENCE_RUNNING=true
          ok "Passive map learning + persistence is running"
        fi
      fi
      if ${MAP_PERSISTENCE_RUNNING}; then
        ok "Map output: ${MAP_SAVE_PREFIX}.yaml/.pgm"
      else
        warn "Map output files will not be refreshed until /tmp/map_manager.log is fixed."
      fi
    else
      warn "Map persistence disabled (--no-work-mapping)."
    fi
  fi
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
if ${SENSOR_ONLY}; then
  if [[ "${LIDAR_MODE}" != "rplidar" ]]; then
    echo -e "    ${CYN}rostopic hz /unitree/scan${NC}                  — Unitree filtered scan rate"
    echo -e "    ${CYN}rostopic echo -n 1 /unilidar/cloud${NC}         — Unitree raw point cloud"
  fi
  if [[ "${LIDAR_MODE}" == "dual" ]]; then
    echo -e "    ${CYN}rostopic hz /rplidar/scan_filtered${NC}         — RPLIDAR filtered scan rate"
  elif [[ "${LIDAR_MODE}" == "rplidar" ]]; then
    echo -e "    ${CYN}rostopic hz /scan_filtered${NC}                 — RPLIDAR filtered scan rate"
  fi
  echo -e "    ${CYN}docker exec ${DOCKER_NAME} tail -f /tmp/lidar_bringup.log${NC} — lidar bringup log"
else
  echo -e "    ${CYN}rostopic echo /target_follower/status${NC}   — IDLE|EXPLORING|TRACKING|REACQUIRE_TARGET|REACHED|WAITING_ACTION|RETREATING|LOST|FAILED"
  echo -e "    ${CYN}rostopic echo /target_follower/result${NC}   — True/False (dialogue trigger)"
  echo -e "    ${CYN}rostopic echo /trash_action${NC}             — True=接受 / False=拒绝 (对话结果)"
  echo -e "    ${CYN}rostopic echo /target_pose${NC}              — current target pose"
  if ${WORK_MAPPING}; then
    if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
      echo -e "    ${CYN}rostopic hz /map${NC}                          — online SLAM map publish rate"
    else
      echo -e "    ${CYN}rostopic hz /work_map${NC}                     — passive mapping publish rate"
    fi
  fi
  echo -e "    ${CYN}tail -f /tmp/handobj.log${NC}                  — detection log"
  echo -e "    ${CYN}tail -f /tmp/dialogue_bridge.log${NC}           — dialogue bridge log"
  echo -e "    ${CYN}tail -f /tmp/dialogue_runner.log${NC}           — dialogue runner log"
  if ${MAP_PERSISTENCE_RUNNING}; then
    if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
      echo -e "    ${CYN}tail -f /tmp/target_follow.log${NC}             — nav + online SLAM log"
    else
      echo -e "    ${CYN}tail -f /tmp/passive_mapping.log${NC}           — passive gmapping log"
    fi
    echo -e "    ${CYN}tail -f /tmp/map_manager.log${NC}               — map merge/save log"
    echo -e "    ${CYN}ls ${MAP_SAVE_PREFIX}.yaml ${MAP_SAVE_PREFIX}.pgm${NC} — latest by-product map"
  elif ${WORK_MAPPING}; then
    if [[ "${NAV_PROFILE}" == "online_slam_task" ]]; then
      echo -e "    ${CYN}tail -f /tmp/target_follow.log${NC}             — nav + online SLAM log"
    else
      echo -e "    ${CYN}tail -f /tmp/passive_mapping.log${NC}           — passive gmapping log"
    fi
    echo -e "    ${CYN}tail -f /tmp/map_manager.log${NC}               — map persistence failure log"
  fi
  echo -e "    ${CYN}tail -f /tmp/demo_dashboard.log${NC}            — dashboard log"
fi
echo ""

if ${SENSOR_ONLY}; then
  while true; do
    sleep 5
  done
fi

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
    REACQUIRE_TARGET) S_COLOR="${CYN}${STATUS}${NC}" ;;
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
