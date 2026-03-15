#!/usr/bin/env bash
# ============================================================================
# setup_rplidar_a2.sh — 安装配置 RPLIDAR A2M12 (ROS1 Noetic)
#
# 内容：
#   1) 安装 rplidar_ros 驱动（apt 优先）
#   2) 确保串口权限（dialout）
#   3) 建议使用 /dev/serial/by-id 固定设备路径
#   4) 提供快速验证命令
# ============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
log() { echo -e "${CYAN}[INFO]${NC} $*"; }
ok()  { echo -e "${GREEN}[OK]${NC}   $*"; }
err() { echo -e "${RED}[ERR]${NC}  $*"; }

USE_SOURCE=false
for arg in "$@"; do
  case "$arg" in
    --source) USE_SOURCE=true ;;
  esac
done

log "Installing base dependencies..."
sudo apt-get update
sudo apt-get install -y ros-noetic-rplidar-ros

if $USE_SOURCE; then
  log "--source enabled: cloning rplidar_ros into catkin_ws/src"
  REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
  WS_DIR="${REPO_DIR}/catkin_ws"
  mkdir -p "${WS_DIR}/src"
  if [[ ! -d "${WS_DIR}/src/rplidar_ros" ]]; then
    git clone https://github.com/Slamtec/rplidar_ros.git "${WS_DIR}/src/rplidar_ros"
  else
    log "rplidar_ros already exists in workspace, skipping clone"
  fi
  source /opt/ros/noetic/setup.bash
  cd "${WS_DIR}"
  catkin_make
fi

if ! groups | grep -q dialout; then
  log "Adding ${USER} to dialout group"
  sudo usermod -aG dialout "${USER}"
  log "Please re-login (or run: newgrp dialout) before using the lidar"
fi

log "Serial device hints:"
ls -la /dev/serial/by-id 2>/dev/null || true
ls -la /dev/ttyUSB* 2>/dev/null || true

ok "Setup complete"
echo ""
echo "Quick test:"
echo "  roslaunch p3at_lms_navigation real_robot_mapping_rplidar.launch rplidar_port:=/dev/ttyUSB0"
echo ""
echo "Recommended stable port via by-id (example):"
echo "  rplidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*"
