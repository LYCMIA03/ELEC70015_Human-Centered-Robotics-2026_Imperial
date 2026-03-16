#!/usr/bin/env bash
# ============================================================================
# setup_unitree_lidar.sh — 安装配置 Unitree 4D Lidar L1 ROS 驱动
#
# 此脚本帮助在 Jetson Orin Nano (或任何 Ubuntu 20.04 + ROS Noetic 机器) 上:
#   1. 克隆 unilidar_sdk 仓库
#   2. 编译 unitree_lidar_ros ROS 包
#   3. 设置 USB 串口权限
#   4. 验证驱动安装
#
# 用法:
#   ./setup_unitree_lidar.sh
#   ./setup_unitree_lidar.sh --skip-clone   # 如果已克隆过
# ============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
log() { echo -e "${CYAN}[INFO]${NC} $*"; }
warn() { echo -e "${RED}[WARN]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="${SCRIPT_DIR}/catkin_ws"
SKIP_CLONE=false

for arg in "$@"; do
  case "$arg" in
    --skip-clone) SKIP_CLONE=true ;;
  esac
done

_extract_serial_token() {
  local byid_name="$1"
  sed -E 's/.*_([^_]+)-if[0-9]+-port[0-9]+$/\1/' <<<"${byid_name}"
}

_detect_unitree_serial() {
  local candidates=()
  if [[ -d /dev/serial/by-id ]]; then
    mapfile -t candidates < <(ls -1 /dev/serial/by-id 2>/dev/null | grep -E 'CP2104_USB_to_UART_Bridge' || true)
  fi
  if [[ "${#candidates[@]}" -eq 1 ]]; then
    _extract_serial_token "${candidates[0]}"
    return 0
  fi
  return 1
}

# ===== Step 1: Clone unilidar_sdk =====
SDK_DIR="${WS_DIR}/src/unilidar_sdk"
if $SKIP_CLONE || [[ -d "$SDK_DIR" ]]; then
  log "unilidar_sdk already exists, skipping clone."
else
  log "Cloning unilidar_sdk..."
  cd "${WS_DIR}/src"
  git clone https://github.com/unitreerobotics/unilidar_sdk.git
fi

# ===== Step 2: Create symlink for ROS package =====
ROS_PKG_SRC="${SDK_DIR}/unitree_lidar_ros"
ROS_PKG_LINK="${WS_DIR}/src/unitree_lidar_ros"

if [[ -d "$ROS_PKG_SRC" ]]; then
  if [[ ! -L "$ROS_PKG_LINK" && ! -d "$ROS_PKG_LINK" ]]; then
    log "Creating symlink: unitree_lidar_ros → unilidar_sdk/unitree_lidar_ros"
    ln -sf "$ROS_PKG_SRC" "$ROS_PKG_LINK"
  else
    log "unitree_lidar_ros package already linked."
  fi
else
  log "Warning: ${ROS_PKG_SRC} not found."
  log "The SDK structure may differ. Please check: ls ${SDK_DIR}/"
  log "You may need to manually copy/link the ROS package to ${WS_DIR}/src/"
fi

# ===== Step 3: Install dependencies =====
log "Installing ROS dependencies..."
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.zsh
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# ===== Step 4: Build =====
log "Building catkin workspace (including unitree_lidar_ros)..."
catkin_make || { echo -e "${RED}Build failed${NC}"; exit 1; }
source devel/setup.bash 2>/dev/null || source devel/setup.zsh

# ===== Step 5: USB Serial Permissions =====
log "Setting up USB serial port permissions..."
if ! groups | grep -q dialout; then
  sudo usermod -aG dialout "$USER"
  log "Added ${USER} to 'dialout' group. Please log out and back in for this to take effect."
fi

# Create udev rule for Unitree Lidar L1
UDEV_RULE="/etc/udev/rules.d/99-unitree-lidar.rules"
if [[ ! -f "$UDEV_RULE" ]]; then
  if UNITREE_SERIAL="$(_detect_unitree_serial)"; then
    log "Creating serial-pinned udev rule for Unitree Lidar (serial=${UNITREE_SERIAL})..."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"${UNITREE_SERIAL}\", MODE:=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", SYMLINK+=\"unitree_lidar\"" | sudo tee "$UDEV_RULE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    log "Udev rule created. Unitree Lidar will appear as /dev/unitree_lidar"
  else
    warn "Cannot uniquely detect Unitree CP2104 serial from /dev/serial/by-id."
    warn "Skipped writing ${UDEV_RULE} to avoid ambiguous VID/PID-only mapping."
    warn "Please reconnect only Unitree and rerun this script, or set unitree_port:=/dev/serial/by-id/..."
  fi
else
  log "Udev rule already exists."
fi

# ===== Step 6: Verify =====
log "Verifying installation..."
if rospack find unitree_lidar_ros 2>/dev/null; then
  echo -e "${GREEN}✓ unitree_lidar_ros package found at: $(rospack find unitree_lidar_ros)${NC}"
else
  echo -e "${RED}✗ unitree_lidar_ros package not found. Check the SDK structure.${NC}"
  echo "  Manual steps:"
  echo "  1. ls ${SDK_DIR}/"
  echo "  2. Find the ROS1 package directory"
  echo "  3. ln -sf <path> ${WS_DIR}/src/unitree_lidar_ros"
  echo "  4. catkin_make"
fi

echo ""
log "Setup complete!"
echo ""
echo "=== Quick Start Guide ==="
echo ""
echo "1. Connect Unitree Lidar L1 via USB"
echo "2. Check device: ls -la /dev/ttyUSB* /dev/unitree_lidar"
echo "3. Test driver:"
echo "   roslaunch p3at_lms_navigation real_robot_mapping_unitree.launch unitree_port:=/dev/unitree_lidar"
echo ""
echo "4. Sim test (no hardware needed):"
echo "   roslaunch p3at_lms_navigation mapping_unitree.launch"
echo ""
echo "5. Full pipeline:"
echo "   ./run_full_pipeline_unitree.sh"
echo ""
