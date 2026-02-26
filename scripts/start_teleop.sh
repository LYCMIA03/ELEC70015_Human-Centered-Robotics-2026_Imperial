#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
# ============================================================================
# start_teleop.sh
# 键盘遥操控制 — 通过 /cmd_vel 驱动 P3-AT 底盘
#
# 默认在笔记本电脑上运行（role=laptop），亦可在 Jetson 上运行（role=jetson）：
#
#   # 在笔记本上运行（需先在 deploy.env 中设置 LAPTOP_IP）：
#   ./scripts/start_teleop.sh
#
#   # 在 Jetson 上直接运行（不需要设置 LAPTOP_IP）：
#   ./scripts/start_teleop.sh jetson
#
# 键盘控制说明（teleop_twist_keyboard）：
#   u  i  o     前左 / 前进 / 前右
#   j  k  l     左转 / 停止 / 右转
#   m  ,  .     后左 / 后退 / 后右
#   q / z       加速 / 减速（线速度）
#   w / x       加速 / 减速（角速度）
#   空格 / k    紧急停止
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 支持通过第一个参数指定 role（默认 laptop）；也可用环境变量覆盖
ROLE="${1:-${TELEOP_ROLE:-laptop}}"
MASTER_HOST="${MASTER_HOST:-jetson}"

if [[ "${ROLE}" != "jetson" && "${ROLE}" != "laptop" && "${ROLE}" != "raspi" ]]; then
  echo "用法: $0 [laptop|jetson]"
  echo "  laptop  — 在笔记本电脑上运行（需在 deploy.env 设置 LAPTOP_IP）"
  echo "  jetson  — 在 Jetson 上本地运行"
  exit 1
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" "${ROLE}" "${MASTER_HOST}"

# 检查 teleop_twist_keyboard 是否已安装
if ! rospack find teleop_twist_keyboard &>/dev/null; then
  echo "[teleop] 未找到 teleop_twist_keyboard，尝试安装..."
  sudo apt-get install -y ros-noetic-teleop-twist-keyboard
fi

echo ""
echo "========================================"
echo " P3-AT 键盘遥操 — 发布至 /cmd_vel"
echo " ROS_MASTER_URI = ${ROS_MASTER_URI}"
echo " ROS_IP         = ${ROS_IP}"
echo "========================================"
echo " 控制键：i=前进  ,=后退  j=左转  l=右转  k/空格=停止"
echo " q/z=调线速  w/x=调角速"
echo "========================================"
echo ""

exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
