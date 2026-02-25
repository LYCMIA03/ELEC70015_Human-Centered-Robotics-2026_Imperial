#!/usr/bin/env bash
# ============================================================================
# start_target_follow.sh
# 真机目标追踪 — Target Following Overlay
#
# 运行机器（Jetson Orin Nano — Docker 容器内）：
#   ./scripts/start_target_follow.sh
#
# 可选参数（透传给 roslaunch）：
#   standoff_distance:=0.5       在距目标多远处停下 (m), 默认 0.5
#   face_target:=true            到达后是否朝向目标, 默认 true
#   target_timeout:=5.0          目标消失多久后取消导航 (s), 默认 5.0
#   udp_port:=16031              UDP 端口, 默认 16031
#   camera_optical_frame:=true   相机坐标系是否为光学坐标系, 默认 true
#
# 前提：
#   - 导航栈已启动 (real_robot_nav_unitree.launch 或 real_robot_mapping_unitree.launch)
#   - YOLO 检测已在 host 上运行 (predict_15cls_rgbd.py --udp-enable)
#
# 结果 topic：
#   /target_follower/result   (std_msgs/Bool)   True=到达, False=失败
#   /target_follower/status   (std_msgs/String)  IDLE|TRACKING|REACHED|LOST|FAILED
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"

exec roslaunch target_follower target_follow_real.launch "$@"
