#!/usr/bin/env bash

# Usage:
#   source scripts/env.sh <role> [master_host]
#
# role: jetson | raspi | laptop
# master_host: jetson | raspi | laptop   (default: jetson)
#
# Examples:
#   source scripts/env.sh jetson jetson
#   source scripts/env.sh raspi jetson
#   source scripts/env.sh laptop laptop

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Use: source scripts/env.sh <role> [master_host]"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROLE="${1:-}"
MASTER_HOST="${2:-${MASTER_HOST:-jetson}}"
CATKIN_WS="${CATKIN_WS:-${REPO_ROOT}/catkin_ws}"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

JETSON_IP="${JETSON_IP:-127.0.0.1}"
RASPI_IP="${RASPI_IP:-127.0.0.1}"
LAPTOP_IP="${LAPTOP_IP:-127.0.0.1}"

if [[ -z "${ROLE}" ]]; then
  echo "[env] Missing role. Use one of: jetson | raspi | laptop"
  return 1
fi

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "[env] /opt/ros/noetic/setup.bash not found. Install ROS Noetic first."
  return 1
fi

if [[ -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${CATKIN_WS}/devel/setup.bash"
else
  echo "[env] Warning: ${CATKIN_WS}/devel/setup.bash not found. Run catkin_make first."
fi

case "${MASTER_HOST}" in
  jetson) MASTER_IP="${JETSON_IP}" ;;
  raspi) MASTER_IP="${RASPI_IP}" ;;
  laptop) MASTER_IP="${LAPTOP_IP}" ;;
  *)
    echo "[env] Invalid master_host: ${MASTER_HOST}. Use jetson | raspi | laptop"
    return 1
    ;;
esac

case "${ROLE}" in
  jetson) SELF_IP="${JETSON_IP}" ;;
  raspi) SELF_IP="${RASPI_IP}" ;;
  laptop) SELF_IP="${LAPTOP_IP}" ;;
  *)
    echo "[env] Invalid role: ${ROLE}. Use jetson | raspi | laptop"
    return 1
    ;;
esac

export ROS_MASTER_URI="http://${MASTER_IP}:11311"
export ROS_IP="${SELF_IP}"
unset ROS_HOSTNAME

echo "[env] role=${ROLE} master_host=${MASTER_HOST}"
echo "[env] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[env] ROS_IP=${ROS_IP}"
