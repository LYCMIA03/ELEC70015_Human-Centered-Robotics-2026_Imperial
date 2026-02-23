#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" jetson "${MASTER_HOST}"

UDP_PORT_ARG_SET=0
for arg in "$@"; do
  if [[ "${arg}" == udp_bind_port:=* ]]; then
    UDP_PORT_ARG_SET=1
    break
  fi
done

if [[ "${UDP_PORT_ARG_SET}" -eq 1 ]]; then
  exec roslaunch p3at_lms_navigation real_robot_nav.launch "$@"
else
  exec roslaunch p3at_lms_navigation real_robot_nav.launch udp_bind_port:="${TRASH_UDP_PORT}" "$@"
fi
