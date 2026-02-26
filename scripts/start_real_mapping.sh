#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
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
  exec roslaunch p3at_lms_navigation real_robot_mapping.launch "$@"
else
  exec roslaunch p3at_lms_navigation real_robot_mapping.launch udp_bind_port:="${TRASH_UDP_PORT}" "$@"
fi
