#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_SYNC_SCRIPT="${SCRIPT_DIR}/sync_time_from_master.sh"

# Default real-robot topology: RasPi node connects to Jetson master.
MASTER_HOST="${MASTER_HOST:-jetson}"

if [[ "${SKIP_TIME_SYNC:-0}" != "1" && -x "${TIME_SYNC_SCRIPT}" ]]; then
  # Keep Pi time close to the Jetson master before RosAria starts publishing odom/TF.
  "${TIME_SYNC_SCRIPT}" --master-host "${MASTER_HOST}" --retries "${TIME_SYNC_RETRIES:-10}" --retry-delay-s "${TIME_SYNC_RETRY_DELAY_S:-3}"
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" raspi "${MASTER_HOST}"
exec roslaunch p3at_base base.launch "$@"
