#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default real-robot topology: RasPi node connects to Jetson master.
MASTER_HOST="${MASTER_HOST:-jetson}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" raspi "${MASTER_HOST}"
exec roslaunch p3at_base base.launch "$@"
