#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Local simulation default: laptop as both node host and master.
MASTER_HOST="${MASTER_HOST:-laptop}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" laptop "${MASTER_HOST}"
exec roslaunch p3at_lms_navigation mapping.launch use_gazebo_target:=false "$@"
