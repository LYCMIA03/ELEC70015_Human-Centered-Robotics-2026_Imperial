#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Local simulation default: laptop as both node host and master.
MASTER_HOST="${MASTER_HOST:-laptop}"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" laptop "${MASTER_HOST}"
exec roslaunch p3at_lms_navigation nav.launch "$@"
