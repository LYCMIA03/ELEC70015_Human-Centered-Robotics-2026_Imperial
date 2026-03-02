#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROLE="${1:-laptop}"
if [[ "${ROLE}" != "jetson" && "${ROLE}" != "laptop" ]]; then
  echo "Usage: $0 [jetson|laptop]"
  exit 1
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh" "${ROLE}" "${ROLE}"
exec roscore
