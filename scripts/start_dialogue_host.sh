#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/start_dialogue_udp_runner.sh" "$@"
