#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

DIALOGUE_TRIGGER_HOST="${DIALOGUE_TRIGGER_HOST:-0.0.0.0}"
DIALOGUE_TRIGGER_UDP_PORT="${DIALOGUE_TRIGGER_UDP_PORT:-16041}"
DIALOGUE_RESULT_HOST="${DIALOGUE_RESULT_HOST:-127.0.0.1}"
DIALOGUE_ACTION_UDP_PORT="${DIALOGUE_ACTION_UDP_PORT:-16032}"

cd "${REPO_ROOT}"
exec python3 dialogue/dialogue_udp_runner.py \
  --listen-host "${DIALOGUE_TRIGGER_HOST}" \
  --listen-port "${DIALOGUE_TRIGGER_UDP_PORT}" \
  --send-host "${DIALOGUE_RESULT_HOST}" \
  --send-port "${DIALOGUE_ACTION_UDP_PORT}" \
  "$@"

