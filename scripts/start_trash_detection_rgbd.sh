#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Load deploy.env when present for shared parameters (e.g., TRASH_UDP_PORT).
if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

UDP_HOST="${UDP_HOST:-127.0.0.1}"
UDP_PORT="${UDP_PORT:-${TRASH_UDP_PORT:-16031}}"
UDP_FRAME_ID="${UDP_FRAME_ID:-camera_link}"
UDP_KIND="${UDP_KIND:-person}"
UDP_RATE="${UDP_RATE:-10}"

# Jetson local cusparseLt fallback for PyTorch wheels that depend on libcusparseLt.so.0.
if [[ -d "${HOME}/.local/cusparselt" ]]; then
  export LD_LIBRARY_PATH="${HOME}/.local/cusparselt:${LD_LIBRARY_PATH:-}"
fi

cd "${REPO_ROOT}"
exec python3 trash_detection/predict_15cls_rgbd.py \
  --nearest-person \
  --print-xyz \
  --headless \
  --udp-enable \
  --udp-host "${UDP_HOST}" \
  --udp-port "${UDP_PORT}" \
  --udp-frame-id "${UDP_FRAME_ID}" \
  --udp-kind "${UDP_KIND}" \
  --udp-rate "${UDP_RATE}" \
  "$@"
