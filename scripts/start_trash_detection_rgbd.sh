#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
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
DETECTOR="trash"

show_usage() {
  cat <<'EOF'
Usage:
  ./scripts/start_trash_detection_rgbd.sh [--detector trash|handobj] [extra detector args...]

Examples:
  ./scripts/start_trash_detection_rgbd.sh
  ./scripts/start_trash_detection_rgbd.sh --detector handobj
  ./scripts/start_trash_detection_rgbd.sh --detector trash --color-res 720p
EOF
}

PASS_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --detector)
      if [[ $# -lt 2 ]]; then
        echo "Error: --detector requires a value: trash|handobj" >&2
        show_usage
        exit 2
      fi
      DETECTOR="$2"
      shift 2
      ;;
    --detector=*)
      DETECTOR="${1#*=}"
      shift
      ;;
    -h|--help)
      show_usage
      exit 0
      ;;
    *)
      PASS_ARGS+=("$1")
      shift
      ;;
  esac
done

# Jetson local cusparseLt fallback for PyTorch wheels that depend on libcusparseLt.so.0.
if [[ -d "${HOME}/.local/cusparselt" ]]; then
  export LD_LIBRARY_PATH="${HOME}/.local/cusparselt:${LD_LIBRARY_PATH:-}"
fi

cd "${REPO_ROOT}"
case "${DETECTOR}" in
  trash)
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
      "${PASS_ARGS[@]}"
    ;;
  handobj)
    exec python3 handobj_detection/handobj_detection_rgbd.py \
      --print-xyz \
      --headless \
      --udp-enable \
      --udp-host "${UDP_HOST}" \
      --udp-port "${UDP_PORT}" \
      --udp-frame-id "${UDP_FRAME_ID}" \
      --udp-rate "${UDP_RATE}" \
      "${PASS_ARGS[@]}"
    ;;
  *)
    echo "Error: unsupported detector '${DETECTOR}'. Use trash|handobj." >&2
    show_usage
    exit 2
    ;;
esac
