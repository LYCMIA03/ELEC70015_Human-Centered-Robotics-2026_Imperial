#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
fi

MASTER_HOST="${MASTER_HOST:-jetson}"
MASTER_URI="${ROS_MASTER_URI:-}"
MAX_SKEW_S="${MAX_SKEW_S:-5}"
RETRIES="${RETRIES:-10}"
RETRY_DELAY_S="${RETRY_DELAY_S:-3}"
QUIET=0

usage() {
  cat <<'EOF'
Usage: sync_time_from_master.sh [options]

Sync the local system clock from the HTTP Date header exposed by the ROS master.
This is mainly intended for the Raspberry Pi, which may boot without RTC/NTP.

Options:
  --master-uri URI        Use this ROS master URI directly.
  --master-host NAME      Resolve host from deploy.env (jetson|raspi|laptop).
  --max-skew-s N          Skip updates when skew is <= N seconds (default: 5).
  --retries N             Number of fetch attempts before failing (default: 10).
  --retry-delay-s SEC     Delay between retries in seconds (default: 3).
  --quiet                 Reduce log noise.
  -h, --help              Show this help.
EOF
}

log() {
  if [[ "${QUIET}" -eq 0 ]]; then
    echo "[time-sync] $*"
  fi
}

warn() {
  echo "[time-sync] Warning: $*" >&2
}

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

resolve_master_uri() {
  if [[ -n "${MASTER_URI}" ]]; then
    printf '%s\n' "${MASTER_URI}"
    return
  fi

  case "${MASTER_HOST}" in
    jetson)
      printf 'http://%s:11311\n' "${JETSON_IP:-192.168.50.1}"
      ;;
    raspi)
      printf 'http://%s:11311\n' "${RASPI_IP:-192.168.50.2}"
      ;;
    laptop)
      printf 'http://%s:11311\n' "${LAPTOP_IP:-127.0.0.1}"
      ;;
    *)
      warn "Unsupported master host '${MASTER_HOST}'"
      return 1
      ;;
  esac
}

fetch_master_date() {
  local uri="$1"
  python3 - "$uri" <<'PY'
import http.client
import sys
import urllib.parse

uri = sys.argv[1]
parsed = urllib.parse.urlparse(uri)
host = parsed.hostname
port = parsed.port or (443 if parsed.scheme == "https" else 80)
path = parsed.path or "/"

conn = http.client.HTTPConnection(host, port, timeout=3)
conn.request("HEAD", path)
resp = conn.getresponse()
date_header = resp.getheader("Date")
if not date_header:
    raise SystemExit(2)
print(date_header)
PY
}

abs_value() {
  local value="$1"
  if (( value < 0 )); then
    echo $(( -value ))
  else
    echo "${value}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --master-uri)
      MASTER_URI="${2:?Missing value for --master-uri}"
      shift 2
      ;;
    --master-host)
      MASTER_HOST="${2:?Missing value for --master-host}"
      shift 2
      ;;
    --max-skew-s)
      MAX_SKEW_S="${2:?Missing value for --max-skew-s}"
      shift 2
      ;;
    --retries)
      RETRIES="${2:?Missing value for --retries}"
      shift 2
      ;;
    --retry-delay-s)
      RETRY_DELAY_S="${2:?Missing value for --retry-delay-s}"
      shift 2
      ;;
    --quiet)
      QUIET=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      warn "Unknown option: $1"
      usage
      exit 2
      ;;
  esac
done

MASTER_URI="$(resolve_master_uri)"
DATE_HEADER=""

for attempt in $(seq 1 "${RETRIES}"); do
  if DATE_HEADER="$(fetch_master_date "${MASTER_URI}" 2>/dev/null)"; then
    break
  fi

  if (( attempt < RETRIES )); then
    log "Master ${MASTER_URI} not ready yet (attempt ${attempt}/${RETRIES}); retrying in ${RETRY_DELAY_S}s"
    sleep "${RETRY_DELAY_S}"
  fi
done

if [[ -z "${DATE_HEADER}" ]]; then
  warn "Could not fetch Date header from ${MASTER_URI}"
  exit 1
fi

MASTER_EPOCH="$(date -u -d "${DATE_HEADER}" +%s)"
LOCAL_EPOCH="$(date -u +%s)"
SKEW_S=$(( MASTER_EPOCH - LOCAL_EPOCH ))
ABS_SKEW_S="$(abs_value "${SKEW_S}")"

if (( ABS_SKEW_S <= MAX_SKEW_S )); then
  log "Clock skew ${ABS_SKEW_S}s is within threshold; no sync needed."
  exit 0
fi

TARGET_UTC="$(date -u -d "@${MASTER_EPOCH}" '+%Y-%m-%d %H:%M:%S')"
log "Syncing system time from ${MASTER_URI} (skew ${SKEW_S}s -> ${TARGET_UTC} UTC)"
run_as_root date -u -s "${TARGET_UTC}" >/dev/null

UPDATED_EPOCH="$(date -u +%s)"
UPDATED_SKEW_S=$(( MASTER_EPOCH - UPDATED_EPOCH ))
UPDATED_ABS_SKEW_S="$(abs_value "${UPDATED_SKEW_S}")"

if (( UPDATED_ABS_SKEW_S > MAX_SKEW_S )); then
  warn "Clock still differs from master by ${UPDATED_SKEW_S}s after sync"
  exit 1
fi

log "Clock synchronized successfully."
