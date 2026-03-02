#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -uo pipefail

INTERVAL="${INTERVAL:-1}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-0.4}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval) INTERVAL="$2"; shift 2 ;;
    --topic-timeout) TOPIC_TIMEOUT="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage (run inside docker):
  ./scripts/demo_dashboard_docker.sh [--interval 1] [--topic-timeout 0.4]

Show ROS-side dashboard fields:
  /target_follower/status
  /target_follower/result
  /trash_detection/target_point (x,z)
  /target_follower/target_distance
EOF
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

topic_bool() {
  local t="$1"
  timeout "${TOPIC_TIMEOUT}" rostopic echo -n 1 "${t}" 2>/dev/null | awk '/data:/{print $2; exit}' || true
}

topic_str() {
  local t="$1"
  timeout "${TOPIC_TIMEOUT}" rostopic echo -n 1 "${t}" 2>/dev/null | sed -n 's/^data: //p' | tr -d '"' || true
}

topic_xz() {
  local t="$1"
  local v
  v="$(timeout "${TOPIC_TIMEOUT}" rostopic echo -n 1 "${t}" 2>/dev/null | awk '
    /^point:/{p=1;next}
    p && /^  x:/{x=$2}
    p && /^  z:/{z=$2}
    END{
      if (x=="" || z=="") print "?,?";
      else printf "%.3f,%.3f", x, z
    }' 2>/dev/null || true)"
  v="$(printf "%s" "${v}" | head -n 1 | tr -d '\r')"
  [[ -z "${v}" ]] && v="?,?"
  echo "${v}"
}

as01nah() {
  local v="$1"
  case "${v}" in
    True|true|1) echo "1" ;;
    False|false|0) echo "0" ;;
    *) echo "nah" ;;
  esac
}

printf "%-19s | %-8s | %-6s | %-8s | %-8s | %-9s\n" \
  "time" "status" "res" "tgt_x" "tgt_z" "dist(m)"

while true; do
  now="$(date '+%F %T')"
  status="$(topic_str /target_follower/status)"; [[ -z "${status}" ]] && status="nah"
  result_raw="$(topic_bool /target_follower/result)"
  result="$(as01nah "${result_raw}")"
  xz="$(topic_xz /trash_detection/target_point)"
  tgt_x="${xz%,*}"
  tgt_z="${xz#*,}"
  dist="$(topic_bool /target_follower/target_distance)"; [[ -z "${dist}" ]] && dist="nah"

  printf "%-19s | %-8s | %-6s | %-8s | %-8s | %-9s\n" \
    "${now}" "${status}" "${result}" "${tgt_x}" "${tgt_z}" "${dist}"
  sleep "${INTERVAL}"
done
