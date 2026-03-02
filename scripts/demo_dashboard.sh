#!/usr/bin/env bash

__SINGLETON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/single_instance.sh
source "${__SINGLETON_DIR}/lib/single_instance.sh"
single_instance::activate "$(basename "$0")"
set -uo pipefail

INTERVAL="${INTERVAL:-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval) INTERVAL="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage (run on host):
  ./scripts/demo_dashboard.sh [--interval 1]

Host-side dashboard fields:
  dialogue_phase  (from /tmp/dialogue_runner.log)
  runner          (dialogue runner process)
  n2u             (navigation_success_udp_bridge process)
  u2r             (udp_trash_action_bridge process)
EOF
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

dialogue_phase() {
  if ! pgrep -af "dialogue/dialogue_udp_runner.py" >/dev/null 2>&1; then
    echo "down"
    return
  fi
  if [[ ! -f /tmp/dialogue_runner.log ]]; then
    echo "idle"
    return
  fi
  tail -n 200 /tmp/dialogue_runner.log 2>/dev/null | awk '
    /\[Trigger\]/{phase="triggered"}
    /Listening for one utterance/{phase="waiting_reply"}
    /\[Round [12]\]/{phase="processing"}
    /\[Result\]/{phase="idle"}
    END{
      if (phase=="") phase="idle";
      print phase
    }'
}

proc_up() {
  local pat="$1"
  pgrep -af "$pat" >/dev/null 2>&1 && echo "up" || echo "down"
}

printf "%-19s | %-13s | %-6s | %-6s | %-6s\n" \
  "time" "dialogue_phase" "runner" "n2u" "u2r"

while true; do
  now="$(date '+%F %T')"
  dphase="$(dialogue_phase)"
  runner="$(proc_up "dialogue/dialogue_udp_runner.py")"
  n2u="$(proc_up "navigation_success_udp_bridge.py")"
  u2r="$(proc_up "udp_trash_action_bridge.py")"

  printf "%-19s | %-13s | %-6s | %-6s | %-6s\n" \
    "${now}" "${dphase}" "${runner}" "${n2u}" "${u2r}"
  sleep "${INTERVAL}"
done

