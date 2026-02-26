#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROUNDS=3
TIMEOUT_SEC=20
DOCKER_NAME="ros_noetic"
JETSON_IP="192.168.50.1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rounds) ROUNDS="$2"; shift 2 ;;
    --timeout) TIMEOUT_SEC="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/dialogue_stability_check.sh [--rounds N] [--timeout SEC]

This script:
  1) starts dialogue runner in SIM mode (no microphone dependency)
  2) starts docker bridges
  3) sends false->true trigger N rounds
  4) checks each round receives /trash_action
EOF
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
  docker start "${DOCKER_NAME}" >/dev/null
fi

docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "pgrep -x rosmaster >/dev/null || (source /opt/ros/noetic/setup.bash && nohup roscore >/tmp/roscore.log 2>&1 &)"

# Clean old processes
PIDS="$(ps -eo pid=,cmd= | awk '/dialogue\/dialogue_udp_runner.py/ && !/awk/ {print $1}')"
if [[ -n "${PIDS}" ]]; then
  echo "${PIDS}" | xargs -r kill
fi
pkill -f '[s]tart_dialogue_docker_bridges.sh' 2>/dev/null || true
docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "pkill -f '[n]avigation_success_udp_bridge.py' 2>/dev/null || true; pkill -f '[u]dp_trash_action_bridge.py' 2>/dev/null || true"

sleep 1

: > /tmp/dialogue_runner.log
: > /tmp/dialogue_bridge.log

# Start SIM runner (fully deterministic, no mic needed)
nohup env PYTHONUNBUFFERED=1 python3 "${REPO_ROOT}/dialogue/dialogue_udp_runner.py" \
  --listen-host 0.0.0.0 \
  --listen-port 16041 \
  --send-host 127.0.0.1 \
  --send-port 16032 \
  --sim \
  --first-user-wav "${REPO_ROOT}/dialogue/voice_data/sim_user_answer_affirmative_a.wav" \
  --second-user-wav "${REPO_ROOT}/dialogue/voice_data/sim_user_answer_affirmative_b.wav" \
  > /tmp/dialogue_runner.log 2>&1 &
RUNNER_PID=$!

nohup "${SCRIPT_DIR}/start_dialogue_docker_bridges.sh" > /tmp/dialogue_bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 2

if ! kill -0 "${RUNNER_PID}" 2>/dev/null; then
  echo "[FAIL] runner failed to start"
  tail -n 80 /tmp/dialogue_runner.log || true
  exit 1
fi

if ! ss -lunp 2>/dev/null | grep -q "0.0.0.0:16041"; then
  echo "[FAIL] runner not listening on 16041"
  exit 1
fi

echo "[INFO] runner pid=${RUNNER_PID}, bridge launcher pid=${BRIDGE_PID}"
echo "[INFO] testing ${ROUNDS} rounds, timeout=${TIMEOUT_SEC}s"

PASS=0
for i in $(seq 1 "${ROUNDS}"); do
  echo "[ROUND ${i}] trigger false->true"

  docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc "
    source /opt/ros/noetic/setup.bash && \
    source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
    export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP} && \
    rostopic pub -1 /target_follower/result std_msgs/Bool 'data: false' >/dev/null 2>&1 && \
    sleep 0.4 && \
    rostopic pub -1 /target_follower/result std_msgs/Bool 'data: true' >/dev/null 2>&1
  "

  RESP="$(
    docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc "
      source /opt/ros/noetic/setup.bash && \
      source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
      export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP} && \
      ( timeout ${TIMEOUT_SEC} rostopic echo -n 1 /trash_action 2>/dev/null | awk '/data:/{print \\$2}' ) || true
    "
  )"

  if [[ "${RESP}" == "True" || "${RESP}" == "False" ]]; then
    echo "[ROUND ${i}] PASS trash_action=${RESP}"
    PASS=$((PASS + 1))
  else
    echo "[ROUND ${i}] FAIL no /trash_action within ${TIMEOUT_SEC}s"
  fi
done

echo ""
echo "[SUMMARY] pass=${PASS}/${ROUNDS}"
echo "[INFO] tail /tmp/dialogue_runner.log"
tail -n 80 /tmp/dialogue_runner.log || true

if [[ "${PASS}" -ne "${ROUNDS}" ]]; then
  exit 2
fi

