#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKER_NAME="${DOCKER_NAME:-ros_noetic}"
JETSON_IP="${JETSON_IP:-192.168.50.1}"
DEVICE="${DEVICE:-24}"
WAIT_SEC="${WAIT_SEC:-90}"

if [[ -f "${SCRIPT_DIR}/deploy.env" ]]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/deploy.env"
  JETSON_IP="${JETSON_IP:-192.168.50.1}"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --ip) JETSON_IP="$2"; shift 2 ;;
    --wait) WAIT_SEC="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/test_dialogue_chain.sh [--device 24] [--ip 192.168.50.1] [--wait 90]

Flow:
  1) start docker + roscore (if needed)
  2) restart dialogue runner + docker bridges
  3) send false->true to /target_follower/result
  4) wait /trash_action response
EOF
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
  docker start "${DOCKER_NAME}" >/dev/null
fi

docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "pgrep -x rosmaster >/dev/null || (source /opt/ros/noetic/setup.bash && nohup roscore >/tmp/roscore.log 2>&1 &)"

echo "[1/4] cleaning old runner/bridges..."
ps -eo pid=,cmd= | awk '/python3 .*dialogue\/dialogue_udp_runner.py|start_dialogue_docker_bridges.sh|navigation_success_udp_bridge.py|udp_trash_action_bridge.py/ && !/awk/ {print $1}' | xargs -r kill
docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "pkill -f 'navigation_success_udp_bridge.py' 2>/dev/null || true; pkill -f 'udp_trash_action_bridge.py' 2>/dev/null || true"
sleep 1

echo "[2/4] starting runner + bridges..."
nohup env PYTHONUNBUFFERED=1 python3 "${REPO_ROOT}/dialogue/dialogue_udp_runner.py" \
  --listen-host 0.0.0.0 \
  --listen-port 16041 \
  --send-host 127.0.0.1 \
  --send-port 16032 \
  --device "${DEVICE}" \
  > /tmp/dialogue_runner.log 2>&1 &
RUNNER_PID=$!

nohup docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP}; \
   source /opt/ros/noetic/setup.bash && \
   source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
   exec python3 ${REPO_ROOT}/catkin_ws/src/target_follower/scripts/navigation_success_udp_bridge.py \
     _in_topic:=/target_follower/result _out_host:=127.0.0.1 _out_port:=16041" \
  > /tmp/dialogue_bridge_nav2udp.log 2>&1 &

nohup docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP}; \
   source /opt/ros/noetic/setup.bash && \
   source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
   exec python3 ${REPO_ROOT}/catkin_ws/src/target_follower/scripts/udp_trash_action_bridge.py \
     _bind_port:=16032 _out_topic:=/trash_action" \
  > /tmp/dialogue_bridge_udp2ros.log 2>&1 &

sleep 2
kill -0 "${RUNNER_PID}" 2>/dev/null || { echo "[FAIL] runner failed"; tail -n 80 /tmp/dialogue_runner.log; exit 1; }
ss -lunp | grep -q '0.0.0.0:16041' || { echo "[FAIL] runner not listening 16041"; exit 1; }

echo "[3/4] sending trigger false->true..."
docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
  "source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
   export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP}; \
   rostopic pub -1 /target_follower/result std_msgs/Bool 'data: false' >/dev/null 2>&1; \
   sleep 0.5; \
   rostopic pub -1 /target_follower/result std_msgs/Bool 'data: true' >/dev/null 2>&1"

echo "[4/4] waiting /trash_action (say yes/no now)..."
RESP="$(
  docker exec --user "$(id -u):$(id -g)" "${DOCKER_NAME}" bash -lc \
    "source /opt/ros/noetic/setup.bash && source ${REPO_ROOT}/catkin_ws/devel/setup.bash && \
     export ROS_MASTER_URI=http://${JETSON_IP}:11311 ROS_IP=${JETSON_IP}; \
     ( timeout ${WAIT_SEC} rostopic echo -n 1 /trash_action 2>/dev/null | awk '/data:/{print \\$2}' ) || true"
)"

echo "trash_action=${RESP:-<none>}"
echo "--- runner tail ---"
tail -n 80 /tmp/dialogue_runner.log || true
