#!/usr/bin/env bash
set -euo pipefail

rplidar_preflight::parse_arg() {
  local key="$1"
  shift
  local token
  for token in "$@"; do
    if [[ "$token" == "$key":=* ]]; then
      echo "${token#*=}"
      return 0
    fi
  done
  return 1
}

rplidar_preflight::check_ros_master() {
  if ! timeout 3 rostopic list >/dev/null 2>&1; then
    echo "[rplidar_preflight] ERROR: ROS master unreachable (check start_master.sh)" >&2
    return 1
  fi
}

rplidar_preflight::check_device() {
  local port="$1"
  if [[ ! -e "$port" ]]; then
    echo "[rplidar_preflight] ERROR: device not found: $port" >&2
    return 1
  fi
  if [[ ! -r "$port" || ! -w "$port" ]]; then
    echo "[rplidar_preflight] ERROR: no rw permission on $port" >&2
    ls -l "$port" >&2 || true
    id >&2 || true
    return 1
  fi
}

rplidar_preflight::probe_scan_activity() {
  local port="$1"
  local topic="$2"
  local timeout_s="$3"
  local log_file="/tmp/rplidar_preflight_probe.log"

  echo "[rplidar_preflight] probing scan stream on $topic via $port"
  rosrun rplidar_ros rplidarNode \
    __name:=rplidar_probe_node \
    _serial_port:="$port" \
    _serial_baudrate:=256000 \
    _frame_id:=laser \
    _inverted:=false \
    _angle_compensate:=true >"$log_file" 2>&1 &
  local probe_pid=$!

  sleep 1
  if ! kill -0 "$probe_pid" >/dev/null 2>&1; then
    echo "[rplidar_preflight] ERROR: probe driver exited early" >&2
    tail -n 80 "$log_file" >&2 || true
    return 1
  fi

  if ! timeout "$timeout_s" rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    echo "[rplidar_preflight] ERROR: no scan message on $topic within ${timeout_s}s" >&2
    tail -n 80 "$log_file" >&2 || true
    kill "$probe_pid" >/dev/null 2>&1 || true
    wait "$probe_pid" >/dev/null 2>&1 || true
    return 1
  fi

  if ! rostopic info "$topic" 2>/dev/null | grep -q "rplidar_probe_node"; then
    echo "[rplidar_preflight] WARNING: $topic active but probe publisher not detected" >&2
  fi

  kill "$probe_pid" >/dev/null 2>&1 || true
  wait "$probe_pid" >/dev/null 2>&1 || true
  echo "[rplidar_preflight] scan activity check passed"
}

rplidar_preflight::run() {
  local default_port="/dev/ttyUSB0"
  local port
  if port="$(rplidar_preflight::parse_arg rplidar_port "$@")"; then
    :
  else
    port="$default_port"
  fi

  local topic="/scan"
  local timeout_s="8"

  rplidar_preflight::check_ros_master
  rplidar_preflight::check_device "$port"
  rplidar_preflight::probe_scan_activity "$port" "$topic" "$timeout_s"
}
