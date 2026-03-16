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

rplidar_preflight::print_failure_hints() {
  local log_file="$1"
  local port="$2"
  local baud="$3"

  echo "[rplidar_preflight] ---- diagnostics ----" >&2
  echo "[rplidar_preflight] port=${port} baud=${baud}" >&2
  ls -l "${port}" >&2 || true
  if [[ -d /dev/serial/by-id ]]; then
    ls -l /dev/serial/by-id >&2 || true
  fi
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n "${port}" 2>/dev/null \
      | grep -E '^(ID_VENDOR=|ID_MODEL=|ID_SERIAL=|ID_VENDOR_ID=|ID_MODEL_ID=|DEVPATH=)' >&2 || true
  fi

  if grep -q "RESULT_OPERATION_TIMEOUT" "$log_file"; then
    echo "[rplidar_preflight] HINT: serial opened but no valid RPLIDAR reply." >&2
    echo "[rplidar_preflight] HINT: per Slamtec rplidar_ros/rplidar_sdk docs + A2M12 datasheet, check:" >&2
    echo "  1) Power: A2M12 needs stable 5V (startup current can be high)." >&2
    echo "  2) Motor control: A2/A3 require accessory-board PWM/MOTOCTL path." >&2
    echo "  3) Wiring/adapter: USB-UART only TX/RX/GND is not enough if MOTOCTL is missing." >&2
    echo "  4) Port identity: ensure this is the real RPLIDAR serial endpoint." >&2
  fi
  if grep -q "cannot bind to the specified serial port" "$log_file"; then
    echo "[rplidar_preflight] HINT: serial port cannot be opened (permissions/port busy/wrong path)." >&2
  fi
}

rplidar_preflight::cleanup_probe_node() {
  local probe_pid="${1:-}"

  # Prefer ROS-level shutdown so the node unregisters from master cleanly.
  if rosnode list 2>/dev/null | grep -qx "/rplidar_probe_node"; then
    rosnode kill /rplidar_probe_node >/dev/null 2>&1 || true
  fi

  # Fallback: terminate the whole process group started by setsid.
  if [[ -n "${probe_pid}" ]]; then
    kill -INT -- "-${probe_pid}" >/dev/null 2>&1 || true
    sleep 0.2
    kill -TERM -- "-${probe_pid}" >/dev/null 2>&1 || true
    wait "${probe_pid}" >/dev/null 2>&1 || true
  fi

  # Purge stale registrations (non-interactive).
  printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true
}

rplidar_preflight::probe_scan_activity() {
  local port="$1"
  local topic="$2"
  local timeout_s="$3"
  local baud="$4"
  local pre_start_motor="$5"
  local pre_start_motor_pwm="$6"
  local pre_start_motor_warmup_s="$7"
  local log_file="/tmp/rplidar_preflight_probe.log"

  echo "[rplidar_preflight] probing scan stream on $topic via $port"
  setsid rosrun rplidar_ros rplidarNode \
    __name:=rplidar_probe_node \
    _serial_port:="$port" \
    _serial_baudrate:="$baud" \
    _pre_start_motor:="$pre_start_motor" \
    _pre_start_motor_pwm:="$pre_start_motor_pwm" \
    _pre_start_motor_warmup_s:="$pre_start_motor_warmup_s" \
    _frame_id:=laser \
    _inverted:=false \
    _angle_compensate:=true >"$log_file" 2>&1 &
  local probe_pid=$!

  sleep 1
  if ! kill -0 "$probe_pid" >/dev/null 2>&1; then
    echo "[rplidar_preflight] ERROR: probe driver exited early" >&2
    tail -n 80 "$log_file" >&2 || true
    rplidar_preflight::print_failure_hints "$log_file" "$port" "$baud"
    return 1
  fi

  if ! timeout "$timeout_s" rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    echo "[rplidar_preflight] ERROR: no scan message on $topic within ${timeout_s}s" >&2
    tail -n 80 "$log_file" >&2 || true
    rplidar_preflight::print_failure_hints "$log_file" "$port" "$baud"
    rplidar_preflight::cleanup_probe_node "$probe_pid"
    return 1
  fi

  if ! rostopic info "$topic" 2>/dev/null | grep -q "rplidar_probe_node"; then
    echo "[rplidar_preflight] WARNING: $topic active but probe publisher not detected" >&2
  fi

  rplidar_preflight::cleanup_probe_node "$probe_pid"
  echo "[rplidar_preflight] scan activity check passed"
}

rplidar_preflight::run() {
  local default_port="/dev/rplidar_lidar"
  local port
  if port="$(rplidar_preflight::parse_arg rplidar_port "$@")"; then
    :
  else
    port="$default_port"
  fi

  local topic="/scan"
  local timeout_s="8"
  local baud
  if baud="$(rplidar_preflight::parse_arg rplidar_baud "$@")"; then
    :
  else
    baud="256000"
  fi
  local pre_start_motor
  if pre_start_motor="$(rplidar_preflight::parse_arg rplidar_pre_start_motor "$@")"; then
    :
  else
    pre_start_motor="false"
  fi
  local pre_start_motor_pwm
  if pre_start_motor_pwm="$(rplidar_preflight::parse_arg rplidar_pre_start_motor_pwm "$@")"; then
    :
  else
    pre_start_motor_pwm="600"
  fi
  local pre_start_motor_warmup_s
  if pre_start_motor_warmup_s="$(rplidar_preflight::parse_arg rplidar_pre_start_motor_warmup_s "$@")"; then
    :
  else
    pre_start_motor_warmup_s="0.0"
  fi

  rplidar_preflight::check_ros_master
  rplidar_preflight::check_device "$port"
  rplidar_preflight::probe_scan_activity \
    "$port" "$topic" "$timeout_s" "$baud" \
    "$pre_start_motor" "$pre_start_motor_pwm" "$pre_start_motor_warmup_s"
}
