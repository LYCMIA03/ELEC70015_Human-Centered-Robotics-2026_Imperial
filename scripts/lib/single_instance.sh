#!/usr/bin/env bash
# Shared singleton guard for scripts under ./scripts.
# Behavior: starting a new instance will terminate an old running instance
# of the same script key, then continue with the new one.

single_instance::_proc_start_time() {
  local pid="$1"
  # /proc/<pid>/stat field 22 is process start time in clock ticks since boot.
  # shellcheck disable=SC2016
  awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true
}

single_instance::activate() {
  local key="${1:-$(basename "$0")}"
  local user_name="${USER:-$(id -un)}"
  local runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
  local state_dir="${runtime_dir}/hcr_singleton_${user_name}"
  local pid_file="${state_dir}/${key}.pid"

  mkdir -p "${state_dir}"

  if [[ -f "${pid_file}" ]]; then
    local old_pid old_start current_start
    old_pid="$(awk 'NR==1{print $1}' "${pid_file}" 2>/dev/null || true)"
    old_start="$(awk 'NR==2{print $1}' "${pid_file}" 2>/dev/null || true)"
    if [[ "${old_pid}" =~ ^[0-9]+$ ]] && [[ "${old_pid}" != "$$" ]]; then
      current_start="$(single_instance::_proc_start_time "${old_pid}")"
      if [[ -n "${old_start}" ]] && [[ -n "${current_start}" ]] && [[ "${current_start}" == "${old_start}" ]]; then
        echo "[singleton] ${key}: stopping previous instance pid=${old_pid}" >&2
        kill "${old_pid}" 2>/dev/null || true
        local i
        for i in $(seq 1 20); do
          current_start="$(single_instance::_proc_start_time "${old_pid}")"
          if [[ "${current_start}" != "${old_start}" ]]; then
            break
          fi
          sleep 0.1
        done
        current_start="$(single_instance::_proc_start_time "${old_pid}")"
        if [[ "${current_start}" == "${old_start}" ]]; then
          echo "[singleton] ${key}: force stopping previous instance pid=${old_pid}" >&2
          kill -9 "${old_pid}" 2>/dev/null || true
        fi
      fi
    fi
  fi

  {
    echo "$$"
    single_instance::_proc_start_time "$$"
  } > "${pid_file}"

  trap '
    if [[ -f "'"${pid_file}"'" ]] && [[ "$(awk '"'"'NR==1{print $1}'"'"' "'"${pid_file}"'" 2>/dev/null || true)" == "$$" ]]; then
      rm -f "'"${pid_file}"'"
    fi
  ' EXIT
}
