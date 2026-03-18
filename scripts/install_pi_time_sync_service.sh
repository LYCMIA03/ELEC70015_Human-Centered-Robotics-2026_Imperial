#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="p3at-time-sync.service"
INSTALL_PATH="/usr/local/sbin/p3at-sync-time-from-master"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

run_as_root install -m 0755 "${SCRIPT_DIR}/sync_time_from_master.sh" "${INSTALL_PATH}"

tmp_service="$(mktemp)"
cat > "${tmp_service}" <<EOF
[Unit]
Description=Sync Raspberry Pi system time from Jetson ROS master
Wants=network-online.target
After=network-online.target

[Service]
Type=oneshot
ExecStart=${INSTALL_PATH} --quiet --retries 20 --retry-delay-s 3
Restart=on-failure
RestartSec=15

[Install]
WantedBy=multi-user.target
EOF

run_as_root install -m 0644 "${tmp_service}" "${SERVICE_PATH}"
rm -f "${tmp_service}"

run_as_root systemctl daemon-reload
run_as_root systemctl enable "${SERVICE_NAME}"
run_as_root systemctl restart "${SERVICE_NAME}"

echo "[time-sync] Installed ${SERVICE_NAME} and started it."
