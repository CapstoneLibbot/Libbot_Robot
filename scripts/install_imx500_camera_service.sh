#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Install this service on Raspberry Pi OS, not inside the ROS container." >&2
  exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

if ! command -v systemctl >/dev/null 2>&1 || ! command -v apt-get >/dev/null 2>&1; then
  echo "This installer requires Raspberry Pi OS with systemd and apt." >&2
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
service_user="${SUDO_USER:-}"
if [[ -z "${service_user}" || "${service_user}" == "root" ]]; then
  service_user="$(id -un)"
fi
if [[ "${service_user}" == "root" ]]; then
  echo "Run this installer from your normal Raspberry Pi user account." >&2
  exit 1
fi

apt-get update
apt-get install -y python3-picamera2 python3-pil python3-pyzbar libzbar0 openssl

install -d -m 0755 /etc/libot /usr/local/lib/libot
environment_file="/etc/libot/imx500-camera.env"
if [[ ! -f "${environment_file}" ]]; then
  stream_token="$(openssl rand -hex 32)"
  printf 'LIBOT_CAMERA_STREAM_TOKEN=%s\n' "${stream_token}" > "${environment_file}"
  chmod 0600 "${environment_file}"
  chown root:root "${environment_file}"
  echo "Camera stream token for backend RobotRuntimes__real__CameraStreamToken: ${stream_token}"
fi

launcher="/usr/local/lib/libot/start-imx500-camera.sh"
{
  printf '#!/usr/bin/env bash\nset -euo pipefail\n'
  printf 'repo_root=%q\n' "${repo_root}"
  printf 'export PYTHONPATH="${repo_root}/robot_app${PYTHONPATH:+:${PYTHONPATH}}"\n'
  printf 'cd "${repo_root}"\n'
  printf 'exec /usr/bin/python3 "${repo_root}/robot_app/hardware/imx500_scanner_server.py" --host 127.0.0.1 --port 9011 --stream-host 0.0.0.0 --stream-port 9012\n'
} > "${launcher}"
chmod 0755 "${launcher}"
chown root:root "${launcher}"

unit_file="/etc/systemd/system/libot-imx500-camera.service"
cat > "${unit_file}" <<EOF
[Unit]
Description=LiBot IMX500 QR scanner and camera stream
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${service_user}
SupplementaryGroups=video
EnvironmentFile=-${environment_file}
ExecStart=${launcher}
Restart=on-failure
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 "${unit_file}"
chown root:root "${unit_file}"

systemctl daemon-reload
systemctl enable --now libot-imx500-camera.service
echo "IMX500 scanner is enabled at boot. SCAN steps start camera capture automatically."
