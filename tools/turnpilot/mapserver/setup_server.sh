#!/usr/bin/env bash
# One-time setup of the map server VM (Debian). Run on the VM as a sudoer; deploy.sh copies it there.
#
#   setup_server.sh <public-ip>
#
# Installs pyvalhalla in /opt/turnpilot-maps/venv, makes the service user, a 1 GB swap file, a random API key
# and a self-signed certificate for the IP, and the systemd unit. Safe to run again: it keeps an existing key
# and certificate, so the comma's copies stay valid.
set -euo pipefail

IP="${1:?usage: $0 <public-ip>}"
PYVALHALLA_VERSION="${PYVALHALLA_VERSION:-3.9.1}"
ETC=/etc/turnpilot-maps
OPT=/opt/turnpilot-maps
SRV=/srv/turnpilot-maps

sudo apt-get update -q
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q python3-venv rsync openssl

id -u tpmaps >/dev/null 2>&1 || sudo useradd --system --home "$SRV" --shell /usr/sbin/nologin tpmaps
sudo mkdir -p "$ETC" "$OPT" "$SRV/regions" "$SRV/staging"
sudo chown "$USER" "$OPT" "$SRV/staging"

if [ ! -f /swapfile ]; then
  sudo fallocate -l 1G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
fi
sudo swapon /swapfile 2>/dev/null || true

[ -x "$OPT/venv/bin/python" ] || python3 -m venv "$OPT/venv"
"$OPT/venv/bin/pip" install -q "pyvalhalla==$PYVALHALLA_VERSION"

if [ ! -f "$ETC/key" ]; then
  openssl rand -hex 32 | sudo tee "$ETC/key" >/dev/null
fi
if [ ! -f "$ETC/cert.pem" ]; then
  sudo openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes -days 3650 \
    -subj "/CN=turnpilot-maps" -addext "subjectAltName=IP:$IP" \
    -keyout "$ETC/cert.key" -out "$ETC/cert.pem"
fi
sudo chown root:tpmaps "$ETC/key" "$ETC/cert.key" "$ETC/cert.pem"
sudo chmod 640 "$ETC/key" "$ETC/cert.key"
sudo chmod 644 "$ETC/cert.pem"

sudo tee /etc/systemd/system/turnpilot-maps.service >/dev/null <<EOF
[Unit]
Description=TurnPilot map server
After=network-online.target
Wants=network-online.target

[Service]
User=tpmaps
ExecStart=$OPT/venv/bin/python $OPT/server.py --regions $SRV/regions --tmp-dir /var/cache/turnpilot-maps --log-dir /var/lib/turnpilot-maps-logs
WorkingDirectory=$OPT
Restart=on-failure
RestartSec=5
AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
# corridors are built here before they are sent, on disk: /tmp may be a tmpfs that counts against MemoryMax
CacheDirectory=turnpilot-maps
# the cars' drive logs (logstore.py), kept until someone removes them
StateDirectory=turnpilot-maps-logs
TasksMax=64
ReadOnlyPaths=$SRV $ETC $OPT
MemoryMax=850M

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable turnpilot-maps >/dev/null
echo "setup done"
