#!/usr/bin/env bash
# setup_jetson.sh — install systemd service that runs `jetson_clocks` at every
# boot, locking CPU/GPU/EMC frequencies to MAX for deterministic SLAM latency.
#
# Run once with:
#     sudo bash setup_jetson.sh
#
# Effect after install:
#   - Boot:     jetson_clocks (lock all rails to MAX)
#   - Shutdown: jetson_clocks --restore (return to schedutil)
#   - Manual:   sudo systemctl {start,stop,status} jetson_clocks
#
# Warning: locking to MAX increases power draw + thermals. Verify throttle
# counters stay 0 under your workload (see blackbox plot throttle_*.png).

set -e

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: this script must run as root. Try: sudo bash $0"
  exit 1
fi

if ! command -v jetson_clocks >/dev/null 2>&1; then
  echo "ERROR: /usr/bin/jetson_clocks not found. Are you on a Jetson?"
  exit 1
fi

SERVICE_PATH=/etc/systemd/system/jetson_clocks.service

echo "[1/4] writing ${SERVICE_PATH}"
cat > "$SERVICE_PATH" <<'EOF'
[Unit]
Description=NVIDIA Jetson Clocks — lock CPU/GPU/EMC to MAX
After=multi-user.target nvfancontrol.service
Wants=nvfancontrol.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
ExecStop=/usr/bin/jetson_clocks --restore
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "[2/4] systemctl daemon-reload"
systemctl daemon-reload

echo "[3/4] enable + start jetson_clocks.service"
systemctl enable jetson_clocks.service
systemctl start  jetson_clocks.service

echo "[4/4] status check"
systemctl status jetson_clocks.service --no-pager || true

echo
echo "Done. jetson_clocks is now active and will auto-run on every boot."
echo "Verify lock with: sudo jetson_clocks --show | head"
echo "Disable later  : sudo systemctl disable --now jetson_clocks"
