#!/bin/bash
# livox-gps-sync.service installer
# MID-360 GPS (GPRMC + PPS) time sync systemd unit 생성 및 활성화
#
# Usage: sudo bash scripts/setup_gps_sync.sh

set -e

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: sudo 로 실행해주세요"
  echo "  sudo bash $0"
  exit 1
fi

SVC='livox-gps-sync.service'
UNIT="/etc/systemd/system/$SVC"
USER_NAME='aluxorin22'
SDK_DIR='/home/aluxorin22/ros2_dep_ws/src/Livox-SDK2'
BIN="$SDK_DIR/build/samples/livox_lidar_rmc_time_sync/livox_lidar_rmc_time_sync"
CFG="$SDK_DIR/samples/livox_lidar_rmc_time_sync/mid360_config.json"
LOG='/tmp/livox_gps_sync.log'

cat > "$UNIT" << EOF
[Unit]
Description=Livox MID-360 GPS time sync (RMC feeder + PTP master blocker)
After=network-online.target
Wants=network-online.target
# PTP 마스터와 동시 실행 금지
Conflicts=ptp4l-master.service ptp4l-mid360.service

[Service]
Type=simple
User=$USER_NAME
ExecStartPre=/bin/bash -c '[ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ] || /bin/chmod 666 /dev/ttyUSB0'
ExecStart=$BIN $CFG
Restart=always
RestartSec=5
StandardOutput=append:$LOG
StandardError=append:$LOG

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now "$SVC"

echo "OK: $UNIT 생성 및 활성화됨"
systemctl status "$SVC" --no-pager
