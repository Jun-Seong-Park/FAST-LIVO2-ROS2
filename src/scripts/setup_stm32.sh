#!/bin/bash
# ttyUSB udev rule installer
# Permanently sets MODE=0666 on /dev/ttyUSB* so the GPS feeder
# (livox_lidar_rmc_time_sync) can access the STM32 Blue Pill without sudo.
#
# Usage: sudo bash scripts/setup_stm32.sh

set -e

# Grant execute permission to all scripts in this directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x "$SCRIPT_DIR"/*.sh 2>/dev/null || true

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: please run with sudo"
  echo "  sudo bash $0"
  exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-ttyusb.rules"

cat > "$RULE_FILE" << 'EOF'
# ttyUSB — GPS feeder (STM32 Blue Pill GPRMC) access permissions
KERNEL=="ttyUSB*", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "OK: $RULE_FILE installed"
echo "    /dev/ttyUSB* will be fixed at mode 0666."
echo "    Unplug and replug the device to apply immediately."
