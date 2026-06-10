#!/bin/bash
# See3CAM_24CUG camera one-time setup script
# VID=2560 / PID=c128
#
# Includes:
#   1. udev: /dev/24cug (V4L2), /dev/24cug_hid (HID) symlinks + permissions
#   2. udev: permanently disable USB autosuspend (default 2000ms → -1)
#   3. uvcvideo nodrop=1 — prevent incomplete triggered frames from being dropped
#
# Usage: sudo bash scripts/setup_see3cam.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x "$SCRIPT_DIR"/*.sh 2>/dev/null || true

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: please run with sudo"
  echo "  sudo bash $0"
  exit 1
fi

# ── 1. udev rules ────────────────────────────────────────────────────────────
RULE_FILE="/etc/udev/rules.d/99-see3cam-24cug.rules"

cat > "$RULE_FILE" << 'EOF'
# See3CAM_24CUG — V4L2 / HID symlinks + permissions
SUBSYSTEM=="video4linux", ATTR{name}=="See3CAM_24CUG", ATTR{index}=="0", SYMLINK+="24cug", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{product}=="See3CAM_24CUG", SYMLINK+="24cug_hid", MODE="0666"

# Disable USB autosuspend — default 2000ms, confirmed to add latency on resume
SUBSYSTEM=="usb", ATTRS{idVendor}=="2560", ATTRS{idProduct}=="c128", ATTR{power/autosuspend_delay_ms}="-1", ATTR{power/control}="on"
EOF

udevadm control --reload-rules
udevadm trigger
echo "OK: udev rules installed ($RULE_FILE)"
echo "--- contents ---"
cat "$RULE_FILE"
echo "----------------"

# ── 2. uvcvideo nodrop=1 ─────────────────────────────────────────────────────
MODPROBE_FILE="/etc/modprobe.d/uvcvideo.conf"
if ! grep -q "nodrop=1" "$MODPROBE_FILE" 2>/dev/null; then
  echo "options uvcvideo nodrop=1" > "$MODPROBE_FILE"
  echo "OK: uvcvideo nodrop=1 set ($MODPROBE_FILE)"
else
  echo "OK: uvcvideo nodrop=1 already set"
fi

echo ""
echo "=== Done ==="
echo "Please unplug and replug the camera."
