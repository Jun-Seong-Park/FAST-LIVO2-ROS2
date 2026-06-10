#!/bin/bash
# Cold reset the Jetson Orin NX USB3 controller (tegra-xusb / bus 2).
#
# When to use:
#   - e-con camera (VID 2560) not visible in lsusb
#   - /dev/24cug, /dev/video0 missing
#   - dmesg shows "usb 2-X: device not accepting address ... error -71"
#     followed by "usb usb2-port?: unable to enumerate USB device"
#
# Root cause: See3CAM_24CUG USB3 descriptor / LPM quirk causes enumeration failure
#             → xhci locks the port as dormant. Replugging does not recover it.
#
# Fix: unbind tegra-xusb platform driver → wait 1s → bind.
#      Forces PHY recalibration + slot context reallocation → link training retry.
#
# Usage:
#   bash scripts/usb-reset.sh
#   bash scripts/usb-reset.sh --force   # force reset even if camera is already enumerated

set -e

DEV="3610000.usb"
DRV=/sys/bus/platform/drivers/tegra-xusb
FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

if [[ $FORCE -eq 0 ]] && lsusb | grep -qi "2560:"; then
  echo "e-con camera already enumerated. skipping (use --force to override)."
  lsusb | grep -i "2560:"
  ls -la /dev/24cug /dev/24cug_hid /dev/video* 2>/dev/null || true
  exit 0
fi

echo ">>> unbind $DEV"
echo "$DEV" | sudo tee "$DRV/unbind" >/dev/null

sleep 1

echo ">>> bind $DEV"
echo "$DEV" | sudo tee "$DRV/bind" >/dev/null

sleep 3

echo
echo ">>> result"
if lsusb | grep -qi "2560:"; then
  lsusb | grep -i "2560:"
  ls -la /dev/24cug /dev/24cug_hid /dev/video* 2>/dev/null
  PORT=$(ls /sys/bus/usb/devices/ | grep -E "^2-[0-9]+$" | head -1)
  echo "USB3 port: $PORT  (controller = $DEV, all ports share the same controller)"
  echo "OK"
else
  echo "FAIL — e-con camera still not visible. check dmesg:"
  sudo dmesg | tail -20
  exit 1
fi
