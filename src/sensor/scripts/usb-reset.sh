#!/bin/bash
# Jetson Orin NX USB3 (tegra-xusb / bus 2) 컨트롤러 콜드 리셋.
#
# 언제 쓰나:
#   - lsusb 에 e-con (VID 2560) 안 보임
#   - /dev/24cug, /dev/video0 없음
#   - dmesg 에 "usb 2-X: device not accepting address ... error -71" 후
#     "usb usb2-port?: unable to enumerate USB device" 가 떴음
#
# 원인: See3CAM_24CUG 의 USB3 디스크립터/LPM quirk 로 enumeration 실패 →
#       xhci 가 포트를 dormant 로 잠금. 재꽂기로는 안 풀림.
#
# 조치: tegra-xusb 플랫폼 드라이버 unbind → 1초 대기 → bind.
#       PHY 재캘리브레이션 + slot context 재할당 → link training 재시도.
#
# 사용:
#   bash ~/ros2_dep_ws2/scripts/usb-reset.sh
#   bash ~/ros2_dep_ws2/scripts/usb-reset.sh --force   # 이미 enumerate 돼 있어도 강제

set -e

DEV="3610000.usb"
DRV=/sys/bus/platform/drivers/tegra-xusb
FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

if [[ $FORCE -eq 0 ]] && lsusb | grep -qi "2560:"; then
  echo "이미 e-con 카메라 enumerate 됨. --force 옵션 없으면 스킵."
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
echo ">>> 결과"
if lsusb | grep -qi "2560:"; then
  lsusb | grep -i "2560:"
  ls -la /dev/24cug /dev/24cug_hid /dev/video* 2>/dev/null
  PORT=$(ls /sys/bus/usb/devices/ | grep -E "^2-[0-9]+$" | head -1)
  echo "잡힌 USB3 포트: $PORT  (컨트롤러 = $DEV, 모든 포트 같은 컨트롤러)"
  echo "OK"
else
  echo "FAIL — e-con 아직 안 보임. dmesg 확인:"
  sudo dmesg | tail -20
  exit 1
fi
