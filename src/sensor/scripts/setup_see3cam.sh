#!/bin/bash
# See3CAM_24CUG 카메라 설정 통합 스크립트
# VID=2560 / PID=c128
#
# 포함 항목:
#   1. udev: /dev/24cug (V4L2), /dev/24cug_hid (HID) 심링크 + 권한
#   2. udev: USB autosuspend 영구 비활성화 (기본 2000ms → -1)
#   3. uvcvideo nodrop=1  — 트리거 프레임 incomplete 드롭 방지
#
# Usage: sudo bash scripts/setup_see3cam.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x "$SCRIPT_DIR"/*.sh 2>/dev/null || true

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: sudo 로 실행해주세요"
  echo "  sudo bash $0"
  exit 1
fi

# ── 1. udev rules ────────────────────────────────────────────────────────────
RULE_FILE="/etc/udev/rules.d/99-see3cam-24cug.rules"

cat > "$RULE_FILE" << 'EOF'
# See3CAM_24CUG — V4L2 / HID 심링크 + 권한
SUBSYSTEM=="video4linux", ATTR{name}=="See3CAM_24CUG", ATTR{index}=="0", SYMLINK+="24cug", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{product}=="See3CAM_24CUG", SYMLINK+="24cug_hid", MODE="0666"

# USB autosuspend 비활성화 — 기본값 2000ms, 절전 복귀 지연으로 레이턴시 영향 확인됨
SUBSYSTEM=="usb", ATTRS{idVendor}=="2560", ATTRS{idProduct}=="c128", ATTR{power/autosuspend_delay_ms}="-1", ATTR{power/control}="on"
EOF

udevadm control --reload-rules
udevadm trigger
echo "OK: udev rules 설치됨 ($RULE_FILE)"
echo "--- 내용 ---"
cat "$RULE_FILE"
echo "------------"

# ── 2. uvcvideo nodrop=1 ─────────────────────────────────────────────────────
MODPROBE_FILE="/etc/modprobe.d/uvcvideo.conf"
if ! grep -q "nodrop=1" "$MODPROBE_FILE" 2>/dev/null; then
  echo "options uvcvideo nodrop=1" > "$MODPROBE_FILE"
  echo "OK: uvcvideo nodrop=1 설정됨 ($MODPROBE_FILE)"
else
  echo "OK: uvcvideo nodrop=1 이미 설정됨"
fi

echo ""
echo "=== 완료 ==="
echo "카메라를 뺐다 다시 꽂아주세요."
