#!/bin/bash
# ttyUSB udev rule installer
# GPS feeder (livox_lidar_rmc_time_sync) 가 /dev/ttyUSB* 에 sudo 없이 접근하도록
# MODE=0666 을 영구 설정한다.
#
# Usage: sudo bash scripts/setup_stm32.sh

set -e

# 이 스크립트가 속한 scripts/ 폴더의 모든 sh/bash 파일에 실행 권한 부여
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x "$SCRIPT_DIR"/*.sh 2>/dev/null || true

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: sudo 로 실행해주세요"
  echo "  sudo bash $0"
  exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-ttyusb.rules"

cat > "$RULE_FILE" << 'EOF'
# ttyUSB — GPS feeder (STM32 Blue Pill GPRMC) 접근 권한
KERNEL=="ttyUSB*", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "OK: $RULE_FILE 설치됨"
echo "    /dev/ttyUSB* 권한이 0666 으로 고정됩니다."
echo "    장치를 뺐다 다시 꽂으면 즉시 적용됩니다."
