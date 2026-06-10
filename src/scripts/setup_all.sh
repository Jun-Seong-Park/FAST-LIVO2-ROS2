#!/usr/bin/env bash
# Run all one-time setup scripts in order.
#
# Usage: sudo bash scripts/setup_all.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: please run with sudo"
  echo "  sudo bash $0"
  exit 1
fi

run() {
  local script="$SCRIPT_DIR/$1"
  echo ""
  echo "========================================="
  echo " Running: $1"
  echo "========================================="
  bash "$script"
}

run setup_jetson.sh
run setup_stm32.sh
run setup_see3cam.sh
run setup_fastdds.sh

echo ""
echo "========================================="
echo " All setup scripts completed."
echo "========================================="
