#!/usr/bin/env bash
# livox_custom_stop — MID-360 을 IDLE 로 보낸다.
#
# 흐름:
#   (1) ros2 driver 살아있으면 종료 — SDK 포트 회수
#   (2) livox_idle --reboot  → UDP cmd 0x0200, LiDAR 가 자동 IDLE 진입
#   (3) livox_idle --info    → cmd 0x0101 으로 work_state 조회 (verify)
#
# 사용:
#   ./livox_custom_stop.sh
#
# 환경변수 (CLAUDE.md 하드코딩 금지):
#   LIVOX_IDLE     livox_idle 바이너리 경로
#   LIVOX_CFG      SDK config (host_ip 가 우리 호스트와 일치해야 함)
#   KILL_WAIT_S    SIGTERM 후 대기 (default 2)
#   REBOOT_WAIT_S  reboot 후 verify 까지 대기 (default 6, POST/SELFCHECK 시간)
#
# 종료 코드:
#   0  성공 (work_state = IDLE 또는 transitional)
#   1  설정 / 바이너리 / config 누락
#   2  livox_idle --reboot 자체 실패

set -uo pipefail

LIVOX_IDLE=${LIVOX_IDLE:-$HOME/ros2_dep_ws/build/livox_sdk2/samples/livox_idle/livox_idle}
LIVOX_CFG=${LIVOX_CFG:-$HOME/ros2_dep_ws/src/Livox-SDK2/samples/livox_lidar_rmc_time_sync/mid360_config.json}
KILL_WAIT_S=${KILL_WAIT_S:-2}
REBOOT_WAIT_S=${REBOOT_WAIT_S:-6}

DRIVER_PATTERN='livox_ros_driver2(_sync)?_node'

log() { printf '[stop] %s\n' "$*"; }
err() { printf '[stop] ERROR: %s\n' "$*" >&2; }

# ── 사전 점검 ────────────────────────────────────────────────────────
[[ -x "$LIVOX_IDLE" ]] || { err "binary 없음: $LIVOX_IDLE"; exit 1; }
[[ -f "$LIVOX_CFG"  ]] || { err "config 없음: $LIVOX_CFG"; exit 1; }

# ── (1) ros2 driver 종료 ────────────────────────────────────────────
if pgrep -f "$DRIVER_PATTERN" >/dev/null; then
  log "ros2 driver SIGTERM"
  pkill -TERM -f "$DRIVER_PATTERN" || true
  sleep "$KILL_WAIT_S"
  if pgrep -f "$DRIVER_PATTERN" >/dev/null; then
    log "SIGTERM 무응답 → SIGKILL"
    pkill -KILL -f "$DRIVER_PATTERN" || true
    sleep 1
  fi
fi

# ── (2) reboot 명령 ─────────────────────────────────────────────────
log "MID-360 reboot 명령 전송 (UDP cmd 0x0200)"
if ! "$LIVOX_IDLE" "$LIVOX_CFG" --reboot; then
  err "livox_idle --reboot 실패"
  exit 2
fi

# ── (3) verify ──────────────────────────────────────────────────────
log "POST/SELFCHECK 대기 ${REBOOT_WAIT_S}s"
sleep "$REBOOT_WAIT_S"

log "work_state 조회 (UDP cmd 0x0101)"
"$LIVOX_IDLE" "$LIVOX_CFG" --info || true

log "done"
