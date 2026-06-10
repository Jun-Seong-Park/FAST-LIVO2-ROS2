#!/usr/bin/env bash
# livox_custom_stop — send MID-360 to IDLE state.
#
# Flow:
#   (1) terminate ros2 driver if running — release SDK port
#   (2) livox_idle --reboot  → UDP cmd 0x0200, LiDAR enters IDLE automatically
#   (3) livox_idle --info    → cmd 0x0101, query work_state (verify)
#
# Usage:
#   ./livox_custom_stop.sh
#
# Environment variables:
#   LIVOX_IDLE     path to livox_idle binary
#   LIVOX_CFG      SDK config (host_ip must match this host)
#   KILL_WAIT_S    wait after SIGTERM (default 2)
#   REBOOT_WAIT_S  wait after reboot before verify (default 6, POST/SELFCHECK time)
#
# Exit codes:
#   0  success (work_state = IDLE or transitional)
#   1  missing config / binary / sdk config
#   2  livox_idle --reboot failed

set -uo pipefail

LIVOX_IDLE=${LIVOX_IDLE:-$HOME/ros2_dep_ws/build/livox_sdk2/samples/livox_idle/livox_idle}
LIVOX_CFG=${LIVOX_CFG:-$HOME/ros2_dep_ws/src/Livox-SDK2/samples/livox_lidar_rmc_time_sync/mid360_config.json}
KILL_WAIT_S=${KILL_WAIT_S:-2}
REBOOT_WAIT_S=${REBOOT_WAIT_S:-6}

DRIVER_PATTERN='livox_ros_driver2(_sync)?_node'

log() { printf '[stop] %s\n' "$*"; }
err() { printf '[stop] ERROR: %s\n' "$*" >&2; }

# ── pre-checks ───────────────────────────────────────────────────────
[[ -x "$LIVOX_IDLE" ]] || { err "binary not found: $LIVOX_IDLE"; exit 1; }
[[ -f "$LIVOX_CFG"  ]] || { err "config not found: $LIVOX_CFG"; exit 1; }

# ── (1) terminate ros2 driver ────────────────────────────────────────
if pgrep -f "$DRIVER_PATTERN" >/dev/null; then
  log "sending SIGTERM to ros2 driver"
  pkill -TERM -f "$DRIVER_PATTERN" || true
  sleep "$KILL_WAIT_S"
  if pgrep -f "$DRIVER_PATTERN" >/dev/null; then
    log "SIGTERM ignored → SIGKILL"
    pkill -KILL -f "$DRIVER_PATTERN" || true
    sleep 1
  fi
fi

# ── (2) reboot command ───────────────────────────────────────────────
log "sending reboot command to MID-360 (UDP cmd 0x0200)"
if ! "$LIVOX_IDLE" "$LIVOX_CFG" --reboot; then
  err "livox_idle --reboot failed"
  exit 2
fi

# ── (3) verify ───────────────────────────────────────────────────────
log "waiting ${REBOOT_WAIT_S}s for POST/SELFCHECK"
sleep "$REBOOT_WAIT_S"

log "querying work_state (UDP cmd 0x0101)"
"$LIVOX_IDLE" "$LIVOX_CFG" --info || true

log "done"
