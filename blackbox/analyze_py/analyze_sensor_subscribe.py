#!/usr/bin/env python3
"""
Sensor subscriber trace analyzer (lidar + imu + image).

bin inputs (blackbox dump):
  trace/log/lidar_subrecord.bin  — LidarSubRecord  (24B)
  trace/log/imu_subrecord.bin    — ImuSubRecord    (24B)
  trace/log/image_subrecord.bin  — ImageSubRecord  (24B)

SubRecord (24B): {seq, header_stamp, t_cbk_ns}

sub seq = publisher seq at receive time.
seq_diff > 1  → subscriber missed frames (pub-side drop or DDS drop).
seq_diff == 2 consistently → subscriber processing at half publisher rate.

usage:
  python3 analyze_sensor_subscribe.py
  python3 analyze_sensor_subscribe.py --out-dir /path/to/folder
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


DT_SUB = np.dtype([
    ('seq',          '<u8'),
    ('header_stamp', '<u8'),
    ('t_cbk_ns',     '<u8'),
])
assert DT_SUB.itemsize == 24


# ────────────────────────────────────────────────────────────
# plot target periods [ms] — match each stream's publish period
# ────────────────────────────────────────────────────────────
CAM_TARGET_MS   = 100.0
LIDAR_TARGET_MS = 100.0
IMU_TARGET_MS   = 5.0

# diff_status thresholds (% max-dev vs target). Also used as the status_ylim zoom width.
STATUS_PCT = {'GOOD': 2.0, 'WARN': 5.0}


# ────────────────────────────────────────────────────────────
# style helpers
# ────────────────────────────────────────────────────────────

def style(ax):
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3, color='gray')
    ax.set_axisbelow(True)
    ax.spines[['top', 'right']].set_visible(False)


def legend(ax):
    ax.legend(loc='upper right', fontsize=10, frameon=True,
              framealpha=0.9, edgecolor='gray')


def savefig(fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)


# ────────────────────────────────────────────────────────────
# status thresholds (cascading max-deviation vs reference)
# ────────────────────────────────────────────────────────────
# All values within ref ±GOOD% → GOOD. If any exceeds, judge against ±WARN% → WARN.
# If any exceeds that too, BAD.
def diff_status(values: np.ndarray, ref: float) -> str:
    if ref <= 0 or len(values) == 0:
        return 'GOOD'
    max_dev_pct = float(np.max(np.abs(values - ref)) / ref * 100.0)
    if max_dev_pct <= STATUS_PCT['GOOD']: return 'GOOD'
    if max_dev_pct <= STATUS_PCT['WARN']: return 'WARN'
    return 'BAD'


def status_ylim(ax, target_ms: float, status: str) -> None:
    """GOOD: ±2% zoom, WARN: ±5% zoom, BAD: auto (full data range)."""
    if status == 'BAD':
        return
    pct = STATUS_PCT[status] / 100.0
    ax.set_ylim(target_ms * (1 - pct), target_ms * (1 + pct))


# ────────────────────────────────────────────────────────────
# loader
# ────────────────────────────────────────────────────────────

def load(path: Path, label: str) -> np.ndarray | None:
    if not path.exists():
        print(f'[skip] {label}: not found ({path})')
        return None
    arr = np.fromfile(path, dtype=DT_SUB)
    arr = arr[arr['seq'] > 0]
    if len(arr) == 0:
        print(f'[skip] {label}: empty after seq>0 filter ({path})')
        return None
    print(f'loaded {len(arr)} records for {label}')
    return arr


# ────────────────────────────────────────────────────────────
# metrics
# ────────────────────────────────────────────────────────────

def sub_metrics(a: np.ndarray, label: str) -> dict:
    arrive_dt_ms = np.diff(a['t_cbk_ns'].astype(np.int64))     / 1e6
    stamp_dt_ms  = np.diff(a['header_stamp'].astype(np.int64)) / 1e6
    seq          = a['seq'].astype(np.int64)
    seq_diff     = np.diff(seq)
    gaps         = np.where(seq_diff > 1)[0]
    stale        = np.where(np.diff(a['header_stamp']) == 0)[0]

    dur_s = (int(a['t_cbk_ns'][-1]) - int(a['t_cbk_ns'][0])) / 1e9 if len(a) >= 2 else 0.0
    rate  = len(a) / dur_s if dur_s > 0 else 0.0

    print(f'[{label}]  records={len(a)}  rate={rate:.2f} Hz  gaps={len(gaps)}')
    return dict(arrive_dt_ms=arrive_dt_ms, stamp_dt_ms=stamp_dt_ms,
                seq=seq, seq_diff=seq_diff, gaps=gaps, stale=stale,
                rate=rate, label=label)


# ────────────────────────────────────────────────────────────
# plot
# ────────────────────────────────────────────────────────────

def _title(ax, t):
    ax.set_title(t, fontsize=14, fontweight='bold', loc='center', pad=12)


def sub_plot_combined(a: np.ndarray, m: dict, out_path: Path,
                      target_ms: float):
    label         = m['label']
    t_axis        = (a['t_cbk_ns'][1:].astype(np.int64) - int(a['t_cbk_ns'][0])) / 1e9
    arrive_dt_ms  = m['arrive_dt_ms']
    stamp_diff_ms = np.diff(a['header_stamp'].astype(np.int64)) / 1e6

    fig, axs = plt.subplots(3, 2, figsize=(12, 10), dpi=150)

    # ── [0,0] Stamp Delta Full Range (placeholder, to be replaced later) ──────
    ax = axs[0, 0]
    ax.plot(t_axis, stamp_diff_ms, linewidth=1.5, color='C3')
    _title(ax, f'{label}  Stamp Delta Full Range')
    ax.set_ylabel('Stamp Delta [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax)

    # ── [0,1] Monotonic Diff Distribution (t_cbk_ns diff) ────────────────────
    ax = axs[0, 1]
    ax.hist(arrive_dt_ms, bins=40, color='C0', alpha=0.8, edgecolor='none')
    ax.axvline(arrive_dt_ms.mean(), color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {arrive_dt_ms.mean():.2f} ms')
    ax.axvline(np.percentile(arrive_dt_ms, 99), color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(arrive_dt_ms, 99):.2f} ms')
    _title(ax, f'{label}  Monotonic Diff Distribution')
    ax.set_xlabel('Cbk Delta [ms]', fontsize=12)
    ax.set_ylabel('Count', fontsize=12)
    style(ax); legend(ax)

    # ── [1,0] Monotonic Diff (t_cbk_ns diff) ─────────────────────────────────
    arr_mean_ms  = float(arrive_dt_ms.mean())
    arr_p99_ms   = float(np.percentile(arrive_dt_ms, 99))
    arr_worst_ms = float(arrive_dt_ms[np.argmax(np.abs(arrive_dt_ms - target_ms))])
    arr_status   = diff_status(arrive_dt_ms, target_ms)
    ax = axs[1, 0]
    ax.plot(t_axis, arrive_dt_ms, linewidth=1.5, color='C3')
    ax.axhline(target_ms,    color='black', linestyle=':',  linewidth=1.0,
               label=f'Target {target_ms:g} ms')
    ax.axhline(arr_mean_ms,  color='black', linestyle='-',  linewidth=1.0,
               label=f'Mean {arr_mean_ms:.2f} ms')
    ax.axhline(arr_p99_ms,   color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {arr_p99_ms:.2f} ms')
    ax.axhline(arr_worst_ms, color='C3',    linestyle='-.', linewidth=1.0,
               label=f'Worst {arr_worst_ms:.2f} ms')
    status_ylim(ax, target_ms, arr_status)
    _title(ax, f'{label}  Monotonic Diff  [{arr_status}]')
    ax.set_ylabel('Cbk Delta [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    # ── [1,1] Stamp Diff (header_stamp diff) ─────────────────────────────────
    stamp_mean_ms  = float(stamp_diff_ms.mean())
    stamp_p99_ms   = float(np.percentile(stamp_diff_ms, 99))
    stamp_worst_ms = float(stamp_diff_ms[np.argmax(np.abs(stamp_diff_ms - target_ms))])
    stamp_status   = diff_status(stamp_diff_ms, target_ms)
    ax = axs[1, 1]
    ax.plot(t_axis, stamp_diff_ms, linewidth=1.5, color='C3')
    ax.axhline(target_ms,      color='black', linestyle=':',  linewidth=1.0,
               label=f'Target {target_ms:g} ms')
    ax.axhline(stamp_mean_ms,  color='black', linestyle='-',  linewidth=1.0,
               label=f'Mean {stamp_mean_ms:.2f} ms')
    ax.axhline(stamp_p99_ms,   color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {stamp_p99_ms:.2f} ms')
    ax.axhline(stamp_worst_ms, color='C3',    linestyle='-.', linewidth=1.0,
               label=f'Worst {stamp_worst_ms:.2f} ms')
    status_ylim(ax, target_ms, stamp_status)
    _title(ax, f'{label}  Stamp Diff  [{stamp_status}]')
    ax.set_ylabel('Stamp Delta [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    # ── [2,0] Header Stamp vs Index (unchanged) ──────────────────────────────
    idx     = np.arange(len(a['header_stamp']))
    stamp_s = (a['header_stamp'].astype(np.int64) - int(a['header_stamp'][0])) / 1e9
    ax = axs[2, 0]
    ax.plot(idx, stamp_s, linewidth=1.5, color='C1')
    _title(ax, f'{label}  Header Stamp vs Index')
    ax.set_ylabel('Stamp [sec]', fontsize=12)
    ax.set_xlabel('Index [Sample]', fontsize=12)
    style(ax)

    # ── [2,1] Drop Count seq_diff (moved from [0,1]) ─────────────────────────
    seq_diff  = m['seq_diff']
    gap_n     = len(m['gaps'])
    drop_stat = 'GOOD' if gap_n == 0 else ('WARN' if gap_n < 100 else 'BAD')
    ax = axs[2, 1]
    ax.plot(t_axis, seq_diff, linewidth=1.5, color='C4')
    ax.axhline(1, color='black', linestyle=':', linewidth=1.0, label='No drop (=1)')
    ax.axhline(2, color='black', linestyle='--', linewidth=0.8, label='Halved rate (=2)')
    ax.set_ylim(0, max(3.0, float(seq_diff.max()) + 0.5))
    _title(ax, f'{label}  Drop Count = {gap_n}  [{drop_stat}]')
    ax.set_ylabel('Seq Diff', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    savefig(fig, out_path)


def process_sub(arr: np.ndarray, label: str, prefix: str,
                out_dir: Path, target_ms: float):
    m = sub_metrics(arr, label)
    sub_plot_combined(arr, m, out_dir / f'{prefix}_sub.png',
                      target_ms=target_ms)


# ────────────────────────────────────────────────────────────
# main
# ────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description='Sensor subscriber trace analyzer (lidar + imu + image)')
    log_dir = Path(__file__).resolve().parent.parent / 'log'
    out_dir = Path(__file__).resolve().parent.parent / 'plots'

    p.add_argument('--lidar',   type=Path, default=log_dir / 'lidar_subrecord.bin')
    p.add_argument('--imu',     type=Path, default=log_dir / 'imu_subrecord.bin')
    p.add_argument('--image',   type=Path, default=log_dir / 'image_subrecord.bin')
    p.add_argument('--out-dir', type=Path, default=out_dir)
    args = p.parse_args()

    for sub in ('image', 'lidar', 'imu'):
        (args.out_dir / sub).mkdir(parents=True, exist_ok=True)

    lidar = load(args.lidar, 'LIDAR')
    imu   = load(args.imu,   'IMU')
    image = load(args.image, 'IMAGE')

    if lidar is not None:
        process_sub(lidar, 'LIDAR', 'lidar', args.out_dir / 'lidar',
                    target_ms=LIDAR_TARGET_MS)
    if imu is not None:
        process_sub(imu, 'IMU', 'imu', args.out_dir / 'imu',
                    target_ms=IMU_TARGET_MS)
    if image is not None:
        process_sub(image, 'CAM', 'image', args.out_dir / 'image',
                    target_ms=CAM_TARGET_MS)

    print(f'\nplots saved to {args.out_dir}')


if __name__ == '__main__':
    main()
