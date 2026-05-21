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

def sub_plot_combined(a: np.ndarray, m: dict, out_path: Path,
                      target_ms: float, tol_pct: float):
    t_axis        = (a['t_cbk_ns'][1:].astype(np.int64) - int(a['t_cbk_ns'][0])) / 1e9
    stamp_diff_ms = np.diff(a['header_stamp'].astype(np.int64)) / 1e6
    pct_stale     = 100.0 * len(m['stale']) / max(1, len(stamp_diff_ms))

    fig, axs = plt.subplots(3, 2, figsize=(12, 10), dpi=150)

    # ── left column ───────────────────────────────────────────────────────────
    d = m['arrive_dt_ms']

    ax = axs[0, 0]
    ax.plot(t_axis, d, linewidth=1.5)
    ax.axhline(d.mean(), color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {d.mean():.2f} ms')
    ax.axhline(np.percentile(d, 99), color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(d, 99):.2f} ms')
    ax.set_title(f'{m["label"]} SUB  Arrival Interval',
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_ylabel('Interval [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    ax = axs[1, 0]
    ax.hist(d, bins=40, color='C0', alpha=0.8, edgecolor='none')
    ax.axvline(d.mean(), color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {d.mean():.2f} ms')
    ax.axvline(np.percentile(d, 99), color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(d, 99):.2f} ms')
    ax.set_title(f'{m["label"]} SUB  Arrival Interval Distribution',
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_xlabel('Interval [ms]', fontsize=12)
    ax.set_ylabel('Count', fontsize=12)
    style(ax); legend(ax)

    idx     = np.arange(len(a['header_stamp']))
    stamp_s = (a['header_stamp'].astype(np.int64) - int(a['header_stamp'][0])) / 1e9
    ax = axs[2, 0]
    ax.plot(idx, stamp_s, linewidth=1.5, color='C1')
    ax.set_title(f'{m["label"]} SUB  Header Stamp vs Index',
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_ylabel('Stamp [sec]', fontsize=12)
    ax.set_xlabel('Index [Sample]', fontsize=12)
    style(ax)

    # ── right column: seq & stamp health ─────────────────────────────────────
    seq_diff = m['seq_diff']
    gap_n    = len(m['gaps'])
    status   = '[GOOD]' if gap_n == 0 else ('[WARN]' if gap_n < 100 else '[BAD]')

    ax = axs[0, 1]
    ax.plot(t_axis, seq_diff, linewidth=1.5, color='C4')
    ax.axhline(1, color='black', linestyle=':', linewidth=1.0, label='No drop (=1)')
    ax.axhline(2, color='black', linestyle='--', linewidth=0.8, label='Halved rate (=2)')
    ax.set_ylim(0, max(3.0, float(seq_diff.max()) + 0.5))
    ax.set_title(f'{m["label"]} SUB  Seq Diff  gaps={gap_n}  {status}',
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_ylabel('Seq Diff', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    tol     = tol_pct / 100.0
    mean_ms = float(stamp_diff_ms.mean())
    p99_ms  = float(np.percentile(stamp_diff_ms, 99))

    ax = axs[1, 1]
    ax.plot(t_axis, stamp_diff_ms, linewidth=1.5, color='C3')
    ax.axhline(target_ms, color='black', linestyle=':', linewidth=1.0,
               label=f'Target {target_ms:g} ms')
    ax.axhline(mean_ms, color='black', linestyle='-',  linewidth=1.0,
               label=f'Mean {mean_ms:.2f} ms')
    ax.axhline(p99_ms,  color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {p99_ms:.2f} ms')
    ax.set_ylim(target_ms * (1 - tol), target_ms * (1 + tol))
    ax.set_title((f'{m["label"]} SUB  Stamp Delta  Zoom +/-{tol_pct:g}%'
                  f'  Stale {len(m["stale"])} ({pct_stale:.2f}%)'),
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_ylabel('Stamp Delta [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax); legend(ax)

    ax = axs[2, 1]
    ax.plot(t_axis, stamp_diff_ms, linewidth=1.5, color='C3')
    ax.set_title(f'{m["label"]} SUB  Stamp Delta Full Range',
                 fontsize=14, fontweight='bold', loc='center', pad=12)
    ax.set_ylabel('Stamp Delta [ms]', fontsize=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    style(ax)

    savefig(fig, out_path)


def process_sub(arr: np.ndarray, label: str, prefix: str,
                out_dir: Path, target_ms: float, tol_pct: float):
    m = sub_metrics(arr, label)
    sub_plot_combined(arr, m, out_dir / f'{prefix}_sub.png',
                      target_ms=target_ms, tol_pct=tol_pct)


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
                    target_ms=100.0, tol_pct=2.0)
    if imu is not None:
        process_sub(imu, 'IMU', 'imu', args.out_dir / 'imu',
                    target_ms=5.0, tol_pct=30.0)
    if image is not None:
        process_sub(image, 'IMAGE', 'image', args.out_dir / 'image',
                    target_ms=100.0, tol_pct=5.0)

    print(f'\nplots saved to {args.out_dir}')


if __name__ == '__main__':
    main()
