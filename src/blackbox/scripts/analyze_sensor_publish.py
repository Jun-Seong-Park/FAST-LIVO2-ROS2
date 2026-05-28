#!/usr/bin/env python3
"""
Sensor publisher trace analyzer (camera + lidar + imu).

bin inputs (blackbox dump):
  trace/log/see3cam24cug_<variant>_image_pub.bin  — ImagePubRecord (48B)
  trace/log/lidar_pubrecord.bin                    — LidarPubRecord (24B)
  trace/log/imu_pubrecord.bin                      — ImuPubRecord   (24B)

ImagePubRecord (blackbox spec §3.5):
  {seq, header_stamp, t_dqbuf_ns, t_cbk_ns, t_pub_ns, gst_done, ros_done, _pad[6]}

usage:
  python3 analyze_sensor_publish.py
  python3 analyze_sensor_publish.py --resolution hd
  python3 analyze_sensor_publish.py --out-dir /path/to/folder
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


# ────────────────────────────────────────────────────────────
# External reference measurements (v4l2_tester, 2026-05-18)
# ────────────────────────────────────────────────────────────
# UYVY SD 1280x720→640x360 60fps bufs=2, jetson_clocks+autosuspend on
# Measurement A: SOE→DQBUF  = 11.37 ms  (exposure + USB transfer, structural floor)
# Measurement C: DQBUF→done =  4.65 ms  (swscale only, EOF-based reference)
SOE_TO_DQBUF_MS = 11.37

# ────────────────────────────────────────────────────────────
# dtypes (blackbox spec §3.5)
# ────────────────────────────────────────────────────────────

DT_IMG = np.dtype([
    ('seq',          '<u8'),
    ('header_stamp', '<u8'),
    ('t_dqbuf_ns',   '<u8'),
    ('t_cbk_ns',     '<u8'),
    ('t_pub_ns',     '<u8'),
    ('gst_done',     '<u1'),
    ('ros_done',     '<u1'),
    ('_pad',         '<u1', 6),
])
assert DT_IMG.itemsize == 48

DT_PUB = np.dtype([
    ('seq',          '<u8'),
    ('header_stamp', '<u8'),
    ('t_pub_ns',     '<u8'),
])
assert DT_PUB.itemsize == 24


# ────────────────────────────────────────────────────────────
# plot target periods [ms] — match each stream's publish period
# tol_pct: y-zoom half-width for Stamp Diff plot (status는 diff_status가 별도 판정)
# ────────────────────────────────────────────────────────────
CAM_TARGET_MS   = 100.0
LIDAR_TARGET_MS = 100.0
LIDAR_TOL_PCT   = 2.0
IMU_TARGET_MS   = 5.0
IMU_TOL_PCT     = 30.0

# diff_status thresholds (% max-dev vs target). 동시에 status_ylim zoom 폭으로도 사용.
STATUS_PCT = {'GOOD': 2.0, 'WARN': 5.0}


# ────────────────────────────────────────────────────────────
# style helpers
# ────────────────────────────────────────────────────────────

def style(ax):
    ax.grid(True, which='major', linestyle='-', linewidth=0.5,
            alpha=0.3, color='gray')
    ax.set_axisbelow(True)
    ax.spines[['top', 'right']].set_visible(False)


def title(ax, t):  ax.set_title(t, fontsize=14, fontweight='bold', loc='center', pad=12)
def xlabel(ax, t): ax.set_xlabel(t, fontsize=12)
def ylabel(ax, t): ax.set_ylabel(t, fontsize=12)


def legend(ax, loc='upper right'):
    ax.legend(loc=loc, fontsize=10, frameon=True,
              framealpha=0.9, edgecolor='gray')


def savefig(fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)


# ────────────────────────────────────────────────────────────
# status thresholds (cascading max-deviation vs reference)
# ────────────────────────────────────────────────────────────
# 모든 값이 ref ±GOOD% 안 → GOOD. 하나라도 벗어나면 ±WARN% 로 판단 → WARN.
# 하나라도 벗어나면 BAD.
def diff_status(values: np.ndarray, ref: float) -> str:
    if ref <= 0 or len(values) == 0:
        return 'GOOD'
    max_dev_pct = float(np.max(np.abs(values - ref)) / ref * 100.0)
    if max_dev_pct <= STATUS_PCT['GOOD']: return 'GOOD'
    if max_dev_pct <= STATUS_PCT['WARN']: return 'WARN'
    return 'BAD'


def status_ylim(ax, target_ms: float, status: str) -> None:
    """GOOD: ±2% zoom, WARN: ±5% zoom, BAD: auto (data 끝까지)."""
    if status == 'BAD':
        return
    pct = STATUS_PCT[status] / 100.0
    ax.set_ylim(target_ms * (1 - pct), target_ms * (1 + pct))


# ────────────────────────────────────────────────────────────
# loaders
# ────────────────────────────────────────────────────────────

def load(path: Path, label: str, dtype) -> np.ndarray | None:
    if not path.exists():
        print(f'[skip] {label}: not found ({path})')
        return None
    arr = np.fromfile(path, dtype=dtype)
    arr = arr[arr['seq'] > 0]
    if len(arr) == 0:
        print(f'[skip] {label}: empty after seq>0 filter ({path})')
        return None
    print(f'loaded {len(arr)} records for {label}')
    return arr


# ────────────────────────────────────────────────────────────
# camera analysis
# ────────────────────────────────────────────────────────────

def cam_metrics(a: np.ndarray) -> dict:
    pub_valid   = a['ros_done'] == 1
    gst_proc_ms = ((a[pub_valid]['t_cbk_ns'].astype(np.int64)
                    - a[pub_valid]['t_dqbuf_ns'].astype(np.int64)) / 1e6)
    cbk_dt_ms   = ((a[pub_valid]['t_pub_ns'].astype(np.int64)
                    - a[pub_valid]['t_cbk_ns'].astype(np.int64)) / 1e6)
    e2e_ms      = gst_proc_ms + cbk_dt_ms
    gst_dt_ms   = gst_proc_ms

    seq      = a['seq'].astype(np.int64)
    seq_diff = np.diff(seq)
    gaps     = np.where(seq_diff > 1)[0]
    stale    = np.where(np.diff(a['header_stamp']) == 0)[0]

    t_first   = int(a['t_cbk_ns'][0])
    t_last    = int(a[pub_valid]['t_pub_ns'][-1]) if pub_valid.any() else int(a['t_cbk_ns'][-1])
    dur_s     = (t_last - t_first) / 1e9 if len(a) >= 2 else 0.0
    rate      = len(a) / dur_s if dur_s > 0 else 0.0
    n_partial = int(np.sum(~pub_valid))

    print(f'[CAMERA]  records={len(a)}  partial={n_partial}  '
          f'rate={rate:.2f} Hz  gaps={len(gaps)}')

    return dict(gst_dt_ms=gst_dt_ms, gst_proc_ms=gst_proc_ms,
                cbk_dt_ms=cbk_dt_ms, e2e_ms=e2e_ms,
                seq=seq, seq_diff=seq_diff, gaps=gaps, stale=stale,
                pub_valid=pub_valid, a=a)


def cam_plot_combined(a: np.ndarray, m: dict, out_path: Path,
                      target_ms: float = CAM_TARGET_MS):
    t_all_s        = (a['t_cbk_ns'].astype(np.int64) - int(a['t_cbk_ns'][0])) / 1e9
    t_valid_s      = t_all_s[m['pub_valid']]
    t_axis         = t_all_s[1:]
    stamp_diff_ms  = np.diff(a['header_stamp'].astype(np.int64)) / 1e6

    t_pub_valid_ns = a[m['pub_valid']]['t_pub_ns'].astype(np.int64)
    t_pub_diff_ms  = np.diff(t_pub_valid_ns) / 1e6
    t_pub_axis_s   = (t_pub_valid_ns[1:] - int(a['t_cbk_ns'][0])) / 1e9

    fig, axs = plt.subplots(3, 2, figsize=(12, 10), dpi=150)

    # ── [0,0] stacked area: SOE→DQBUF / GStreamer / cbk→pub over time ────────
    ax = axs[0, 0]
    import matplotlib.colors as mcolors
    soe_arr = np.full(len(m['gst_proc_ms']), SOE_TO_DQBUF_MS)
    ax.stackplot(
        t_valid_s, soe_arr, m['gst_proc_ms'], m['cbk_dt_ms'],
        colors=[mcolors.to_hex('C1'), mcolors.to_hex('C0'), mcolors.to_hex('C2')],
        labels=[f'SOE→DQBUF  {SOE_TO_DQBUF_MS:.2f} ms',
                f'GStreamer  {m["gst_proc_ms"].mean():.2f} ms (mean)',
                f'cbk→Pub   {m["cbk_dt_ms"].mean():.2f} ms (mean)'],
    )
    title(ax, 'CAM  Stacked Latency'); ylabel(ax, 'Latency [ms]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    # ── [0,1] histogram: MONOTONIC_RAW Diff distribution ─────────────────────
    ax = axs[0, 1]
    ax.hist(t_pub_diff_ms, bins=40, color='C0', alpha=0.8, edgecolor='none')
    ax.axvline(t_pub_diff_ms.mean(),
               color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {t_pub_diff_ms.mean():.2f} ms')
    ax.axvline(np.percentile(t_pub_diff_ms, 99),
               color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(t_pub_diff_ms, 99):.2f} ms')
    title(ax, 'CAM  MONOTONIC_RAW Diff Distribution')
    xlabel(ax, 'Pub Delta [ms]'); ylabel(ax, 'Count')
    style(ax); legend(ax)

    # ── [1,0] MONOTONIC_RAW Diff (new) ───────────────────────────────────────
    pub_mean_ms  = float(t_pub_diff_ms.mean())
    pub_p99_ms   = float(np.percentile(t_pub_diff_ms, 99))
    pub_worst_ms = float(t_pub_diff_ms[np.argmax(np.abs(t_pub_diff_ms - target_ms))])
    pub_status   = diff_status(t_pub_diff_ms, target_ms)
    ax = axs[1, 0]
    ax.plot(t_pub_axis_s, t_pub_diff_ms, linewidth=1.5, color='C3')
    ax.axhline(target_ms,    color='black', linestyle=':',  linewidth=1.0,
               label=f'Target {target_ms:g} ms')
    ax.axhline(pub_mean_ms,  color='black', linestyle='-',  linewidth=1.0,
               label=f'Mean {pub_mean_ms:.2f} ms')
    ax.axhline(pub_p99_ms,   color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {pub_p99_ms:.2f} ms')
    ax.axhline(pub_worst_ms, color='C3',    linestyle='-.', linewidth=1.0,
               label=f'Worst {pub_worst_ms:.2f} ms')
    status_ylim(ax, target_ms, pub_status)
    title(ax, f'CAM  MONOTONIC_RAW Diff  [{pub_status}]')
    ylabel(ax, 'Pub Delta [ms]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    # ── [1,1] Stamp Diff ─────────────────────────────────────────────────────
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
    title(ax, f'CAM  Stamp Diff  [{stamp_status}]')
    ylabel(ax, 'Stamp Delta [ms]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    # ── [2,0] header stamp vs index (unchanged) ──────────────────────────────
    ax = axs[2, 0]
    idx     = np.arange(len(a['header_stamp']))
    stamp_s = (a['header_stamp'].astype(np.int64) - int(a['header_stamp'][0])) / 1e9
    ax.plot(idx, stamp_s, linewidth=1.5, color='C1')
    title(ax, 'CAM  Header Stamp vs Index')
    ylabel(ax, 'Stamp [sec]'); xlabel(ax, 'Index [Sample]')
    style(ax)

    # ── [2,1] Drop Count seq_diff (moved from [0,1]) ─────────────────────────
    gap_n     = len(m['gaps'])
    drop_stat = 'GOOD' if gap_n == 0 else ('WARN' if gap_n < 100 else 'BAD')
    ax = axs[2, 1]
    ax.plot(t_axis, m['seq_diff'], linewidth=1.5, color='C4')
    ax.axhline(1, color='black', linestyle=':', linewidth=1.0, label='Expected (=1)')
    ax.set_ylim(0, 2)
    title(ax, f'CAM  Drop Count = {gap_n}  [{drop_stat}]')
    ylabel(ax, 'Seq Diff [Frame]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    savefig(fig, out_path)


def cam_plot_breakdown(m, out_path):
    soe_ms  = SOE_TO_DQBUF_MS
    gst     = m['gst_dt_ms'].mean()
    cbk     = m['cbk_dt_ms'].mean()
    full_ms = soe_ms + m['e2e_ms']   # SOE→Pub distribution

    fig = plt.figure(figsize=(13, 5), dpi=150)
    gs  = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.3], wspace=0.30)
    ax_pie = fig.add_subplot(gs[0, 0])
    ax_bar = fig.add_subplot(gs[0, 1])

    total = soe_ms + gst + cbk
    ax_pie.pie(
        [soe_ms, gst, cbk],
        labels=[f'SOE→DQBUF\n{soe_ms:.2f} ms',
                f'GStreamer\n{gst:.2f} ms',
                f'Callback→Pub\n{cbk:.2f} ms'],
        colors=['C1', 'C0', 'C2'], autopct='%1.2f%%', startangle=90,
        wedgeprops=dict(edgecolor='white', linewidth=1.4),
        textprops={'fontsize': 11},
    )
    title(ax_pie, f'Average Breakdown  (Total {total:.2f} ms)')
    ax_pie.set_xlabel(
        'End-to-end image latency: sensor exposure (SOE) $\\rightarrow$ ROS publish',
        fontsize=11, labelpad=10)

    bar_labels = ['SOE→DQBUF', 'GStreamer', 'Callback→Pub', 'Total']
    bar_series = [
        np.array([soe_ms]),
        m['gst_dt_ms'],
        m['cbk_dt_ms'],
        full_ms,
    ]
    means = [s.mean() for s in bar_series]
    p99s  = [np.percentile(s, 99) for s in bar_series]
    maxs  = [s.max() for s in bar_series]

    y    = np.arange(len(bar_labels))[::-1]
    h    = 0.25
    cols = ['C1', 'C0', 'C2', 'C4']
    ax_bar.barh(y + h, means, h, color=cols)
    ax_bar.barh(y,     p99s,  h, color=cols, alpha=0.65, hatch='//')
    ax_bar.barh(y - h, maxs,  h, color=cols, alpha=0.45, hatch='..')
    for yy, v in zip(y + h, means): ax_bar.text(v, yy, f' {v:.2f}', va='center', fontsize=9)
    for yy, v in zip(y,     p99s ): ax_bar.text(v, yy, f' {v:.2f}', va='center', fontsize=9)
    for yy, v in zip(y - h, maxs ): ax_bar.text(v, yy, f' {v:.2f}', va='center', fontsize=9)
    ax_bar.set_yticks(y)
    ax_bar.set_yticklabels(bar_labels, fontsize=11)
    xlabel(ax_bar, 'Latency [ms]')
    title(ax_bar, 'Per-Metric Statistics')
    style(ax_bar)
    ax_bar.grid(True, which='major', axis='x', linestyle='-',
                linewidth=0.5, alpha=0.3, color='gray')
    legend_handles = [
        Patch(facecolor='white', edgecolor='black', linewidth=0.5, label='Mean'),
        Patch(facecolor='white', edgecolor='black', linewidth=0.5, hatch='//', label='P99'),
        Patch(facecolor='white', edgecolor='black', linewidth=0.5, hatch='..', label='Max'),
    ]
    ax_bar.legend(handles=legend_handles, loc='center right', fontsize=10,
                  frameon=True, framealpha=0.9, edgecolor='gray')
    savefig(fig, out_path)


def process_camera(arr, out_dir: Path, prefix='cam'):
    m = cam_metrics(arr)
    cam_plot_combined(arr, m, out_dir / f'{prefix}_pub.png')
    cam_plot_breakdown(m,     out_dir / f'{prefix}_breakdown.png')


# ────────────────────────────────────────────────────────────
# lidar / imu analysis
# ────────────────────────────────────────────────────────────

def pub_metrics(a: np.ndarray, label: str) -> dict:
    pub_dt_ms   = np.diff(a['t_pub_ns'].astype(np.int64))     / 1e6
    stamp_dt_ms = np.diff(a['header_stamp'].astype(np.int64)) / 1e6
    seq         = a['seq'].astype(np.int64)
    seq_diff    = np.diff(seq)
    gaps        = np.where(seq_diff > 1)[0]
    stale       = np.where(np.diff(a['header_stamp']) == 0)[0]

    dur_s = (int(a['t_pub_ns'][-1]) - int(a['t_pub_ns'][0])) / 1e9 if len(a) >= 2 else 0.0
    rate  = len(a) / dur_s if dur_s > 0 else 0.0

    print(f'[{label}]  records={len(a)}  rate={rate:.2f} Hz  gaps={len(gaps)}')

    return dict(pub_dt_ms=pub_dt_ms, stamp_dt_ms=stamp_dt_ms,
                seq=seq, seq_diff=seq_diff, gaps=gaps, stale=stale,
                rate=rate, label=label)


def pub_plot_combined(a: np.ndarray, m: dict, out_path: Path,
                      target_ms: float, tol_pct: float):
    t_axis        = (a['t_pub_ns'][1:].astype(np.int64) - int(a['t_pub_ns'][0])) / 1e9
    stamp_diff_ms = np.diff(a['header_stamp'].astype(np.int64)) / 1e6
    pct_stale     = 100.0 * len(m['stale']) / max(1, len(stamp_diff_ms))

    fig, axs = plt.subplots(3, 2, figsize=(12, 10), dpi=150)

    # ── left column ───────────────────────────────────────────────────────────
    d = m['pub_dt_ms']

    ax = axs[0, 0]
    ax.plot(t_axis, d, linewidth=1.5)
    ax.axhline(d.mean(), color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {d.mean():.2f} ms')
    ax.axhline(np.percentile(d, 99), color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(d, 99):.2f} ms')
    title(ax, f'{m["label"]}  Publish Period')
    ylabel(ax, 'Period [ms]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    ax = axs[1, 0]
    ax.hist(d, bins=40, color='C0', alpha=0.8, edgecolor='none')
    ax.axvline(d.mean(), color='black', linestyle=':', linewidth=1.0,
               label=f'Mean {d.mean():.2f} ms')
    ax.axvline(np.percentile(d, 99), color='black', linestyle='--', linewidth=1.0,
               label=f'P99 {np.percentile(d, 99):.2f} ms')
    title(ax, f'{m["label"]}  Publish Period Distribution')
    xlabel(ax, 'Period [ms]'); ylabel(ax, 'Count')
    style(ax); legend(ax)

    idx     = np.arange(len(a['header_stamp']))
    stamp_s = (a['header_stamp'].astype(np.int64) - int(a['header_stamp'][0])) / 1e9
    ax = axs[2, 0]
    ax.plot(idx, stamp_s, linewidth=1.5, color='C1')
    title(ax, f'{m["label"]}  Header Stamp vs Index')
    ylabel(ax, 'Stamp [sec]'); xlabel(ax, 'Index [Sample]')
    style(ax)

    # ── right column: seq & stamp health ─────────────────────────────────────
    gap_n  = len(m['gaps'])
    status = '[GOOD]' if gap_n == 0 else ('[WARN]' if gap_n < 100 else '[BAD]')

    ax = axs[0, 1]
    ax.plot(t_axis, m['seq_diff'], linewidth=1.5, color='C4')
    ax.axhline(1, color='black', linestyle=':', linewidth=1.0, label='Expected (=1)')
    ax.set_ylim(0, 2)
    title(ax, f'{m["label"]}  Drop Count = {gap_n}  {status}')
    ylabel(ax, 'Seq Diff'); xlabel(ax, 'Time [sec]')
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
    title(ax, f'{m["label"]}  Stamp Delta  Zoom +/-{tol_pct:g}%  '
              f'Stale {len(m["stale"])} ({pct_stale:.2f}%)')
    ylabel(ax, 'Stamp Delta [ms]'); xlabel(ax, 'Time [sec]')
    style(ax); legend(ax)

    ax = axs[2, 1]
    t_pub_all_ns  = a['t_pub_ns'].astype(np.int64)
    t_pub_diff_ms = np.diff(t_pub_all_ns) / 1e6
    ax.plot(t_axis, t_pub_diff_ms, linewidth=1.5, color='C3')
    title(ax, f'{m["label"]}  t_pub_ns Diff (MONOTONIC_RAW)')
    ylabel(ax, 'Pub Delta [ms]'); xlabel(ax, 'Time [sec]')
    style(ax)

    savefig(fig, out_path)


def process_pub(arr, label: str, prefix: str, out_dir: Path,
                target_ms: float, tol_pct: float):
    m = pub_metrics(arr, label)
    pub_plot_combined(arr, m, out_dir / f'{prefix}_pub.png',
                      target_ms=target_ms, tol_pct=tol_pct)


# ────────────────────────────────────────────────────────────
# main
# ────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description='Sensor publisher trace analyzer (camera + lidar + imu)')
    log_dir = Path(__file__).resolve().parent.parent / 'log'
    out_dir = Path(__file__).resolve().parent.parent / 'plots'

    p.add_argument('--resolution', default='sd',
                   choices=['sd', 'hd', 'wuxga', 'wuxga_mjpg'],
                   help='Camera resolution variant prefix (default: sd)')
    p.add_argument('--cam',     type=Path, default=None,
                   help='Override cam bin path')
    p.add_argument('--lidar',   type=Path, default=log_dir / 'lidar_pubrecord.bin')
    p.add_argument('--imu',     type=Path, default=log_dir / 'imu_pubrecord.bin')
    p.add_argument('--out-dir', type=Path, default=out_dir)
    args = p.parse_args()

    if args.cam is None:
        args.cam = log_dir / f'see3cam24cug_{args.resolution}_image_pub.bin'

    for sub in ('image', 'lidar', 'imu'):
        (args.out_dir / sub).mkdir(parents=True, exist_ok=True)

    cam   = load(args.cam,   'CAMERA', DT_IMG)
    lidar = load(args.lidar, 'LIDAR',  DT_PUB)
    imu   = load(args.imu,   'IMU',    DT_PUB)

    if cam is not None:
        process_camera(cam, args.out_dir / 'image', prefix='image')
    if lidar is not None:
        process_pub(lidar, 'LIDAR', 'lidar', args.out_dir / 'lidar',
                    target_ms=LIDAR_TARGET_MS, tol_pct=LIDAR_TOL_PCT)
    if imu is not None:
        process_pub(imu, 'IMU', 'imu', args.out_dir / 'imu',
                    target_ms=IMU_TARGET_MS, tol_pct=IMU_TOL_PCT)

    print(f'\nplots saved to {args.out_dir}')


if __name__ == '__main__':
    main()
