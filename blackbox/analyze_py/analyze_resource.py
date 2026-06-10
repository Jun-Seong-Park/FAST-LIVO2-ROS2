#!/usr/bin/env python3
"""
Resource trace analyzer — splits every metric into its own .png.

Inputs (blackbox dump, 1 Hz):
  log/global.bin       — GlobalRecord (424B)
  log/image_proc.bin   — ProcRecord (48B)
  log/slam_proc.bin    — ProcRecord (48B)
  log/livox_proc.bin   — ProcRecord (48B)
  log/lidar_resource.bin — LidarResourceRecord (56B)

Outputs (plots/resource/, one png per metric):
  cpu0.png … cpu7.png            per-core CPU%
  cpu_freq0.png … cpu_freq7.png  per-core CPU freq
  ram.png                        memory used
  gpu_load.png  gpu_freq.png
  throttle_cpu0.png  throttle_cpu4.png  throttle_gpu.png
  net_<iface>.png                per iface rx+tx throughput
  wifi_link.png  wifi_level.png
  tcp_retrans.png
  proc_image_cpu.png  proc_image_rss.png
  proc_slam_cpu.png   proc_slam_rss.png
  proc_livox_cpu.png  proc_livox_rss.png
  lidar_core_temp.png  lidar_work_state.png  lidar_diag.png  lidar_hms.png

Usage:
  python3 analyze_resource.py [--log-dir DIR] [--out-dir DIR]
"""

import argparse
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


# ─── dtypes (blackbox.hpp ABI) ───────────────────────────────────────────────
DT_PROC = np.dtype([
    ('seq', '<u8'), ('t_sample_ns', '<u8'),
    ('pid', '<u4'), ('_pad', '<u4'),
    ('utime', '<u8'), ('stime', '<u8'), ('rss_bytes', '<u8'),
])

DT_NET = np.dtype([
    ('iface', 'S16'),
    ('rx_bytes', '<u8'), ('tx_bytes', '<u8'),
    ('rx_drop', '<u8'), ('tx_drop', '<u8'),
])

DT_GLOBAL = np.dtype([
    ('seq', '<u8'), ('t_sample_ns', '<u8'),
    ('cpu_total', '<u8', 8), ('cpu_idle', '<u8', 8),
    ('cpu_freq', '<u4', 8),
    ('mem_total_kb', '<u8'), ('mem_avail_kb', '<u8'),
    ('gpu_load', '<u4'), ('gpu_freq_hz', '<u4'),
    ('thr_cpu0', 'u1'), ('thr_cpu4', 'u1'), ('thr_gpu', 'u1'), ('_pad0', 'u1'),
    ('valid', '<u4'),
    ('net', DT_NET, 4),
    ('wifi_link', '<u4'), ('wifi_level', '<i4'),
    ('tcp_out', '<u8'), ('tcp_retrans', '<u8'),
])

DT_LIDAR_RES = np.dtype([
    ('seq', '<u8'), ('t_sample_ns', '<u8'),
    ('core_temp_centi', '<i4'),
    ('lidar_diag_status', '<u2'),
    ('cur_work_state', 'u1'), ('_pad0', 'u1'),
    ('hms_code', '<u4', 8),
])

V_CPU      = 1 << 0
V_MEM      = 1 << 1
V_CPUFREQ  = 1 << 2
V_GPU      = 1 << 3
V_THROTTLE = 1 << 4
V_NET      = 1 << 5
V_WIFI     = 1 << 6
V_TCP      = 1 << 7

assert DT_PROC.itemsize      == 48
assert DT_NET.itemsize       == 48
assert DT_GLOBAL.itemsize    == 424
assert DT_LIDAR_RES.itemsize == 56

CLK_TCK = 100


# ─── style (plot-style skill) ────────────────────────────────────────────────
def style(ax):
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3, color='gray')
    ax.set_axisbelow(True)
    ax.spines[['top', 'right']].set_visible(False)


def legend(ax):
    ax.legend(loc='upper right', fontsize=10, frameon=True,
              framealpha=0.9, edgecolor='gray')


def new_fig():
    return plt.subplots(figsize=(10, 4), dpi=150)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f'  saved {path}')


def load_bin(path, dtype):
    if not path.is_file():
        return None
    arr = np.fromfile(path, dtype=dtype)
    if arr.size == 0:
        return None
    return arr[arr['seq'] > 0]


def t_seconds(ns):
    return (ns.astype(np.int64) - np.int64(ns[0])) / 1e9


# ─── per-core CPU usage (single 4x2 grid png) ───────────────────────────────
def plot_cpu_usage(g, out_dir):
    if g is None or g.size < 2 or not (g['valid'] & V_CPU).all():
        return
    t = t_seconds(g['t_sample_ns'])
    dt_total = np.diff(g['cpu_total'], axis=0).astype(np.float64)
    dt_idle  = np.diff(g['cpu_idle'],  axis=0).astype(np.float64)
    with np.errstate(divide='ignore', invalid='ignore'):
        cpu_pct = np.where(dt_total > 0, 100.0 * (dt_total - dt_idle) / dt_total, 0.0)

    fig, axs = plt.subplots(4, 2, figsize=(14, 10), dpi=150, sharex=True, sharey=True)
    fig.suptitle('CPU Utilization (per core)', fontsize=15, fontweight='bold')
    for c in range(8):
        ax = axs[c // 2, c % 2]
        style(ax)
        ax.plot(t[1:], cpu_pct[:, c], linewidth=1.5, label=f'cpu{c}')
        ax.set_ylim(0, 110)
        ax.tick_params(labelbottom=True, labelleft=True)
        legend(ax)
        if c // 2 == 3:
            ax.set_xlabel('Time [sec]', fontsize=12)
        if c % 2 == 0:
            ax.set_ylabel('CPU [%]', fontsize=12)
    save(fig, out_dir / 'cpu_usage.png')


def _cpu_max_freq_khz(core):
    p = Path(f'/sys/devices/system/cpu/cpu{core}/cpufreq/cpuinfo_max_freq')
    if not p.is_file():
        return None
    try:
        return int(p.read_text().strip())
    except (ValueError, OSError):
        return None


def plot_cpu_freq(g, out_dir):
    if g is None or g.size < 1 or not (g['valid'] & V_CPUFREQ).any():
        return
    t = t_seconds(g['t_sample_ns'])
    fig, axs = plt.subplots(4, 2, figsize=(14, 10), dpi=150, sharex=True, sharey=True)
    fig.suptitle('CPU Frequency (per core)', fontsize=15, fontweight='bold')
    max_mhz_all = 0.0
    for c in range(8):
        ax = axs[c // 2, c % 2]
        style(ax)
        ax.plot(t, g['cpu_freq'][:, c] / 1000.0, linewidth=1.5, label=f'cpu{c}')
        max_khz = _cpu_max_freq_khz(c)
        if max_khz is not None:
            max_mhz = max_khz / 1000.0
            ax.axhline(max_mhz, color='C7', linestyle='--', linewidth=1.0,
                       label=f'max {max_mhz:.0f} MHz')
            max_mhz_all = max(max_mhz_all, max_mhz)
        ax.tick_params(labelbottom=True, labelleft=True)
        legend(ax)
        if c // 2 == 3:
            ax.set_xlabel('Time [sec]', fontsize=12)
        if c % 2 == 0:
            ax.set_ylabel('Frequency [MHz]', fontsize=12)
    if max_mhz_all > 0:
        axs[0, 0].set_ylim(0, max_mhz_all * 1.05)
    save(fig, out_dir / 'cpu_freq.png')


# ─── RAM (with total capacity line + peak annotation) ───────────────────────
def plot_ram(g, out_dir):
    if g is None or g.size < 1 or not (g['valid'] & V_MEM).any():
        return
    t = t_seconds(g['t_sample_ns'])
    used_mb  = (g['mem_total_kb'] - g['mem_avail_kb']) / 1024.0
    total_mb = g['mem_total_kb'][-1] / 1024.0
    peak_mb  = float(used_mb.max())
    peak_t   = float(t[int(used_mb.argmax())])

    fig, ax = new_fig()
    style(ax)
    ax.plot(t, used_mb, linewidth=1.5, label='RAM used')
    ax.axhline(total_mb, color='C7', linestyle='--', linewidth=1.0,
               label=f'total = {total_mb:.0f} MB')
    ax.axhline(peak_mb, color='C3', linestyle=':', linewidth=1.0,
               label=f'peak = {peak_mb:.0f} MB')
    ax.annotate(f'peak {peak_mb:.0f} MB',
                xy=(peak_t, peak_mb),
                xytext=(8, 8), textcoords='offset points',
                fontsize=10, color='C3',
                arrowprops=dict(arrowstyle='->', color='C3', lw=0.8))
    ax.set_title('Memory Used', fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    ax.set_ylabel('RAM [MB]', fontsize=12)
    ax.set_ylim(0, total_mb * 1.05)
    legend(ax)
    save(fig, out_dir / 'ram.png')


# ─── GPU (load + freq in one 2x1 png) ────────────────────────────────────────
def _gpu_max_freq_hz():
    p = Path('/sys/class/devfreq/17000000.gpu/max_freq')
    if not p.is_file():
        return None
    try:
        return int(p.read_text().strip())
    except (ValueError, OSError):
        return None


def plot_gpu(g, out_dir):
    if g is None or g.size < 1 or not (g['valid'] & V_GPU).any():
        return
    t = t_seconds(g['t_sample_ns'])
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), dpi=150, sharex=True)
    fig.suptitle('GPU (utilization + frequency)', fontsize=15, fontweight='bold')
    for ax in axs:
        style(ax)
        ax.tick_params(labelbottom=True)

    axs[0].plot(t, g['gpu_load'] / 10.0, linewidth=1.5, label='GPU load')
    axs[0].set_ylabel('GPU load [%]', fontsize=12)
    axs[0].set_ylim(0, 105)
    legend(axs[0])

    axs[1].plot(t, g['gpu_freq_hz'] / 1e6, linewidth=1.5, label='GPU freq', color='C1')
    max_hz = _gpu_max_freq_hz()
    if max_hz is not None:
        max_mhz = max_hz / 1e6
        axs[1].axhline(max_mhz, color='C7', linestyle='--', linewidth=1.0,
                       label=f'max {max_mhz:.0f} MHz')
        axs[1].set_ylim(0, max_mhz * 1.05)
    axs[1].set_ylabel('Frequency [MHz]', fontsize=12)
    axs[1].set_xlabel('Time [sec]', fontsize=12)
    legend(axs[1])

    save(fig, out_dir / 'gpu.png')


# ─── throttle (cluster0 + cluster1 + gpu in one 3x1 png) ────────────────────
def plot_throttle(g, out_dir):
    if g is None or g.size < 1 or not (g['valid'] & V_THROTTLE).any():
        return
    t = t_seconds(g['t_sample_ns'])
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), dpi=150, sharex=True)
    fig.suptitle('Throttle (cluster0 cpu0-3 / cluster1 cpu4-7 / gpu)',
                 fontsize=15, fontweight='bold')
    for ax in axs:
        style(ax)
        ax.tick_params(labelbottom=True)

    rows = [
        (axs[0], 'thr_cpu0', 25, 'cluster0 (cpu0-3)'),
        (axs[1], 'thr_cpu4', 25, 'cluster1 (cpu4-7)'),
        (axs[2], 'thr_gpu',   6, 'gpu'),
    ]
    for ax, field, ymax, label in rows:
        ax.plot(t, g[field], linewidth=1.5, label=label)
        ax.set_ylabel('Throttle [step]', fontsize=12)
        ax.set_ylim(-0.5, ymax + 0.5)
        legend(ax)
    axs[-1].set_xlabel('Time [sec]', fontsize=12)
    save(fig, out_dir / 'throttle.png')


# ─── network (4 iface 2x2 grid, single png) ─────────────────────────────────
def plot_net(g, out_dir):
    if g is None or g.size < 2 or not (g['valid'] & V_NET).all():
        return
    t    = t_seconds(g['t_sample_ns'])
    dt_s = np.diff(g['t_sample_ns'].astype(np.int64)) / 1e9

    fig, axs = plt.subplots(2, 2, figsize=(14, 8), dpi=150, sharex=True)
    fig.suptitle('Network Throughput (per iface)', fontsize=15, fontweight='bold')
    for s in range(4):
        ax = axs[s // 2, s % 2]
        style(ax)
        ax.tick_params(labelbottom=True, labelleft=True)
        iface = g['net'][0, s]['iface'].decode('ascii', errors='ignore').rstrip('\x00')
        if not iface:
            ax.set_title('(empty slot)', fontsize=12, pad=8)
            continue
        rx = np.diff(g['net'][:, s]['rx_bytes'].astype(np.float64))
        tx = np.diff(g['net'][:, s]['tx_bytes'].astype(np.float64))
        with np.errstate(divide='ignore', invalid='ignore'):
            rx_mbs = np.where(dt_s > 0, rx / dt_s / 1e6, 0.0)
            tx_mbs = np.where(dt_s > 0, tx / dt_s / 1e6, 0.0)
        ax.plot(t[1:], rx_mbs, linewidth=1.5, label='rx')
        ax.plot(t[1:], tx_mbs, linewidth=1.5, label='tx')
        ax.set_title(iface, fontsize=12, pad=8)
        if s // 2 == 1:
            ax.set_xlabel('Time [sec]', fontsize=12)
        if s % 2 == 0:
            ax.set_ylabel('Throughput [MB/sec]', fontsize=12)
        legend(ax)
    save(fig, out_dir / 'net.png')


# ─── wifi + tcp (link + level + tcp retrans in one 3x1 png) ─────────────────
def plot_wifi(g, out_dir):
    if g is None or g.size < 1:
        return
    if not ((g['valid'] & V_WIFI).any() or (g['valid'] & V_TCP).any()):
        return
    t = t_seconds(g['t_sample_ns'])
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), dpi=150, sharex=True)
    fig.suptitle('WiFi + TCP (link quality / signal level / retrans rate)',
                 fontsize=15, fontweight='bold')
    for ax in axs:
        style(ax)
        ax.tick_params(labelbottom=True)

    axs[0].plot(t, g['wifi_link'], linewidth=1.5, label='link quality')
    axs[0].set_ylabel('Quality [score]', fontsize=12)
    legend(axs[0])

    axs[1].plot(t, g['wifi_level'], linewidth=1.5, label='signal level', color='C1')
    axs[1].set_ylabel('Level [dBm]', fontsize=12)
    legend(axs[1])

    if g.size >= 2 and (g['valid'] & V_TCP).any():
        d_out = np.diff(g['tcp_out'].astype(np.float64))
        d_ret = np.diff(g['tcp_retrans'].astype(np.float64))
        with np.errstate(divide='ignore', invalid='ignore'):
            ret_pct = np.where(d_out > 0, 100.0 * d_ret / d_out, 0.0)
        axs[2].plot(t[1:], ret_pct, linewidth=1.5, label='TCP retrans', color='C3')
    axs[2].set_ylabel('Retrans [%]', fontsize=12)
    axs[2].set_xlabel('Time [sec]', fontsize=12)
    legend(axs[2])

    save(fig, out_dir / 'wifi.png')


# ─── per-process CPU + RSS + etc row (3 process + etc) × 2 metric grid ─────
def plot_proc(procs, g, out_dir):
    """
    procs: dict {label: ndarray(DT_PROC) or None}, ordered.
    g    : GlobalRecord ndarray (for etc row: etc = global busy - tracked sum).
    """
    names = list(procs.keys()) + ['etc']
    n = len(names)
    fig, axs = plt.subplots(n, 2, figsize=(14, 3 * n), dpi=150, sharex=True)
    fig.suptitle('Per-Process Resources (CPU left, RSS right)',
                 fontsize=15, fontweight='bold')

    for row, name in enumerate(names):
        ax_cpu = axs[row, 0]
        ax_rss = axs[row, 1]
        style(ax_cpu)
        style(ax_rss)
        ax_cpu.tick_params(labelbottom=True, labelleft=True)
        ax_rss.tick_params(labelbottom=True, labelleft=True)
        ax_cpu.set_ylabel('CPU [%]', fontsize=12)
        ax_rss.set_ylabel('RSS [MB]', fontsize=12)

        if name == 'etc':
            # etc CPU% = global busy - sum(tracked) — same t_sample tick.
            # RSS for etc is omitted (RSS sum across procs double-counts shared memory).
            if g is None or g.size < 2:
                ax_cpu.text(0.5, 0.5, 'etc: global.bin missing',
                            ha='center', va='center', transform=ax_cpu.transAxes)
            else:
                t_g  = t_seconds(g['t_sample_ns'])
                dt_g = np.diff(g['t_sample_ns'].astype(np.int64)) / 1e9
                d_total = np.diff(g['cpu_total'].sum(axis=1).astype(np.int64))
                d_idle  = np.diff(g['cpu_idle'].sum(axis=1).astype(np.int64))
                busy_jif = (d_total - d_idle).astype(np.float64)

                # sum tracked Δ(utime+stime) aligned to global t_sample pairs
                tracked_jif = np.zeros_like(busy_jif)
                for p in procs.values():
                    if p is None or p.size < 2:
                        continue
                    cpu_raw = (p['utime'] + p['stime']).astype(np.int64)
                    p_map = dict(zip(p['t_sample_ns'].tolist(), cpu_raw.tolist()))
                    for i in range(len(busy_jif)):
                        ts_a = int(g['t_sample_ns'][i])
                        ts_b = int(g['t_sample_ns'][i + 1])
                        a = p_map.get(ts_a)
                        b = p_map.get(ts_b)
                        if a is not None and b is not None and b >= a:
                            tracked_jif[i] += (b - a)

                etc_jif = busy_jif - tracked_jif
                with np.errstate(divide='ignore', invalid='ignore'):
                    etc_pct = np.where(dt_g > 0, (etc_jif / CLK_TCK) / dt_g * 100, np.nan)
                ax_cpu.plot(t_g[1:], etc_pct, linewidth=1.5, label='etc CPU', color='C3')

            ax_rss.text(0.5, 0.5, 'etc: N/A (RSS sum double-counts shared mem)',
                        ha='center', va='center', transform=ax_rss.transAxes)
        else:
            p = procs[name]
            if p is None or p.size < 2:
                ax_cpu.text(0.5, 0.5, f'{name}: no data',
                            ha='center', va='center', transform=ax_cpu.transAxes)
                ax_rss.text(0.5, 0.5, f'{name}: no data',
                            ha='center', va='center', transform=ax_rss.transAxes)
            else:
                t     = t_seconds(p['t_sample_ns'])
                dt_s  = np.diff(p['t_sample_ns'].astype(np.int64)) / 1e9
                d_cpu = np.diff((p['utime'] + p['stime']).astype(np.int64))
                with np.errstate(divide='ignore', invalid='ignore'):
                    cpu_pct = np.where((dt_s > 0) & (d_cpu >= 0),
                                       100.0 * (d_cpu / CLK_TCK) / dt_s, np.nan)
                rss_mb = p['rss_bytes'] / (1024.0 * 1024.0)
                ax_cpu.plot(t[1:], cpu_pct, linewidth=1.5, label=f'{name} CPU')
                ax_rss.plot(t,     rss_mb,  linewidth=1.5, label=f'{name} RSS')

        legend(ax_cpu)
        legend(ax_rss)
        if row == n - 1:
            ax_cpu.set_xlabel('Time [sec]', fontsize=12)
            ax_rss.set_xlabel('Time [sec]', fontsize=12)
    save(fig, out_dir / 'proc.png')


# ─── LiDAR device resource ──────────────────────────────────────────────────
def plot_lidar_resource(res, out_dir):
    if res is None or res.size < 1:
        return
    t = t_seconds(res['t_sample_ns'])

    fig, ax = new_fig()
    style(ax)
    ax.plot(t, res['core_temp_centi'] / 100.0, linewidth=1.5, label='core_temp')
    ax.set_title('LiDAR Core Temperature', fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    ax.set_ylabel('Temperature [degC]', fontsize=12)
    legend(ax)
    save(fig, out_dir / 'lidar_core_temp.png')

    fig, ax = new_fig()
    style(ax)
    ax.step(t, res['cur_work_state'], linewidth=1.5, where='post', label='work_state')
    ax.set_title('LiDAR Work State', fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    ax.set_ylabel('Work State [enum]', fontsize=12)
    ax.set_ylim(-0.5, 6.5)
    legend(ax)
    save(fig, out_dir / 'lidar_work_state.png')

    fig, ax = new_fig()
    style(ax)
    diag = res['lidar_diag_status']
    for name, sh in [('system', 0), ('scan', 1), ('ranging', 2), ('comm', 3)]:
        nib = (diag >> (4 * sh)) & 0xF
        ax.plot(t, nib, linewidth=1.2, label=name)
    ax.set_title('LiDAR Module Diagnostic Status', fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    ax.set_ylabel('Status [code]', fontsize=12)
    ax.set_ylim(-0.5, 4.0)
    legend(ax)
    save(fig, out_dir / 'lidar_diag.png')

    fig, ax = new_fig()
    style(ax)
    hms = res['hms_code']
    nonzero_mask = (hms != 0).any(axis=1)
    if nonzero_mask.any():
        ax.scatter(t[nonzero_mask], np.full(nonzero_mask.sum(), 1),
                   marker='x', s=30, label='HMS fault event')
    ax.set_title('LiDAR HMS Faults', fontsize=14, fontweight='bold', pad=12)
    ax.set_xlabel('Time [sec]', fontsize=12)
    ax.set_ylabel('Event [marker]', fontsize=12)
    ax.set_ylim(-0.5, 2.0)
    legend(ax)
    save(fig, out_dir / 'lidar_hms.png')


# ─── main ───────────────────────────────────────────────────────────────────
def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description='blackbox resource plot (one png per metric)')
    ap.add_argument('--log-dir', type=Path,
                    default=here.parent / 'log')
    ap.add_argument('--out-dir', type=Path,
                    default=here.parent / 'plots' / 'resource')
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(f'[resource] log_dir = {args.log_dir}')
    print(f'[resource] out_dir = {args.out_dir}')

    g          = load_bin(args.log_dir / 'global.bin',         DT_GLOBAL)
    proc_image = load_bin(args.log_dir / 'image_proc.bin',     DT_PROC)
    proc_slam  = load_bin(args.log_dir / 'slam_proc.bin',      DT_PROC)
    proc_livox = load_bin(args.log_dir / 'livox_proc.bin',     DT_PROC)
    lidar_res  = load_bin(args.log_dir / 'lidar_resource.bin', DT_LIDAR_RES)

    plot_cpu_usage(g, args.out_dir)
    plot_cpu_freq (g, args.out_dir)
    plot_ram             (g, args.out_dir)
    plot_gpu             (g, args.out_dir)
    plot_throttle        (g, args.out_dir)
    plot_net             (g, args.out_dir)
    plot_wifi            (g, args.out_dir)
    plot_proc({'image': proc_image, 'slam': proc_slam, 'livox': proc_livox},
              g, args.out_dir)
    plot_lidar_resource(lidar_res, args.out_dir)

    print(f'[resource] done.')


if __name__ == '__main__':
    main()
