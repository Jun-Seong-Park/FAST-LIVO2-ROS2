#!/usr/bin/env python3
"""
Publisher vs Subscriber blackbox comparison.

Usage:
  python3 compare_pub_sub.py [--stream lidar|imu|image|all]
                              [--dir trace/log] [--out-dir trace/plots]

File pairs:
  lidar : lidar_pubrecord.bin (24B)  vs  lidar_subrecord.bin (24B)
  imu   : imu_pubrecord.bin   (24B)  vs  imu_subrecord.bin   (24B)
  image : see3cam24cug_sd_image_pub.bin (48B)  vs  image_subrecord.bin (24B)

Match key: header_stamp
Plots saved to: <out-dir>/{lidar,imu,image}/{stream}_compare.png
"""

import argparse
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ─── dtypes (blackbox.hpp ABI) ────────────────────────────────────────────────

PUB_24 = np.dtype([
    ("seq",          "<u8"),
    ("header_stamp", "<u8"),
    ("t_pub_ns",     "<u8"),
])

PUB_IMAGE_48 = np.dtype([
    ("seq",          "<u8"),
    ("header_stamp", "<u8"),
    ("t_dqbuf_ns",   "<u8"),
    ("t_cbk_ns",     "<u8"),
    ("t_pub_ns",     "<u8"),
    ("gst_done",     "u1"),
    ("ros_done",     "u1"),
    ("_pad",         "u1", (6,)),
])

SUB_24 = np.dtype([
    ("seq",          "<u8"),
    ("header_stamp", "<u8"),
    ("t_cbk_ns",     "<u8"),
])

STREAM_MAP = {
    "lidar": {
        "pub_file": "lidar_pubrecord.bin",
        "sub_file": "lidar_subrecord.bin",
        "pub_dtype": PUB_24,
        "pub_t_col": "t_pub_ns",
    },
    "imu": {
        "pub_file": "imu_pubrecord.bin",
        "sub_file": "imu_subrecord.bin",
        "pub_dtype": PUB_24,
        "pub_t_col": "t_pub_ns",
    },
    "image": {
        "pub_file": "see3cam24cug_sd_image_pub.bin",
        "sub_file": "image_subrecord.bin",
        "pub_dtype": PUB_IMAGE_48,
        "pub_t_col": "t_pub_ns",
    },
}

# stream → plots subfolder
STREAM_FOLDER = {"lidar": "lidar", "imu": "imu", "image": "image"}


# ─── loaders ─────────────────────────────────────────────────────────────────

def load(path: str, dtype: np.dtype) -> np.ndarray | None:
    if not os.path.exists(path):
        print(f'  [WARN] not found: {path}')
        return None
    size = os.path.getsize(path)
    n = size // dtype.itemsize
    if n == 0:
        return None
    arr = np.fromfile(path, dtype=dtype, count=n)
    arr = arr[arr['seq'] > 0]
    if len(arr) == 0:
        print(f'  [WARN] all seq=0: {path}')
        return None
    return arr


# ─── style helpers ────────────────────────────────────────────────────────────

def _style(ax):
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3, color='gray')
    ax.set_axisbelow(True)
    ax.spines[['top', 'right']].set_visible(False)


def _legend(ax):
    ax.legend(loc='upper right', fontsize=10, frameon=True,
              framealpha=0.9, edgecolor='gray')


def _savefig(fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)


# ─── rolling drop rate ───────────────────────────────────────────────────────

def rolling_drop_rate(t_pub_s: np.ndarray, t_match_s: np.ndarray,
                      window_s: float = 1.0):
    """Sliding window (50% overlap) drop rate. Returns (centers_s, rate_pct)."""
    if len(t_pub_s) == 0:
        return np.array([]), np.array([])
    step = window_s / 2
    centers = np.arange(t_pub_s[0] + window_s / 2, t_pub_s[-1], step)
    rates = []
    for tc in centers:
        lo, hi  = tc - window_s / 2, tc + window_s / 2
        n_pub   = int(np.sum((t_pub_s   >= lo) & (t_pub_s   < hi)))
        n_match = int(np.sum((t_match_s >= lo) & (t_match_s < hi)))
        rates.append((n_pub - n_match) / n_pub * 100 if n_pub > 0 else 0.0)
    return centers, np.array(rates)


# ─── plot ─────────────────────────────────────────────────────────────────────

def cmp_plot(stream: str, pub: np.ndarray, sub: np.ndarray,
             common: list, pub_set: dict, sub_set: dict,
             cfg: dict, out_path: Path) -> None:
    if not common:
        return

    t_pub_col = cfg["pub_t_col"]
    t0 = int(pub[t_pub_col][0])

    t_pub_all_s   = (pub[t_pub_col].astype(np.int64) - t0) / 1e9
    t_match_s     = np.array([(int(pub[t_pub_col][pub_set[s]]) - t0) / 1e9
                               for s in common])
    lat_ms        = np.array([(int(sub["t_cbk_ns"][sub_set[s]])
                                - int(pub[t_pub_col][pub_set[s]])) / 1e6
                               for s in common])
    t_first_sub_s = (int(sub["t_cbk_ns"][0])  - t0) / 1e9
    t_last_sub_s  = (int(sub["t_cbk_ns"][-1]) - t0) / 1e9

    win_s = 0.5 if stream == 'imu' else 1.0
    dr_centers, dr_rate = rolling_drop_rate(t_pub_all_s, t_match_s, window_s=win_s)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), dpi=150, sharex=True)
    for ax in (ax1, ax2):
        ax.tick_params(labelbottom=True)

    # ── top: latency line ─────────────────────────────────────────────────────
    ax1.plot(t_match_s, lat_ms, linewidth=0.8, color='C0',
             label=f'Latency  n={len(common):,}')
    ax1.axhline(lat_ms.mean(), color='black', linestyle=':', linewidth=1.0,
                label=f'Mean {lat_ms.mean():.1f} ms')
    ax1.axhline(np.percentile(lat_ms, 99), color='black', linestyle='--', linewidth=1.0,
                label=f'P99 {np.percentile(lat_ms, 99):.1f} ms')
    ax1.set_title(f'{stream.upper()}  Latency  (t_cbk_sub - t_pub)',
                  fontsize=14, fontweight='bold', loc='center', pad=12)
    ax1.set_ylabel('Latency [ms]', fontsize=12)
    _style(ax1); _legend(ax1)

    # ── bottom: rolling drop rate ─────────────────────────────────────────────
    if len(dr_centers):
        ax2.plot(dr_centers, dr_rate, linewidth=1.5, color='C0')
    ax2.axhline(0, color='black', linestyle=':', linewidth=0.8)
    ax2.set_ylim(bottom=0)
    ax2.set_title(f'{stream.upper()}  Drop Rate  (window = {win_s:g} s)',
                  fontsize=14, fontweight='bold', loc='center', pad=12)
    ax2.set_ylabel('Drop Rate [%]', fontsize=12)
    ax2.set_xlabel('Time [sec]', fontsize=12)
    _style(ax2)

    # ── x축 범위: FAST-LIVO2 active 구간으로 제한 ────────────────────────────
    ax2.set_xlim(t_first_sub_s, t_last_sub_s)

    _savefig(fig, out_path)
    print(f'  saved {out_path}')


# ─── compare ─────────────────────────────────────────────────────────────────

def compare(stream: str, log_dir: str, out_dir: Path) -> None:
    cfg      = STREAM_MAP[stream]
    pub_path = os.path.join(log_dir, cfg["pub_file"])
    sub_path = os.path.join(log_dir, cfg["sub_file"])

    pub = load(pub_path, cfg["pub_dtype"])
    sub = load(sub_path, SUB_24)
    if pub is None or sub is None:
        return

    pub_set = {int(s): i for i, s in enumerate(pub["header_stamp"])}
    sub_set = {int(s): i for i, s in enumerate(sub["header_stamp"])}

    common   = sorted(set(pub_set) & set(sub_set))
    pub_only = len(pub_set) - len(common)
    sub_only = len(sub_set) - len(common)
    drop_pct = pub_only / max(len(pub_set), 1) * 100

    print(f'[{stream.upper()}]  pub={len(pub_set):,}  matched={len(common):,}  '
          f'drop={pub_only:,} ({drop_pct:.1f}%)  sub_only={sub_only:,}')

    sub_dir = out_dir / STREAM_FOLDER[stream]
    sub_dir.mkdir(parents=True, exist_ok=True)
    cmp_plot(stream, pub, sub, common, pub_set, sub_set, cfg,
             sub_dir / f'{stream}_compare.png')


# ─── main ────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stream", choices=list(STREAM_MAP) + ["all"], default="all")
    default_log = os.path.join(os.environ.get("HOME", "/tmp"), "FAST-LIVO2-ROS2/trace/log")
    default_out = str(Path(__file__).resolve().parent.parent / 'plots')
    ap.add_argument("--dir",     default=default_log, help="blackbox log directory")
    ap.add_argument("--out-dir", default=default_out,  help="plot output directory")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    streams = list(STREAM_MAP) if args.stream == "all" else [args.stream]
    for s in streams:
        compare(s, args.dir, out_dir)

    print(f'\nplots saved to {out_dir}')


if __name__ == "__main__":
    main()
