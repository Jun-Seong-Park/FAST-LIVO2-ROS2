#!/usr/bin/env python3
"""
FAST-LIVO2 pub/sub trace analyzer.

Reads binary traces written by:
  publisher:
    /home/aluxorin22/trace/log/pub_trace_lidar.bin
    /home/aluxorin22/trace/log/pub_trace_imu.bin
    /home/aluxorin22/trace/log/pub_trace_camera.bin
    /home/aluxorin22/trace/log/see3cam_drop_trace.bin
  subscriber:
    /home/aluxorin22/trace/log/sub_trace_lidar.bin
    /home/aluxorin22/trace/log/sub_trace_imu.bin
    /home/aluxorin22/trace/log/sub_trace_image.bin

The goal is to answer:
  - Did the publisher produce samples at the expected rate?
  - Did FAST-LIVO2 subscriber callbacks receive the same stamps?
  - If not, where are the gaps and how large are they?
  - What is pub->sub callback latency on the same machine?
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, pstdev


PUB_FMT = struct.Struct("<QQ")
SUB_FMT = struct.Struct("<QQQ")
CAM_PUB_FMT = struct.Struct("<QQQQQQQQ")
CAM_DROP_FMT = struct.Struct("<QQQQQQQQQQQQQII")


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    mono_ns: int
    index: int


@dataclass(frozen=True)
class CameraPubSample:
    stamp_ns: int
    pts_ns: int
    t0_enter_ns: int
    t1_pull_ns: int
    t2_map_ns: int
    t3_cvt_ns: int
    t4_mcpy_ns: int
    t5_pub_ns: int

    @property
    def mono_ns(self) -> int:
        return self.t5_pub_ns


def ns_to_s(ns: int) -> float:
    return ns / 1e9


def ns_to_ms(ns: int) -> float:
    return ns / 1e6


def pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    xs = sorted(values)
    k = (len(xs) - 1) * p / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return xs[int(k)]
    return xs[lo] * (hi - k) + xs[hi] * (k - lo)


def stats(values: list[float]) -> str:
    if not values:
        return "n/a"
    return (
        f"mean={mean(values):.3f} std={pstdev(values):.3f} "
        f"p50={pct(values, 50):.3f} p95={pct(values, 95):.3f} "
        f"p99={pct(values, 99):.3f} min={min(values):.3f} max={max(values):.3f}"
    )


def read_records(path: Path, fmt: struct.Struct) -> list[tuple[int, ...]]:
    if not path.exists():
        return []
    data = path.read_bytes()
    usable = len(data) - (len(data) % fmt.size)
    if usable != len(data):
        print(f"[warn] {path}: ignoring trailing {len(data) - usable} bytes")
    return [fmt.unpack_from(data, off) for off in range(0, usable, fmt.size)]


def read_pub(path: Path) -> list[Sample]:
    return [Sample(stamp, mono, i) for i, (stamp, mono) in enumerate(read_records(path, PUB_FMT))]


def read_sub(path: Path) -> list[Sample]:
    out = []
    for fallback_i, (stamp, mono, cb_index) in enumerate(read_records(path, SUB_FMT)):
        out.append(Sample(stamp, mono, int(cb_index) if cb_index <= 2**63 else fallback_i))
    return out


def read_camera_pub(path: Path) -> list[Sample]:
    out = []
    for i, rec in enumerate(read_records(path, CAM_PUB_FMT)):
        c = CameraPubSample(*rec)
        out.append(Sample(c.stamp_ns, c.mono_ns, i))
    return out


def read_camera_drop(path: Path) -> list[tuple[int, ...]]:
    return read_records(path, CAM_DROP_FMT)


def dts_ms(samples: list[Sample], field: str) -> list[float]:
    if len(samples) < 2:
        return []
    vals = [getattr(s, field) for s in samples]
    return [ns_to_ms(vals[i] - vals[i - 1]) for i in range(1, len(vals))]


def duration_s(samples: list[Sample]) -> float:
    if len(samples) < 2:
        return 0.0
    return ns_to_s(samples[-1].mono_ns - samples[0].mono_ns)


def print_stream(name: str, samples: list[Sample], expected_ms: float | None, gap_factor: float) -> None:
    print(f"\n[{name}]")
    if not samples:
        print("  missing or empty")
        return
    dur = duration_s(samples)
    rate = (len(samples) - 1) / dur if dur > 0 else 0.0
    print(f"  count={len(samples)} duration={dur:.3f}s rate={rate:.3f}Hz")
    print(f"  stamp_dt_ms: {stats(dts_ms(samples, 'stamp_ns'))}")
    print(f"  mono_dt_ms : {stats(dts_ms(samples, 'mono_ns'))}")

    if expected_ms is not None:
        gaps = find_gaps(samples, expected_ms, gap_factor)
        print(f"  gaps>{expected_ms * gap_factor:.3f}ms: {len(gaps)}")
        for g in gaps[:8]:
            i, dt_ms, prev_stamp, stamp = g
            print(f"    i={i} dt={dt_ms:.3f}ms prev={ns_to_s(prev_stamp):.9f} stamp={ns_to_s(stamp):.9f}")
        if len(gaps) > 8:
            print(f"    ... {len(gaps) - 8} more")


def find_gaps(samples: list[Sample], expected_ms: float, factor: float) -> list[tuple[int, float, int, int]]:
    out = []
    threshold = expected_ms * factor
    for i in range(1, len(samples)):
        dt = ns_to_ms(samples[i].stamp_ns - samples[i - 1].stamp_ns)
        if dt < -1.0 or dt > threshold:
            out.append((i, dt, samples[i - 1].stamp_ns, samples[i].stamp_ns))
    return out


def match_pub_sub(pub: list[Sample], sub: list[Sample], name: str) -> tuple[list[float], int, int]:
    if not pub or not sub:
        print(f"\n[{name} pub->sub] missing")
        return [], 0, 0

    pub_by_stamp = {}
    dup_pub = 0
    for p in pub:
        if p.stamp_ns in pub_by_stamp:
            dup_pub += 1
            continue
        pub_by_stamp[p.stamp_ns] = p

    sub_counter = Counter(s.stamp_ns for s in sub)
    matched = []
    missing_sub = 0
    for p in pub:
        if sub_counter[p.stamp_ns] <= 0:
            missing_sub += 1
        else:
            sub_counter[p.stamp_ns] -= 1

    missing_pub = 0
    for s in sub:
        p = pub_by_stamp.get(s.stamp_ns)
        if p is None:
            missing_pub += 1
            continue
        matched.append(ns_to_ms(s.mono_ns - p.mono_ns))

    loss_pct = 100.0 * missing_sub / len(pub) if pub else 0.0
    print(f"\n[{name} pub->sub]")
    print(f"  pub={len(pub)} sub={len(sub)} matched={len(matched)}")
    print(f"  pub_without_sub={missing_sub} ({loss_pct:.3f}% of pub)")
    print(f"  sub_without_pub={missing_pub} duplicate_pub_stamps={dup_pub}")
    print(f"  latency_ms: {stats(matched)}")
    print_stamp_overlap(pub, sub)
    print_nearest_stamp_delta(pub, sub)
    match_pub_sub_overlap(pub, sub, name)
    return matched, missing_sub, missing_pub


def match_pub_sub_overlap(pub: list[Sample], sub: list[Sample], name: str) -> None:
    pub_min = min(s.stamp_ns for s in pub)
    pub_max = max(s.stamp_ns for s in pub)
    sub_min = min(s.stamp_ns for s in sub)
    sub_max = max(s.stamp_ns for s in sub)
    lo = max(pub_min, sub_min)
    hi = min(pub_max, sub_max)
    if hi < lo:
        print("  overlap_match: n/a (no stamp overlap)")
        return

    pub_o = [s for s in pub if lo <= s.stamp_ns <= hi]
    sub_o = [s for s in sub if lo <= s.stamp_ns <= hi]
    if not pub_o or not sub_o:
        print("  overlap_match: n/a (empty overlap after filtering)")
        return

    sub_counter = Counter(s.stamp_ns for s in sub_o)
    missing_sub = 0
    for p in pub_o:
        if sub_counter[p.stamp_ns] <= 0:
            missing_sub += 1
        else:
            sub_counter[p.stamp_ns] -= 1

    pub_by_stamp = {}
    for p in pub_o:
        pub_by_stamp.setdefault(p.stamp_ns, p)
    matched = []
    sub_without_pub = 0
    for s in sub_o:
        p = pub_by_stamp.get(s.stamp_ns)
        if p is None:
            sub_without_pub += 1
        else:
            matched.append(ns_to_ms(s.mono_ns - p.mono_ns))

    loss_pct = 100.0 * missing_sub / len(pub_o)
    print(
        f"  overlap_match: pub={len(pub_o)} sub={len(sub_o)} matched={len(matched)} "
        f"pub_without_sub={missing_sub} ({loss_pct:.3f}%) sub_without_pub={sub_without_pub}"
    )
    print(f"  overlap_latency_ms: {stats(matched)}")


def print_stamp_overlap(pub: list[Sample], sub: list[Sample]) -> None:
    pub_min = min(s.stamp_ns for s in pub)
    pub_max = max(s.stamp_ns for s in pub)
    sub_min = min(s.stamp_ns for s in sub)
    sub_max = max(s.stamp_ns for s in sub)
    overlap_min = max(pub_min, sub_min)
    overlap_max = min(pub_max, sub_max)
    overlap_s = ns_to_s(overlap_max - overlap_min) if overlap_max >= overlap_min else 0.0
    print(
        "  stamp_range_s: "
        f"pub=[{ns_to_s(pub_min):.9f}, {ns_to_s(pub_max):.9f}] "
        f"sub=[{ns_to_s(sub_min):.9f}, {ns_to_s(sub_max):.9f}] "
        f"overlap={overlap_s:.3f}s"
    )
    if overlap_s <= 0.0:
        print("  note: pub/sub traces are from different time windows or different runs")


def nearest_delta_ms(stamps: list[int], targets: list[int]) -> list[float]:
    if not stamps or not targets:
        return []
    targets_sorted = sorted(targets)
    out = []
    for stamp in stamps:
        lo = 0
        hi = len(targets_sorted)
        while lo < hi:
            mid = (lo + hi) // 2
            if targets_sorted[mid] < stamp:
                lo = mid + 1
            else:
                hi = mid
        candidates = []
        if lo < len(targets_sorted):
            candidates.append(abs(targets_sorted[lo] - stamp))
        if lo > 0:
            candidates.append(abs(targets_sorted[lo - 1] - stamp))
        if candidates:
            out.append(ns_to_ms(min(candidates)))
    return out


def print_nearest_stamp_delta(pub: list[Sample], sub: list[Sample]) -> None:
    sub_stamps = [s.stamp_ns for s in sub]
    # Sampling keeps huge IMU traces readable without changing qualitative diagnosis.
    step = max(1, len(pub) // 5000)
    sampled_pub = [s.stamp_ns for s in pub[::step]]
    deltas = nearest_delta_ms(sampled_pub, sub_stamps)
    print(f"  nearest_pub_to_sub_stamp_delta_ms: {stats(deltas)}")


def write_gaps_csv(out_dir: Path, name: str, samples: list[Sample], expected_ms: float, factor: float) -> None:
    gaps = find_gaps(samples, expected_ms, factor)
    if not gaps:
        return
    path = out_dir / f"{name}_gaps.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "dt_ms", "prev_stamp_s", "stamp_s"])
        for i, dt_ms, prev_stamp, stamp in gaps:
            w.writerow([i, f"{dt_ms:.6f}", f"{ns_to_s(prev_stamp):.9f}", f"{ns_to_s(stamp):.9f}"])


def summarize_camera_drop(records: list[tuple[int, ...]]) -> None:
    print("\n[camera drop trace]")
    if not records:
        print("  missing or empty")
        return
    last = records[-1]
    # DropSample layout:
    # frame, stamp, pts, offset, offset_end, duration, t0, t5,
    # stamped, fallback, partial, pool_drop, publish_throw, gst_flags, event_flags
    print(f"  records={len(records)}")
    print(f"  last_frame={last[0]} stamped={last[8]} fallback={last[9]} partial={last[10]} "
          f"pool_drop={last[11]} publish_throw={last[12]}")
    events = Counter(r[14] for r in records)
    interesting = {k: v for k, v in events.items() if k != 1}
    print(f"  non-normal event_flags counts={interesting if interesting else '{}'}")


def load_all(args: argparse.Namespace) -> dict[str, list[Sample]]:
    trace_dir = args.trace_dir
    camera_dir = args.camera_trace_dir
    return {
        "pub_lidar": read_pub(trace_dir / "pub_trace_lidar.bin"),
        "pub_imu": read_pub(trace_dir / "pub_trace_imu.bin"),
        "pub_image": read_camera_pub(camera_dir / "pub_trace_camera.bin"),
        "sub_lidar": read_sub(trace_dir / "sub_trace_lidar.bin"),
        "sub_imu": read_sub(trace_dir / "sub_trace_imu.bin"),
        "sub_image": read_sub(trace_dir / "sub_trace_image.bin"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze FAST-LIVO2 publisher/subscriber traces")
    parser.add_argument("--trace-dir", type=Path, default=Path("/home/aluxorin22/trace/log"))
    parser.add_argument("--camera-trace-dir", type=Path, default=Path("/home/aluxorin22/trace/log"))
    parser.add_argument("--out-dir", type=Path, default=Path("/home/aluxorin22/trace/analysis"))
    parser.add_argument("--lidar-ms", type=float, default=100.0, help="expected lidar period in ms")
    parser.add_argument("--imu-ms", type=float, default=5.0, help="expected imu period in ms")
    parser.add_argument("--image-ms", type=float, default=100.0, help="expected image period in ms")
    parser.add_argument("--gap-factor", type=float, default=1.5)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    streams = load_all(args)

    print("=" * 92)
    print("FAST-LIVO2 trace analysis")
    print(f"trace_dir={args.trace_dir}")
    print(f"camera_trace_dir={args.camera_trace_dir}")
    print(f"out_dir={args.out_dir}")
    print("=" * 92)

    print_stream("publisher lidar", streams["pub_lidar"], args.lidar_ms, args.gap_factor)
    print_stream("subscriber lidar", streams["sub_lidar"], args.lidar_ms, args.gap_factor)
    match_pub_sub(streams["pub_lidar"], streams["sub_lidar"], "lidar")

    print_stream("publisher imu", streams["pub_imu"], args.imu_ms, args.gap_factor)
    print_stream("subscriber imu", streams["sub_imu"], args.imu_ms, args.gap_factor)
    match_pub_sub(streams["pub_imu"], streams["sub_imu"], "imu")

    print_stream("publisher image", streams["pub_image"], args.image_ms, args.gap_factor)
    print_stream("subscriber image", streams["sub_image"], args.image_ms, args.gap_factor)
    match_pub_sub(streams["pub_image"], streams["sub_image"], "image")

    summarize_camera_drop(read_camera_drop(args.camera_trace_dir / "see3cam_drop_trace.bin"))

    write_gaps_csv(args.out_dir, "pub_lidar", streams["pub_lidar"], args.lidar_ms, args.gap_factor)
    write_gaps_csv(args.out_dir, "sub_lidar", streams["sub_lidar"], args.lidar_ms, args.gap_factor)
    write_gaps_csv(args.out_dir, "pub_imu", streams["pub_imu"], args.imu_ms, args.gap_factor)
    write_gaps_csv(args.out_dir, "sub_imu", streams["sub_imu"], args.imu_ms, args.gap_factor)
    write_gaps_csv(args.out_dir, "pub_image", streams["pub_image"], args.image_ms, args.gap_factor)
    write_gaps_csv(args.out_dir, "sub_image", streams["sub_image"], args.image_ms, args.gap_factor)

    print(f"\nCSV gap reports written under: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
