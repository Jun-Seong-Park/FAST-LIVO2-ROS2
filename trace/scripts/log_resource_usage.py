#!/usr/bin/env python3
"""
Lightweight resource logger for FAST-LIVO2 sensor stack.

Writes CSV files under <sensor_ws>/trace/log:
  - resource_process.csv: per-process CPU%, RSS, VSZ, thread count
  - resource_system.csv : system load, memory, swap, disk, temperature, network counters
  - resource_tegrastats.log: raw tegrastats output when available

Default process matching is intentionally broad enough for the current stack:
  fastlivo_mapping2, livox_ros_driver2_sync_node, see3cam_trigger,
  component_container, rviz2
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import shutil
import signal
import subprocess
import time
from pathlib import Path


TRACE_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = TRACE_DIR / "log"
DEFAULT_PATTERNS = [
    "fastlivo_mapping2",
    "fast_livo2_node",
    "livox_ros_driver2_sync_node",
    "livox_lidar_publisher",
    "see3cam_trigger",
    "component_container",
    "rviz2",
]


RUNNING = True


def on_signal(signum, frame) -> None:
    global RUNNING
    RUNNING = False


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def read_proc_stat(pid: int) -> dict | None:
    stat = read_text(Path(f"/proc/{pid}/stat"))
    if not stat:
        return None
    # comm may contain spaces, so split around the final ")".
    try:
        left, right = stat.rsplit(") ", 1)
        comm = left.split("(", 1)[1]
        fields = right.split()
    except ValueError:
        return None
    try:
        return {
            "pid": pid,
            "comm": comm,
            "state": fields[0],
            "ppid": int(fields[1]),
            "utime": int(fields[11]),
            "stime": int(fields[12]),
            "num_threads": int(fields[17]),
            "vsize": int(fields[20]),
            "rss_pages": int(fields[21]),
        }
    except (IndexError, ValueError):
        return None


def read_cmdline(pid: int) -> str:
    raw = Path(f"/proc/{pid}/cmdline")
    try:
        data = raw.read_bytes()
    except OSError:
        return ""
    return data.replace(b"\0", b" ").decode(errors="replace").strip()


def process_matches(cmd: str, comm: str, patterns: list[str]) -> bool:
    hay = f"{comm} {cmd}"
    return any(p in hay for p in patterns)


def list_processes(patterns: list[str]) -> list[dict]:
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        st = read_proc_stat(pid)
        if st is None:
            continue
        cmd = read_cmdline(pid)
        if process_matches(cmd, st["comm"], patterns):
            st["cmdline"] = cmd
            out.append(st)
    return out


def read_total_jiffies() -> int:
    line = read_text(Path("/proc/stat")).splitlines()[0]
    vals = [int(x) for x in line.split()[1:]]
    return sum(vals)


def meminfo() -> dict[str, int]:
    out = {}
    for line in read_text(Path("/proc/meminfo")).splitlines():
        key, rest = line.split(":", 1)
        parts = rest.strip().split()
        if parts:
            out[key] = int(parts[0]) * 1024
    return out


def loadavg() -> tuple[float, float, float]:
    parts = read_text(Path("/proc/loadavg")).split()
    return float(parts[0]), float(parts[1]), float(parts[2])


def read_temp_c() -> float | None:
    temps = []
    base = Path("/sys/class/thermal")
    for path in base.glob("thermal_zone*/temp"):
        try:
            val = int(path.read_text().strip())
        except (OSError, ValueError):
            continue
        if val > 1000:
            temps.append(val / 1000.0)
        else:
            temps.append(float(val))
    return max(temps) if temps else None


def disk_free_bytes(path: Path) -> int:
    try:
        return shutil.disk_usage(path).free
    except OSError:
        return 0


def netdev_totals() -> dict[str, int]:
    totals = {
        "rx_bytes": 0,
        "tx_bytes": 0,
        "rx_drop": 0,
        "tx_drop": 0,
        "rx_errs": 0,
        "tx_errs": 0,
    }
    for line in read_text(Path("/proc/net/dev")).splitlines()[2:]:
        if ":" not in line:
            continue
        iface, rest = line.split(":", 1)
        iface = iface.strip()
        if iface == "lo":
            continue
        vals = rest.split()
        if len(vals) < 16:
            continue
        totals["rx_bytes"] += int(vals[0])
        totals["rx_errs"] += int(vals[2])
        totals["rx_drop"] += int(vals[3])
        totals["tx_bytes"] += int(vals[8])
        totals["tx_errs"] += int(vals[10])
        totals["tx_drop"] += int(vals[11])
    return totals


def write_header_if_new(path: Path, header: list[str]) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    with path.open("w", newline="") as f:
        csv.writer(f).writerow(header)


def start_tegrastats(interval_ms: int, path: Path) -> subprocess.Popen | None:
    if shutil.which("tegrastats") is None:
        return None
    f = path.open("ab", buffering=0)
    try:
        proc = subprocess.Popen(
            ["tegrastats", "--interval", str(interval_ms)],
            stdout=f,
            stderr=subprocess.STDOUT,
        )
    except OSError:
        f.close()
        return None
    proc._trace_file = f  # type: ignore[attr-defined]
    return proc


def stop_tegrastats(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)
    f = getattr(proc, "_trace_file", None)
    if f is not None:
        f.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Log FAST-LIVO2 process/system resource usage")
    parser.add_argument("--interval", type=float, default=1.0, help="sample interval in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl-C")
    parser.add_argument("--pattern", action="append", default=[], help="extra process match pattern")
    parser.add_argument("--quiet", action="store_true", help="do not print live status lines")
    parser.add_argument("--no-tegrastats", action="store_true")
    args = parser.parse_args()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    patterns = DEFAULT_PATTERNS + args.pattern

    proc_csv = LOG_DIR / "resource_process.csv"
    sys_csv = LOG_DIR / "resource_system.csv"
    tegra_log = LOG_DIR / "resource_tegrastats.log"

    write_header_if_new(proc_csv, [
        "time_s", "pid", "ppid", "comm", "state", "cpu_pct",
        "rss_mb", "vsz_mb", "threads", "cmdline",
    ])
    write_header_if_new(sys_csv, [
        "time_s", "load1", "load5", "load15", "mem_used_mb", "mem_avail_mb",
        "swap_used_mb", "max_temp_c", "trace_free_gb",
        "net_rx_mbps", "net_tx_mbps", "net_rx_drop_delta", "net_tx_drop_delta",
        "net_rx_err_delta", "net_tx_err_delta",
    ])

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    tegra_proc = None
    if not args.no_tegrastats:
        tegra_proc = start_tegrastats(max(100, int(args.interval * 1000)), tegra_log)

    page_size = os.sysconf("SC_PAGE_SIZE")
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    prev_proc_ticks: dict[int, int] = {}
    prev_total = read_total_jiffies()
    prev_net = netdev_totals()
    cpu_count = os.cpu_count() or 1

    start = time.monotonic()
    print(f"resource logs -> {LOG_DIR}")
    print(f"process patterns -> {', '.join(patterns)}")
    if tegra_proc is None:
        print("tegrastats -> unavailable or disabled")
    else:
        print(f"tegrastats -> {tegra_log}")
    sys.stdout.flush()
    try:
        while RUNNING:
            now = time.time()
            total = read_total_jiffies()
            total_delta = max(1, total - prev_total)
            prev_total = total
            net_now = netdev_totals()
            interval = max(args.interval, 1e-6)
            rx_mbps = (net_now["rx_bytes"] - prev_net["rx_bytes"]) * 8.0 / interval / 1e6
            tx_mbps = (net_now["tx_bytes"] - prev_net["tx_bytes"]) * 8.0 / interval / 1e6
            rx_drop_delta = net_now["rx_drop"] - prev_net["rx_drop"]
            tx_drop_delta = net_now["tx_drop"] - prev_net["tx_drop"]
            rx_err_delta = net_now["rx_errs"] - prev_net["rx_errs"]
            tx_err_delta = net_now["tx_errs"] - prev_net["tx_errs"]
            prev_net = net_now

            rows = []
            for p in list_processes(patterns):
                ticks = p["utime"] + p["stime"]
                prev_ticks = prev_proc_ticks.get(p["pid"], ticks)
                prev_proc_ticks[p["pid"]] = ticks
                cpu_pct = 100.0 * cpu_count * (ticks - prev_ticks) / total_delta
                rows.append([
                    f"{now:.3f}",
                    p["pid"],
                    p["ppid"],
                    p["comm"],
                    p["state"],
                    f"{cpu_pct:.3f}",
                    f"{p['rss_pages'] * page_size / 1024 / 1024:.3f}",
                    f"{p['vsize'] / 1024 / 1024:.3f}",
                    p["num_threads"],
                    p["cmdline"],
                ])

            with proc_csv.open("a", newline="") as f:
                csv.writer(f).writerows(rows)

            mi = meminfo()
            load1, load5, load15 = loadavg()
            mem_total = mi.get("MemTotal", 0)
            mem_avail = mi.get("MemAvailable", 0)
            swap_total = mi.get("SwapTotal", 0)
            swap_free = mi.get("SwapFree", 0)
            temp_c = read_temp_c()
            with sys_csv.open("a", newline="") as f:
                csv.writer(f).writerow([
                    f"{now:.3f}",
                    f"{load1:.3f}",
                    f"{load5:.3f}",
                    f"{load15:.3f}",
                    f"{(mem_total - mem_avail) / 1024 / 1024:.3f}",
                    f"{mem_avail / 1024 / 1024:.3f}",
                    f"{(swap_total - swap_free) / 1024 / 1024:.3f}",
                    "" if temp_c is None else f"{temp_c:.3f}",
                    f"{disk_free_bytes(TRACE_DIR) / 1024 / 1024 / 1024:.3f}",
                    f"{rx_mbps:.3f}",
                    f"{tx_mbps:.3f}",
                    rx_drop_delta,
                    tx_drop_delta,
                    rx_err_delta,
                    tx_err_delta,
                ])

            if not args.quiet:
                top = sorted(rows, key=lambda r: float(r[5]), reverse=True)[:3]
                top_s = "; ".join(f"{r[3]} pid={r[1]} cpu={r[5]} rss={r[6]}MB" for r in top)
                if not top_s:
                    top_s = "no matched processes"
                print(
                    f"t={now:.0f} matched={len(rows)} load1={load1:.2f} "
                    f"mem_used={(mem_total - mem_avail)/1024/1024:.0f}MB "
                    f"net rx={rx_mbps:.2f}Mbps tx={tx_mbps:.2f}Mbps "
                    f"drops rx/tx={rx_drop_delta}/{tx_drop_delta} | {top_s}"
                )
                sys.stdout.flush()

            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break
            time.sleep(args.interval)
    finally:
        stop_tegrastats(tegra_proc)

    print(f"resource logs written to {LOG_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
