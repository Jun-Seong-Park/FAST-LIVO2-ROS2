# Blackbox

> **Last update**: 2026-05-29  
> **Author**: jspark2954@aluxonline.com


## Contents

1. [Concept](#1-concept)
2. [Records](#2-records)
3. [Usage](#3-usage)
4. [How It Works](#4-how-it-works)
5. [Analysis](#5-analysis)



# 1. Concept

Blackbox is an always-on *flight recorder* for the FAST-LIVO2 sensor/SLAM stack
(camera + LiDAR + IMU). When a run goes wrong — a latency spike, a dropped frame,
a sensor desync, a thermal throttle — the state you need is the state *right
before and during* the incident. Like an aircraft flight recorder, Blackbox
exists to make that state recoverable after the fact.

Ordinary logging cannot fill this role:

- **`printf` / `rosout`** — ASCII formatting plus an I/O flush on every event is
  far too heavy for a hot path running at 200 Hz (IMU) or 30 Hz (camera). It adds
  jitter to the very timing it is meant to measure, and in practice it is switched
  off in production — empty exactly when an incident happens.
- **ROS bag** — records full payloads (images, point clouds): gigabytes per
  minute, plus serialization and DDS cost on the hot path. It cannot run for hours
  on an embedded host.

Blackbox records neither. Per event it stores **only small, fixed-size metadata** —
a sequence number, the header stamp, and a few monotonic timestamps — never the
payload. That is cheap enough to leave on permanently.

## Design tenets

- **Always-on, never a debug switch.** The most valuable data is the second before
  a fault. A logger you disable in production is blank when you need it most, so
  Blackbox is built to cost nothing to leave running.
- **The hot path costs nanoseconds.** Logging must not perturb what it measures.
  Each call is one lock-free `atomic fetch_add` plus a single struct store — no
  malloc, no lock, no syscall, no formatting. Multiple sensor threads write
  concurrently without contention.
- **Binary, fixed ABI.** Every record is a packed struct whose size is frozen by
  `static_assert`. Writing is one store; offline parsing is a `memcpy` over the
  file. That frozen layout is the contract between the C++ writer and the Python
  analyzers.
- **Survives the crash.** Records live in an `mmap`-backed file that a background
  thread flushes to disk once per second. If the process dies, everything up to
  ~1 s ago is already on disk — the recorder outlives the crash, as a flight
  recorder must.
- **Monotonic-raw time.** Every timestamp comes from `CLOCK_MONOTONIC_RAW`, which
  never steps with NTP/PTP corrections. Pipeline latency is about *interval
  consistency* (ΔT), not wall-clock accuracy — a clock that cannot jump backward
  is what matters.

## What it answers

Where a latency or drop originated along the **sensor → publish → subscribe**
path, and whether the host was saturated or throttling at that moment — by
pairing per-event timestamps with 1 Hz host and device health (CPU, RAM, GPU,
thermal, network, LiDAR core temperature).

## Non-goals

Blackbox does not capture sensor payloads. It records *when and whether* an event
happened, not *what* the data contained. For payload capture, run a ROS bag
alongside it.


# 2. Records

A record is a fixed-size struct. Nothing variable-length, nothing optional,
nothing serialized. Writing one is a single store; reading one is a `memcpy`. The
size of every record is frozen by `static_assert` — change a field and the build
breaks, on purpose, because a Python analyzer on the other end depends on that
exact byte layout.

## Shared invariants

These hold for every stream:

- Every field is 8-byte aligned. Every timestamp is `ns` from `CLOCK_MONOTONIC_RAW`.
- `header_stamp` = `msg->header.stamp` in ns (MID-360 GPS-epoch domain). It is the
  **pub ↔ sub match key** — the same physical event carries the same stamp on both
  sides.
- `seq` is a per-stream counter. A gap in `seq` is a drop.
- Unwritten slots are zero (from `fallocate`), so analyzers keep only `seq > 0`.
- A trailing byte acts as a valid flag **only** where two threads fill one record
  (the camera). Single-thread streams cannot tear, so they carry no flag.

## Stream catalog

Nine streams, four producers. `N` is the slot count (`kBufN`); capacity is how
long that fills at the stream's rate.

| Stream | Record | Size | File | N | Capacity | Written by |
|---|---|---|---|---|---|---|
| `image` (pub) | `ImagePubRecord` | 48 B | `see3cam24cug_trig_image_pub.bin` | 2²⁰ | 48 MB ≈ 9.7 h @30 Hz | `see3cam24cug_trig.cpp` |
| `lidar` (pub) | `LidarPubRecord` | 24 B | `lidar_pubrecord.bin` | 2²⁰ | 24 MB ≈ 29 h @10 Hz | `lddc.cpp` |
| `imu` (pub) | `ImuPubRecord` | 24 B | `imu_pubrecord.bin` | 2²⁰ | 24 MB ≈ **87 min @200 Hz** | `lddc.cpp` |
| `sub_lidar` | `LidarSubRecord` | 24 B | `lidar_subrecord.bin` | 2²⁰ | 24 MB ≈ 29 h | `LIVMapper.cpp` |
| `sub_imu` | `ImuSubRecord` | 24 B | `imu_subrecord.bin` | 2²⁰ | 24 MB ≈ 87 min | `LIVMapper.cpp` |
| `sub_image` | `ImageSubRecord` | 24 B | `image_subrecord.bin` | 2²⁰ | 24 MB ≈ 9.7 h | `LIVMapper.cpp` |
| `lidar_resource` | `LidarResourceRecord` | 56 B | `lidar_resource.bin` | 2¹⁶ | 3.6 MB ≈ 18.2 h @1 Hz | `lds_lidar.cpp` |
| `resource` (host) | `GlobalRecord` | 424 B | `global.bin` | 2¹⁶ | 28 MB ≈ 18.2 h @1 Hz | `resource_logger` |
| `resource` (proc ×3) | `ProcRecord` | 48 B | `{image,slam,livox}_proc.bin` | 2¹⁶ | 3 MB each ≈ 18.2 h @1 Hz | `resource_logger` |

**IMU at 200 Hz is the binding constraint** — 24 MB fills in 87 min. A single
flight is well under that. To go longer, bump only the IMU stream's `N`.

## Record ABI

### `ImagePubRecord` — 48 B

The camera is the one record filled by two threads, so the last written byte
(`ros_done`) marks it complete.

| Off | Size | Field | Meaning |
|---|---|---|---|
| 0 | 8 | `seq` | drop detector |
| 8 | 8 | `header_stamp` | msg stamp ns (match key) |
| 16 | 8 | `t_dqbuf_ns` | DQBUF done = pipeline entry (v4l2 src pad probe) |
| 24 | 8 | `t_cbk_ns` | appsink callback entry |
| 32 | 8 | `t_pub_ns` | `publish()` return |
| 40 | 1 | `gst_done` | reached the callback |
| 41 | 1 | `ros_done` | publish done — **valid flag** |
| 42 | 6 | `_pad` | align to 48 |

### `LidarPubRecord` / `ImuPubRecord` / `*SubRecord` — 24 B

cbk = publish (single thread), so one call fills the whole record. No flag needed.

| Off | Size | Field | Meaning |
|---|---|---|---|
| 0 | 8 | `seq` | drop detector |
| 8 | 8 | `header_stamp` | msg stamp ns (match key) |
| 16 | 8 | `t_pub_ns` (pub) / `t_cbk_ns` (sub) | publish return / subscriber-callback entry |

### `LidarResourceRecord` — 56 B

MID-360 device health from the SDK push message (1 Hz).

| Off | Size | Field | Meaning |
|---|---|---|---|
| 0 | 8 | `seq` | record counter |
| 8 | 8 | `t_sample_ns` | sample time (MonoRaw) |
| 16 | 4 | `core_temp_centi` | core temp, 0.01 °C |
| 20 | 2 | `lidar_diag_status` | 4 modules × 4 bits |
| 22 | 1 | `cur_work_state` | work-state enum |
| 23 | 1 | `_pad0` | align |
| 24 | 32 | `hms_code[8]` | 8 fault-code slots |

### `ProcRecord` — 48 B

Per-process CPU and memory. Counters are stored **raw**; the analyzer differences
them.

| Off | Size | Field | Meaning |
|---|---|---|---|
| 0 | 8 | `seq` | sample counter |
| 8 | 8 | `t_sample_ns` | sample time (shared with `GlobalRecord`) |
| 16 | 4 | `pid` | restart detector (change = counter reset) |
| 20 | 4 | `_pad` | align |
| 24 | 8 | `utime` | `/proc/[pid]/stat` utime (raw jiffies) |
| 32 | 8 | `stime` | `/proc/[pid]/stat` stime (raw jiffies) |
| 40 | 8 | `rss_bytes` | resident set, bytes |

### `GlobalRecord` — 424 B

Whole-host health on the same 1 Hz tick. Counters raw; a `valid` bitmask says
which sources parsed to a meaningful number this tick (a read can succeed yet
return `N/A`).

| Group | Fields |
|---|---|
| header | `seq`, `t_sample_ns` |
| CPU (8 cores) | `cpu_total[8]`, `cpu_idle[8]` (raw jiffies), `cpu_freq[8]` (kHz) |
| RAM | `mem_total_kb`, `mem_avail_kb` |
| GPU | `gpu_load` (‰), `gpu_freq_hz` |
| throttle | `thr_cpu0`, `thr_cpu4`, `thr_gpu` (cooling cur_state) |
| validity | `valid` (`GlobalValid` bitmask) |
| network | `net[4]` (`NetSlot`: iface, rx/tx bytes, rx/tx drop), `wifi_link`, `wifi_level` |
| TCP | `tcp_out`, `tcp_retrans` |

## Measurement intervals

Every timestamp is the same `CLOCK_MONOTONIC_RAW` domain, so any two subtract
directly:

| Interval | Formula |
|---|---|
| GStreamer processing | `t_cbk_ns − t_dqbuf_ns` |
| callback → publish | `t_pub_ns − t_cbk_ns` |
| transport (pub → sub) | sub `t_cbk_ns` − pub `t_pub_ns` (matched on `header_stamp`) |


# 3. Usage

Three calls. `init()` in the constructor, `log()` on every event, `shutdown()` in
the destructor. That is the whole API.

```cpp
blackbox::<stream>::init(path);   // open + fallocate + mmap + start writer
blackbox::<stream>::log(...);     // hot path — one store
blackbox::<stream>::shutdown();   // stop writer + final fdatasync + munmap + close
```

## Rules

- **`init()` opens the file or throws.** `fallocate`/`mmap` failure raises
  `std::runtime_error`. There is no silent-disable mode — a recorder that quietly
  turned itself off would be worse than one that refused to start.
- **`log_*()` never crashes the caller.** Before `init` (or after a failed one)
  `fd < 0` and the call is a no-op. When the buffer fills, it stops silently. The
  hot path cannot take down the node it is measuring.
- **Once each.** `init()` once per stream in the node constructor, `shutdown()`
  once in the destructor. A clean SIGINT/SIGTERM reaches the destructor through
  `rclcpp`, so a normal stop loses zero records.

## Where each stream is driven

| Node | Stream(s) | Calls |
|---|---|---|
| `see3cam24cug_trig.cpp` (camera) | `image` | `init` → `log_cbk` → `log_pub` → `shutdown` |
| `lddc.cpp` (Livox driver) | `lidar`, `imu` | `init` → `log(stamp)` → `shutdown` |
| `lds_lidar.cpp` (Livox device) | `lidar_resource` | `log(temp, diag, state, hms)` |
| `LIVMapper.cpp` (fast_livo2) | `sub_lidar`, `sub_imu`, `sub_image` | `init` → `log(stamp)` → `shutdown` |
| `resource_logger` | `resource` | `init(dir)` → 1 Hz sample → `shutdown` |

Paths come from `blackbox::log_dir()`, so a caller writes
`blackbox::log_dir() + "/<name>.bin"`.

## The camera cascade — the one exception

Every other stream does cbk = publish on one thread, so a single `log(stamp)` call
fills the record. The camera is split across two threads, so its record is filled
in two stages, handed off by slot index:

1. A v4l2 `src` pad probe stamps `t_dqbuf` the instant DQBUF completes (pipeline
   entry).
2. The appsink callback calls `log_cbk(header_stamp, t_dqbuf)` — it writes
   `seq`/`header`/`t_dqbuf`/`t_cbk`, sets `gst_done`, and **returns a slot index**.
3. The publish loop, right after `publish()`, calls `log_pub(idx)` — it stamps
   `t_pub` and sets `ros_done` in that same slot.

That is why `ImagePubRecord` carries `gst_done`/`ros_done`: two threads, one
record, and the last byte written marks it complete.

```cpp
// ctor
blackbox::image::init(blackbox::log_dir() + "/see3cam24cug_trig_image_pub.bin");
// cbk thread
size_t idx = blackbox::image::log_cbk(header_stamp_ns, t_dqbuf_ns);
// publish loop, immediately after publish()
blackbox::image::log_pub(idx);
// dtor
blackbox::image::shutdown();
```

## The resource logger

Host resources are not a callback hot path, so they get a dedicated node. It reads
its config (`proc_prefixes`, `net_ifaces`, `sample_period_s`, `log_dir`), polls
`/proc` and `/sys` once per second, and writes all four resource files on the same
tick. Launch it alongside the sensor stack:

```bash
ros2 launch blackbox launch.py
```


# 4. How It Works

There is one mechanism: `detail::Box<Record, N>`. Every stream is that template
with a different `Record` and a different `N`. Understand the Box and you
understand the entire blackbox.

## The Box, step by step

- **Allocate everything up front.** `init()` does
  `open(O_TRUNC)` → `fallocate(N × sizeof(Record))` → `mmap(MAP_SHARED | MAP_POPULATE)`.
  `fallocate` reserves the extents immediately; `MAP_POPULATE` prefaults the page
  tables. The first `log()` hits zero allocation and zero page-fault jitter — the
  cost was paid at startup, not on the hot path. (`ftruncate` only writes
  metadata, so the first store would page-fault. We don't use it.)
- **The hot path is three operations.**
  `i = idx.fetch_add(1, relaxed); if (i < N) base[i] = record;`
  One atomic increment, one bounds check, one struct store. No lock, no malloc, no
  syscall except the clock read. `fetch_add` is lock-free, so multiple sensor
  threads write concurrently and never block each other.
- **A writer thread does the I/O.** One background thread sleeps 1 s, calls
  `fdatasync(fd)`, repeats. The hot path never touches the disk; flushing is
  batched and asynchronous. The kernel owns the `mmap` dirty pages, so the data is
  durable in the page cache even between syncs.
- **Buffer full = silent stop.** Past `N`, `next_idx()` returns `SIZE_MAX` and
  `log()` becomes a no-op. The Box never reallocates, never overwrites, never
  blocks. It just stops — and it is sized so that never happens within a flight.
- **Shutdown is deterministic.** `running = false` → join the writer (≤ 1 s) →
  final `fdatasync` → `munmap` → `close`.

## What was deleted, and why

The design is defined as much by what is absent:

- **No ring buffer, no rotation.** A flight fits in one fixed-size file. Rotation
  is a part you don't need, so it isn't there.
- **No formatting.** Binary structs parsed offline. ASCII spends a runtime cost to
  buy an offline convenience — the wrong trade for a hot path.
- **No locks.** A hot-path mutex would couple the sensor threads. `fetch_add`
  doesn't.
- **No silent failure.** `init` throws; it never half-runs. The only quiet path is
  buffer-full, which is sized never to occur.

## Durability — what survives

| Event | Loss |
|---|---|
| Clean exit | **0** |
| SIGKILL / segfault | **≤ 1 s** (kernel flushes the `mmap` dirty pages) |
| Kernel panic | **≤ 1 s** |
| Power cut (non-PLP NVMe) | NVMe DRAM cache — unrecoverable in software |

## Time base

Every timestamp is `CLOCK_MONOTONIC_RAW`. On aarch64 it is a real syscall
(~100–300 ns, not vDSO) — negligible against a millisecond budget — and it never
steps with NTP/PTP. Because every stream shares this one clock, any two timestamps
subtract directly. That is what makes cross-stream latency measurable at all.

## Where it lives

Blackbox is a header-only `INTERFACE` library. `BLACKBOX_ROOT_DIR` is injected by
CMake as the package source path, so `log_dir()` / `plots_dir()` / `scripts_dir()`
resolve to `src/blackbox/{log, plots, scripts}`. A caller includes
`<blackbox/blackbox.hpp>` and links the interface target — nothing to compile.


# 5. Analysis

The read side. Binary in, PNG out. Four analyzers and one wrapper, all under
`scripts/`.

`plot.py` runs the four in sequence:

| # | Script | What it produces |
|---|---|---|
| 1 | `analyze_sensor_publish.py` | pub-side stats (image/lidar/imu) → `plots/{image,lidar,imu}/` |
| 2 | `analyze_sensor_subscribe.py` | sub-side stats → same dirs |
| 3 | `analyze_sensor_compare.py` | pub vs sub matched on `header_stamp` → `*_compare.png` |
| 4 | `analyze_resource.py` | host + LiDAR device resource → `plots/resource/`, one PNG per metric |

## How to read the output

- **A `seq` gap is a drop.** `seq` is a per-stream counter; a jump > 1 means lost
  frames. On the sub side, a steady `seq_diff == 2` means the subscriber is running
  at half the publisher's rate.
- **Latency is timestamp subtraction.** GStreamer = `t_cbk − t_dqbuf`;
  callback→publish = `t_pub − t_cbk`; transport = sub `t_cbk` − pub `t_pub`. All
  `MONOTONIC_RAW`, all directly subtractable, pub and sub matched by `header_stamp`.
- **Valid filters.** Keep `seq > 0` (skip empty `fallocate`'d slots). For
  `ImagePubRecord`, additionally keep `ros_done == 1` (skip torn records). For
  `GlobalRecord`, use a field only when its `valid` bit is set.

## Resource derivations

Counters are stored raw and differenced in the analyzer:

| Metric | Formula |
|---|---|
| per-core CPU % | `(Δcpu_total − Δcpu_idle) / Δcpu_total × 100` |
| RAM used | `mem_total_kb − mem_avail_kb` |
| process CPU % | `(Δ(utime + stime) / CLK_TCK) / Δt × 100`, `CLK_TCK = 100` |
| process RSS | `rss_bytes` |
| TCP retrans rate | `tcp_retrans / tcp_out` |

A change in `ProcRecord.pid` is a process restart — the `utime` counter resets, so
the analyzer breaks the line there.

## The payoff

Resource records share `t_sample_ns` with the sensor streams' clock domain. Put a
resource trace next to a sensor's `t_pub_ns`/`t_cbk_ns` and you see exactly what
the host was doing — CPU saturation, thermal throttle, network drop — at the
instant a frame dropped or the SLAM diverged. That correlation is the whole point.

> The camera variant is selectable: the publish/compare analyzers read
> `see3cam24cug_<variant>_image_pub.bin`; set `RESOLUTION` at the top of `plot.py`.