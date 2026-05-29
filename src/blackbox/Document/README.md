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


# 3. Usage


# 4. How It Works



# 5. Analysis