#pragma once
/**
 * @file blackbox.hpp
 * @brief Blackbox (livox_ros_driver2 vendored subset) — lidar / imu / lidar_resource streams.
 *
 * mmap-fixed-array + 1 s fdatasync writer thread.
 *
 * Vendored per the package-independence decision (spec: storage.md "배포 형태"):
 * this package owns the LidarPubRecord / ImuPubRecord / LidarResourceRecord ABI.
 * Other streams (image pub, sub_*, host resource) live in their owning packages.
 * The Box mechanism's single source of truth is the spec (storage.md), not a shared header.
 *
 * Lifecycle (caller's responsibility):
 *   const std::string session = blackbox::session_dir();   // capture ONCE in ctor
 *   blackbox::<stream>::init(session + "/<name>.bin")      — open + fallocate + mmap + writer thread
 *   blackbox::<stream>::log(...)                           — hot path
 *   blackbox::<stream>::shutdown()                         — stop writer + final fdatasync + munmap + close
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace blackbox {

// ─────────────────────────────────────────────────────────────
// Log session, ros2-logger style:  ~/.blackbox/log/<YYYY-MM-DD-HH-MM-SS>-<pid>/
// A fresh directory each run → every session is preserved, never overwritten.
// session_dir() re-stamps the time on every call — capture it ONCE in the ctor.
// ─────────────────────────────────────────────────────────────

inline std::string log_root() {
  const char* home = std::getenv("HOME");
  return std::string(home && *home ? home : "/tmp") + "/.blackbox/log";
}

inline std::string session_dir() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char ts[20];
  std::strftime(ts, sizeof(ts), "%Y-%m-%d-%H-%M-%S", &tm);
  return log_root() + "/" + ts + "-" + std::to_string(::getpid());
}

// ─────────────────────────────────────────────────────────────
// Common: mono_raw_ns
// ─────────────────────────────────────────────────────────────

inline uint64_t mono_raw_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

// ─────────────────────────────────────────────────────────────
// Records — this package is the ABI owner (spec: lidar.md, resource.md)
// ─────────────────────────────────────────────────────────────

struct LidarPubRecord {
  uint64_t seq;
  uint64_t header_stamp;
  uint64_t t_pub_ns;
};
static_assert(sizeof(LidarPubRecord) == 24, "LidarPubRecord ABI fixed at 24B");

struct ImuPubRecord {
  uint64_t seq;
  uint64_t header_stamp;
  uint64_t t_pub_ns;
};
static_assert(sizeof(ImuPubRecord) == 24, "ImuPubRecord ABI fixed at 24B");

/**
 * LiDAR device resource (MID-360 push msg, 1 Hz). 4 keys always co-bundled
 * per packet (wire-verified 30/30, 2026-05-26). Missing-key flag NOT needed
 * since the record is omitted entirely if push msg drops (seq gap).
 */
struct LidarResourceRecord {
  uint64_t seq;                  // 0   record counter (monotonic from init)
  uint64_t t_sample_ns;          // 8   mono_raw_ns() at callback entry
  int32_t  core_temp_centi;      // 16  key 0x8007, unit 0.01 deg C
  uint16_t lidar_diag_status;    // 20  key 0x800E, 4 modules x 4 bits packed
  uint8_t  cur_work_state;       // 22  key 0x8006, enum
  uint8_t  _pad0;                // 23  align (NOT a valid flag)
  uint32_t hms_code[8];          // 24  key 0x8011, 8 fault-code slots
                                 // 56  end
};
static_assert(sizeof(LidarResourceRecord) == 56, "LidarResourceRecord ABI fixed at 56B");
static_assert(alignof(LidarResourceRecord) == 8, "LidarResourceRecord align must be 8B");

// ─────────────────────────────────────────────────────────────
// detail: generic mmap-fixed-array + 1s writer thread
// (mechanism truth = spec/storage.md — fix there first, then propagate)
// ─────────────────────────────────────────────────────────────

namespace detail {

inline void mkdir_for_file(const std::string& path) {
  size_t slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) { return; }
  std::string dir = path.substr(0, slash);
  size_t pos = 0;
  while ((pos = dir.find('/', pos + 1)) != std::string::npos) {
    ::mkdir(dir.substr(0, pos).c_str(), 0755);
  }
  ::mkdir(dir.c_str(), 0755);
}

template <typename Record, size_t N>
struct Box {
  Record*                  base{nullptr};
  int                      fd{-1};
  std::atomic<size_t>      idx{0};
  std::atomic<uint64_t>    seq{0};
  std::atomic<bool>        running{false};
  std::thread              writer;
  std::mutex               init_mtx;

  void init(const std::string& path) {
    std::lock_guard<std::mutex> lk(init_mtx);
    if (fd >= 0) { return; }          // already initialized
    mkdir_for_file(path);
    fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) { throw std::runtime_error("blackbox: open failed: " + path); }
    const size_t bytes = N * sizeof(Record);
    if (::fallocate(fd, 0, 0, bytes) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("blackbox: fallocate failed: " + path);
    }
    void* p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_POPULATE, fd, 0);
    if (p == MAP_FAILED) {
      ::close(fd); fd = -1;
      throw std::runtime_error("blackbox: mmap failed: " + path);
    }
    base = static_cast<Record*>(p);
    running.store(true, std::memory_order_release);
    writer = std::thread([this]() {
      while (running.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!running.load(std::memory_order_acquire)) { break; }
        if (fd >= 0) { ::fdatasync(fd); }
      }
    });
  }

  void shutdown() {
    std::lock_guard<std::mutex> lk(init_mtx);
    if (fd < 0) { return; }
    running.store(false, std::memory_order_release);
    if (writer.joinable()) { writer.join(); }
    if (fd >= 0) { ::fdatasync(fd); }
    if (base) {
      ::munmap(base, N * sizeof(Record));
      base = nullptr;
    }
    ::close(fd);
    fd = -1;
  }

  size_t next_idx() {
    size_t i = idx.fetch_add(1, std::memory_order_relaxed);
    return i < N ? i : SIZE_MAX;
  }

  uint64_t next_seq() {
    return seq.fetch_add(1, std::memory_order_relaxed) + 1;
  }
};

}  // namespace detail

// ─────────────────────────────────────────────────────────────
// LiDAR stream — single thread (cbk = publish)
// ─────────────────────────────────────────────────────────────

namespace lidar {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 29h @ 10Hz
inline detail::Box<LidarPubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) { return; }
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) { return; }
  LidarPubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_pub_ns     = mono_raw_ns();
}

}  // namespace lidar

// ─────────────────────────────────────────────────────────────
// IMU stream — single thread (cbk = publish)
// ─────────────────────────────────────────────────────────────

namespace imu {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 87 min @ 200Hz
inline detail::Box<ImuPubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) { return; }
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) { return; }
  ImuPubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_pub_ns     = mono_raw_ns();
}

}  // namespace imu

// ─────────────────────────────────────────────────────────────
// LiDAR device resource stream — 1 Hz, SDK push msg callback thread
//   kBufN = 1<<16 (3.6 MB, 18.2 h @ 1 Hz). Smaller N than sensor streams.
//   Box hot-path is lock-free atomic fetch_add, multi-writer safe.
// ─────────────────────────────────────────────────────────────

namespace lidar_resource {

inline constexpr size_t kBufN = 1 << 16;
inline detail::Box<LidarResourceRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(int32_t        core_temp_centi,
                uint16_t       lidar_diag_status,
                uint8_t        cur_work_state,
                const uint32_t hms_code[8]) {
  if (g_box.fd < 0) { return; }
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) { return; }
  LidarResourceRecord& r = g_box.base[i];
  r.seq               = g_box.next_seq();
  r.t_sample_ns       = mono_raw_ns();
  r.core_temp_centi   = core_temp_centi;
  r.lidar_diag_status = lidar_diag_status;
  r.cur_work_state    = cur_work_state;
  r._pad0             = 0;
  std::memcpy(r.hms_code, hms_code, sizeof(r.hms_code));
}

}  // namespace lidar_resource

}  // namespace blackbox
