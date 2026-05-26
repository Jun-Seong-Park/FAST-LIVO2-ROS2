#pragma once
// blackbox — mmap-fixed-array + 1s writer thread.
//
// Spec: ~/.claude/skills/blackbox/Knowledge/blackbox_spec.md
//
// Lifecycle (caller 책임):
//   blackbox::<stream>::init(path)     — ctor. open + fallocate + mmap + writer thread
//   blackbox::<stream>::log_*(...)     — hot path
//   blackbox::<stream>::shutdown()     — dtor. stop writer + final fdatasync + munmap + close

#include <atomic>
#include <chrono>
#include <cstdint>
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
// Blackbox root: src/blackbox/{log,plots,scripts}  (package-internal)
//   BLACKBOX_ROOT_DIR is injected by CMakeLists as absolute path to the
//   package source dir (same pattern as fast_livo2's ROOT_DIR).
// ─────────────────────────────────────────────────────────────

#ifndef BLACKBOX_ROOT_DIR
#error "BLACKBOX_ROOT_DIR not defined — CMakeLists must inject it via target_compile_definitions"
#endif

inline std::string log_dir()     { return std::string(BLACKBOX_ROOT_DIR) + "log"; }
inline std::string plots_dir()   { return std::string(BLACKBOX_ROOT_DIR) + "plots"; }
inline std::string scripts_dir() { return std::string(BLACKBOX_ROOT_DIR) + "scripts"; }

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
// Records (blackbox_spec.md §3.5)
// ─────────────────────────────────────────────────────────────

struct ImagePubRecord {
  uint64_t seq;              // 8B
  uint64_t header_stamp;     // 8B
  uint64_t t_dqbuf_ns;   // 8B  pad probe: mono_raw_ns() — DQBUF 완료, pipeline 진입 시각
  uint64_t t_cbk_ns;         // 8B  cbk: mono_raw_ns()
  uint64_t t_pub_ns;         // 8B  pub: mono_raw_ns()
  uint8_t  gst_done;         // 1B  flag
  uint8_t  ros_done;         // 1B  flag
  uint8_t  _pad[6];          // 6B
  // 구간 (모두 MONOTONIC_RAW):
  //   GStreamer 처리: t_cbk_ns - t_dqbuf_ns
  //   cbk → pub:     t_pub_ns  - t_cbk_ns
};
static_assert(sizeof(ImagePubRecord) == 48, "ImagePubRecord ABI fixed at 48B");

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
 * Host resource records (spec/resource.md "Record ABI"). 1 Hz polling.
 *   ProcRecord   48 B  per-process CPU/RSS (image / slam / livox)
 *   NetSlot      48 B  fixed iface slot inside GlobalRecord
 *   GlobalRecord 424 B CPU per-core / RAM / GPU / throttle / net / wifi / TCP
 */
struct ProcRecord {
  uint64_t seq;            // 0   sample counter (seq>0 = valid slot)
  uint64_t t_sample_ns;    // 8   MonoRaw — same tick as GlobalRecord
  uint32_t pid;            // 16  restart detector
  uint32_t _pad;           // 20  align
  uint64_t utime;          // 24  /proc/[pid]/stat utime (raw jiffies)
  uint64_t stime;          // 32  /proc/[pid]/stat stime (raw jiffies)
  uint64_t rss_bytes;      // 40  /proc/[pid]/statm resident * PAGE
};                          // 48
static_assert(sizeof(ProcRecord) == 48, "ProcRecord ABI fixed at 48B");

struct NetSlot {
  char     iface[16];      // 0
  uint64_t rx_bytes;       // 16
  uint64_t tx_bytes;       // 24
  uint64_t rx_drop;        // 32
  uint64_t tx_drop;        // 40
};                          // 48
static_assert(sizeof(NetSlot) == 48, "NetSlot ABI fixed at 48B");

enum GlobalValid : uint32_t {
  V_CPU      = 1u << 0,
  V_MEM      = 1u << 1,
  V_CPUFREQ  = 1u << 2,
  V_GPU      = 1u << 3,
  V_THROTTLE = 1u << 4,
  V_NET      = 1u << 5,
  V_WIFI     = 1u << 6,
  V_TCP      = 1u << 7,
};

struct GlobalRecord {
  uint64_t seq;             // 0
  uint64_t t_sample_ns;     // 8
  uint64_t cpu_total[8];    // 16   /proc/stat cpuN sum(user..guest_nice)
  uint64_t cpu_idle[8];     // 80   /proc/stat cpuN idle
  uint32_t cpu_freq[8];     // 144  scaling_cur_freq (kHz)
  uint64_t mem_total_kb;    // 176  /proc/meminfo MemTotal
  uint64_t mem_avail_kb;    // 184  MemAvailable
  uint32_t gpu_load;        // 192  permille 0-1000
  uint32_t gpu_freq_hz;     // 196  Hz
  uint8_t  thr_cpu0;        // 200  cooling cur_state cpufreq-cpu0 (0-25)
  uint8_t  thr_cpu4;        // 201  cpufreq-cpu4
  uint8_t  thr_gpu;         // 202  devfreq-17000000.gpu (0-6)
  uint8_t  _pad0;           // 203
  uint32_t valid;           // 204  GlobalValid bitmask
  NetSlot  net[4];          // 208
  uint32_t wifi_link;       // 400
  int32_t  wifi_level;      // 404  dBm (negative)
  uint64_t tcp_out;         // 408  /proc/net/snmp OutSegs
  uint64_t tcp_retrans;     // 416  RetransSegs
};                           // 424
static_assert(sizeof(GlobalRecord) == 424, "GlobalRecord ABI fixed at 424B");

/**
 * LiDAR device resource (MID-360 push msg, 1 Hz). 4 keys always co-bundled
 * per packet (wire-verified 30/30, 2026-05-26). Missing-key flag NOT needed
 * since the record is omitted entirely if push msg drops (seq gap).
 * See references/logging/resource.md "결정 잠금 2026-05-26".
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
// ─────────────────────────────────────────────────────────────

namespace detail {

inline void mkdir_for_file(const std::string& path) {
  size_t slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) return;
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
    if (fd >= 0) return;          // already initialized
    mkdir_for_file(path);
    fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) throw std::runtime_error("blackbox: open failed: " + path);
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
        if (!running.load(std::memory_order_acquire)) break;
        if (fd >= 0) ::fdatasync(fd);
      }
    });
  }

  void shutdown() {
    std::lock_guard<std::mutex> lk(init_mtx);
    if (fd < 0) return;
    running.store(false, std::memory_order_release);
    if (writer.joinable()) writer.join();
    if (fd >= 0) ::fdatasync(fd);
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
// Image stream — cbk + publish_loop 두 thread 의 cascade
// ─────────────────────────────────────────────────────────────

namespace image {

inline constexpr size_t kBufN = 1 << 20;   // 48 MB ≈ 9.7h @ 30Hz
inline detail::Box<ImagePubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

// cbk thread: seq/header_stamp/gst_dt/t_cbk 박고 record_idx 반환.
// 가득 차면 SIZE_MAX 반환 → caller 가 publish 후 log_pub(SIZE_MAX) 호출하면 silent stop.
inline size_t log_cbk(uint64_t header_stamp_ns, uint64_t t_dqbuf_ns) {
  if (g_box.fd < 0) return SIZE_MAX;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return SIZE_MAX;
  ImagePubRecord& r = g_box.base[i];
  r.seq              = g_box.next_seq();
  r.header_stamp     = header_stamp_ns;
  r.t_dqbuf_ns   = t_dqbuf_ns;
  r.t_cbk_ns         = mono_raw_ns();
  r.t_pub_ns         = 0;
  r.gst_done         = 1;
  r.ros_done         = 0;
  return i;
}

// publish thread: t_pub + ros_done 박음.
inline void log_pub(size_t i) {
  if (g_box.fd < 0 || i == SIZE_MAX || i >= kBufN) return;
  ImagePubRecord& r = g_box.base[i];
  r.t_pub_ns = mono_raw_ns();
  r.ros_done = 1;
}

}  // namespace image

// ─────────────────────────────────────────────────────────────
// LiDAR stream — single thread (cbk = publish)
// ─────────────────────────────────────────────────────────────

namespace lidar {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 29h @ 10Hz
inline detail::Box<LidarPubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
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
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
  ImuPubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_pub_ns     = mono_raw_ns();
}

}  // namespace imu

// ─────────────────────────────────────────────────────────────
// Subscriber records — stamp match key for pub vs sub comparison
// ─────────────────────────────────────────────────────────────

struct LidarSubRecord {
  uint64_t seq;           // callback counter (0, 1, 2, …)
  uint64_t header_stamp;  // msg->header.stamp as ns
  uint64_t t_cbk_ns;      // CLOCK_MONOTONIC_RAW at callback entry
};
static_assert(sizeof(LidarSubRecord) == 24, "LidarSubRecord ABI fixed at 24B");

struct ImuSubRecord {
  uint64_t seq;
  uint64_t header_stamp;
  uint64_t t_cbk_ns;
};
static_assert(sizeof(ImuSubRecord) == 24, "ImuSubRecord ABI fixed at 24B");

struct ImageSubRecord {
  uint64_t seq;
  uint64_t header_stamp;
  uint64_t t_cbk_ns;
};
static_assert(sizeof(ImageSubRecord) == 24, "ImageSubRecord ABI fixed at 24B");

namespace sub_lidar {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 29h @ 10Hz
inline detail::Box<LidarSubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
  LidarSubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_cbk_ns     = mono_raw_ns();
}

}  // namespace sub_lidar

namespace sub_imu {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 87 min @ 200Hz
inline detail::Box<ImuSubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
  ImuSubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_cbk_ns     = mono_raw_ns();
}

}  // namespace sub_imu

namespace sub_image {

inline constexpr size_t kBufN = 1 << 20;   // 24 MB ≈ 9.7h @ 30Hz
inline detail::Box<ImageSubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

inline void log(uint64_t header_stamp_ns) {
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
  ImageSubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_cbk_ns     = mono_raw_ns();
}

}  // namespace sub_image

// ─────────────────────────────────────────────────────────────
// LiDAR device resource stream — 1 Hz, SDK push msg callback thread
//   kBufN = 1<<16 (3.6 MB, 18.2 h @ 1 Hz). Smaller N than sensor streams.
//   thread context: SDK push callback (mid360_command_handler.cpp:86) →
//     Box hot-path is lock-free atomic fetch_add, multi-writer safe.
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
  if (g_box.fd < 0) return;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return;
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

// ─────────────────────────────────────────────────────────────
// Host resource stream — 1 Hz polling from ResourceLogger node.
//   4 files share one tick (same t_sample_ns): global.bin + 3 proc .bin.
//   kBufN = 1<<16 (18.2 h @ 1 Hz). global.bin 28 MB, *_proc.bin 3 MB.
// ─────────────────────────────────────────────────────────────

namespace resource {

inline constexpr size_t kBufN = 1 << 16;

enum ProcSlot : int { kImage = 0, kSlam = 1, kLivox = 2, kProcSlotCount = 3 };

inline detail::Box<GlobalRecord, kBufN> g_global;
inline detail::Box<ProcRecord,   kBufN> g_image_proc;
inline detail::Box<ProcRecord,   kBufN> g_slam_proc;
inline detail::Box<ProcRecord,   kBufN> g_livox_proc;

inline void init(const std::string& dir) {
  g_global.init    (dir + "/global.bin");
  g_image_proc.init(dir + "/image_proc.bin");
  g_slam_proc.init (dir + "/slam_proc.bin");
  g_livox_proc.init(dir + "/livox_proc.bin");
}

inline void shutdown() {
  g_global.shutdown();
  g_image_proc.shutdown();
  g_slam_proc.shutdown();
  g_livox_proc.shutdown();
}

// store: sampler fills payload, box assigns seq/t_sample_ns.
inline void store_global(GlobalRecord r, uint64_t t_sample_ns) {
  if (g_global.fd < 0) return;
  size_t i = g_global.next_idx();
  if (i == SIZE_MAX) return;
  r.seq         = g_global.next_seq();
  r.t_sample_ns = t_sample_ns;
  g_global.base[i] = r;
}

inline void store_proc(ProcSlot slot, ProcRecord r, uint64_t t_sample_ns) {
  detail::Box<ProcRecord, kBufN>* box = nullptr;
  switch (slot) {
    case kImage: box = &g_image_proc; break;
    case kSlam:  box = &g_slam_proc;  break;
    case kLivox: box = &g_livox_proc; break;
    default:     return;
  }
  if (box->fd < 0) return;
  size_t i = box->next_idx();
  if (i == SIZE_MAX) return;
  r.seq         = box->next_seq();
  r.t_sample_ns = t_sample_ns;
  box->base[i] = r;
}

}  // namespace resource

}  // namespace blackbox
