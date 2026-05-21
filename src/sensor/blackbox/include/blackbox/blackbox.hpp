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
// Trace root: $HOME/FAST-LIVO2-ROS2/trace/log
// ─────────────────────────────────────────────────────────────

inline std::string trace_log_dir() {
  const char* home = ::getenv("HOME");
  return std::string(home ? home : "/tmp") + "/FAST-LIVO2-ROS2/trace/log";
}

// ─────────────────────────────────────────────────────────────
// Common: MonoRawNs
// ─────────────────────────────────────────────────────────────

inline uint64_t MonoRawNs() {
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
  uint64_t t_dqbuf_ns;   // 8B  pad probe: MonoRawNs() — DQBUF 완료, pipeline 진입 시각
  uint64_t t_cbk_ns;         // 8B  cbk: MonoRawNs()
  uint64_t t_pub_ns;         // 8B  pub: MonoRawNs()
  uint8_t  gst_done;         // 1B
  uint8_t  ros_done;         // 1B
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
  r.t_cbk_ns         = MonoRawNs();
  r.t_pub_ns         = 0;
  r.gst_done         = 1;
  r.ros_done         = 0;
  return i;
}

// publish thread: t_pub + ros_done 박음.
inline void log_pub(size_t i) {
  if (g_box.fd < 0 || i == SIZE_MAX || i >= kBufN) return;
  ImagePubRecord& r = g_box.base[i];
  r.t_pub_ns = MonoRawNs();
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
  r.t_pub_ns     = MonoRawNs();
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
  r.t_pub_ns     = MonoRawNs();
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
  r.t_cbk_ns     = MonoRawNs();
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
  r.t_cbk_ns     = MonoRawNs();
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
  r.t_cbk_ns     = MonoRawNs();
}

}  // namespace sub_image

}  // namespace blackbox
