#pragma once
// blackbox (econ_ros2_driver vendored subset) — image-publish stream only.
// mmap-fixed-array + 1 s fdatasync writer thread.
//
// Vendored from the fast_livo2 blackbox package; only the parts this node uses are kept
// (mono_raw_ns, log_dir, the image-publish cascade). Other streams (lidar/imu/host/resource)
// were intentionally dropped.
//
// Lifecycle (caller 책임):
//   blackbox::image::init(path)    — open + fallocate + mmap + writer thread
//   blackbox::image::log_cbk(...)  — hot path (cbk thread): returns record idx
//   blackbox::image::log_pub(idx)  — hot path (publish): stamps t_pub
//   blackbox::image::shutdown()    — stop writer + final fdatasync + munmap + close

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <ctime>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace blackbox {

// Per-run log session, ros2-logger style:  ~/.blackbox/log/<YYYY-MM-DD-HH-MM-SS>-<pid>/
// A fresh directory each run → every session is preserved, never overwritten.
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

inline uint64_t mono_raw_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

// Image publish record (cbk → publish cascade). All times MONOTONIC_RAW.
struct ImagePubRecord {
  uint64_t seq;              // 8B
  uint64_t header_stamp;     // 8B  ROS header.stamp (GPS-epoch ns from mmap)
  uint64_t t_capture_ns;     // 8B  backend capture stamp (gst src-pad probe / v4l2 DQBUF)
  uint64_t t_cbk_ns;         // 8B  log_cbk 시각
  uint64_t t_pub_ns;         // 8B  log_pub 시각
  uint8_t  cap_done;         // 1B  flag
  uint8_t  ros_done;         // 1B  flag
  uint8_t  _pad[6];          // 6B
  // 구간:  캡처→cbk = t_cbk_ns - t_capture_ns,  cbk→pub = t_pub_ns - t_cbk_ns
};
static_assert(sizeof(ImagePubRecord) == 48, "ImagePubRecord ABI fixed at 48B");

// ─────────────────────────────────────────────────────────────
// detail: generic mmap-fixed-array + 1 s writer thread
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
  Record*               base{nullptr};
  int                   fd{-1};
  std::atomic<size_t>   idx{0};
  std::atomic<uint64_t> seq{0};
  std::atomic<bool>     running{false};
  std::thread           writer;
  std::mutex            init_mtx;

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
    if (base) { ::munmap(base, N * sizeof(Record)); base = nullptr; }
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
// Image stream — cbk + publish cascade (single grab thread here)
// ─────────────────────────────────────────────────────────────

namespace image {

inline constexpr size_t kBufN = 1 << 20;   // 48 MB ≈ 9.7 h @ 30 Hz
inline detail::Box<ImagePubRecord, kBufN> g_box;

inline void init(const std::string& path) { g_box.init(path); }
inline void shutdown() { g_box.shutdown(); }

// cbk: stamp seq/header/capture/cbk, return record idx (SIZE_MAX if full → caller skips log_pub).
inline size_t log_cbk(uint64_t header_stamp_ns, uint64_t t_capture_ns) {
  if (g_box.fd < 0) return SIZE_MAX;
  size_t i = g_box.next_idx();
  if (i == SIZE_MAX) return SIZE_MAX;
  ImagePubRecord& r = g_box.base[i];
  r.seq          = g_box.next_seq();
  r.header_stamp = header_stamp_ns;
  r.t_capture_ns = t_capture_ns;
  r.t_cbk_ns     = mono_raw_ns();
  r.t_pub_ns     = 0;
  r.cap_done     = 1;
  r.ros_done     = 0;
  return i;
}

// publish: stamp t_pub + ros_done.
inline void log_pub(size_t i) {
  if (g_box.fd < 0 || i == SIZE_MAX || i >= kBufN) return;
  ImagePubRecord& r = g_box.base[i];
  r.t_pub_ns = mono_raw_ns();
  r.ros_done = 1;
}

}  // namespace image

}  // namespace blackbox
