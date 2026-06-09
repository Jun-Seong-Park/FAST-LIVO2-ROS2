#pragma once

/**
 * @file mmap_stamper.hpp
 * @brief Reads the GPS-epoch ns timestamp that the LiDAR side shares via an mmap file.
 *
 * The LiDAR side (livox_ros_driver2_sync) writes GPS epoch ns into an mmap file; the
 * camera side reads it and stamps it onto image header.stamp, guaranteeing the same
 * time domain.
 *
 * The layout stays compatible with LIV_handhold grab_trigger.cpp (16B fixed).
 * ARM64: an 8B-aligned 64-bit load is architecturally atomic — no barrier needed.
 */

#include <cstdint>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

struct SharedTimestamp {
  int64_t high; /// legacy field, unused
  int64_t low;  /// GPS epoch ns written by the LiDAR side
};
static_assert(sizeof(SharedTimestamp) == 16,
              "must match LIV_handhold grab_trigger layout");

class MmapStamper {
 public:
  explicit MmapStamper(std::string path) : path_(std::move(path)) {}
  ~MmapStamper() { close(); }

  /// Lazy open — on the first try the LiDAR may not have created the mmap file yet.
  /// No-op once it succeeds. Called on the hot path every callback, so keep it a cheap path.
  void try_open() {
    if (opened_) { return; }
    fd_ = ::open(path_.c_str(), O_RDWR);
    if (fd_ < 0) { return; }
    void *m = mmap(nullptr, sizeof(SharedTimestamp),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (m == MAP_FAILED) { ::close(fd_); fd_ = -1; return; }
    stamp_ = static_cast<SharedTimestamp *>(m);
    opened_ = true;
  }

  /// Returns 0 if not opened yet; the caller checks for 0 and falls to error.
  int64_t read_low_ns() const {
    return stamp_ ? stamp_->low : 0;
  }

  bool opened() const { return opened_; }
  const std::string &path() const { return path_; }

  void close() {
    if (stamp_) { munmap(stamp_, sizeof(SharedTimestamp)); stamp_ = nullptr; }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    opened_ = false;
  }

 private:
  std::string      path_;
  SharedTimestamp* stamp_  = nullptr;
  int              fd_     = -1;
  bool             opened_ = false;
};
