#pragma once
// LiDAR 측 (livox_ros_driver2_sync) 가 mmap 파일에 GPS epoch ns 를 기록 →
// 카메라 측에서 읽어서 image header.stamp 에 박음. 같은 time domain 보장.
//
// 레이아웃은 LIV_handhold grab_trigger.cpp 와 호환 유지 (16B fixed).
// ARM64: 8B aligned 64-bit load 는 architecturally atomic — barrier 불필요.

#include <cstdint>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

namespace see3cam {

struct SharedTimestamp {
  int64_t high;
  int64_t low;
};
static_assert(sizeof(SharedTimestamp) == 16,
              "must match LIV_handhold grab_trigger layout");

class MmapStamper {
 public:
  explicit MmapStamper(std::string path) : path_(std::move(path)) {}
  ~MmapStamper() { close(); }

  // lazy open — 첫 try 시 LiDAR 가 아직 mmap 파일 안 만들었을 수 있음.
  // 한 번 성공하면 이후 no-op. 핫패스에서 매 콜백마다 호출되니 cheap path 보장.
  void try_open() {
    if (opened_) return;
    fd_ = ::open(path_.c_str(), O_RDWR);
    if (fd_ < 0) return;
    void *m = mmap(nullptr, sizeof(SharedTimestamp),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (m == MAP_FAILED) { ::close(fd_); fd_ = -1; return; }
    stamp_ = static_cast<SharedTimestamp *>(m);
    opened_ = true;
  }

  // 못 열렸으면 0 반환. 호출자는 0 검사 후 fallback now() 결정.
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

}  // namespace see3cam
