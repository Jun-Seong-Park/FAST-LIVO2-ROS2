// gprmc_feeder.h
//
// 호스트 직렬포트에서 GPRMC 한 줄씩 읽어 LiDAR 에 forward (SDK 의
// SetLivoxLidarRmcSyncTime 호출).  livox-gps-sync.service 가 하던 일을
// 드라이버 안으로 흡수.
//
// 흐름:
//   1. DriverNode 가 ROS param 읽어 Configure(port, baud, enabled) 호출.
//   2. SDK 의 LidarInfoChangeCallback 안에서 OnLidarHandle(handle) 호출
//      → 첫 호출 시 Synchro 스레드 시작, 그 핸들로 GPRMC forward 시작.
//   3. 드라이버 종료 시 Stop().
//
// `enabled = false` 면 일체 작동 안 함 (원본 vanilla 와 동일).

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace livox_ros {

class GprmcFeeder {
 public:
  static GprmcFeeder& GetInstance();

  void Configure(const std::string& port, int baud, const std::string& parity, bool enabled);
  void OnLidarHandle(uint32_t handle);
  void Stop();

  GprmcFeeder(const GprmcFeeder&) = delete;
  GprmcFeeder& operator=(const GprmcFeeder&) = delete;

 private:
  GprmcFeeder() = default;
  ~GprmcFeeder() = default;

  std::mutex mu_;
  std::string port_{"/dev/ttyUSB0"};
  int baud_{9600};
  std::string parity_{"8N1"};
  bool enabled_{false};
  std::atomic<bool> started_{false};
  std::atomic<uint32_t> handle_{0};
};

}  // namespace livox_ros
