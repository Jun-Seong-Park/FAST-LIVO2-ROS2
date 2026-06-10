// gprmc_feeder.h
//
// Reads GPRMC line by line from the host serial port and forwards it to the LiDAR
// (calls the SDK's SetLivoxLidarRmcSyncTime).  Absorbs the work that
// livox-gps-sync.service used to do into the driver.
//
// Flow:
//   1. DriverNode reads ROS params and calls Configure(port, baud, enabled).
//   2. OnLidarHandle(handle) is called inside the SDK's LidarInfoChangeCallback
//      → on the first call, starts the Synchro thread and begins forwarding GPRMC with that handle.
//   3. Stop() on driver shutdown.
//
// If `enabled = false`, nothing runs at all (same as the vanilla original).

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
