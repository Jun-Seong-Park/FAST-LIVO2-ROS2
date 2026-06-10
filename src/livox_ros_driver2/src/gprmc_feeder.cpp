// gprmc_feeder.cpp
//
// Scope of responsibility:
//   - Uses the Synchro instance (serial port reader) as-is
//   - Starts only once (started_ flag) — guards against duplicate LiDAR detection callbacks
//   - Calls SetLivoxLidarRmcSyncTime(handle, ...) inside the GPRMC callback
//
// See gprmc_feeder.h for the detailed flow.

#include "gprmc_feeder.h"

#include <iostream>

#include "livox_lidar_api.h"
#include "synchro.h"

namespace livox_ros {

namespace {

BaudRate ParseBaud(int baud) {
  switch (baud) {
    case 2400:    return BR2400;
    case 4800:    return BR4800;
    case 9600:    return BR9600;
    case 19200:   return BR19200;
    case 38400:   return BR38400;
    case 57600:   return BR57600;
    case 115200:  return BR115200;
    case 230400:  return BR230400;
    default:      return BR9600;
  }
}

Parity ParseParity(const std::string& s) {
  if (s == "7E1") return P_7E1;
  if (s == "7O1") return P_7O1;
  if (s == "7S1") return P_7S1;
  return P_8N1;  // default
}

}  // namespace

GprmcFeeder& GprmcFeeder::GetInstance() {
  static GprmcFeeder inst;
  return inst;
}

void GprmcFeeder::Configure(const std::string& port, int baud,
                            const std::string& parity, bool enabled) {
  std::lock_guard<std::mutex> lk(mu_);
  port_ = port;
  baud_ = baud;
  parity_ = parity;
  enabled_ = enabled;
}

void GprmcFeeder::OnLidarHandle(uint32_t handle) {
  if (!enabled_) return;

  // Start Synchro only on the first call — protects against multi-lidar / re-fired callbacks
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true)) {
    handle_.store(handle);  // On subsequent callbacks, only update the handle
    return;
  }

  handle_.store(handle);

  std::string port_local;
  int baud_local;
  std::string parity_local;
  {
    std::lock_guard<std::mutex> lk(mu_);
    port_local = port_;
    baud_local = baud_;
    parity_local = parity_;
  }

  Synchro& syn = Synchro::GetInstance();
  syn.SetPortName(port_local);
  syn.SetBaudRate(ParseBaud(baud_local));
  syn.SetParity(ParseParity(parity_local));

  // Inside the callback, read handle_ atomically each time to use the latest value
  syn.SetSyncTimerCallback([this](const char* rmc, uint16_t rmc_length) {
    uint32_t h = handle_.load();
    if (h == 0) return;
    SetLivoxLidarRmcSyncTime(
        h, rmc, rmc_length,
        [](livox_status status, uint32_t /*handle*/,
           LivoxLidarRmcSyncTimeResponse* /*data*/, void* /*client_data*/) {
          (void)status;  // ack only — ignore the response
        },
        nullptr);
  });

  if (!syn.Start()) {
    std::cerr << "[gprmc_feeder] Synchro::Start failed (port="
              << port_local << ")" << std::endl;
    started_.store(false);  // so it can be retried
  }
}

void GprmcFeeder::Stop() {
  if (!started_.load()) return;
  Synchro::GetInstance().Stop();
  started_.store(false);
  handle_.store(0);
}

}  // namespace livox_ros
