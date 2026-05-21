// gprmc_feeder.cpp
//
// 책임 범위:
//   - Synchro 인스턴스(직렬 포트 reader)는 그대로 사용
//   - 한 번만 시작 (started_ flag) — 여러 LiDAR detection 콜백 중복 방어
//   - GPRMC 콜백 안에서 SetLivoxLidarRmcSyncTime(handle, ...) 호출
//
// 자세한 흐름은 gprmc_feeder.h 참조.

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

  // 첫 호출에서만 Synchro 시작 — 멀티 lidar / 콜백 재발생 보호
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true)) {
    handle_.store(handle);  // 후속 콜백에서도 handle 만 갱신
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

  // 콜백 안에서는 handle_ 을 매번 atomic 으로 읽어 최신값 사용
  syn.SetSyncTimerCallback([this](const char* rmc, uint16_t rmc_length) {
    uint32_t h = handle_.load();
    if (h == 0) return;
    SetLivoxLidarRmcSyncTime(
        h, rmc, rmc_length,
        [](livox_status status, uint32_t /*handle*/,
           LivoxLidarRmcSyncTimeResponse* /*data*/, void* /*client_data*/) {
          (void)status;  // ack 만 — 응답은 무시
        },
        nullptr);
  });

  if (!syn.Start()) {
    std::cerr << "[gprmc_feeder] Synchro::Start failed (port="
              << port_local << ")" << std::endl;
    started_.store(false);  // 다시 시도 가능하도록
  }
}

void GprmcFeeder::Stop() {
  if (!started_.load()) return;
  Synchro::GetInstance().Stop();
  started_.store(false);
  handle_.store(0);
}

}  // namespace livox_ros
