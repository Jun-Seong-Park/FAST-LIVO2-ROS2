#pragma once
// See3CAM_24CUG HID Extension Unit 제어.
// hidraw character device 를 통해 stream mode (TRIGGER/MASTER) 와 exposure 설정.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

namespace see3cam::hid {

// HID 프로토콜 상수
inline constexpr uint8_t kCameraControl24CUG = 0xA8;
inline constexpr uint8_t kGetStreamMode      = 0x1B;
inline constexpr uint8_t kSetStreamMode      = 0x1C;
inline constexpr uint8_t kOsCode             = 0x70;
inline constexpr uint8_t kLinuxOs            = 0x01;
inline constexpr uint8_t kSetExposureComp    = 0x12;

inline constexpr int kHidPkt  = 65;
inline constexpr int kHidResp = 64;

inline bool write_read(int fd, const uint8_t *cmd, uint8_t *resp,
                       int timeout_sec, rclcpp::Logger logger)
{
  if (write(fd, cmd, kHidPkt) != kHidPkt) {
    RCLCPP_ERROR(logger, "HID write failed: %s", strerror(errno));
    return false;
  }
  fd_set rfds; FD_ZERO(&rfds); FD_SET(fd, &rfds);
  struct timeval tv = {timeout_sec, 0};
  int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
  if (ret <= 0) {
    RCLCPP_ERROR(logger, "HID select %s", ret == 0 ? "timeout" : "error");
    return false;
  }
  for (int i = 0; i < 10; i++) {
    int n = read(fd, resp, kHidResp);
    if (n >= kHidResp && resp[0] == cmd[1] && resp[1] == cmd[2]) return true;
    if (n < 0 && errno == EAGAIN) { usleep(100000); continue; }
    if (n > 0) continue;
    RCLCPP_ERROR(logger, "HID read failed: n=%d errno=%d", n, errno);
    return false;
  }
  RCLCPP_ERROR(logger, "HID response not matched after retries");
  return false;
}

inline void drain(int fd, rclcpp::Logger /*logger*/)
{
  uint8_t junk[kHidResp];
  int drained = 0;
  while (read(fd, junk, kHidResp) > 0) drained++;
  (void)drained;
}

inline bool init(int fd, rclcpp::Logger logger)
{
  drain(fd, logger);
  int desc_size = 0;
  struct hidraw_report_descriptor rpt_desc = {};
  char name[256] = {};
  ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
  rpt_desc.size = desc_size;
  ioctl(fd, HIDIOCGRDESC, &rpt_desc);
  ioctl(fd, HIDIOCGRAWNAME(256), name);
  (void)name;
  struct hidraw_devinfo info = {};
  ioctl(fd, HIDIOCGRAWINFO, &info);  

  uint8_t out[kHidPkt] = {};
  out[1] = kOsCode; out[2] = kLinuxOs;
  if (write(fd, out, kHidPkt) != kHidPkt) {
    RCLCPP_ERROR(logger, "OS code write failed"); return false;
  }
  for (int attempt = 0; attempt < 3; attempt++) {
    usleep(500000);
    uint8_t in[kHidResp] = {};
    int n = read(fd, in, kHidResp);
    if (n > 0 && in[0] == kOsCode && in[1] == kLinuxOs && in[2] == 0x01) {
    }
  }  
  return true;
}

inline bool set_stream_mode(int fd, uint8_t mode, uint8_t afl, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kSetStreamMode;
  cmd[3] = mode; cmd[4] = afl;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 5, logger)) return false;
  bool ok = (resp[6] == 0x01);
  if (!ok) {
    RCLCPP_ERROR(logger, "\033[31mSET stream mode: %s AFL=%s FAILED\033[0m",
                 mode == 0x01 ? "TRIGGER" : "MASTER",
                 afl  == 0x01 ? "ON"      : "OFF");
  }
  return ok;
}

inline bool set_exposure(int fd, uint32_t exposure_us, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG;
  cmd[2] = kSetExposureComp;
  cmd[3] = (exposure_us >> 24) & 0xFF;   // big-endian
  cmd[4] = (exposure_us >> 16) & 0xFF;
  cmd[5] = (exposure_us >>  8) & 0xFF;
  cmd[6] =  exposure_us        & 0xFF;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 7, logger)) return false;
  bool ok = (resp[6] == 0x01);
  if (!ok) {
    RCLCPP_ERROR(logger, "\033[31mSET exposure: %u us FAILED\033[0m", exposure_us);
  }
  return ok;  
}

inline bool get_stream_mode(int fd, uint8_t &mode, uint8_t &afl,
                            rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kGetStreamMode;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 5, logger)) return false;
  mode = resp[2]; afl = resp[3];
  if (mode != 0x01) {
    RCLCPP_ERROR(logger, "\033[31mGET stream mode: %s AFL=%s NOT_TRIGGER\033[0m",
                 mode == 0x01 ? "TRIGGER" : "MASTER",
                 afl  == 0x01 ? "ON"      : "OFF");
  }
  return (resp[6] == 0x01);
}

// ─────────────────────────────────────────────────────────────
// 사용자 코드는 이 두 함수만 호출하면 됨.
// SD/HD 노드가 init_hid 멤버 함수 들고 있을 필요 없음.
// ─────────────────────────────────────────────────────────────

inline bool open_and_init_trigger(
    const char* device, uint32_t exposure_us, uint8_t afl_mode,
    int& fd_out, rclcpp::Logger logger)
{
  fd_out = ::open(device, O_RDWR | O_NONBLOCK);
  if (fd_out < 0) {
    RCLCPP_FATAL(logger, "Cannot open HID %s: %s", device, strerror(errno));
    return false;
  }
  if (!init(fd_out, logger))                              return false;
  if (!set_exposure(fd_out, exposure_us, logger))         return false;
  if (!set_stream_mode(fd_out, 0x01, afl_mode, logger))   return false;
  uint8_t mode = 0, afl = 0;
  get_stream_mode(fd_out, mode, afl, logger);
  if (mode != 0x01) {
    RCLCPP_FATAL(logger, "TRIGGER mode verification failed");
    return false;
  }
  return true;
}

inline void close_and_restore(int& fd, bool restore_master, rclcpp::Logger logger)
{
  if (fd < 0) return;
  if (restore_master) set_stream_mode(fd, 0x00, 0x00, logger);
  ::close(fd);
  fd = -1;
}

}  // namespace see3cam::hid
