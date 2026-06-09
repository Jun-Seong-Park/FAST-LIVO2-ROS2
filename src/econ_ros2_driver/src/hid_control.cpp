/**
 * @file hid_control.cpp
 * @brief See3CAM_24CUG HID Extension Unit control — implementation.
 *
 * The public surface is the two functions in hid_control.hpp. Everything else here — HID opcodes,
 * packet framing, write_read transport, per-setting SET/GET, the readback check — is internal to
 * this TU (anonymous namespace). The driver's HID policy is fixed (external TRIGGER, AFL off,
 * sensor flip off); only exposure_us varies, so it is the sole runtime argument.
 */

#include "hid_control.hpp"

#include <rclcpp/logging.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

namespace see3cam24cug::hid {
namespace {

// ── HID protocol constants ──
constexpr uint8_t kCameraControl24CUG = 0xA8;
constexpr uint8_t kGetStreamMode      = 0x1B;
constexpr uint8_t kSetStreamMode      = 0x1C;
constexpr uint8_t kOsCode             = 0x70;
constexpr uint8_t kLinuxOs            = 0x01;
constexpr uint8_t kGetExposureComp    = 0x11;
constexpr uint8_t kSetExposureComp    = 0x12;
constexpr uint8_t kGetFlipMode        = 0x0B;
constexpr uint8_t kSetFlipMode        = 0x0C;

// ── fixed HID policy: the values this driver always SETs and verifies the readback against ──
constexpr uint8_t kStreamTrigger = 0x01;   // stream_mode = TRIGGER
constexpr uint8_t kAflOff        = 0x00;   // Auto Function Lock OFF
constexpr uint8_t kHidFlipOff    = 0x00;   // sensor flip register OFF = upright

constexpr int kHidPkt  = 65;
constexpr int kHidResp = 64;

/// Short timeout (s) for the confirmatory readback GETs. Bounds the constructor stall if a GET
/// hangs; write_read's select() fails gracefully, so a hang becomes GET_FAILED, never a block.
constexpr int kCheckTimeoutSec = 2;

bool write_read(int fd, const uint8_t *cmd, uint8_t *resp,
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
    if (n >= kHidResp && resp[0] == cmd[1] && resp[1] == cmd[2]) { return true; }
    if (n < 0 && errno == EAGAIN) { usleep(100000); continue; }
    if (n > 0) { continue; }
    RCLCPP_ERROR(logger, "HID read failed: n=%d errno=%d", n, errno);
    return false;
  }
  RCLCPP_ERROR(logger, "HID response not matched after retries");
  return false;
}

void drain(int fd, rclcpp::Logger /*logger*/)
{
  uint8_t junk[kHidResp];
  int drained = 0;
  while (read(fd, junk, kHidResp) > 0) { drained++; }
  (void)drained;
}

bool init(int fd, rclcpp::Logger logger)
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

bool set_stream_mode(int fd, uint8_t mode, uint8_t afl, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kSetStreamMode;
  cmd[3] = mode; cmd[4] = afl;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 5, logger)) { return false; }
  bool ok = (resp[6] == 0x01);
  if (!ok) {
    RCLCPP_ERROR(logger, "\033[31mSET stream mode: %s AFL=%s FAILED\033[0m",
                 mode == 0x01 ? "TRIGGER" : "MASTER",
                 afl  == 0x01 ? "ON"      : "OFF");
  }
  return ok;
}

bool set_exposure(int fd, uint32_t exposure_us, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG;
  cmd[2] = kSetExposureComp;
  cmd[3] = (exposure_us >> 24) & 0xFF;   // big-endian
  cmd[4] = (exposure_us >> 16) & 0xFF;
  cmd[5] = (exposure_us >>  8) & 0xFF;
  cmd[6] =  exposure_us        & 0xFF;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 7, logger)) { return false; }
  bool ok = (resp[6] == 0x01);
  if (!ok) {
    RCLCPP_ERROR(logger, "\033[31mSET exposure: %u us FAILED\033[0m", exposure_us);
  }
  return ok;
}

/// Writes the sensor flip/orientation register (0xA8 0x0C) to OFF (kHidFlipOff). The value is fixed;
/// the call is kept on every start because a comment in the original driver claimed the register
/// write re-latches the sensor readout and clears cold-boot tearing. That causal claim is
/// UNVERIFIED, so the write is retained conservatively rather than as a proven fix.
bool set_flip_mode(int fd, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kSetFlipMode;
  cmd[3] = kHidFlipOff;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 5, logger)) { return false; }
  bool ok = (resp[6] == 0x01);
  if (!ok) {
    RCLCPP_ERROR(logger, "\033[31mSET flip mode: 0x%02X FAILED\033[0m", kHidFlipOff);
  }
  return ok;
}

bool get_stream_mode(int fd, uint8_t &mode, uint8_t &afl, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kGetStreamMode;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, 5, logger)) { return false; }
  mode = resp[2]; afl = resp[3];
  if (mode != 0x01) {
    RCLCPP_ERROR(logger, "\033[31mGET stream mode: %s AFL=%s NOT_TRIGGER\033[0m",
                 mode == 0x01 ? "TRIGGER" : "MASTER",
                 afl  == 0x01 ? "ON"      : "OFF");
  }
  return (resp[6] == 0x01);
}

bool get_exposure(int fd, uint32_t &exposure_us, int timeout_sec, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kGetExposureComp;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, timeout_sec, logger)) { return false; }
  exposure_us = (static_cast<uint32_t>(resp[2]) << 24) |   // big-endian 4 bytes
                (static_cast<uint32_t>(resp[3]) << 16) |
                (static_cast<uint32_t>(resp[4]) <<  8) |
                 static_cast<uint32_t>(resp[5]);
  return (resp[6] == 0x01);
}

bool get_flip(int fd, uint8_t &flip, int timeout_sec, rclcpp::Logger logger)
{
  uint8_t cmd[kHidPkt] = {};
  cmd[1] = kCameraControl24CUG; cmd[2] = kGetFlipMode;
  uint8_t resp[kHidResp] = {};
  if (!write_read(fd, cmd, resp, timeout_sec, logger)) { return false; }
  flip = resp[2];
  return (resp[6] == 0x01);
}

/// Reads back the HID settings the driver just SET, compares each to the fixed policy (and the one
/// runtime value, exposure_us), and writes a human-readable snapshot to <session_dir>/econ_hid_check.txt.
/// ONE-SHOT init record (off the hot path) — txt is the right medium; the binary mmap blackbox owns
/// per-frame telemetry.
///
/// n_applied = how many SETs succeeded before this call:
///   -1 device open failed · 0 exposure SET failed · 1 stream_mode SET failed · 2 flip SET failed · 3 all OK.
/// A setting whose SET failed is recorded SET_FAILED; one never reached, NOT_APPLIED (neither is GET'd).
/// Observational: mismatches are logged + written but do NOT fail here. Returns whether stream_mode
/// reads back as TRIGGER — the caller's hard trigger-mode gate.
bool write_settings_check(int fd, uint32_t exposure_us, const char *hid_device,
                          const std::string &session_dir, int n_applied, rclcpp::Logger logger)
{
  const char *stage =
    n_applied < 0  ? "OPEN_FAIL" :
    n_applied == 0 ? "EXPOSURE_SET_FAIL" :
    n_applied == 1 ? "STREAMMODE_SET_FAIL" :
    n_applied == 2 ? "FLIP_SET_FAIL" : "OK";

  // ── device identity: kernel-cached ioctls (no USB wire traffic, no trigger-timeout risk) ──
  char name[256] = {};
  uint16_t vid = 0, pid = 0;
  if (fd >= 0) {
    ioctl(fd, HIDIOCGRAWNAME(256), name);
    struct hidraw_devinfo info = {};
    if (ioctl(fd, HIDIOCGRAWINFO, &info) == 0) {
      vid = static_cast<uint16_t>(info.vendor);
      pid = static_cast<uint16_t>(info.product);
    }
  }

  // ── readback only the settings whose SET succeeded ──
  uint32_t exp_val = 0; bool exp_get = false;
  uint8_t  sm_mode = 0, sm_afl = 0; bool sm_get = false;
  uint8_t  fl_val = 0; bool fl_get = false;
  if (n_applied >= 1) { exp_get = get_exposure(fd, exp_val, kCheckTimeoutSec, logger); }
  if (n_applied >= 2) { sm_get  = get_stream_mode(fd, sm_mode, sm_afl, logger); }
  if (n_applied >= 3) { fl_get  = get_flip(fd, fl_val, kCheckTimeoutSec, logger); }

  // ── per-setting (actual, result) ── compared to the fixed policy + exposure_us ──
  auto u8hex = [](uint8_t v) { char b[8]; std::snprintf(b, sizeof(b), "0x%02X", v); return std::string(b); };
  const std::string sm_name_want = kStreamTrigger == 0x01 ? "0x01 TRIGGER" : "0x00 MASTER";

  std::string exp_actual, exp_result;
  if (n_applied >= 1) {
    exp_actual = exp_get ? std::to_string(exp_val) : "GET_FAILED";
    exp_result = !exp_get ? "GET_FAILED" : (exp_val == exposure_us ? "PASS" : "MISMATCH");
  } else { exp_actual = "-"; exp_result = (n_applied == 0) ? "SET_FAILED" : "NOT_APPLIED"; }

  std::string sm_actual, sm_result, afl_actual, afl_result;
  if (n_applied >= 2) {
    if (!sm_get) { sm_actual = "GET_FAILED"; sm_result = "GET_FAILED"; afl_actual = "GET_FAILED"; afl_result = "GET_FAILED"; }
    else {
      sm_actual  = sm_mode == 0x01 ? "0x01 TRIGGER" : "0x00 MASTER";
      sm_result  = sm_mode == kStreamTrigger ? "PASS" : "MISMATCH";
      afl_actual = sm_afl ? "0x01 ON" : "0x00 OFF";
      afl_result = sm_afl == kAflOff ? "PASS" : "MISMATCH";
    }
  } else {
    sm_actual = "-"; sm_result = (n_applied == 1) ? "SET_FAILED" : "NOT_APPLIED";
    afl_actual = "-"; afl_result = sm_result;
  }

  std::string fl_actual, fl_result;
  if (n_applied >= 3) {
    fl_actual = fl_get ? u8hex(fl_val) : "GET_FAILED";
    fl_result = !fl_get ? "GET_FAILED" : (fl_val == kHidFlipOff ? "PASS" : "MISMATCH");
  } else { fl_actual = "-"; fl_result = (n_applied == 2) ? "SET_FAILED" : "NOT_APPLIED"; }

  // ── timestamps ──
  struct timespec tsr; clock_gettime(CLOCK_MONOTONIC_RAW, &tsr);
  const uint64_t mono_ns = static_cast<uint64_t>(tsr.tv_sec) * 1000000000ULL + tsr.tv_nsec;
  std::time_t wt = std::time(nullptr); std::tm tmv{}; localtime_r(&wt, &tmv);
  char wall[32]; std::strftime(wall, sizeof(wall), "%Y-%m-%d %H:%M:%S", &tmv);

  // ── write the txt (the session dir already exists: blackbox::image::init created it in the ctor) ──
  const std::string path = session_dir + "/econ_hid_check.txt";
  std::ofstream os(path);
  if (!os) {
    RCLCPP_WARN(logger, "HID check: cannot open %s (txt skipped)", path.c_str());
    return (n_applied >= 2) && sm_get && (sm_mode == kStreamTrigger);
  }

  auto row = [&](const char *setting, const char *getop, const std::string &intended,
                 const std::string &actual, const std::string &result) {
    char line[160];
    std::snprintf(line, sizeof(line), "%-14s %-5s %-15s %-15s %s\n",
                  setting, getop, intended.c_str(), actual.c_str(), result.c_str());
    os << line;
  };

  os << "# econ_ros2_driver HID config check\n";
  os << "# stage     : " << stage << "\n";
  os << "# session   : " << session_dir << "\n";
  os << "# wall      : " << wall << " (local)\n";
  os << "# mono_ns   : " << mono_ns << "   (CLOCK_MONOTONIC_RAW, same domain as the .bin)\n";
  os << "# hid_device: " << hid_device << "\n";
  if (fd >= 0) {
    const char *variant = pid == 0xC124 ? "default variant"
                        : pid == 0xC128 ? "alternate variant"
                                        : "unknown variant";
    char idline[96];
    std::snprintf(idline, sizeof(idline), "# vid:pid   : 0x%04X:0x%04X  (%s)\n", vid, pid, variant);
    os << "# hid_name  : " << name << "\n" << idline;
  } else {
    os << "# hid_name  : UNAVAILABLE (device not opened)\n";
  }
  os << "# fw_ver    : UNAVAILABLE (no HID opcode; SDK ReadFirmwareVersion only)\n";
  os << "# unique_id : UNAVAILABLE (no HID opcode; SDK GetCameraUniqueID only)\n";
  os << "# note      : GET issued in TRIGGER mode, pre-STREAMON\n";
  os << "#\n";
  os << "# setting       get   intended        actual          result\n";
  row("exposure_us", "0x11", std::to_string(exposure_us), exp_actual, exp_result);
  row("stream_mode", "0x1B", sm_name_want, sm_actual, sm_result);
  row("  afl", "0x1B", kAflOff ? "0x01 ON" : "0x00 OFF", afl_actual, afl_result);
  row("flip", "0x0B", u8hex(kHidFlipOff), fl_actual, fl_result);
  os << "#\n";
  os << "# not set by this driver (camera defaults, NOT verified):\n";
  os << "#   denoise, q_factor, exp_roi, strobe, frame_rate\n";

  auto tally = [](const std::string &r, int &pass, int &checked) {
    if (r == "NOT_APPLIED") { return; }
    ++checked; if (r == "PASS") { ++pass; }
  };
  int pass = 0, checked = 0;
  tally(exp_result, pass, checked); tally(sm_result, pass, checked);
  tally(afl_result, pass, checked); tally(fl_result, pass, checked);
  os << "#\n# summary: " << pass << "/" << checked << " PASS  (stage " << stage << ")\n";

  if (pass != checked) {
    RCLCPP_WARN(logger, "HID check: %d/%d PASS (stage %s) — see %s", pass, checked, stage, path.c_str());
  } else {
    RCLCPP_INFO(logger, "HID check: %d/%d PASS — %s", pass, checked, path.c_str());
  }
  return (n_applied >= 2) && sm_get && (sm_mode == kStreamTrigger);
}

}  // namespace

bool open_and_init_trigger(const char* device, uint32_t exposure_us,
                           const std::string& session_dir, int& fd_out, rclcpp::Logger logger)
{
  fd_out = ::open(device, O_RDWR | O_NONBLOCK);
  if (fd_out < 0) {
    RCLCPP_FATAL(logger, "Cannot open HID %s: %s", device, strerror(errno));
    write_settings_check(-1, exposure_us, device, session_dir, -1, logger);
    return false;
  }
  init(fd_out, logger);   // OS handshake; best-effort, always returns true
  // SET order follows the official trigger entry sequence (trigger.md): camera settings → trigger
  // mode. The flip register is written too (value OFF) pre-STREAMON; see set_flip_mode for why the
  // call is kept on every start despite the value being fixed.
  if (!set_exposure(fd_out, exposure_us, logger)) {
    write_settings_check(fd_out, exposure_us, device, session_dir, 0, logger);
    return false;
  }
  if (!set_stream_mode(fd_out, kStreamTrigger, kAflOff, logger)) {
    write_settings_check(fd_out, exposure_us, device, session_dir, 1, logger);
    return false;
  }
  if (!set_flip_mode(fd_out, logger)) {
    write_settings_check(fd_out, exposure_us, device, session_dir, 2, logger);
    return false;
  }
  // Full readback verify of all three + device identity → econ_hid_check.txt. Returns the
  // trigger-mode gate; everything else is observational (logged/written, non-fatal).
  if (!write_settings_check(fd_out, exposure_us, device, session_dir, 3, logger)) {
    RCLCPP_FATAL(logger, "TRIGGER mode verification failed");
    return false;
  }
  return true;
}

void close(int& fd)
{
  if (fd < 0) { return; }
  ::close(fd);
  fd = -1;
}

}  // namespace see3cam24cug::hid
