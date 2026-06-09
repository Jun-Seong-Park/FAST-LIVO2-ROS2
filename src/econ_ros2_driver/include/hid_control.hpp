#pragma once

/**
 * @file hid_control.hpp
 * @brief See3CAM_24CUG HID Extension Unit control — public surface (declarations only).
 *
 * This driver's HID policy is fixed: external TRIGGER mode, AFL off, sensor flip off. Those values
 * are baked into hid_control.cpp; the only runtime input is exposure_us. All HID protocol/transport
 * detail (opcodes, packet framing, per-setting SET/GET, the readback check) is internal to the .cpp
 * (anonymous namespace) — user code only needs the two functions below.
 */

#include <rclcpp/logger.hpp>

#include <cstdint>
#include <string>

namespace see3cam24cug::hid {

/// Open the hidraw device, run the OS handshake, then SET pre-STREAMON in trigger-entry order:
///   exposure_us → stream-mode TRIGGER (AFL off) → sensor flip off.
/// Reads every setting back and writes <session_dir>/econ_hid_check.txt. Returns true only when
/// stream-mode reads back as TRIGGER (the hard gate); other mismatches are observational. On
/// success fd_out holds the open descriptor; on failure it is left as opened (caller still closes).
bool open_and_init_trigger(const char* device, uint32_t exposure_us,
                           const std::string& session_dir, int& fd_out, rclcpp::Logger logger);

/// Close the descriptor opened by open_and_init_trigger (no-op if already closed); sets fd to -1.
void close(int& fd);

}  // namespace see3cam24cug::hid
