#pragma once

/**
 * @file econ_ros2_driver.hpp
 * @brief See3CAM_24CUG ROS2 driver node — declaration.
 *
 * Holds the device/exposure/trigger defaults, the resolution profiles, and the node class
 * declaration. Member function bodies live in econ_ros2_driver.cpp.
 *
 * The constexpr k* values are the declare_parameter "defaults"; config/config.yaml (or a launch
 * override) overrides them. Resolution is not a numeric parameter — a resolution-profile string is
 * resolved by resolve_profile_uyvy()/resolve_profile_mjpg() into capture/output/fps (per-format).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#include "gst_backend.hpp"
#include "mmap_stamper.hpp"

// ── Resolution profile ──
// output = published resolution after nvvidconv. Only sd downscales (1280x720→640x360); the rest
// are full-frame. fps is nominal for caps negotiation — under trigger the real rate is external.
// Each table is a 1:1 copy of v4l2-ctl --list-formats-ext (/dev/24cug); the per-format menus differ
// (UYVY vs MJPG), so the profiles are split per format.
struct Resolution {
  int capture_width;
  int capture_height;
  int output_width;
  int output_height;
  int fps;
};

/// A 90° rotation (CCW=1 / CW=3) swaps output width·height. 0(none)·2(180°) keep dimensions.
inline bool is_quarter_turn(int flip_method) { return flip_method == 1 || flip_method == 3; }

/// UYVY (raw, compressed=false) capture profile.
/// v4l2-ctl UYVY menu: 1280x720=60/120, 1920x1080=60, 1920x1200=55 (no 60).
inline Resolution resolve_profile_uyvy(const std::string& profile) {
  if (profile == "hd")    { return {1280,  720, 1280,  720, 60}; }
  if (profile == "fhd")   { return {1920, 1080, 1920, 1080, 60}; }
  if (profile == "wuxga") { return {1920, 1200, 1920, 1200, 55}; } // UYVY 1920x1200 negotiates only at 55
  // "sd" and anything else → default sd (capture HD, output 640x360 downscale)
  return {1280, 720, 640, 360, 60};
}

/// MJPG (compressed=true) capture profile.
/// v4l2-ctl MJPG menu: 1280x720=60/120, 1920x1080=30/60/120, 1920x1200=60/114.
inline Resolution resolve_profile_mjpg(const std::string& profile) {
  if (profile == "hd")    { return {1280,  720, 1280,  720, 60}; }
  if (profile == "fhd")   { return {1920, 1080, 1920, 1080, 60}; }
  if (profile == "wuxga") { return {1920, 1200, 1920, 1200, 60}; } // MJPG 1920x1200 = 60/114
  // "sd" and anything else → default sd (capture HD, output 640x360 downscale)
  return {1280, 720, 640, 360, 60};
}

// ── Runtime parameters ──
struct Params {
  std::string device;          // udev symlink (capture)
  std::string hid_device;      // udev symlink (HID control)
  std::string resolution;      // profile string (sd | hd | fhd | wuxga)
  Resolution  res;             // result of resolving resolution via resolve_profile()
  int         flip_method;     // nvvidconv rotation enum (0 none / 1 CCW / 2 180 / 3 CW). gstreamer only
  int         pub_width;       // derived — published width after rotation (output_height on 90° rotation)
  int         pub_height;      // derived — published height after rotation (output_width on 90° rotation)
  size_t      expected_size;   // derived — pub_width * pub_height * 3 (BGR 3ch). rotation-invariant
  int         exposure_us;
  bool        compressed;      // true → MJPG CompressedImage, false → raw bgr8 Image
  std::string topic_name;
  std::string frame_id;
};

class EconRos2Driver : public rclcpp::Node
{
 public:
  EconRos2Driver();
  ~EconRos2Driver() override;

 private:
  // ── declare_parameter defaults (overridable via config/config.yaml) ──
  static constexpr const char* kDevice     = "/dev/24cug";
  static constexpr const char* kHidDevice  = "/dev/24cug_hid";
  static constexpr const char* kResolution = "sd";
  static constexpr int         kExposureUs = 10000;   // 10 ms
  static constexpr int         kFlipMethod = 0;       // no rotation by default
  static constexpr uint8_t     kAflMode    = 0x00;    // Auto Frame Length OFF (hardcoded)
  static constexpr bool        kCompressed = false;
  static constexpr const char* kTopicName  = "/camera/image";
  static constexpr const char* kFrameId    = "camera_init";

  /// mmap file shared with the LiDAR trigger process (GPS-epoch ns timestamp source).
  static std::string resolve_shared_path();
  /// Declare each parameter with the defaults above, read overrides, and derive res/dimensions.
  Params load_params();
  /// Only the active format's publisher/buffer is set up; the other stays unused.
  void   setup_publisher();
  /// HID external-trigger mode + exposure.
  bool   init_hid();
  /// Start the GStreamer capture+convert backend in the chosen format.
  bool   init_backend();
  /// Main capture loop (grab thread): grab → publish → release.
  void   loop();
  /// Stamp from the shared mmap (same time domain as LiDAR), copy into the reusable buffer, publish.
  void   publish(const Frame& frame);
  /// Stop the grab loop, join the thread, stop the backend, close the stamper and HID.
  void   shutdown();

  // QoS: best effort
  const rclcpp::SensorDataQoS qos_;

  Params                              p_;            // runtime config
  MmapStamper                         stamper_;
  int                                 hid_fd_;
  GstBackend                          backend_;      // GStreamer capture+convert (only backend)
  std::thread                         grab_thread_;
  std::atomic<bool>                   stop_loop_;
  uint64_t                            n_pull_{0};           // successful grabs (single thread)
  uint64_t                            last_push_seen_{0};
  uint64_t                            total_drops_{0};
  sensor_msgs::msg::Image             msg_;          // reusable raw buffer (compressed=false)
  sensor_msgs::msg::CompressedImage   cmsg_;         // reusable MJPG buffer (compressed=true)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_;   // raw
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cpublisher_;  // compressed
};
