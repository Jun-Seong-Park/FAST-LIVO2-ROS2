/**
 * @file econ_ros2_driver.cpp
 * @brief See3CAM_24CUG ROS2 driver node — implementation + entry point.
 *
 * Owns orchestration + the publish path: param load → HID stream-mode init → backend start →
 * grab → mmap GPS-epoch stamp → publish (Image bgr8 | CompressedImage jpeg) + blackbox.
 * Capture+convert detail lives in the GStreamer backend; this file never touches V4L2/GStreamer.
 *
 * One reusable publish buffer per format: inter-process publish() finishes the synchronous
 * DDS copy before returning, so a single buffer suffices (single grab thread + sync publish).
 */

#include "econ_ros2_driver.hpp"

#include <cstdlib>
#include <cstring>
#include <memory>

#include "blackbox.hpp"
#include "hid_control.hpp"

namespace hid = see3cam::hid;

// ── lifecycle ──────────────────────────────────────────────────────────────

EconRos2Driver::EconRos2Driver()
  : Node("econ_ros2_driver")
{
  // params
  p_ = load_params();
  RCLCPP_INFO(get_logger(),
    "backend=gstreamer resolution=%s capture=%dx%d output=%dx%d flip=%d pub=%dx%d compressed=%s",
    p_.resolution.c_str(),
    p_.res.capture_width, p_.res.capture_height,
    p_.res.output_width, p_.res.output_height,
    p_.flip_method, p_.pub_width, p_.pub_height,
    p_.compressed ? "true" : "false");

  // blackbox path
  blackbox::image::init(blackbox::session_dir() + "/econ_ros2_driver_pub.bin");

  setup_publisher();

  if (!init_hid())     { return; }   // HID stream-mode + exposure
  if (!init_backend()) { return; }   // capture+convert pipeline (gstreamer)

  grab_thread_ = std::thread(&EconRos2Driver::loop, this);
}

EconRos2Driver::~EconRos2Driver()
{
  shutdown();
  blackbox::image::shutdown();
}

// ── init ───────────────────────────────────────────────────────────────────

std::string EconRos2Driver::resolve_shared_path()
{
  const char* home = std::getenv("HOME");
  return std::string(home && *home ? home : "/tmp") + "/timeshare";
}

/// Declares each parameter with the k* defaults (overridden by yaml/launch if present), reads them,
/// and returns Params. res / expected_size are not parameters — derived from the profile, last.
Params EconRos2Driver::load_params()
{
  Params p;
  p.device      = declare_parameter<std::string>("device",      kDevice);
  p.hid_device  = declare_parameter<std::string>("hid_device",  kHidDevice);
  p.resolution  = declare_parameter<std::string>("resolution",  kResolution);
  p.exposure_us = declare_parameter<int>("exposure_us",         kExposureUs);
  p.compressed  = declare_parameter<bool>("compressed",         kCompressed);
  p.flip_method = declare_parameter<int>("flip_method",         kFlipMethod);
  p.topic_name  = declare_parameter<std::string>("topic_name",  kTopicName);
  p.frame_id    = declare_parameter<std::string>("frame_id",    kFrameId);
  // compressed decides the capture format → pulls fps correctly from the per-format profile table.
  p.res = p.compressed ? resolve_profile_mjpg(p.resolution)
                       : resolve_profile_uyvy(p.resolution);
  // On a 90° rotation, swap published width·height (nvvidconv outputs after rotating, so dimensions change).
  const bool quarter = is_quarter_turn(p.flip_method);
  p.pub_width     = quarter ? p.res.output_height : p.res.output_width;
  p.pub_height    = quarter ? p.res.output_width  : p.res.output_height;
  p.expected_size = static_cast<size_t>(p.pub_width) * p.pub_height * 3;
  return p;
}

void EconRos2Driver::setup_publisher()
{
  if (p_.compressed) {
    // CompressedImage: capture-resolution MJPG, variable byte size per frame.
    cpublisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      p_.topic_name + "/compressed", qos_);
    cmsg_.header.frame_id = p_.frame_id;
    cmsg_.format          = "jpeg";
  } else {
    // raw bgr8 Image: fixed output resolution and byte size.
    publisher_ = create_publisher<sensor_msgs::msg::Image>(p_.topic_name, qos_);
    msg_.header.frame_id = p_.frame_id;
    msg_.encoding        = "bgr8";
    msg_.is_bigendian    = 0;
    msg_.width  = p_.pub_width;
    msg_.height = p_.pub_height;
    msg_.step   = p_.pub_width * 3;
    msg_.data.resize(p_.expected_size);
  }
}

bool EconRos2Driver::init_hid()
{
  return hid::open_and_init_trigger(p_.hid_device.c_str(), p_.exposure_us,
                                    kAflMode, hid_fd_, get_logger());
}

bool EconRos2Driver::init_backend()
{
  const ImageFormat format = p_.compressed ? ImageFormat::kJpeg
                                           : ImageFormat::kRawBgr8;
  return backend_.start(p_, format, get_logger());
}

// ── capture loop ───────────────────────────────────────────────────────────

void EconRos2Driver::loop()
{
  // stop_loop_ is read with memory_order_relaxed: it is a plain termination flag with no other
  // data to synchronize, and observing the store one iteration late (the next grab is at
  // most 100 ms away) is harmless, so this hot loop avoids a memory barrier.
  while (!stop_loop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
    Frame frame = backend_.grab(100);   // 100 ms timeout; invalid Frame on timeout
    if (!frame.valid()) { // timeout and error will be logged in gst_backend
      if (blackbox::mono_raw_ns() - backend_.t_capture_ns() > 200000000)   // >200 ms (2 trigger periods) with no frame
      RCLCPP_WARN(get_logger(),
        "\033[33mgrab timeout — no frames received for >100 ms\033[0m");
      continue;
    }
    ++n_pull_;

    // Drop accounting: driver pushes seen since last grab, minus the one just received.
    const uint64_t push_now     = backend_.push_count();
    const uint64_t pushed_since = push_now - last_push_seen_;
    last_push_seen_ = push_now;
    if (pushed_since > 1) {
      const uint64_t drops = pushed_since - 1;
      total_drops_ += drops;
      RCLCPP_WARN_STREAM(get_logger(),
        "\033[33mros2 publish slower than gstreamer, drop +" << drops
        << " total=" << total_drops_
        << " (produced=" << push_now << " published=" << n_pull_ << ")\033[0m");
    }

    publish(frame);
    backend_.release();   // backend reclaims the frame buffer (gst sample unref)
  }
}

void EconRos2Driver::publish(const Frame& frame)
{
  stamper_.try_open();
  const int64_t ns = stamper_.read_low_ns();
  /// If the stamper is not yet ready or there's an issue reading, fallback to ROS time (not GPS-synced).
  const rclcpp::Time stamp = (ns > 0) ? rclcpp::Time(ns, RCL_ROS_TIME) : now();

  if (p_.compressed) {
    // MJPG: variable byte size — copy exactly frame.size into the CompressedImage buffer.
    cmsg_.header.stamp = stamp;
    cmsg_.data.resize(frame.size);
    std::memcpy(cmsg_.data.data(), frame.data, frame.size);
  } else {
    // raw: backend caps fix the frame size, but guard the memcpy source.
    if (frame.size < p_.expected_size) {
      RCLCPP_WARN(get_logger(),
        "\033[33m[econ_ros2_driver] frame smaller than expected (%zu < %zu)\033[0m",
        frame.size, p_.expected_size);
      return;
    }
    msg_.header.stamp = stamp;
    std::memcpy(msg_.data.data(), frame.data, p_.expected_size);
  }

  const uint64_t header_stamp_ns = static_cast<uint64_t>(stamp.nanoseconds());
  const uint64_t t_capture       = backend_.t_capture_ns();   // MONO_RAW at capture
  const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_capture);

  // inter-process publish finishes the synchronous DDS copy before returning,
  // so the same buffer can be reused for the next frame
  if (p_.compressed) { cpublisher_->publish(cmsg_); }
  else               { publisher_->publish(msg_); }
  blackbox::image::log_pub(record_idx);
}

void EconRos2Driver::shutdown()
{
  // grab() has a 100 ms timeout, so the loop exits within ~100 ms of stop_loop_; join BEFORE
  // backend_.stop() frees capture resources the grab thread may still touch.
  stop_loop_.store(true); // signal the grab loop to exit so the join() below can return
  if (grab_thread_.joinable()) { grab_thread_.join(); }
  backend_.stop();   // idempotent — safe even if start() never ran
  stamper_.close();
  hid::close(hid_fd_);
}

// ── entry point ────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EconRos2Driver>());
  rclcpp::shutdown();
  return 0;
}
