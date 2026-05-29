// econ_ros2_driver — See3CAM_24CUG ROS2 driver node (single entry executable).
// Owns orchestration only: param load → HID stream-mode init → backend select →
// grab → mmap GPS-epoch stamp → publish (Image bgr8 | CompressedImage jpeg) + blackbox.
// Capture+convert detail lives in the selected backend (gstreamer | v4l2_opencv);
// this file never touches V4L2/GStreamer/OpenCV directly.
//
// One reusable publish buffer per format: inter-process publish() finishes the synchronous
// DDS copy before returning, so a single buffer suffices (single grab thread + sync publish).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "config.hpp"
#include "camera_backend.hpp"
#include "gst_backend.hpp"
#include "opencv_backend.hpp"
#include "hid_control.hpp"
#include "mmap_stamper.hpp"
#include "blackbox.hpp"

namespace cfg = see3cam;
namespace hid = see3cam::hid;

class EconRos2Driver : public rclcpp::Node
{
 public:
  EconRos2Driver()
    : Node("econ_ros2_driver"),
      stamper_(resolve_shared_path()),
      hid_fd_(-1),
      stop_(false)
  {
    // Parameters (config/config.yaml overrides config.hpp defaults)
    p_ = cfg::load_params(*this);
    RCLCPP_INFO(get_logger(),
      "[econ_ros2_driver] backend=%s profile=%s capture=%dx%d output=%dx%d compressed=%s",
      p_.backend.c_str(), p_.resolution.c_str(),
      p_.res.capture_width, p_.res.capture_height,
      p_.res.output_width, p_.res.output_height,
      p_.compressed ? "true" : "false");

    // ros2-logger style: fresh ~/.blackbox/log/<ts>-<pid>/ each run (no overwrite)
    blackbox::image::init(blackbox::session_dir() + "/econ_ros2_driver_image_pub.bin");

    setup_publisher();

    if (!init_hid())     return;   // HID stream-mode (TRIGGER / MASTER) + exposure
    if (!init_backend()) return;   // capture+convert pipeline (gstreamer / v4l2_opencv)

    grab_thread_ = std::thread(&EconRos2Driver::loop, this);
  }

  ~EconRos2Driver() override
  {
    shutdown();
    blackbox::image::shutdown();
  }

 private:
  // mmap file shared with the LiDAR trigger process (GPS-epoch ns timestamp source)
  static std::string resolve_shared_path()
  {
    const char* home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }

  // Only the active format's publisher/buffer is set up; the other stays unused.
  void setup_publisher()
  {
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    if (p_.compressed) {
      // CompressedImage: capture-resolution MJPG, variable byte size per frame.
      cpublisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        p_.topic_name + "/compressed", qos);
      cmsg_.header.frame_id = p_.frame_id;
      cmsg_.format          = "jpeg";
    } else {
      // raw bgr8 Image: fixed output resolution and byte size.
      publisher_ = create_publisher<sensor_msgs::msg::Image>(p_.topic_name, qos);
      msg_.header.frame_id = p_.frame_id;
      msg_.encoding        = "bgr8";
      msg_.is_bigendian    = 0;
      msg_.width  = p_.res.output_width;
      msg_.height = p_.res.output_height;
      msg_.step   = p_.res.output_width * 3;
      msg_.data.resize(p_.expected_size);
    }
  }

  // HID external-trigger mode + exposure.
  bool init_hid()
  {
    return hid::open_and_init_trigger(p_.hid_device.c_str(), p_.exposure_us,
                                      p_.afl_mode, hid_fd_, get_logger());
  }

  // Factory: one backend per the `backend` param. Unknown name → FATAL (caller stops the node).
  bool init_backend()
  {
    if      (p_.backend == "gstreamer") backend_ = std::make_unique<cfg::GstBackend>();
    else if (p_.backend == "opencv")    backend_ = std::make_unique<cfg::V4l2OpencvBackend>();
    else {
      RCLCPP_FATAL(get_logger(),
        "\033[31m[econ_ros2_driver] unknown backend '%s' (gstreamer | opencv)\033[0m",
        p_.backend.c_str());
      return false;
    }
    // Node owns the format decision; backend captures in whatever it is handed.
    const cfg::ImageFormat format = p_.compressed ? cfg::ImageFormat::kJpeg
                                                  : cfg::ImageFormat::kRawBgr8;
    return backend_->start(p_, format, get_logger());
  }

  void loop()
  {
    while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      cfg::Frame frame = backend_->grab(100);   // 100 ms timeout; invalid Frame on timeout
      if (!frame.valid()) continue;
      ++n_pull_;

      // Drop accounting: driver pushes seen since last grab, minus the one just received.
      const uint64_t push_now     = backend_->push_count();
      const uint64_t pushed_since = push_now - last_push_seen_;
      last_push_seen_ = push_now;
      if (pushed_since > 1) {
        const uint64_t drops = pushed_since - 1;
        total_drops_ += drops;
        RCLCPP_WARN_STREAM(get_logger(),
          "\033[33m[econ_ros2_driver] drop +" << drops
          << " total=" << total_drops_
          << " (push=" << push_now << " pull=" << n_pull_ << ")\033[0m");
      }

      publish(frame);
      backend_->release();   // backend reclaims the frame buffer (gst unref / v4l2 requeue)
    }
  }

  // Stamp from the shared mmap (same time domain as LiDAR), copy into the reusable buffer, publish.
  void publish(const cfg::Frame& frame)
  {
    stamper_.try_open();
    const int64_t ns = stamper_.read_low_ns();
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
    const uint64_t t_capture       = backend_->t_capture_ns();   // MONO_RAW at capture
    const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_capture);

    // inter-process publish finishes the synchronous DDS copy before returning,
    // so the same buffer can be reused for the next frame
    if (p_.compressed) cpublisher_->publish(cmsg_);
    else               publisher_->publish(msg_);
    blackbox::image::log_pub(record_idx);
  }

  void shutdown()
  {
    // grab() has a 100 ms timeout, so the loop exits within ~100 ms of stop_; join BEFORE
    // backend_->stop() frees capture resources the grab thread may still touch.
    stop_.store(true);
    if (grab_thread_.joinable()) grab_thread_.join();
    if (backend_) backend_->stop();
    stamper_.close();
    hid::close_and_restore(hid_fd_, p_.restore_master, get_logger());
  }

  cfg::Params                         p_;          // runtime config
  cfg::MmapStamper                    stamper_;
  int                                 hid_fd_;
  std::unique_ptr<cfg::CameraBackend> backend_;    // gstreamer | v4l2_opencv
  std::thread                         grab_thread_;
  std::atomic<bool>                   stop_;
  uint64_t                            n_pull_{0};         // successful grabs (single thread)
  uint64_t                            last_push_seen_{0};
  uint64_t                            total_drops_{0};
  sensor_msgs::msg::Image             msg_;        // reusable raw buffer (compressed=false)
  sensor_msgs::msg::CompressedImage   cmsg_;       // reusable MJPG buffer (compressed=true)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_;   // raw
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cpublisher_;  // compressed
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EconRos2Driver>());
  rclcpp::shutdown();
  return 0;
}
