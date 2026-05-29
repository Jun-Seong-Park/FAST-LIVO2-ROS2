// See3CAM_24CUG trigger node — single-thread pull → mmap-stamp → move-publish.
// GStreamer pipeline/negotiation lives in gstream_pipeline.hpp; this file only orchestrates:
//   pull (BGR) → mmap GPS-epoch stamp → memcpy into reusable Image → publish + blackbox.
//
// One reusable publish buffer: inter-process publish() finishes the synchronous DDS copy before
// returning, so a single buffer suffices (single thread + synchronous publish).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <gst/gst.h>

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "config.hpp"
#include "gstream_pipeline.hpp"
#include "hid_control.hpp"
#include "mmap_stamper.hpp"
#include "blackbox/blackbox.hpp"

namespace cfg = see3cam;
namespace hid = see3cam::hid;

class See3cam24cugTrig : public rclcpp::Node
{
 public:
  See3cam24cugTrig()
    : Node("see3cam24cug_trig"),
      stamper_(resolve_shared_path()),
      hid_fd_(-1),
      stop_(false)
  {
    // Parameters (config/config.yaml or `resolution:=` launch arg override config.hpp defaults)
    p_ = cfg::load_params(*this);
    RCLCPP_INFO(get_logger(),
      "[see3cam24cug_trig] profile=%s capture=%dx%d output=%dx%d fps=%d compressed=%s "
      "(nominal; trigger ~10Hz)",
      p_.resolution.c_str(), p_.res.capture_width, p_.res.capture_height,
      p_.res.output_width, p_.res.output_height, p_.res.fps, p_.compressed ? "true" : "false");

    // Blackbox
    blackbox::image::init(blackbox::log_dir() + "/see3cam24cug_trig_image_pub.bin");

    // Publisher + reusable buffer fixed fields (stamp refreshed per frame). Only the active
    // format's publisher/buffer is set up; the other stays unused.
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

    // HID open + external-trigger mode
    if (!hid::open_and_init_trigger(p_.hid_device.c_str(), p_.exposure_us,
                                    p_.afl_mode, hid_fd_, get_logger())) return;

    // GStreamer pipeline (element-based build + negotiation in gstream_pipeline.hpp)
    if (!gst_.start(p_.device, p_.res, p_.compressed, get_logger())) return;

    // Pipeline worker thread
    pipeline_thread_ = std::thread(&See3cam24cugTrig::pipeline_loop, this);
  }

  ~See3cam24cugTrig() override {
    shutdown();
    blackbox::image::shutdown();
  }

 private:

  // mmap file path for the timestamp shared with the trigger process
  static std::string resolve_shared_path() {
    const char* home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }

  void pipeline_loop() {
    // Pull until stop_ is set or rclcpp shuts down; 100 ms pull timeout
    while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      GstSample* sample = gst_.try_pull(100);
      gst_.drain_bus(get_logger());

      if (!sample) continue;
      ++n_pull_;

      // Drop accounting: pushes seen since last pull, minus the one just received
      const uint64_t push_now     = gst_.push_count();
      const uint64_t pushed_since = push_now - last_push_seen_;
      last_push_seen_ = push_now;
      if (pushed_since > 1) {
        const uint64_t drops = pushed_since - 1;
        total_drops_ += drops;
        RCLCPP_WARN_STREAM(get_logger(),
          "\033[33m[see3cam24cug_trig] drop +" << drops
          << " total=" << total_drops_
          << " (push=" << push_now << " pull=" << n_pull_ << ")\033[0m");
      }

      process_sample(sample);
      gst_sample_unref(sample);  // also releases the buffer/caps (owned by appsink)
    }
  }

  // Fill the reusable buffer from `sample`, then publish it.
  void process_sample(GstSample* sample) {
    GstBuffer* buf = gst_sample_get_buffer(sample);
    if (!buf) {
      RCLCPP_WARN(get_logger(), "\033[33m[see3cam24cug_trig] WARN — buf nullptr\033[0m");
      return;
    }

    GstMemory* memory = gst_buffer_get_memory(buf, 0);
    if (!memory) {
      RCLCPP_WARN(get_logger(), "\033[33m[see3cam24cug_trig] WARN — memory nullptr\033[0m");
      return;
    }
    GstMapInfo info;
    if (!gst_memory_map(memory, &info, GST_MAP_READ)) {
      gst_memory_unref(memory);
      RCLCPP_WARN(get_logger(), "\033[33m[see3cam24cug_trig] WARN — memory map fail\033[0m");
      return;
    }

    stamper_.try_open();
    const int64_t ns = stamper_.read_low_ns();
    const rclcpp::Time stamp = (ns > 0) ? rclcpp::Time(ns, RCL_ROS_TIME) : now();

    if (p_.compressed) {
      // MJPG: variable byte size — copy exactly info.size into the CompressedImage buffer.
      cmsg_.header.stamp = stamp;
      cmsg_.data.resize(info.size);
      std::memcpy(cmsg_.data.data(), info.data, info.size);
    } else {
      // raw: negotiated caps fix the frame size, but guard the memcpy source
      if (info.size < p_.expected_size) {
        gst_memory_unmap(memory, &info);
        gst_memory_unref(memory);
        RCLCPP_WARN(get_logger(), "\033[33m[see3cam24cug_trig] WARN — frame smaller than expected\033[0m");
        return;
      }
      msg_.header.stamp = stamp;
      std::memcpy(msg_.data.data(), info.data, p_.expected_size);
    }

    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);

    const uint64_t header_stamp_ns = static_cast<uint64_t>(stamp.nanoseconds());
    const uint64_t t_push = gst_.t_push_ns();  // src-pad probe value (MONOTONIC_RAW)
    const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_push);

    // inter-process publish finishes the synchronous DDS copy before returning,
    // so the same buffer can be reused for the next frame
    if (p_.compressed) cpublisher_->publish(cmsg_);
    else               publisher_->publish(msg_);
    blackbox::image::log_pub(record_idx);
  }

  void shutdown() {
    // try_pull has a 100 ms timeout, so the loop exits within ~100 ms of stop_; join BEFORE
    // gst_.stop() frees the appsink — otherwise the pulling thread would touch freed memory.
    stop_.store(true);
    if (pipeline_thread_.joinable()) pipeline_thread_.join();
    gst_.stop();
    stamper_.close();
    hid::close_and_restore(hid_fd_, p_.restore_master, get_logger());
  }

  cfg::Params            p_;        // runtime config (config.hpp defaults overridden by yaml / launch arg)
  cfg::MmapStamper       stamper_;
  int                    hid_fd_;
  cfg::GstCameraPipeline gst_;      // all GStreamer build / negotiation / probe / bus / pull
  std::thread            pipeline_thread_;
  std::atomic<bool>      stop_;
  uint64_t               n_pull_{0};         // successful pulls (single thread: pipeline_loop)
  uint64_t               last_push_seen_{0};
  uint64_t               total_drops_{0};
  sensor_msgs::msg::Image           msg_;    // reusable raw buffer (compressed=false)
  sensor_msgs::msg::CompressedImage cmsg_;   // reusable MJPG buffer (compressed=true)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           publisher_;   // raw
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cpublisher_;  // compressed
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<See3cam24cugTrig>());
  rclcpp::shutdown();
  return 0;
}
