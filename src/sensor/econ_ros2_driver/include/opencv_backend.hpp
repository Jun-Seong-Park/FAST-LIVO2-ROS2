#pragma once
// V4l2OpencvBackend — direct V4L2 ioctl/mmap capture + OpenCV convert (CameraBackend impl).
// Declaration only; implementation in src/opencv_backend.cpp.
//
// kRawBgr8: V4L2 UYVY → cv::cvtColor(UYVY→BGR) → cv::resize (downscale when output != capture)
// kJpeg:    V4L2 MJPEG → buffer bytes as-is (no decode, no rescale)
//
// grab() DQBUFs one buffer; raw conversion copies into bgr_, jpeg borrows the mmap buffer.
// Either way the V4L2 buffer is requeued in release(). push_count derives from
// v4l2_buffer.sequence so the node's drop accounting matches the GStreamer backend.

#include "camera_backend.hpp"
#include "config.hpp"

#include <rclcpp/logger.hpp>

#include <opencv2/core.hpp>   // cv::Mat members

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace see3cam {

class V4l2OpencvBackend : public CameraBackend {
 public:
  V4l2OpencvBackend() = default;
  ~V4l2OpencvBackend() override;

  V4l2OpencvBackend(const V4l2OpencvBackend&)            = delete;
  V4l2OpencvBackend& operator=(const V4l2OpencvBackend&) = delete;

  bool     start(const Params& p, ImageFormat format, rclcpp::Logger logger) override;
  Frame    grab(int timeout_ms) override;
  void     release() override;
  void     stop() override;
  uint64_t push_count()   const override;
  uint64_t t_capture_ns() const override;

 private:
  static constexpr int kBufCount = 4;
  struct MmapBuf { void* start = nullptr; size_t length = 0; };

  bool set_format();
  bool request_buffers();

  rclcpp::Logger        logger_ = rclcpp::get_logger("econ_v4l2_backend");
  int                   fd_ = -1;
  ImageFormat           format_ = ImageFormat::kRawBgr8;
  Resolution            res_{};
  bool                  need_resize_ = false;
  MmapBuf               bufs_[kBufCount];
  int                   n_bufs_ = 0;
  int                   current_index_ = -1;     // DQBUF'd buffer awaiting release()/QBUF
  cv::Mat               conv_;                    // capture-size BGR (only when downscaling)
  cv::Mat               bgr_;                     // output-size BGR returned to the node
  std::atomic<uint64_t> n_push_{0};              // v4l2_buffer.sequence + 1 (driver frame count)
  uint64_t              t_capture_ns_{0};         // MONO_RAW at last DQBUF
};

}  // namespace see3cam
