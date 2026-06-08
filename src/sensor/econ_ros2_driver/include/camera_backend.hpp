#pragma once
// CameraBackend — capture+convert abstraction behind the econ_ros2_driver node.
// The node never touches V4L2/GStreamer/OpenCV; it only drives this interface:
//   start() once → loop { grab() → (use Frame) → release() } → stop().
//
// Concrete backends:
//   GstBackend         (gst_backend.hpp)         — v4l2src + nvvidconv/videoconvert via GStreamer
//   V4l2OpencvBackend  (opencv_backend.hpp) — direct V4L2 ioctl/mmap + OpenCV convert
//
// The node decides the output ImageFormat and passes it to start(); the backend never reads
// Params::compressed itself. Frame bytes then follow that format:
//   kRawBgr8 → BGR, exactly output_width*output_height*3 bytes (raw bgr8 Image)
//   kJpeg    → MJPG bytes, variable size                      (CompressedImage jpeg)

#include <cstddef>
#include <cstdint>

#include <rclcpp/logger.hpp>

#include "config.hpp"

namespace see3cam {

// Output format the node wants — chosen by the node, handed to the backend at start().
enum class ImageFormat {
  kRawBgr8,   // raw BGR, fixed output_width*output_height*3 bytes
  kJpeg,      // MJPG bytes, variable size
};

// A borrowed view of one captured frame. data is owned by the backend and stays valid
// only until the matching release() — the node must copy out before releasing.
struct Frame {
  const uint8_t* data = nullptr;
  size_t         size = 0;
  bool valid() const { return data != nullptr && size > 0; }
};

class CameraBackend {
 public:
  virtual ~CameraBackend() = default;

  // Open the device (Params: device/resolution) and begin streaming in `format`.
  // Returns false on any setup failure (logs FATAL).
  virtual bool start(const Params& p, ImageFormat format, rclcpp::Logger logger) = 0;

  // Block up to timeout_ms for the next frame. Returns an invalid Frame on timeout.
  // The returned data is valid until release() is called.
  virtual Frame grab(int timeout_ms) = 0;

  // Release the frame returned by the last grab() (gst sample unref / V4L2 buffer requeue).
  virtual void release() = 0;

  // Stop streaming and free all capture resources. Call only after the grab thread has stopped.
  virtual void stop() = 0;

  // Frames the driver has pushed so far (node uses the delta for drop accounting).
  virtual uint64_t push_count() const = 0;

  // MONOTONIC_RAW ns at the last capture (node logs it as the blackbox t_capture field).
  virtual uint64_t t_capture_ns() const = 0;
};

}  // namespace see3cam
