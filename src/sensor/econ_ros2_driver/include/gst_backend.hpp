#pragma once

/**
 * @file gst_backend.hpp
 * @brief GstBackend — the GStreamer capture+convert backend (the driver's only backend). Jetson only.
 *
 * Declaration only; implementation in src/gst_backend.cpp.
 *
 * The node never touches V4L2/GStreamer directly; it drives this backend:
 *   start() once → loop { grab() → (use Frame) → release() } → stop().
 * The node decides the output ImageFormat and passes it to start(); Frame bytes follow that format:
 *   kRawBgr8 → BGR, exactly pub_width*pub_height*3 bytes (raw bgr8 Image)
 *   kJpeg    → MJPG bytes, variable size                  (CompressedImage jpeg)
 *
 * kRawBgr8:
 *   v4l2src ─ caps(UYVY WxH @fps) ─ nvvidconv(flip-method) ─ caps(BGRx pub WxH) ─ videoconvert ─ caps(BGR) ─ appsink
 *     - nvvidconv does color + downscale + rotation on the Tegra ISP (HW); sd downscales here, others pass through
 *     - flip_method 90° (1/3) swaps output WxH → caps use pub_width/pub_height (derived in config.hpp)
 * kJpeg:
 *   v4l2src ─ caps(image/jpeg WxH @fps) ─ appsink   (no encoder, no rescale)
 *
 * caps filters pin every negotiation boundary → deterministic caps.
 * Push-side observation (t_capture, push_count) comes from the v4l2src src-pad probe.
 */

#include <rclcpp/logger.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstddef>
#include <cstdint>

// Params/Resolution are defined in econ_ros2_driver.hpp; GstBackend only takes them by const&,
// so a forward declaration is enough here. (gst_backend.cpp includes the full definition.)
struct Params;
struct Resolution;

/// Output format the node wants — chosen by the node, handed to the backend at start().
enum class ImageFormat {
  kRawBgr8,   // raw BGR, fixed pub_width*pub_height*3 bytes
  kJpeg,      // MJPG bytes, variable size
};

/// A borrowed view of one captured frame. data is owned by the backend and stays valid
/// only until the matching release() — the node must copy out before releasing.
struct Frame {
  const uint8_t* data = nullptr;
  size_t         size = 0;
  bool valid() const { return data != nullptr && size > 0; }
};

class GstBackend {
 public:
  GstBackend() = default;
  ~GstBackend();

  GstBackend(const GstBackend&)            = delete;
  GstBackend& operator=(const GstBackend&) = delete;

  /// Open the device (Params: device/resolution) and begin streaming in `format`.
  /// Returns false on any setup failure (logs FATAL).
  bool     start(const Params& p, ImageFormat format, rclcpp::Logger logger);
  /// Block up to timeout_ms for the next frame. Returns an invalid Frame on timeout;
  /// the returned data is valid until release().
  Frame    grab(int timeout_ms);
  /// Release the frame returned by the last grab() (gst sample unref).
  void     release();
  /// Stop streaming and free all capture resources. Call only after the grab thread has stopped.
  void     stop();
  /// Frames v4l2src has pushed so far (node uses the delta for drop accounting).
  uint64_t push_count()   const;
  /// MONOTONIC_RAW ns at the last capture (node logs it as the blackbox t_capture field).
  uint64_t t_capture_ns() const;

 private:
  bool build_raw_chain(GstElement* src, GstElement* sink, const Params& p);
  bool build_jpeg_chain(GstElement* src, GstElement* sink, const Resolution& res);
  void drain_bus();
  void discard_sample();
  static GstCaps* make_uyvy_caps(const Resolution& res);
  static void     set_caps(GstElement* capsfilter, GstCaps* caps);
  static GstPadProbeReturn on_src_push(GstPad*, GstPadProbeInfo*, gpointer data);

  rclcpp::Logger        logger_ = rclcpp::get_logger("econ_gst_backend");
  GstElement*           pipeline_ = nullptr;
  GstAppSink*           sink_     = nullptr;   // owned by pipeline_ (raw ptr for pull)
  GstSample*            sample_   = nullptr;   // current grab() sample (released by release())
  GstMemory*            memory_   = nullptr;   // current grab() memory
  GstMapInfo            info_{};               // current grab() map info
  std::atomic<uint64_t> n_push_{0};            // buffers pushed by v4l2src
  uint64_t              t_push_ns_{0};         // MONO_RAW at last push
};
