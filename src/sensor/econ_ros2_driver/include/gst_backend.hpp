#pragma once
// GstBackend — GStreamer capture+convert backend (CameraBackend impl). Jetson only.
// Declaration only; implementation in src/gst_backend.cpp.
//
// kRawBgr8:
//   v4l2src ─ caps(UYVY WxH @fps) ─ nvvidconv(flip-method) ─ caps(BGRx pub WxH) ─ videoconvert ─ caps(BGR) ─ appsink
//     - nvvidconv does color + downscale + rotation on the Tegra ISP (HW); sd downscales here, others pass through
//     - flip_method 90° (1/3) swaps output WxH → caps use pub_width/pub_height (config.hpp 파생)
// kJpeg:
//   v4l2src ─ caps(image/jpeg WxH @fps) ─ appsink   (no encoder, no rescale)
//
// caps filters pin every negotiation boundary → deterministic caps.
// Push-side observation (t_capture, push_count) comes from the v4l2src src-pad probe.

#include "camera_backend.hpp"
#include "config.hpp"

#include <rclcpp/logger.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdint>

namespace see3cam {

class GstBackend : public CameraBackend {
 public:
  GstBackend() = default;
  ~GstBackend() override;

  GstBackend(const GstBackend&)            = delete;
  GstBackend& operator=(const GstBackend&) = delete;

  bool     start(const Params& p, ImageFormat format, rclcpp::Logger logger) override;
  Frame    grab(int timeout_ms) override;
  void     release() override;
  void     stop() override;
  uint64_t push_count()   const override;
  uint64_t t_capture_ns() const override;

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

}  // namespace see3cam
