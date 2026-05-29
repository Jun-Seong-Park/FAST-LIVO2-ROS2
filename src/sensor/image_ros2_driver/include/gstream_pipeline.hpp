#pragma once
// See3CAM_24CUG GStreamer pipeline — element-based construction + caps negotiation.
// All GStreamer concerns (build, link, caps negotiation, src-pad probe, bus, pull, teardown)
// live here so the ROS node only orchestrates: pull → stamp → publish.
//
// Raw path (compressed=false):
//   v4l2src ─ capsfilter(UYVY WxH @fps) ─ nvvidconv ─ capsfilter(BGRx WxH)
//           ─ videoconvert ─ capsfilter(BGR) ─ appsink
//   - capsfilters pin every negotiation boundary → deterministic caps (no fuzzy auto-negotiation)
//   - sd downscales at nvvidconv (capture != output); hd/fhd/wuxga pass through full-frame
// Compressed path (compressed=true):
//   v4l2src ─ capsfilter(image/jpeg, capture WxH @fps) ─ appsink
//   - camera emits MJPG over V4L2 → v4l2src delivers image/jpeg directly (no encoder)
//   - JPEG is not rescalable in-stream → capture resolution as-is (output/downscale ignored)
//   - fps in capture caps is nominal — trigger-mode effective rate is the external trigger (~10 Hz)
//
// Push-side observation (t_push, n_push) is a GStreamer-side fact, so it is owned here and exposed
// via getters; the node reads them for drop accounting and the blackbox t_push field.

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdint>
#include <string>

#include "config.hpp"
#include "blackbox/blackbox.hpp"

namespace see3cam {

class GstCameraPipeline {
 public:
  GstCameraPipeline() = default;
  ~GstCameraPipeline() { stop(); }

  GstCameraPipeline(const GstCameraPipeline&)            = delete;
  GstCameraPipeline& operator=(const GstCameraPipeline&) = delete;

  // Build element pipeline for device + resolution, negotiate caps, set PLAYING.
  // compressed selects the chain (raw BGR vs MJPG image/jpeg) between v4l2src and appsink.
  // Returns false on any factory/link/state failure (logs FATAL).
  bool start(const std::string& device, const Resolution& res, bool compressed,
             rclcpp::Logger logger) {
    gst_init(nullptr, nullptr);  // idempotent — safe to call repeatedly

    pipeline_ = gst_pipeline_new("see3cam_pipeline");
    GstElement* src  = gst_element_factory_make("v4l2src", "src");
    GstElement* sink = gst_element_factory_make("appsink", "sink");

    if (!pipeline_ || !src || !sink) {
      RCLCPP_FATAL(logger, "\033[31m[gst] element factory failed (missing plugin?)\033[0m");
      return false;
    }

    // v4l2 source device
    g_object_set(src, "device", device.c_str(), nullptr);

    // appsink: no signals (we pull), no clock sync, keep newest buffer only
    g_object_set(sink, "emit-signals", FALSE, "sync", FALSE,
                 "max-buffers", 1, "drop", TRUE, nullptr);
    sink_ = GST_APP_SINK(sink);

    const bool linked = compressed ? build_jpeg_chain(src, sink, res, logger)
                                   : build_raw_chain(src, sink, res, logger);
    if (!linked) return false;

    // src-pad probe: stamp MONO_RAW at each v4l2src push (push-side timing)
    GstPad* src_pad = gst_element_get_static_pad(src, "src");
    gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, on_src_push, this, nullptr);
    gst_object_unref(src_pad);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(logger, "\033[31m[gst] set PLAYING failed\033[0m");
      return false;
    }
    return true;
  }

  // Pull one BGR sample within timeout_ms; nullptr on timeout. Caller unrefs.
  GstSample* try_pull(int timeout_ms) {
    return gst_app_sink_try_pull_sample(sink_, timeout_ms * GST_MSECOND);
  }

  // Non-blocking: pop + log one ERROR/EOS bus message if present.
  void drain_bus(rclcpp::Logger logger) {
    GstBus*     bus = gst_element_get_bus(pipeline_);
    GstMessage* msg = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (msg) {
      if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        GError* gerr = nullptr;
        gchar*  dbg  = nullptr;
        gst_message_parse_error(msg, &gerr, &dbg);
        RCLCPP_ERROR(logger, "\033[31m[gst] BUS ERROR — %s | %s\033[0m",
                     gerr ? gerr->message : "unknown", dbg ? dbg : "");
        if (gerr) g_error_free(gerr);
        if (dbg)  g_free(dbg);
      } else {
        RCLCPP_WARN(logger, "\033[33m[gst] BUS EOS\033[0m");
      }
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
  }

  // Push-side observation (written by on_src_push on the GStreamer streaming thread).
  uint64_t push_count() const { return n_push_.load(std::memory_order_relaxed); }
  uint64_t t_push_ns()  const { return t_push_ns_; }  // not atomic — 64-bit aligned, torn read unlikely

  // GST_STATE_NULL + unref. Call only after the pulling thread has stopped (frees the appsink).
  void stop() {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);   // frees all child elements (incl. appsink)
      pipeline_ = nullptr;
    }
    sink_ = nullptr;
  }

 private:
  // Raw chain: UYVY capture → nvvidconv (downscale when output != capture) → BGR → appsink.
  bool build_raw_chain(GstElement* src, GstElement* sink, const Resolution& res,
                       rclcpp::Logger logger) {
    GstElement* caps_cap = gst_element_factory_make("capsfilter",   "caps_capture");
    GstElement* nvconv   = gst_element_factory_make("nvvidconv",    "nvconv");
    GstElement* caps_out = gst_element_factory_make("capsfilter",   "caps_output");
    GstElement* vidconv  = gst_element_factory_make("videoconvert", "vidconv");
    GstElement* caps_bgr = gst_element_factory_make("capsfilter",   "caps_bgr");

    if (!caps_cap || !nvconv || !caps_out || !vidconv || !caps_bgr) {
      RCLCPP_FATAL(logger, "\033[31m[gst] raw chain factory failed (missing plugin?)\033[0m");
      return false;
    }

    // Capture caps: UYVY capture WxH @ fps (fps nominal — trigger effective ~10 Hz)
    GstCaps* c_capture = gst_caps_new_simple(
      "video/x-raw",
      "format",    G_TYPE_STRING,     "UYVY",
      "width",     G_TYPE_INT,        res.capture_width,
      "height",    G_TYPE_INT,        res.capture_height,
      "framerate", GST_TYPE_FRACTION, res.fps, 1,
      nullptr);
    g_object_set(caps_cap, "caps", c_capture, nullptr);
    gst_caps_unref(c_capture);

    // nvvidconv output: BGRx output WxH (downscale happens here when output != capture)
    GstCaps* c_output = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "BGRx",
      "width",  G_TYPE_INT,    res.output_width,
      "height", G_TYPE_INT,    res.output_height,
      nullptr);
    g_object_set(caps_out, "caps", c_output, nullptr);
    gst_caps_unref(c_output);

    // Final: BGR (node memcpy's this into sensor_msgs::Image bgr8)
    GstCaps* c_bgr = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", nullptr);
    g_object_set(caps_bgr, "caps", c_bgr, nullptr);
    gst_caps_unref(c_bgr);

    gst_bin_add_many(GST_BIN(pipeline_),
                     src, caps_cap, nvconv, caps_out, vidconv, caps_bgr, sink, nullptr);
    if (!gst_element_link_many(src, caps_cap, nvconv, caps_out, vidconv, caps_bgr, sink, nullptr)) {
      RCLCPP_FATAL(logger, "\033[31m[gst] raw chain link failed (caps negotiation)\033[0m");
      return false;
    }
    return true;
  }

  // Compressed chain: camera MJPG → v4l2src image/jpeg → appsink (no encoder, no rescale).
  bool build_jpeg_chain(GstElement* src, GstElement* sink, const Resolution& res,
                        rclcpp::Logger logger) {
    GstElement* caps_jpeg = gst_element_factory_make("capsfilter", "caps_jpeg");

    if (!caps_jpeg) {
      RCLCPP_FATAL(logger, "\033[31m[gst] jpeg chain factory failed (missing plugin?)\033[0m");
      return false;
    }

    // image/jpeg at capture WxH @ fps — fps must be an MJPG-supported rate for this resolution
    // (See3CAM_24CUG: 1280x720 / 1920x1080 @ 60, 1920x1200 @ 60). fps nominal under trigger mode.
    GstCaps* c_jpeg = gst_caps_new_simple(
      "image/jpeg",
      "width",     G_TYPE_INT,        res.capture_width,
      "height",    G_TYPE_INT,        res.capture_height,
      "framerate", GST_TYPE_FRACTION, res.fps, 1,
      nullptr);
    g_object_set(caps_jpeg, "caps", c_jpeg, nullptr);
    gst_caps_unref(c_jpeg);

    gst_bin_add_many(GST_BIN(pipeline_), src, caps_jpeg, sink, nullptr);
    if (!gst_element_link_many(src, caps_jpeg, sink, nullptr)) {
      RCLCPP_FATAL(logger, "\033[31m[gst] jpeg chain link failed (caps negotiation)\033[0m");
      return false;
    }
    return true;
  }

  // Buffer probe on v4l2src src pad: one clock read per push, buffer passes through unmodified.
  static GstPadProbeReturn on_src_push(GstPad*, GstPadProbeInfo*, gpointer data) {
    auto* self = static_cast<GstCameraPipeline*>(data);
    self->t_push_ns_ = blackbox::mono_raw_ns();
    self->n_push_.fetch_add(1, std::memory_order_relaxed);
    return GST_PAD_PROBE_OK;
  }

  GstElement*           pipeline_  = nullptr;
  GstAppSink*           sink_      = nullptr;  // owned by pipeline_ (raw ptr for pull)
  std::atomic<uint64_t> n_push_{0};            // buffers pushed by v4l2src
  uint64_t              t_push_ns_{0};         // MONO_RAW at last push
};

}  // namespace see3cam
