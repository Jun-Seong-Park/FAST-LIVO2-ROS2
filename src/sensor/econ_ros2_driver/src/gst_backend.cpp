// GstBackend implementation — see gst_backend.hpp for the pipeline overview.

#include "gst_backend.hpp"

#include <rclcpp/logging.hpp>

#include "blackbox.hpp"

namespace see3cam {

GstBackend::~GstBackend() { stop(); }

bool GstBackend::start(const Params& p, ImageFormat format, rclcpp::Logger logger) {
  logger_ = logger;
  gst_init(nullptr, nullptr);  // idempotent — safe to call repeatedly

  pipeline_ = gst_pipeline_new("econ_gst_pipeline");
  GstElement* src  = gst_element_factory_make("v4l2src", "src");
  GstElement* sink = gst_element_factory_make("appsink", "sink");

  if (!pipeline_ || !src || !sink) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] element factory failed (missing plugin?)\033[0m");
    return false;
  }

  g_object_set(src, "device", p.device.c_str(), nullptr);
  // appsink: no signals (we pull), no clock sync, keep newest buffer only
  g_object_set(sink, "emit-signals", FALSE, "sync", FALSE,
               "max-buffers", 1, "drop", TRUE, nullptr);
  sink_ = GST_APP_SINK(sink);

  if (format == ImageFormat::kJpeg && p.flip_method != 0) {
    // jpeg 체인은 passthrough(디코더·nvvidconv 없음) → 회전 불가. 무시하고 경고.
    RCLCPP_WARN(logger_,
      "\033[33m[gst] flip_method=%d ignored — jpeg passthrough chain cannot rotate\033[0m",
      p.flip_method);
  }

  const bool linked = (format == ImageFormat::kJpeg)
    ? build_jpeg_chain(src, sink, p.res)
    : build_raw_chain(src, sink, p);
  if (!linked) return false;

  // src-pad probe: stamp MONO_RAW at each v4l2src push (push-side timing)
  GstPad* src_pad = gst_element_get_static_pad(src, "src");
  gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, on_src_push, this, nullptr);
  gst_object_unref(src_pad);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] set PLAYING failed\033[0m");
    return false;
  }
  return true;
}

// Pull one sample, map its memory, hand back a borrowed view. release() unmaps/unrefs.
Frame GstBackend::grab(int timeout_ms) {
  sample_ = gst_app_sink_try_pull_sample(sink_, timeout_ms * GST_MSECOND);
  drain_bus();
  if (!sample_) return {};

  GstBuffer* buf = gst_sample_get_buffer(sample_);
  if (!buf) { discard_sample(); return {}; }

  memory_ = gst_buffer_get_memory(buf, 0);
  if (!memory_) { discard_sample(); return {}; }

  if (!gst_memory_map(memory_, &info_, GST_MAP_READ)) {
    gst_memory_unref(memory_); memory_ = nullptr;
    discard_sample();
    RCLCPP_WARN(logger_, "\033[33m[gst] memory map fail\033[0m");
    return {};
  }
  return Frame{info_.data, info_.size};
}

void GstBackend::release() {
  if (memory_) { gst_memory_unmap(memory_, &info_); gst_memory_unref(memory_); memory_ = nullptr; }
  if (sample_) { gst_sample_unref(sample_); sample_ = nullptr; }
}

void GstBackend::stop() {
  release();
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);   // frees all child elements (incl. appsink)
    pipeline_ = nullptr;
  }
  sink_ = nullptr;
}

uint64_t GstBackend::push_count()   const { return n_push_.load(std::memory_order_relaxed); }
uint64_t GstBackend::t_capture_ns() const { return t_push_ns_; }  // 64-bit aligned, torn read unlikely

// Raw chain: UYVY capture → nvvidconv (color + downscale + rotation on ISP) → BGRx → videoconvert → BGR.
bool GstBackend::build_raw_chain(GstElement* src, GstElement* sink, const Params& p) {
  const Resolution& res = p.res;
  GstElement* caps_cap = gst_element_factory_make("capsfilter",   "caps_capture");
  GstElement* nvconv   = gst_element_factory_make("nvvidconv",    "nvconv");
  GstElement* caps_out = gst_element_factory_make("capsfilter",   "caps_output");
  GstElement* vidconv  = gst_element_factory_make("videoconvert", "vidconv");
  GstElement* caps_bgr = gst_element_factory_make("capsfilter",   "caps_bgr");

  if (!caps_cap || !nvconv || !caps_out || !vidconv || !caps_bgr) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] raw chain factory failed (missing plugin?)\033[0m");
    return false;
  }

  // 회전 (HW). 0 none / 1 CCW90 / 2 180 / 3 CW90 — config.hpp flip_method.
  g_object_set(nvconv, "flip-method", p.flip_method, nullptr);

  set_caps(caps_cap, make_uyvy_caps(res));
  // nvvidconv output: BGRx pub WxH. 90° 회전 시 pub 가로·세로가 swap 되어 있으므로
  // (config.hpp 파생) rotate 후 출력 치수와 정확히 일치한다 (downscale 도 여기서).
  GstCaps* c_output = gst_caps_new_simple(
    "video/x-raw",
    "format", G_TYPE_STRING, "BGRx",
    "width",  G_TYPE_INT,    p.pub_width,
    "height", G_TYPE_INT,    p.pub_height,
    nullptr);
  set_caps(caps_out, c_output);
  // final BGR (node memcpy's this into sensor_msgs::Image bgr8)
  set_caps(caps_bgr, gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", nullptr));

  gst_bin_add_many(GST_BIN(pipeline_),
                   src, caps_cap, nvconv, caps_out, vidconv, caps_bgr, sink, nullptr);
  if (!gst_element_link_many(src, caps_cap, nvconv, caps_out, vidconv, caps_bgr, sink, nullptr)) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] raw chain link failed (caps negotiation)\033[0m");
    return false;
  }
  return true;
}

// Compressed chain: camera MJPG → v4l2src image/jpeg → appsink (no encoder, no rescale).
bool GstBackend::build_jpeg_chain(GstElement* src, GstElement* sink, const Resolution& res) {
  GstElement* caps_jpeg = gst_element_factory_make("capsfilter", "caps_jpeg");
  if (!caps_jpeg) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] jpeg factory failed (missing plugin?)\033[0m");
    return false;
  }

  GstCaps* c_jpeg = gst_caps_new_simple(
    "image/jpeg",
    "width",     G_TYPE_INT,        res.capture_width,
    "height",    G_TYPE_INT,        res.capture_height,
    "framerate", GST_TYPE_FRACTION, res.fps, 1,
    nullptr);
  set_caps(caps_jpeg, c_jpeg);

  gst_bin_add_many(GST_BIN(pipeline_), src, caps_jpeg, sink, nullptr);
  if (!gst_element_link_many(src, caps_jpeg, sink, nullptr)) {
    RCLCPP_FATAL(logger_, "\033[31m[gst] jpeg link failed (caps negotiation)\033[0m");
    return false;
  }
  return true;
}

// Capture caps: UYVY at capture WxH @ fps (nominal under trigger).
GstCaps* GstBackend::make_uyvy_caps(const Resolution& res) {
  return gst_caps_new_simple(
    "video/x-raw",
    "format",    G_TYPE_STRING,     "UYVY",
    "width",     G_TYPE_INT,        res.capture_width,
    "height",    G_TYPE_INT,        res.capture_height,
    "framerate", GST_TYPE_FRACTION, res.fps, 1,
    nullptr);
}

// Set caps on a capsfilter and drop our ref (g_object_set takes its own).
void GstBackend::set_caps(GstElement* capsfilter, GstCaps* caps) {
  g_object_set(capsfilter, "caps", caps, nullptr);
  gst_caps_unref(caps);
}

// Non-blocking: pop + log one ERROR/EOS bus message if present.
void GstBackend::drain_bus() {
  GstBus*     bus = gst_element_get_bus(pipeline_);
  GstMessage* msg = gst_bus_pop_filtered(
    bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  if (msg) {
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      GError* gerr = nullptr;
      gchar*  dbg  = nullptr;
      gst_message_parse_error(msg, &gerr, &dbg);
      RCLCPP_ERROR(logger_, "\033[31m[gst] BUS ERROR — %s | %s\033[0m",
                   gerr ? gerr->message : "unknown", dbg ? dbg : "");
      if (gerr) g_error_free(gerr);
      if (dbg)  g_free(dbg);
    } else {
      RCLCPP_WARN(logger_, "\033[33m[gst] BUS EOS\033[0m");
    }
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
}

void GstBackend::discard_sample() { gst_sample_unref(sample_); sample_ = nullptr; }

// Buffer probe on v4l2src src pad: one clock read per push, buffer passes through unmodified.
GstPadProbeReturn GstBackend::on_src_push(GstPad*, GstPadProbeInfo*, gpointer data) {
  auto* self = static_cast<GstBackend*>(data);
  self->t_push_ns_ = blackbox::mono_raw_ns();
  self->n_push_.fetch_add(1, std::memory_order_relaxed);
  return GST_PAD_PROBE_OK;
}

}  // namespace see3cam
