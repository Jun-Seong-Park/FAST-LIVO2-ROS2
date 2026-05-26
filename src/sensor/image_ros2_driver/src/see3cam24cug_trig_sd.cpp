// See3CAM_24CUG — SD (640x360) variant. GStreamer NVMM → BGRx → bgr8.
// blackbox publish trace: blackbox::image (mmap-fixed-array + 1s writer thread).
// Pipeline:
//   v4l2src /dev/24cug ! UYVY 1280x720@60 ! nvvidconv ! BGRx 640x360 ! videoconvert ! BGR ! appsink

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

#include "image_ros_driver/config.hpp"
#include "image_ros_driver/hid_control.hpp"
#include "image_ros_driver/mmap_stamper.hpp"
#include "blackbox/blackbox.hpp"

namespace cfg = see3cam;
namespace hid = see3cam::hid;

namespace {
// SD 전용 ────────────────────────────────────────────
constexpr int         kCaptureWidth   = 1280;
constexpr int         kCaptureHeight  = 720;
constexpr int         kOutputWidth    = 640;
constexpr int         kOutputHeight   = 360;
constexpr int         kCaptureFps     = 60;
constexpr const char* kTopicName      = "/camera/image";
constexpr const char* kNodeName       = "see3cam24cug_trig_sd";
constexpr const char* kLogLabel       = "[see3cam24cug_sd]";
const std::string kBlackboxPath = blackbox::log_dir() + "/see3cam24cug_sd_image_pub.bin";
}  // namespace

class See3cam24cugTrigSd : public rclcpp::Node
{
 public:
  See3cam24cugTrigSd()
    : Node(kNodeName),
      stamper_(resolve_shared_path()),
      running_(true),
      hid_fd_(-1),
      pipeline_(nullptr),
      gst_loop_(nullptr),
      appsink_max_buffers_(read_env_int("SEE3CAM_APPSINK_BUFFERS", cfg::kAppsinkMaxBuffers)),
      appsink_drop_(read_env_bool("SEE3CAM_APPSINK_DROP", cfg::kAppsinkDrop)),
      diag_sleep_ms_(read_env_int("SEE3CAM_DIAG_SLEEP_MS", cfg::kDiagSleepMs))
  {
    appsink_max_buffers_ = std::max(1, appsink_max_buffers_);

    blackbox::image::init(kBlackboxPath);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      kTopicName, rclcpp::QoS(rclcpp::KeepLast(60)).best_effort().durability_volatile());

    init_pool();

    publish_thread_run_ = true;
    publish_thread_ = std::thread(&See3cam24cugTrigSd::publish_loop, this);

    set_trigger_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_trigger_mode",
      std::bind(&See3cam24cugTrigSd::on_set_trigger, this,
                std::placeholders::_1, std::placeholders::_2));
    get_trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/get_trigger_mode",
      std::bind(&See3cam24cugTrigSd::on_get_trigger, this,
                std::placeholders::_1, std::placeholders::_2));

    gst_init(nullptr, nullptr);

    if (!hid::open_and_init_trigger(cfg::kHidDevice, cfg::kExposureUs,
                                    cfg::kAflMode, hid_fd_, get_logger())) return;
    if (!init_pipeline()) return;

    init_ready_marker();
  }

  ~See3cam24cugTrigSd() override {
    shutdown();
    blackbox::image::shutdown();
  }

 private:
  static std::string resolve_shared_path() {
    const char *home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }

  static int read_env_int(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    char* end = nullptr;
    errno = 0;
    long parsed = std::strtol(v, &end, 10);
    if (errno || end == v || *end != '\0') return fallback;
    return static_cast<int>(parsed);
  }

  static bool read_env_bool(const char* name, bool fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    if (!std::strcmp(v, "1") || !std::strcmp(v, "true") || !std::strcmp(v, "TRUE")) return true;
    if (!std::strcmp(v, "0") || !std::strcmp(v, "false") || !std::strcmp(v, "FALSE")) return false;
    return fallback;
  }

  void init_pool() {
    for (int i = 0; i < cfg::kPoolSize; ++i) {
      auto* m = new sensor_msgs::msg::Image();
      m->header.frame_id = cfg::kFrameId;
      m->encoding        = "bgr8";
      m->is_bigendian    = 0;
      q_free_.push_back(m);
    }
  }

  bool init_pipeline() {
    char pipe_str[1024];
    std::snprintf(pipe_str, sizeof(pipe_str),
      "v4l2src device=%s name=src"
      " ! video/x-raw,format=UYVY,width=%d,height=%d,framerate=%d/1"
      " ! nvvidconv"
      " ! video/x-raw,format=BGRx,width=%d,height=%d"
      " ! videoconvert"
      " ! video/x-raw,format=BGR"
      " ! appsink name=sink emit-signals=true sync=false max-buffers=%d drop=%s",
      cfg::kDevice,
      kCaptureWidth, kCaptureHeight, kCaptureFps,
      kOutputWidth, kOutputHeight,
      appsink_max_buffers_, appsink_drop_ ? "true" : "false");

    GError *err = nullptr;
    pipeline_ = gst_parse_launch(pipe_str, &err);
    if (!pipeline_ || err) {
      RCLCPP_FATAL(get_logger(), "\033[31m%s GST ... FAIL — parse: %s\033[0m",
                   kLogLabel, err ? err->message : "unknown");
      if (err) g_error_free(err);
      return false;
    }

    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    g_signal_connect(sink, "new-sample", G_CALLBACK(on_new_sample_static), this);
    gst_object_unref(sink);

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
    gst_bus_add_watch(bus, on_bus_message_static, this);
    gst_object_unref(bus);

    gst_loop_ = g_main_loop_new(nullptr, FALSE);
    gst_thread_ = std::thread([this]() { g_main_loop_run(gst_loop_); });

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(get_logger(), "\033[31m%s GST ... FAIL — PLAYING\033[0m", kLogLabel);
      return false;
    }

    // pad probe: v4l2src src pad — DQBUF 완료 직후 mono_raw_ns() 기록
    GstElement *src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (src_elem) {
      GstPad *src_pad = gst_element_get_static_pad(src_elem, "src");
      if (src_pad) {
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          on_v4l2src_push_static, this, nullptr);
        gst_object_unref(src_pad);
      }
      gst_object_unref(src_elem);
    }
    return true;
  }

  static GstPadProbeReturn on_v4l2src_push_static(GstPad*, GstPadProbeInfo*, gpointer data) {
    static_cast<See3cam24cugTrigSd*>(data)->t_dqbuf_ns_ = blackbox::mono_raw_ns();
    return GST_PAD_PROBE_OK;
  }

  void init_ready_marker() {
    ready_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      const uint64_t stamped = stamped_count_;
      if (!ok_reported_ && stamped >= cfg::kReadyFrameThreshold) {
        write_sync_done_marker();
        RCLCPP_INFO(get_logger(), "\033[34m%s SYNC ... OK\033[0m", kLogLabel);
        ok_reported_ = true;
        ready_timer_.reset();
        return;
      }
      if (++ready_tick_count_ >= cfg::kReadyTimeoutSec && !fail_reported_) {
        RCLCPP_ERROR(get_logger(), "\033[31m%s SYNC ... FAIL — stamped_frames=%lu (%ds timeout)\033[0m",
                     kLogLabel,
                     static_cast<unsigned long>(stamped),
                     cfg::kReadyTimeoutSec);
        fail_reported_ = true;
      }
    });
  }

  static void write_sync_done_marker() {
    uint64_t mono_ns = blackbox::mono_raw_ns();
    const std::string sync_path = blackbox::log_dir() + "/sync_done_mono_ns";
    blackbox::detail::mkdir_for_file(sync_path);
    int fd = ::open(sync_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
      ::write(fd, &mono_ns, sizeof(mono_ns));
      ::close(fd);
    }
  }

  // ─── hot path ───────────────────────────────────────────

  void ensure_msg_for_size(sensor_msgs::msg::Image& m, int w, int h) {
    const size_t need = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    if (m.width == static_cast<uint32_t>(w) &&
        m.height == static_cast<uint32_t>(h) &&
        m.data.size() == need) return;
    m.height = static_cast<uint32_t>(h);
    m.width  = static_cast<uint32_t>(w);
    m.step   = static_cast<uint32_t>(w * 3);
    m.data.resize(need);
  }

  void publish_loop() {
    while (publish_thread_run_) {
      ReadySlot rs{};
      {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [this]{ return !q_ready_.empty() || !publish_thread_run_; });
        if (!publish_thread_run_ && q_ready_.empty()) break;
        rs = q_ready_.front(); q_ready_.pop_front();
      }
      try {
        publisher_->publish(*rs.msg);
        if (!image_ok_reported_.exchange(true)) {
          RCLCPP_INFO(get_logger(), "\033[34m%s IMAGE ... OK\033[0m", kLogLabel);
        }
        blackbox::image::log_pub(rs.record_idx);
      } catch (const std::exception&) {
      }
      {
        std::lock_guard<std::mutex> lk(q_mtx_);
        q_free_.push_back(rs.msg);
      }
    }
  }

  static GstFlowReturn on_new_sample_static(GstElement *sink, gpointer data) {
    return static_cast<See3cam24cugTrigSd *>(data)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstElement *sink) {
    if (!running_) return GST_FLOW_EOS;
    if (!gst_ok_reported_.exchange(true)) {
      RCLCPP_INFO(get_logger(), "\033[34m%s GST ... OK\033[0m", kLogLabel);
    }
    if (diag_sleep_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(diag_sleep_ms_));
    }

    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return GST_FLOW_ERROR;

    stamper_.try_open();
    int64_t ns = stamper_.read_low_ns();

    if (skip_remaining_ > 0) {
      skip_remaining_--;
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    const uint64_t t_push = t_dqbuf_ns_;  // pad probe 기록값 (MONOTONIC_RAW)

    GstCaps *caps = gst_sample_get_caps(sample);
    int sample_w = 0, sample_h = 0;
    if (caps) {
      GstStructure *s = gst_caps_get_structure(caps, 0);
      gst_structure_get_int(s, "width",  &sample_w);
      gst_structure_get_int(s, "height", &sample_h);
    }
    if (sample_w <= 0 || sample_h <= 0) {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    sensor_msgs::msg::Image* slot = acquire_slot();
    if (!slot) {
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    ensure_msg_for_size(*slot, sample_w, sample_h);
    const size_t n_pixels = static_cast<size_t>(sample_w) * static_cast<size_t>(sample_h);
    const size_t expected = n_pixels * 3;
    if (map.size < expected) {
      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      release_slot_to_free(slot);
      return GST_FLOW_OK;
    }
    std::memcpy(slot->data.data(), map.data, expected);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    if (ns > 0) {
      slot->header.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
      stamped_count_++;
    } else {
      slot->header.stamp = now();
    }

    const uint64_t header_stamp_ns =
        static_cast<uint64_t>(slot->header.stamp.sec) * 1000000000ULL
      + static_cast<uint64_t>(slot->header.stamp.nanosec);
    const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_push);

    handoff_to_publisher(slot, record_idx);
    return GST_FLOW_OK;
  }

  sensor_msgs::msg::Image* acquire_slot() {
    std::lock_guard<std::mutex> lk(q_mtx_);
    if (!q_free_.empty()) {
      auto* s = q_free_.front(); q_free_.pop_front(); return s;
    }
    if (!q_ready_.empty()) {
      auto* s = q_ready_.front().msg; q_ready_.pop_front();
      return s;
    }
    return nullptr;
  }

  void release_slot_to_free(sensor_msgs::msg::Image* s) {
    std::lock_guard<std::mutex> lk(q_mtx_);
    q_free_.push_back(s);
  }

  void handoff_to_publisher(sensor_msgs::msg::Image* s, size_t record_idx) {
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      q_ready_.push_back(ReadySlot{s, record_idx});
    }
    q_cv_.notify_one();
  }

  // ─── GST bus / shutdown / services ──────────────────────

  static gboolean on_bus_message_static(GstBus *, GstMessage *msg, gpointer data) {
    return static_cast<See3cam24cugTrigSd *>(data)->on_bus_message(msg);
  }

  gboolean on_bus_message(GstMessage *msg) {
    const GstMessageType t = GST_MESSAGE_TYPE(msg);
    if (t == GST_MESSAGE_ERROR) {
      GError *err = nullptr; gchar *dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      if (!gst_fail_reported_.exchange(true)) {
        RCLCPP_ERROR(get_logger(),
                     "\033[31m%s GST ... FAIL — bus: %s\033[0m",
                     kLogLabel,
                     err && err->message ? err->message : "(no message)");
      }
      g_clear_error(&err); g_free(dbg);
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      skip_remaining_ = cfg::kSkipFrames;
    }
    return TRUE;
  }

  void shutdown() {
    running_ = false;
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);

    publish_thread_run_ = false;
    q_cv_.notify_all();
    if (publish_thread_.joinable()) publish_thread_.join();
    for (auto* m : q_free_)  delete m;
    for (auto& rs : q_ready_) delete rs.msg;
    q_free_.clear(); q_ready_.clear();

    if (gst_loop_) {
      g_main_loop_quit(gst_loop_);
      if (gst_thread_.joinable()) gst_thread_.join();
      g_main_loop_unref(gst_loop_); gst_loop_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_); pipeline_ = nullptr;
    }
    stamper_.close();
    hid::close_and_restore(hid_fd_, cfg::kRestoreMaster, get_logger());
  }

  void on_set_trigger(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
  {
    if (hid_fd_ < 0) { resp->success = false; resp->message = "HID N/A"; return; }
    uint8_t mode = req->data ? 0x01 : 0x00;
    bool ok = hid::set_stream_mode(hid_fd_, mode, cfg::kAflMode, get_logger());
    resp->success = ok;
    resp->message = ok ? (req->data ? "TRIGGER" : "MASTER") : "HID failed";
  }

  void on_get_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    if (hid_fd_ < 0) { resp->success = false; resp->message = "HID N/A"; return; }
    uint8_t mode = 0, afl = 0;
    bool ok = hid::get_stream_mode(hid_fd_, mode, afl, get_logger());
    resp->success = ok;
    resp->message = ok ? (mode == 0x01 ? "TRIGGER" : "MASTER") : "HID failed";
  }

  // ─── 멤버 ────────────────────────────────────────────────

  cfg::MmapStamper  stamper_;

  std::atomic<bool> running_;
  int               hid_fd_;
  GstElement       *pipeline_;
  GMainLoop        *gst_loop_;
  std::thread       gst_thread_;
  uint64_t          t_dqbuf_ns_{0};  // pad probe: mono_raw_ns() at v4l2src push

  uint64_t stamped_count_       = 0;
  int      skip_remaining_      = cfg::kSkipFrames;

  int      appsink_max_buffers_;
  bool     appsink_drop_;
  int      diag_sleep_ms_;

  struct ReadySlot {
    sensor_msgs::msg::Image* msg;
    size_t                   record_idx;
  };
  std::mutex                            q_mtx_;
  std::condition_variable               q_cv_;
  std::deque<sensor_msgs::msg::Image*>  q_free_;
  std::deque<ReadySlot>                 q_ready_;
  std::thread                           publish_thread_;
  std::atomic<bool>                     publish_thread_run_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   set_trigger_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   get_trigger_srv_;

  rclcpp::TimerBase::SharedPtr ready_timer_;
  uint32_t ready_tick_count_{0};
  bool     ok_reported_{false};
  bool     fail_reported_{false};
  std::atomic<bool> image_ok_reported_{false};
  std::atomic<bool> gst_ok_reported_{false};
  std::atomic<bool> gst_fail_reported_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<See3cam24cugTrigSd>());
  rclcpp::shutdown();
  return 0;
}
