// See3CAM_24CUG — TEST variant. gscam2 스타일 단일 thread / pull / move publish.
// publish 경로 drop A/B 비교용. 같은 포맷(UYVY 1280x720 → BGRx 640x360 → BGR),
// 같은 mmap stamp 공유, 같은 HID 외부 트리거 진입 — 차이는 소비/발행 구조만.
//
// vs see3cam24cug_trig_sd.cpp 와의 차이 (의도된 단순화):
//   - 풀 / q_free / q_ready / condvar / mutex 없음
//   - cbk(push) 모델 → try_pull_sample(100ms) pull 모델
//   - 매 프레임 std::make_unique<Image>() · publisher_->publish(std::move(img))
//   - blackbox · pad probe · ready timer · bus error restart · set/get_trigger 서비스 없음

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "image_ros_driver/config.hpp"
#include "image_ros_driver/hid_control.hpp"
#include "image_ros_driver/mmap_stamper.hpp"
#include "blackbox/blackbox.hpp"

namespace cfg = see3cam;
namespace hid = see3cam::hid;

class See3cam24cugTest : public rclcpp::Node
{
 public:
  See3cam24cugTest()
    : Node("see3cam24cug_test"),
      stamper_(resolve_shared_path()),
      hid_fd_(-1),
      pipeline_(nullptr),
      sink_(nullptr),
      stop_(false)
  {
    blackbox::image::init(blackbox::log_dir() + "/see3cam24cug_sd_image_pub.bin");

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile());

    gst_init(nullptr, nullptr);

    if (!hid::open_and_init_trigger(cfg::kHidDevice, cfg::kExposureUs,
                                    cfg::kAflMode, hid_fd_, get_logger())) return;
    if (!init_pipeline()) return;

    pipeline_thread_ = std::thread(&See3cam24cugTest::pipeline_loop, this);
  }

  ~See3cam24cugTest() override {
    shutdown();
    blackbox::image::shutdown();
  }

 private:
  static std::string resolve_shared_path() {
    const char* home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }

  bool init_pipeline() {
    // 포맷·디바이스는 sd 와 동일. appsink 옵션만 gscam2 풍으로 단순화:
    //   emit-signals=false (pull 모델), sync=false (트리거 모드라 clock 동기 불필요),
    //   max-buffers/drop 미지정 — gstreamer 기본값에 맡김.
    char pipe_str[1024];
    std::snprintf(pipe_str, sizeof(pipe_str),
      "v4l2src device=/dev/24cug name=src"
      " ! video/x-raw,format=UYVY,width=1280,height=720,framerate=60/1"
      " ! nvvidconv"
      " ! video/x-raw,format=BGRx,width=640,height=360"
      " ! videoconvert"
      " ! video/x-raw,format=BGR"
      " ! appsink name=sink emit-signals=false sync=false"
      );

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipe_str, &err);
    if (!pipeline_ || err) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — parse: %s\033[0m",
                   err ? err->message : "unknown");
      if (err) g_error_free(err);
      return false;
    }

    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink_) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — no sink\033[0m");
      return false;
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — PLAYING\033[0m");
      return false;
    }

    // pad probe: v4l2src src pad — DQBUF 완료 직후 mono_raw_ns() 기록 (sd 와 동일).
    GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (src_elem) {
      GstPad* src_pad = gst_element_get_static_pad(src_elem, "src");
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
    static_cast<See3cam24cugTest*>(data)->t_dqbuf_ns_ = blackbox::mono_raw_ns();
    return GST_PAD_PROBE_OK;
  }

  void pipeline_loop() {
    while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(sink_), 100 * GST_MSECOND);
      if (!sample) continue;
      process_sample(sample);
      gst_sample_unref(sample);
    }
  }

  void process_sample(GstSample* sample) {
    GstBuffer* buf = gst_sample_get_buffer(sample);
    if (!buf) return;

    GstCaps* caps = gst_sample_get_caps(sample);
    int w = 0, h = 0;
    if (caps) {
      GstStructure* s = gst_caps_get_structure(caps, 0);
      gst_structure_get_int(s, "width",  &w);
      gst_structure_get_int(s, "height", &h);
    }
    if (w <= 0 || h <= 0) return;

    GstMemory* memory = gst_buffer_get_memory(buf, 0);
    if (!memory) return;
    GstMapInfo info;
    if (!gst_memory_map(memory, &info, GST_MAP_READ)) {
      gst_memory_unref(memory);
      return;
    }

  std::cout << static_cast<void*>(memory)   << "\n";  // ← GstMemory 구조체 주소 (핸들)
  std::cout << static_cast<void*>(info.data) << "\n";  // ← 실제 BGR 픽셀 시작 주소

    const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    if (info.size < expected) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      return;
    }

    stamper_.try_open();
    const int64_t ns = stamper_.read_low_ns();

    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header.frame_id = cfg::kFrameId;
    img->header.stamp    = (ns > 0) ? rclcpp::Time(ns, RCL_ROS_TIME) : now();
    img->width           = static_cast<uint32_t>(w);
    img->height          = static_cast<uint32_t>(h);
    img->encoding        = "bgr8";
    img->is_bigendian    = 0;
    img->step            = static_cast<uint32_t>(w * 3);
    img->data.resize(expected);
    std::memcpy(img->data.data(), info.data, expected);

    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);

    const uint64_t header_stamp_ns =
        static_cast<uint64_t>(img->header.stamp.sec) * 1000000000ULL
      + static_cast<uint64_t>(img->header.stamp.nanosec);
    const uint64_t t_push = t_dqbuf_ns_;  // pad probe 기록값 (MONOTONIC_RAW)
    const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_push);

    publisher_->publish(std::move(img));
    blackbox::image::log_pub(record_idx);
  }

  void shutdown() {
    stop_.store(true);
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (pipeline_thread_.joinable()) pipeline_thread_.join();
    if (sink_)     { gst_object_unref(sink_);     sink_     = nullptr; }
    if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
    stamper_.close();
    hid::close_and_restore(hid_fd_, cfg::kRestoreMaster, get_logger());
  }

  cfg::MmapStamper  stamper_;
  int               hid_fd_;
  GstElement*       pipeline_;
  GstElement*       sink_;
  std::thread       pipeline_thread_;
  std::atomic<bool> stop_;
  uint64_t          t_dqbuf_ns_{0};  // pad probe: mono_raw_ns() at v4l2src push
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<See3cam24cugTest>());
  rclcpp::shutdown();
  return 0;
}
