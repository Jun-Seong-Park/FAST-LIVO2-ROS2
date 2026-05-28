// See3CAM_24CUG — TEST variant. gscam2 스타일 단일 thread / pull / move publish.
// publish 경로 drop A/B 비교용. 같은 포맷(UYVY 1280x720 → BGRx 640x360 → BGR),
// 같은 mmap stamp 공유, 같은 HID 외부 트리거 진입 — 차이는 소비/발행 구조만.
//
// vs see3cam24cug_trig_sd.cpp 와의 차이 (의도된 단순화):
//   - 풀 / q_free / q_ready / condvar / mutex 없음
//   - cbk(push) 모델 → try_pull_sample(100ms) pull 모델
//   - 재사용 멤버 버퍼 1개 · publisher_->publish(const&) — 별도 프로세스(inter-process)라 publish 가
//     반환 전 DDS 로 동기 복사를 끝내므로(rclcpp publisher.hpp:456 rcl_publish) 버퍼 1개로 충분
//   - blackbox · pad probe · ready timer · bus error restart · set/get_trigger 서비스 없음

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
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
    // 블랙박스 초기화
    blackbox::image::init(blackbox::log_dir() + "/see3cam24cug_sd_image_pub.bin");
    
    // 발행자 초기화
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    // 재사용 버퍼의 고정 필드 1회 설정 (가변 필드 stamp/size 는 매 프레임 갱신)
    msg_.header.frame_id = cfg::kFrameId;
    msg_.encoding        = "bgr8";
    msg_.is_bigendian    = 0;

    // GStreamer 초기화
    gst_init(nullptr, nullptr);

    // HID 초기화 및 트리거 모드 설정
    if (!hid::open_and_init_trigger(cfg::kHidDevice, cfg::kExposureUs,
                                    cfg::kAflMode, hid_fd_, get_logger())) return;

    // GStreamer 파이프라인 초기화
    if (!init_pipeline()) return;

    // 파이프라인 처리 스레드 시작
    pipeline_thread_ = std::thread(&See3cam24cugTest::pipeline_loop, this);
  }

  ~See3cam24cugTest() override {
    shutdown();
    blackbox::image::shutdown();
  }

 private:
  // timestamp ssd 공유 경로 결정: $HOME/timeshare 또는 /tmp/timeshare
  static std::string resolve_shared_path() {
    const char* home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }

  // GStreamer pipeline 을 parse 하고 PLAYING 상태까지 올린다.
  //
  // 입출력 : 인자 없음. 반환 bool — true=정상, false=parse 또는 state 전이 실패.
  // 소유권 : pipeline_/sink_ 멤버에 소유권 보관. 해제는 shutdown() 가 책임 (set_state(NULL) + gst_object_unref).
  //          src pad probe 는 pad 내부에 등록되며 별도 제거 안 함 (pipeline 해제 시 자동 해제).
  // 성능   : 일회성 초기화 (생성자에서 1회 호출). PLAYING 전이는 동기적 — 카메라가 READY 될 때까지 수백 ms 블록 가능.
  // 재진입 : 재진입 불가. 한 객체당 1회 호출 전제 — 두 번 부르면 이전 pipeline 누수.
  // null   : 없음 (호출 인자가 없어서).
  bool init_pipeline() {

    
    // pipeline str 종이 크기 1024 byte 설정 (확인 필요)
    char pipe_str[1024];

    // gstreamer 파이프라인 문자열 작성
    std::snprintf(pipe_str, sizeof(pipe_str),
      "v4l2src device=/dev/24cug name=src"
      " ! video/x-raw,format=UYVY,width=1280,height=720,framerate=60/1"
      " ! nvvidconv"
      " ! video/x-raw,format=BGRx,width=640,height=360"
      " ! videoconvert"
      " ! video/x-raw,format=BGR"
      " ! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
      );

    // Gstreamer 파이프라인 에러 체크 및 생성
    GError* err = nullptr;
    
    // std::cout << "ptr value:" << static_cast<void*>(err) << "\n";  // ← GError 구조체로 만든 포인터 주소 (nullptr 는 0이다)
    // std::cout << "ptr addr" << static_cast<void*>(&err) << "\n";  // ← GError 구조체로 만든 err 이라는 텍스트 변수의 주소

    // Gstreamer 파이프라인 생성
    pipeline_ = gst_parse_launch(pipe_str, &err);
    
    // Pipeline 오류
    if (!pipeline_ || err) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — parse: %s\033[0m",
                   err ? err->message : "unknown");
      if (err) g_error_free(err);
      return false;
    }

    // 
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    // [확인 필요]
    if (!sink_) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — no sink\033[0m");
      return false;
    }
    // gstreamer 가 failure 을 반환하면 fatal 로그를 남기고 false 반환
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(get_logger(), "\033[31m[see3cam24cug_test] GST ... FAIL — PLAYING\033[0m");
      return false;
    }

    // gst element 에서 src 라는 이름의 요소가 GstElement 객체로 저장되어있는데 이걸 src_elem 이라는 포인터 변수가 주소를 가리키게 저장
    GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");

    if (src_elem) {
      // 
      GstPad* src_pad = gst_element_get_static_pad(src_elem, "src");

      //
      if (src_pad) {
        // ㅇㅇ
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                          on_v4l2src_push_static, this, nullptr);
        // ㅇㅇ
        gst_object_unref(src_pad);
      }
      // ㅇㅇ
      gst_object_unref(src_elem);
    }
    return true;
  }

  // v4l2src 가 buffer 를 src pad 로 push 할 때마다 호출되는 probe 콜백.
  // 그 시점의 MONO_RAW 시각을 t_dqbuf_ns_ 에 기록한다 (이후 t_push 로 사용).
  //
  // 입출력 : pad·info 미사용 (이름 생략). data 는 등록 시 넘긴 this — nullptr 불가.
  //          반환은 항상 GST_PAD_PROBE_OK — buffer 를 그대로 통과 (drop/수정 없음).
  // 소유권 : 새 메모리 할당 없음. data 는 객체 lifetime 이라 해제 책임 없음.
  // 성능   : hot path (10 Hz 트리거당 1회). clock_gettime(CLOCK_MONOTONIC_RAW) 1회만.
  // 재진입 : 단일 GStreamer streaming thread 에서만 호출 → 재진입 불가.
  //          단, t_dqbuf_ns_ 는 다른 thread(pipeline_loop) 가 읽음 — atomic 아님,
  //          64bit 정렬 가정으로 torn read 거의 없음 (엄밀히는 보장 X).
  static GstPadProbeReturn on_v4l2src_push_static(GstPad*, GstPadProbeInfo*, gpointer data) {
    auto* self = static_cast<See3cam24cugTest*>(data);
    self->t_dqbuf_ns_ = blackbox::mono_raw_ns();
    self->n_push_.fetch_add(1, std::memory_order_relaxed);
    return GST_PAD_PROBE_OK;
  }

  // GStreamer pipeline 의 appsink 에서 sample 을 pull 해서 처리하는 루프.
  void pipeline_loop() {
    // stop_ 플래그가 켜지거나 ROS가 종료될 때까지 루프. 루프당 최대 100 ms 대기하면서 sample 을 pull 한다.
    while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      
      // 100 ms 대기하면서 sample 을 pull 한다. sample 이 없으면 nullptr 반환 → 루프 계속.
      GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(sink_), 100 * GST_MSECOND);
      
        // sample 이 nullptr 이면 pull 실패 또는 timeout → 루프 계속. sample 이 있으면 처리.
      if (!sample) continue;

      // appsink (max-buffers=1, drop=true) 가 버린 frame 수 추정.
      // pad probe 에서 push 마다 ++n_push_, 여기서 pull 마다 ++n_pull_.
      // 이번 pull 직전까지 들어온 push 가 1개를 초과하면, 그 초과분이 drop.
      ++n_pull_;
      const uint64_t push_now      = n_push_.load(std::memory_order_relaxed);
      const uint64_t pushed_since  = push_now - last_push_seen_;
      last_push_seen_ = push_now;
      if (pushed_since > 1) {
        const uint64_t drops = pushed_since - 1;
        total_drops_ += drops;
        std::cout << "\033[33m[see3cam24cug_test] drop +" << drops
                  << " total=" << total_drops_
                  << " (push=" << push_now << " pull=" << n_pull_ << ")\033[0m\n";
      }

      // sample 을 Image 메시지로 변환 후 발행하는 함수 호출.
      process_sample(sample);
      
      // sample 참조 해제. appsink 내부적으로 관리하므로 gst_sample_unref 만 하면 됨 (gst_buffer · gst_caps 등은 appsink 이 관리).
      gst_sample_unref(sample);
    }
  }

  // 재사용 버퍼의 크기 의존 필드(width/height/step/data)를 필요할 때만 갱신.
  // 크기가 이미 같으면 no-op → resize 재할당 없음.
  void ensure_size(int w, int h) {
    const size_t need = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    if (msg_.width  == static_cast<uint32_t>(w) &&
        msg_.height == static_cast<uint32_t>(h) &&
        msg_.data.size() == need) return;
    msg_.width  = static_cast<uint32_t>(w);
    msg_.height = static_cast<uint32_t>(h);
    msg_.step   = static_cast<uint32_t>(w * 3);
    msg_.data.resize(need);
  }

  // GstSample* sample 을 받아서 재사용 버퍼에 채운 뒤 발행하는 함수.
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

  // std::cout << static_cast<void*>(memory)   << "\n";  // ← GstMemory 구조체 주소 (핸들)
  // std::cout << static_cast<void*>(info.data) << "\n";  // ← 실제 BGR 픽셀 시작 주소

    const size_t expected = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    if (info.size < expected) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      return;
    }

    stamper_.try_open();
    const int64_t ns = stamper_.read_low_ns();

    // 재사용 버퍼: 크기 바뀔 때만 size 필드 갱신, 매 프레임 stamp 와 픽셀만 갱신.
    ensure_size(w, h);
    msg_.header.stamp = (ns > 0) ? rclcpp::Time(ns, RCL_ROS_TIME) : now();
    std::memcpy(msg_.data.data(), info.data, expected);

    gst_memory_unmap(memory, &info);
    gst_memory_unref(memory);

    const uint64_t header_stamp_ns =
        static_cast<uint64_t>(msg_.header.stamp.sec) * 1000000000ULL
      + static_cast<uint64_t>(msg_.header.stamp.nanosec);
    const uint64_t t_push = t_dqbuf_ns_;  // pad probe 기록값 (MONOTONIC_RAW)
    const size_t record_idx = blackbox::image::log_cbk(header_stamp_ns, t_push);

    // inter-process 라 publish 가 반환 전 DDS 로 동기 복사를 끝내므로 같은 버퍼를 다음 프레임에 재사용 가능.
    publisher_->publish(msg_);
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
  std::atomic<uint64_t> n_push_{0};  // pad probe 호출 누적 = v4l2src 가 push 한 buffer 수
  uint64_t          n_pull_{0};      // appsink 에서 pull 성공 누적 (단일 thread: pipeline_loop)
  uint64_t          last_push_seen_{0};
  uint64_t          total_drops_{0};
  sensor_msgs::msg::Image msg_;      // 재사용 발행 버퍼 (단일 thread + 동기 publish 라 1개로 충분)
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<See3cam24cugTest>());
  rclcpp::shutdown();
  return 0;
}
