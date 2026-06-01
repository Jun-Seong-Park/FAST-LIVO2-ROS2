#pragma once
// 디바이스/노출/트리거 기본값 + 해상도 프로파일 + ROS2 파라미터 로더.
// constexpr k* 는 declare_parameter 의 "기본값"이고, config/config.yaml (또는 launch resolution:=) 가
// 이를 오버라이드한다. 해상도는 숫자 파라미터로 받지 않고 resolution 프로파일 문자열을
// resolve_profile() 로 풀어서 캡처/출력/fps 를 결정한다.

#include <cstddef>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace see3cam {

// ── 기본값 (config/config.yaml 로 오버라이드 가능) ──

// 디바이스 (udev symlink — 포트 바뀌어도 카메라 식별로 잡힘)
inline constexpr const char* kDevice    = "/dev/24cug";
inline constexpr const char* kHidDevice = "/dev/24cug_hid";

// 캡처 backend (Jetson 전용 — gstreamer 는 nvvidconv HW 경로)
inline constexpr const char* kBackend  = "gstreamer";  // gstreamer | opencv

// 해상도 프로파일 (sd | hd | fhd | wuxga). 실제 픽셀은 resolve_profile() 참조.
inline constexpr const char* kResolution = "sd";

// 노출/트리거
inline constexpr int     kExposureUs    = 10000;  // 10 ms
inline constexpr uint8_t kAflMode       = 0x00;   // Auto Frame Length OFF (ROS 파라미터는 int — uint8 로 캐스팅)
inline constexpr bool    kRestoreMaster = false;  // shutdown 시 MASTER 복귀 안 함

// 발행 포맷
// false → raw bgr8 Image (nvvidconv/videoconvert 경로).
// true  → 카메라 MJPG 를 v4l2src 가 image/jpeg 로 직접 캡처해 CompressedImage(format=jpeg) 발행.
//         JPEG 는 스트림 내 리스케일 불가 → capture 해상도 그대로 발행(다운스케일·output 무관).
inline constexpr bool    kCompressed    = false;

// ROS 토픽
inline constexpr const char* kTopicName = "/camera/image";
inline constexpr const char* kFrameId   = "camera_init";

// ── 해상도 프로파일 ──
// 캡처 = 센서 native UYVY (HD/FHD/WUXGA 만 존재 — See3CAM_24CUG README 표 근거).
// 출력 = nvvidconv 후 발행 해상도. sd 만 다운스케일(1280x720→640x360), 나머지는 풀프레임.
// fps 는 caps 협상용 명목값 — trigger 모드 실효 프레임레이트는 외부 트리거(~10 Hz)가 결정.
struct Resolution {
  int capture_width;
  int capture_height;
  int output_width;
  int output_height;
  int fps;
};

inline Resolution resolve_profile(const std::string& profile) {
  if (profile == "hd")    return {1280,  720, 1280,  720, 60};
  if (profile == "fhd")   return {1920, 1080, 1920, 1080, 60};
  if (profile == "wuxga") return {1920, 1200, 1920, 1200, 60}; // MJPG 1920x1200 = 60/114 only (55 is UYVY-only → not-negotiated)
  // "sd" 및 그 외 → 기본 sd (캡처 HD, 출력 640x360 다운스케일)
  return {1280, 720, 640, 360, 60};
}

// ── 런타임 파라미터 ──
struct Params {
  std::string device;          // udev symlink (캡처)
  std::string hid_device;      // udev symlink (HID 제어)
  std::string backend;         // gstreamer | v4l2_opencv
  std::string resolution;      // 프로파일 문자열 (sd | hd | fhd | wuxga)
  Resolution  res;             // resolution 을 resolve_profile() 로 푼 결과
  size_t      expected_size;   // 파생값 — output_width * output_height * 3 (BGR 3ch)
  int         exposure_us;
  uint8_t     afl_mode;
  bool        restore_master;
  bool        compressed;      // true → MJPG CompressedImage, false → raw bgr8 Image
  std::string topic_name;
  std::string frame_id;
};

// 각 파라미터를 위 기본값으로 declare 한 뒤(yaml/launch 가 있으면 그 값으로 오버라이드됨) 읽어 Params 로 반환.
// res / expected_size 는 파라미터가 아니라 resolution 프로파일에서 파생 — 마지막에 계산.
inline Params load_params(rclcpp::Node& node) {
  Params p;
  p.device         = node.declare_parameter<std::string>("device",      kDevice);
  p.hid_device     = node.declare_parameter<std::string>("hid_device",  kHidDevice);
  p.backend        = node.declare_parameter<std::string>("backend",     kBackend);
  p.resolution     = node.declare_parameter<std::string>("resolution",  kResolution);
  p.exposure_us    = node.declare_parameter<int>("exposure_us",         kExposureUs);
  p.afl_mode       = static_cast<uint8_t>(node.declare_parameter<int>("afl_mode", kAflMode));
  p.restore_master = node.declare_parameter<bool>("restore_master",     kRestoreMaster);
  p.compressed     = node.declare_parameter<bool>("compressed",         kCompressed);
  p.topic_name     = node.declare_parameter<std::string>("topic_name",  kTopicName);
  p.frame_id       = node.declare_parameter<std::string>("frame_id",    kFrameId);
  p.res            = resolve_profile(p.resolution);
  p.expected_size  = static_cast<size_t>(p.res.output_width) * p.res.output_height * 3;
  return p;
}

}  // namespace see3cam
