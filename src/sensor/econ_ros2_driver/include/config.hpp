#pragma once
// 디바이스/노출/트리거 기본값 + 해상도 프로파일 + ROS2 파라미터 로더.
// constexpr k* 는 declare_parameter 의 "기본값"이고, config/config.yaml (또는 launch resolution:=) 가
// 이를 오버라이드한다. 해상도는 숫자 파라미터로 받지 않고 resolution 프로파일 문자열을
// resolve_profile_uyvy()/resolve_profile_mjpg() 로 풀어서 캡처/출력/fps 를 결정한다(포맷별 표).

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

// 해상도 프로파일 (sd | hd | fhd | wuxga). 실제 픽셀은 resolve_profile_uyvy/mjpg() 참조.
inline constexpr const char* kResolution = "sd";

// 노출/트리거
inline constexpr int     kExposureUs    = 10000;  // 10 ms

// nvvidconv 회전 (HW). gst-inspect-1.0 nvvidconv 이 머신 실측 enum:
//   0 none / 1 counterclockwise(CCW 90°) / 2 rotate-180 / 3 clockwise(CW 90°).
// 90°(1·3)은 출력 가로·세로를 swap → published 치수도 함께 바뀐다(pub_width/height 파생). gstreamer backend 전용.
inline constexpr int     kFlipMethod    = 0;   // 기본 회전 없음
// afl_mode 는 ROS 파라미터로 노출하지 않고 아래 값으로 하드코딩한다 (사용처에서 직접 참조).
inline constexpr uint8_t kAflMode       = 0x00;   // Auto Frame Length OFF (하드코딩)

// 발행 포맷
// false → raw bgr8 Image (nvvidconv/videoconvert 경로).
// true  → 카메라 MJPG 를 v4l2src 가 image/jpeg 로 직접 캡처해 CompressedImage(format=jpeg) 발행.
//         JPEG 는 스트림 내 리스케일 불가 → capture 해상도 그대로 발행(다운스케일·output 무관).
inline constexpr bool    kCompressed    = false;

// ROS 토픽
inline constexpr const char* kTopicName = "/camera/image";
inline constexpr const char* kFrameId   = "camera_init";

// ── 해상도 프로파일 ──
// 출력 = nvvidconv 후 발행 해상도. sd 만 다운스케일(1280x720→640x360), 나머지는 풀프레임.
// fps 는 caps 협상용 명목값 — trigger 모드 실효 프레임레이트는 외부 트리거(~10 Hz)가 결정.
//
// 캡처 fps 메뉴가 포맷마다 달라서(UYVY vs MJPG) 프로파일 표를 포맷별로 분리한다.
// 각 표는 v4l2-ctl --list-formats-ext (/dev/24cug) 출력의 1:1 사본 — 조건 분기로 fps 를 보정하지 않는다.
// 선택은 load_params() 가 compressed 로 한다 (compressed → MJPG, 아니면 UYVY).
struct Resolution {
  int capture_width;
  int capture_height;
  int output_width;
  int output_height;
  int fps;
};

// 90° 회전(CCW=1 / CW=3)이면 출력 가로·세로가 바뀐다. 0(none)·2(180°)는 치수 불변.
inline bool is_quarter_turn(int flip_method) { return flip_method == 1 || flip_method == 3; }

// UYVY(raw, compressed=false) 캡처 프로파일.
// v4l2-ctl UYVY 메뉴: 1280x720=60/120, 1920x1080=60, 1920x1200=55 (60 없음).
inline Resolution resolve_profile_uyvy(const std::string& profile) {
  if (profile == "hd")    return {1280,  720, 1280,  720, 60};
  if (profile == "fhd")   return {1920, 1080, 1920, 1080, 60};
  if (profile == "wuxga") return {1920, 1200, 1920, 1200, 55}; // UYVY 1920x1200 은 55 만 협상됨
  // "sd" 및 그 외 → 기본 sd (캡처 HD, 출력 640x360 다운스케일)
  return {1280, 720, 640, 360, 60};
}

// MJPG(compressed=true) 캡처 프로파일.
// v4l2-ctl MJPG 메뉴: 1280x720=60/120, 1920x1080=30/60/120, 1920x1200=60/114.
inline Resolution resolve_profile_mjpg(const std::string& profile) {
  if (profile == "hd")    return {1280,  720, 1280,  720, 60};
  if (profile == "fhd")   return {1920, 1080, 1920, 1080, 60};
  if (profile == "wuxga") return {1920, 1200, 1920, 1200, 60}; // MJPG 1920x1200 = 60/114
  // "sd" 및 그 외 → 기본 sd (캡처 HD, 출력 640x360 다운스케일)
  return {1280, 720, 640, 360, 60};
}

// ── 런타임 파라미터 ──
struct Params {
  std::string device;          // udev symlink (캡처)
  std::string hid_device;      // udev symlink (HID 제어)
  std::string backend;         // gstreamer | opencv
  std::string resolution;      // 프로파일 문자열 (sd | hd | fhd | wuxga)
  Resolution  res;             // resolution 을 resolve_profile() 로 푼 결과
  int         flip_method;     // nvvidconv 회전 enum (0 none / 1 CCW / 2 180 / 3 CW). gstreamer 전용
  int         pub_width;       // 파생 — 회전 반영한 발행 가로 (90° 회전 시 output_height)
  int         pub_height;      // 파생 — 회전 반영한 발행 세로 (90° 회전 시 output_width)
  size_t      expected_size;   // 파생값 — pub_width * pub_height * 3 (BGR 3ch). 회전 무관(가로·세로 곱 불변)
  int         exposure_us;
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
  p.compressed     = node.declare_parameter<bool>("compressed",         kCompressed);
  p.flip_method    = node.declare_parameter<int>("flip_method",         kFlipMethod);
  p.topic_name     = node.declare_parameter<std::string>("topic_name",  kTopicName);
  p.frame_id       = node.declare_parameter<std::string>("frame_id",    kFrameId);
  // compressed 가 캡처 포맷을 결정 → 포맷별 프로파일 표에서 fps 까지 올바르게 가져온다.
  p.res            = p.compressed ? resolve_profile_mjpg(p.resolution)
                                  : resolve_profile_uyvy(p.resolution);
  // 90° 회전이면 발행 가로·세로 swap (nvvidconv 가 rotate 후 출력하므로 치수가 바뀐다).
  const bool quarter = is_quarter_turn(p.flip_method);
  p.pub_width      = quarter ? p.res.output_height : p.res.output_width;
  p.pub_height     = quarter ? p.res.output_width  : p.res.output_height;
  p.expected_size  = static_cast<size_t>(p.pub_width) * p.pub_height * 3;
  return p;
}

}  // namespace see3cam
