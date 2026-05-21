#pragma once
// 전역 하드코딩 상수. yaml/declare_parameter 사용 안 함.
// 변경 시 여기 한 곳만 수정.

#include <cstdint>

namespace see3cam {

// ── 디바이스 (udev symlink — 포트 바뀌어도 카메라 식별로 잡힘) ──
inline constexpr const char* kDevice    = "/dev/24cug";
inline constexpr const char* kHidDevice = "/dev/24cug_hid";

// ── 해상도/프레임 ──
// 캡처 = 센서 native 1280x720, 출력 = nvvidconv 다운스케일 640x360.
// 캡처 != 출력 (혼동 주의).
inline constexpr int kCaptureWidth  = 1280;
inline constexpr int kCaptureHeight = 720;
inline constexpr int kOutputWidth   = 640;
inline constexpr int kOutputHeight  = 360;
inline constexpr int kCaptureFps    = 60;   // gst-launch 검증 시 사용한 값. trigger 모드에선 명목값 (실효는 STM32 PWM)

// ── 노출/트리거 ──
inline constexpr int     kExposureUs    = 10000;  // 10 ms
inline constexpr uint8_t kAflMode       = 0x00;   // Auto Frame Length OFF
inline constexpr bool    kRestoreMaster = false;  // shutdown 시 MASTER 복귀 안 함

// ── ROS 토픽 ──
inline constexpr const char* kTopicName = "/camera/image";
inline constexpr const char* kFrameId   = "camera_init";

// ── 핫패스 튜닝 ──
inline constexpr int kSkipFrames  = 4;    // 첫 N 프레임 버림 (warmup) -- qtcam reference
inline constexpr int kLogInterval = 50;
inline constexpr int kPoolSize    = 4;    // async publish 슬롯 수
inline constexpr int kAppsinkMaxBuffers = 4;
inline constexpr bool kAppsinkDrop = true;

// ── stdout handshake ──
inline constexpr int kReadyFrameThreshold = 30;   // stamped_count_ >= 30 → CAMERA OK
inline constexpr int kReadyTimeoutSec     = 30;   // 미달 시 CAMERA FAIL

// ── 버퍼 드롭 진단 ──
// 환경변수로만 override. 운영 launch/yaml 표면은 건드리지 않는다.
inline constexpr double kExpectedTriggerHz = 10.0;
inline constexpr int kDiagSleepMs = 0;
inline constexpr const char* kBufferDiagCsv = "/tmp/see3cam_buffer_diag.csv";

}  // namespace see3cam
