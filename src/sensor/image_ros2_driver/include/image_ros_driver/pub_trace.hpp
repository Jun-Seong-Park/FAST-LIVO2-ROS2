#pragma once
// deprecated
// 핫패스 mono clock 트레이스. ros2 topic hz 사용 안 함.
//
// 콜백 안에서 6 구간 시각을 packed struct 로 메모리 ring 에 적재 →
// 종료 시 trace/log/*.bin 으로 dump → 외부 스크립트가 분석.
//
// CLOCK_MONOTONIC_RAW: NTP/PTP 보정 없음. 동일 PC 안 모든 프로세스 같은 도메인.
//
// 6 구간:
//   pts→t0  appsink 큐 대기 (음수 = PTS 가 콜백 진입보다 빠름 = 정상)
//   t0→t1   pull_sample + mmap stamp 읽기
//   t1→t2   gst_buffer_map
//   t2→t3   BGRx→bgr8 strided copy
//   t3→t4   = 0 (legacy 호환용 자리, 중간 memcpy 단계 없음)
//   t4→t5   publish 큐 enqueue
// 처리시간 = t5 - t0

#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

namespace see3cam::trace {

struct __attribute__((packed)) PubSample {
  uint64_t stamp_ns;
  uint64_t pts_ns;
  uint64_t t0_enter_ns;
  uint64_t t1_pull_ns;
  uint64_t t2_map_ns;
  uint64_t t3_cvt_ns;
  uint64_t t4_mcpy_ns;
  uint64_t t5_pub_ns;
};

struct __attribute__((packed)) DropSample {
  uint64_t frame;
  uint64_t stamp_ns;
  uint64_t pts_ns;
  uint64_t gst_offset;
  uint64_t gst_offset_end;
  uint64_t gst_duration_ns;
  uint64_t t0_enter_ns;
  uint64_t t5_pub_ns;
  uint64_t stamped_count;
  uint64_t fallback_count;
  uint64_t partial_count;
  uint64_t pool_drop_count;
  uint64_t publish_throw_count;
  uint32_t gst_flags;
  uint32_t event_flags;
};

inline constexpr uint32_t kEventStamped = 1u << 0;
inline constexpr uint32_t kEventFallback = 1u << 1;
inline constexpr uint32_t kEventPartial = 1u << 2;
inline constexpr uint32_t kEventPoolDrop = 1u << 3;
inline constexpr uint32_t kEventPublishThrowSeen = 1u << 4;

inline constexpr size_t kPubBufN = 1 << 20;   // 10Hz × 64B = 64MB, ~29h
inline constexpr size_t kDropBufN = 1 << 20;  // 10Hz × 112B = ~112MB, ~29h

inline PubSample g_buf[kPubBufN];
inline size_t    g_idx = 0;
inline DropSample g_drop_buf[kDropBufN];
inline size_t     g_drop_idx = 0;

inline uint64_t MonoRawNs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
       + static_cast<uint64_t>(ts.tv_nsec);
}

inline void Record(const PubSample& s) {
  if (g_idx < kPubBufN) g_buf[g_idx++] = s;
}

inline void RecordDrop(const DropSample& s) {
  if (g_drop_idx < kDropBufN) g_drop_buf[g_drop_idx++] = s;
}

inline void EnsureLogDir() {
  ::mkdir("trace", 0755);
  ::mkdir("trace/log", 0755);
}

inline void Dump(const char* path = "trace/log/pub_trace_camera.bin") {
  EnsureLogDir();
  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return;
  const char* p = reinterpret_cast<const char*>(g_buf);
  ssize_t want = static_cast<ssize_t>(sizeof(PubSample) * g_idx);
  while (want > 0) {
    ssize_t w = ::write(fd, p, want);
    if (w <= 0) break;
    p += w; want -= w;
  }
  ::close(fd);
}

inline void DumpDrop(const char* path = "trace/log/see3cam_drop_trace.bin") {
  EnsureLogDir();
  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return;
  const char* p = reinterpret_cast<const char*>(g_drop_buf);
  ssize_t want = static_cast<ssize_t>(sizeof(DropSample) * g_drop_idx);
  while (want > 0) {
    ssize_t w = ::write(fd, p, want);
    if (w <= 0) break;
    p += w; want -= w;
  }
  ::close(fd);
}

}  // namespace see3cam::trace
