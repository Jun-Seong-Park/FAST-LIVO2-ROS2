#pragma once
//
// Subscriber-side callback traces (FAST-LIVO2).
//
// 핫패스에서 ROS 로거·stdout 을 거치지 않고 바이너리 샘플을 메모리 배열에 직접 기록.
// 실행 중에는 1초마다 새 샘플만 append flush 하고, 종료 시 한 번 더 flush.
// 포맷: numpy dtype=[('stamp','u8'),('mono','u8'),('cb_index','u8')] 과 동일 (packed).
//
// 짝: livox_ros_driver2_sync/src/lddc.cpp 의 PubSample / RecordPubImu.
//   - publisher 측  : ~/trace/log/pub_trace_{lidar,imu}.bin  (stamp, mono)         16B
//   - subscriber 측 : ~/trace/log/sub_trace_{lidar,imu,image}.bin (stamp, mono, cb_idx) 24B
//
// drop 검출 = (publisher trace 갯수) - (subscriber trace 갯수).
// jump 검출 = subscriber stamp_dt 가 5 ms 의 정수배에서 벗어나는 지점.

#include <cstdint>
#include <cstddef>
#include <climits>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

namespace fastlivo_trace {

struct __attribute__((packed)) SubSample {
  uint64_t stamp_ns;   // msg_in->header.stamp 를 ns 로 (변형 전 원본)
  uint64_t mono_ns;    // CLOCK_MONOTONIC_RAW (callback 진입 시각)
  uint64_t cb_index;   // callback 진입 카운터 (0,1,2,...)
};

static constexpr size_t kSubBufN = 1 << 20;  // 24MB, IMU 200Hz 기준 약 87 분
static SubSample g_buf_sub_lidar[kSubBufN];
static SubSample g_buf_sub_imu[kSubBufN];
static SubSample g_buf_sub_image[kSubBufN];
static size_t    g_idx_sub_lidar = 0;
static size_t    g_idx_sub_imu = 0;
static size_t    g_idx_sub_image = 0;
static size_t    g_flushed_sub_lidar = 0;
static size_t    g_flushed_sub_imu = 0;
static size_t    g_flushed_sub_image = 0;
static bool      g_trace_files_initialized = false;
static uint64_t  g_last_flush_mono_ns = 0;

static inline uint64_t MonoRawNs() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
       + static_cast<uint64_t>(ts.tv_nsec);
}

static inline const char* TraceLogDir() {
  return "/home/aluxorin22/trace/log";
}

static inline const char* SubLidarPath() {
  return "/home/aluxorin22/trace/log/sub_trace_lidar.bin";
}

static inline const char* SubImuPath() {
  return "/home/aluxorin22/trace/log/sub_trace_imu.bin";
}

static inline const char* SubImagePath() {
  return "/home/aluxorin22/trace/log/sub_trace_image.bin";
}

static inline void EnsureTraceLogDir() {
  ::mkdir("/home/aluxorin22/trace", 0755);
  ::mkdir(TraceLogDir(), 0755);
}

static inline void InitTraceFilesOnce() {
  if (g_trace_files_initialized) return;
  EnsureTraceLogDir();
  const char* paths[] = {
    SubLidarPath(),
    SubImuPath(),
    SubImagePath(),
  };
  for (const char* path : paths) {
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) close(fd);
  }
  g_trace_files_initialized = true;
}

static inline void AppendSamples(const char* path, const SubSample* buf, size_t begin, size_t end) {
  if (end <= begin) return;
  EnsureTraceLogDir();
  int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) return;
  const char* p = reinterpret_cast<const char*>(buf + begin);
  size_t bytes_left = sizeof(SubSample) * (end - begin);
  while (bytes_left > 0) {
    size_t chunk = bytes_left;
    if (chunk > static_cast<size_t>(SSIZE_MAX)) chunk = static_cast<size_t>(SSIZE_MAX);
    ssize_t w = ::write(fd, p, chunk);
    if (w <= 0) break;
    p += w;
    bytes_left -= static_cast<size_t>(w);
  }
  close(fd);
}

static inline void FlushSubTraces() {
  InitTraceFilesOnce();
  AppendSamples(SubLidarPath(), g_buf_sub_lidar, g_flushed_sub_lidar, g_idx_sub_lidar);
  AppendSamples(SubImuPath(), g_buf_sub_imu, g_flushed_sub_imu, g_idx_sub_imu);
  AppendSamples(SubImagePath(), g_buf_sub_image, g_flushed_sub_image, g_idx_sub_image);
  g_flushed_sub_lidar = g_idx_sub_lidar;
  g_flushed_sub_imu = g_idx_sub_imu;
  g_flushed_sub_image = g_idx_sub_image;
  g_last_flush_mono_ns = MonoRawNs();
}

static inline void MaybeFlushSubTraces(uint64_t mono_ns) {
  InitTraceFilesOnce();
  if (g_last_flush_mono_ns == 0 || mono_ns - g_last_flush_mono_ns >= 1000000000ULL) {
    FlushSubTraces();
  }
}

static inline void RecordSubLidar(uint64_t stamp_ns) {
  if (g_idx_sub_lidar < kSubBufN) {
    const uint64_t mono_ns = MonoRawNs();
    g_buf_sub_lidar[g_idx_sub_lidar] = {stamp_ns, mono_ns, g_idx_sub_lidar};
    g_idx_sub_lidar++;
    MaybeFlushSubTraces(mono_ns);
  }
}

static inline void RecordSubImu(uint64_t stamp_ns) {
  if (g_idx_sub_imu < kSubBufN) {
    const uint64_t mono_ns = MonoRawNs();
    g_buf_sub_imu[g_idx_sub_imu] = {stamp_ns, mono_ns, g_idx_sub_imu};
    g_idx_sub_imu++;
    MaybeFlushSubTraces(mono_ns);
  }
}

static inline void RecordSubImage(uint64_t stamp_ns) {
  if (g_idx_sub_image < kSubBufN) {
    const uint64_t mono_ns = MonoRawNs();
    g_buf_sub_image[g_idx_sub_image] = {stamp_ns, mono_ns, g_idx_sub_image};
    g_idx_sub_image++;
    MaybeFlushSubTraces(mono_ns);
  }
}

static inline void DumpSubTraces() {
  FlushSubTraces();
}

}  // namespace fastlivo_trace
