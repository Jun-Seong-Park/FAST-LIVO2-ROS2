// V4l2OpencvBackend implementation — see opencv_backend.hpp for the overview.

#include "opencv_backend.hpp"

#include <rclcpp/logging.hpp>

#include <opencv2/imgproc.hpp>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstring>

#include "blackbox.hpp"

namespace see3cam {

V4l2OpencvBackend::~V4l2OpencvBackend() { stop(); }

bool V4l2OpencvBackend::start(const Params& p, ImageFormat format, rclcpp::Logger logger) {
  logger_      = logger;
  format_      = format;
  res_         = p.res;
  need_resize_ = (res_.output_width  != res_.capture_width ||
                  res_.output_height != res_.capture_height);

  // opencv 경로엔 회전 단계가 없다. flip_method 가 켜지면 node 는 pub(swap) 치수로 발행하지만
  // 버퍼는 회전 안 된 채라 발행 이미지가 깨진다 → 차단(회전이 필요하면 gstreamer backend).
  if (p.flip_method != 0) {
    RCLCPP_FATAL(logger_,
      "\033[31m[v4l2] flip_method=%d unsupported on opencv backend — use backend:=gstreamer\033[0m",
      p.flip_method);
    return false;
  }

  fd_ = ::open(p.device.c_str(), O_RDWR);
  if (fd_ < 0) {
    RCLCPP_FATAL(logger_, "\033[31m[v4l2] cannot open %s: %s\033[0m",
                 p.device.c_str(), strerror(errno));
    return false;
  }

  if (!set_format())      return false;
  if (!request_buffers()) return false;

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_FATAL(logger_, "\033[31m[v4l2] STREAMON failed: %s\033[0m", strerror(errno));
    return false;
  }
  return true;
}

Frame V4l2OpencvBackend::grab(int timeout_ms) {
  fd_set fds; FD_ZERO(&fds); FD_SET(fd_, &fds);
  timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return {};  // timeout / error

  v4l2_buffer buf{};
  buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
    RCLCPP_WARN(logger_, "\033[33m[v4l2] DQBUF failed: %s\033[0m", strerror(errno));
    return {};
  }
  t_capture_ns_ = blackbox::mono_raw_ns();
  n_push_.store(static_cast<uint64_t>(buf.sequence) + 1, std::memory_order_relaxed);
  current_index_ = static_cast<int>(buf.index);

  const uint8_t* src = static_cast<const uint8_t*>(bufs_[buf.index].start);

  if (format_ == ImageFormat::kJpeg) {
    // MJPG bytes as-is — valid until release() requeues this buffer.
    return Frame{src, buf.bytesused};
  }

  // raw UYVY → BGR, then optional downscale into bgr_ (copied out, V4L2 buffer free to requeue)
  cv::Mat uyvy(res_.capture_height, res_.capture_width, CV_8UC2,
               const_cast<uint8_t*>(src));
  if (need_resize_) {
    cv::cvtColor(uyvy, conv_, cv::COLOR_YUV2BGR_UYVY);
    cv::resize(conv_, bgr_, cv::Size(res_.output_width, res_.output_height));
  } else {
    cv::cvtColor(uyvy, bgr_, cv::COLOR_YUV2BGR_UYVY);
  }
  return Frame{bgr_.data, bgr_.total() * bgr_.elemSize()};
}

void V4l2OpencvBackend::release() {
  if (current_index_ < 0) return;
  v4l2_buffer buf{};
  buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index  = static_cast<uint32_t>(current_index_);
  if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
    RCLCPP_WARN(logger_, "\033[33m[v4l2] QBUF requeue failed: %s\033[0m", strerror(errno));
  }
  current_index_ = -1;
}

void V4l2OpencvBackend::stop() {
  if (fd_ < 0) return;
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd_, VIDIOC_STREAMOFF, &type);
  for (int i = 0; i < n_bufs_; ++i) {
    if (bufs_[i].start && bufs_[i].start != MAP_FAILED)
      munmap(bufs_[i].start, bufs_[i].length);
    bufs_[i].start  = nullptr;
    bufs_[i].length = 0;
  }
  n_bufs_        = 0;
  current_index_ = -1;
  ::close(fd_);
  fd_ = -1;
}

uint64_t V4l2OpencvBackend::push_count()   const { return n_push_.load(std::memory_order_relaxed); }
uint64_t V4l2OpencvBackend::t_capture_ns() const { return t_capture_ns_; }

// S_FMT at capture WxH with UYVY (raw) or MJPEG (compressed).
bool V4l2OpencvBackend::set_format() {
  v4l2_format fmt{};
  fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = static_cast<uint32_t>(res_.capture_width);
  fmt.fmt.pix.height      = static_cast<uint32_t>(res_.capture_height);
  fmt.fmt.pix.pixelformat = (format_ == ImageFormat::kJpeg) ? V4L2_PIX_FMT_MJPEG
                                                            : V4L2_PIX_FMT_UYVY;
  fmt.fmt.pix.field       = V4L2_FIELD_NONE;
  if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_FATAL(logger_, "\033[31m[v4l2] S_FMT failed: %s\033[0m", strerror(errno));
    return false;
  }
  return true;
}

// REQBUFS + QUERYBUF/mmap/QBUF for kBufCount MMAP buffers.
bool V4l2OpencvBackend::request_buffers() {
  v4l2_requestbuffers req{};
  req.count  = kBufCount;
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    RCLCPP_FATAL(logger_, "\033[31m[v4l2] REQBUFS failed: %s\033[0m", strerror(errno));
    return false;
  }
  n_bufs_ = static_cast<int>(req.count);

  for (int i = 0; i < n_bufs_; ++i) {
    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = static_cast<uint32_t>(i);
    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      RCLCPP_FATAL(logger_, "\033[31m[v4l2] QUERYBUF failed: %s\033[0m", strerror(errno));
      return false;
    }
    bufs_[i].length = buf.length;
    bufs_[i].start  = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                           fd_, buf.m.offset);
    if (bufs_[i].start == MAP_FAILED) {
      RCLCPP_FATAL(logger_, "\033[31m[v4l2] mmap failed: %s\033[0m", strerror(errno));
      return false;
    }
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_FATAL(logger_, "\033[31m[v4l2] initial QBUF failed: %s\033[0m", strerror(errno));
      return false;
    }
  }
  return true;
}

}  // namespace see3cam
