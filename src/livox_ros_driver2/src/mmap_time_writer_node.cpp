/**
 * @file mmap_time_writer_node.cpp
 * @brief LIV_handhold 방식 mmap 타임스탬프 writer (ROS2)
 *
 * `/livox/lidar` (CustomMsg) 를 구독하여 메시지 도착 직후
 * `/home/$USER/timeshare` 파일에 mmap 된 구조체의 `low` 필드에
 * `msg->timebase` (ns, GPS epoch) 를 쓴다.
 *
 * 동일 파일을 reader 가 mmap 으로 읽어 카메라 프레임 stamp 로 사용함으로써
 * 두 센서가 같은 time domain 을 공유한다.
 *
 * 구조체 레이아웃은 LIV_handhold 의
 *   livox_ros_driver2/src/lddc.cpp, mvs_ros_driver/src/grab_trigger.cpp 와
 * 100% 호환 (`int64_t high; int64_t low;`).
 *
 * 모든 경로는 파라미터로 외부화:
 *   - shared_file : mmap 대상 파일 경로 (default: /home/$USER/timeshare)
 *   - input_topic : 구독할 CustomMsg 토픽 (default: /livox/lidar)
 */

#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

struct SharedTimestamp {
  int64_t high;
  int64_t low;
};
static_assert(sizeof(SharedTimestamp) == 16,
              "Layout must match LIV_handhold grab_trigger / lddc.cpp");

std::string default_shared_path()
{
  const char* home = std::getenv("HOME");
  if (home && *home) return std::string(home) + "/timeshare";

  const char* user = std::getenv("USER");
  if (user && *user) return std::string("/home/") + user + "/timeshare";

  return "/tmp/timeshare";
}

} // namespace

class MmapTimeWriter : public rclcpp::Node
{
public:
  MmapTimeWriter()
    : Node("mmap_time_writer")
  {
    declare_parameter("shared_file", default_shared_path());
    declare_parameter("input_topic", std::string("/livox/lidar"));

    shared_file_ = get_parameter("shared_file").as_string();
    input_topic_ = get_parameter("input_topic").as_string();

    if (!open_shared_mmap()) {
      RCLCPP_FATAL(get_logger(),
                   "mmap writer 초기화 실패 — 종료");
      throw std::runtime_error("mmap init failed");
    }

    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MmapTimeWriter::on_custom_msg, this, std::placeholders::_1));
  }

  ~MmapTimeWriter() override
  {
    if (mapped_ != MAP_FAILED && mapped_ != nullptr) {
      munmap(mapped_, sizeof(SharedTimestamp));
    }
    if (fd_ >= 0) close(fd_);
  }

private:
  bool open_shared_mmap()
  {
    // LIV_handhold 원본과 동일 모드: O_CREAT | O_RDWR | O_TRUNC, 0666.
    // O_TRUNC 는 초기화용 — 파일 길이를 구조체 크기만큼 재확보.
    fd_ = open(shared_file_.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open('%s') failed: %s",
                   shared_file_.c_str(), std::strerror(errno));
      return false;
    }

    // 페이지 크기만큼 확장해야 mmap 이 유효. ftruncate 로 정확한 크기 확보.
    if (ftruncate(fd_, sizeof(SharedTimestamp)) != 0) {
      RCLCPP_ERROR(get_logger(), "ftruncate failed: %s", std::strerror(errno));
      return false;
    }

    mapped_ = mmap(nullptr, sizeof(SharedTimestamp),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapped_ == MAP_FAILED) {
      RCLCPP_ERROR(get_logger(), "mmap failed: %s", std::strerror(errno));
      return false;
    }

    // 초기값 0 으로 명시
    auto* slot = static_cast<SharedTimestamp*>(mapped_);
    slot->high = 0;
    slot->low = 0;
    return true;
  }

  void on_custom_msg(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    if (mapped_ == MAP_FAILED || mapped_ == nullptr) return;

    // timebase 는 첫 포인트의 ns epoch (lidar GPS time).
    // 64-bit aligned store → aarch64 / x86_64 에서 atomic.
    auto* slot = static_cast<SharedTimestamp*>(mapped_);
    slot->low = static_cast<int64_t>(msg->timebase);

    ++count_;
  }

  std::string shared_file_;
  std::string input_topic_;
  int fd_ = -1;
  void* mapped_ = MAP_FAILED;
  std::atomic<uint64_t> count_{0};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MmapTimeWriter>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("mmap_time_writer"),
                 "fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
