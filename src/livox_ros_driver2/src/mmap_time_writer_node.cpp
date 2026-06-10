/**
 * @file mmap_time_writer_node.cpp
 * @brief LIV_handhold-style mmap timestamp writer (ROS2)
 *
 * Subscribes to `/livox/lidar` (CustomMsg) and, immediately upon message arrival,
 * writes `msg->timebase` (ns, GPS epoch) into the `low` field of the struct
 * mmap'd onto the `/home/$USER/timeshare` file.
 *
 * A reader mmaps the same file and uses it as the camera frame stamp, so that
 * the two sensors share the same time domain.
 *
 * The struct layout is 100% compatible with LIV_handhold's
 *   livox_ros_driver2/src/lddc.cpp and mvs_ros_driver/src/grab_trigger.cpp
 * (`int64_t high; int64_t low;`).
 *
 * All paths are externalized as parameters:
 *   - shared_file : mmap target file path (default: /home/$USER/timeshare)
 *   - input_topic : CustomMsg topic to subscribe to (default: /livox/lidar)
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
                   "mmap writer initialization failed — exiting");
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
    // Same mode as the LIV_handhold original: O_CREAT | O_RDWR | O_TRUNC, 0666.
    // O_TRUNC is for initialization — resize the file length to the struct size.
    fd_ = open(shared_file_.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open('%s') failed: %s",
                   shared_file_.c_str(), std::strerror(errno));
      return false;
    }

    // Must extend to the page size for mmap to be valid. ftruncate secures the exact size.
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

    // Explicitly initialize to 0
    auto* slot = static_cast<SharedTimestamp*>(mapped_);
    slot->high = 0;
    slot->low = 0;
    return true;
  }

  void on_custom_msg(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    if (mapped_ == MAP_FAILED || mapped_ == nullptr) return;

    // timebase is the ns epoch of the first point (lidar GPS time).
    // 64-bit aligned store → atomic on aarch64 / x86_64.
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
