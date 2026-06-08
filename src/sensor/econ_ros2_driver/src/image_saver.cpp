// image_saver — periodic frame dumper for verifying the raw bgr8 image stream.
//
// Subscribes to /camera/image (best_effort, matching the driver's publisher QoS),
// keeps the latest frame, and writes one PNG per second into a folder on the Desktop.
// Lets you verify colour / rotation / resolution from static snapshots — no live
// viewer, no remote desktop, loopback only.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSaver : public rclcpp::Node
{
public:
  ImageSaver()
  : Node("image_saver")
  {
    topic_    = declare_parameter<std::string>("topic", "/camera/image");
    save_dir_ = declare_parameter<std::string>("save_dir", default_save_dir());
    const int period_sec = declare_parameter<int>("period_sec", 1);

    std::filesystem::create_directories(save_dir_);

    // Must match the publisher (econ_ros2_driver.cpp): KeepLast(5) + best_effort.
    const rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic_, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { latest_ = msg; });

    timer_ = create_wall_timer(
      std::chrono::seconds(period_sec), [this]() { save_latest(); });

    RCLCPP_INFO(get_logger(),
      "image_saver: %s (best_effort) -> %s, one PNG every %d s",
      topic_.c_str(), save_dir_.c_str(), period_sec);
  }

private:
  static std::string default_save_dir()
  {
    const char* home = std::getenv("HOME");
    const std::string base = (home && *home) ? home : "/tmp";
    return base + "/FAST-LIVO2-ROS2/src/sensor/econ_ros2_driver/Log";
  }

  void save_latest()
  {
    const sensor_msgs::msg::Image::SharedPtr msg = latest_;
    if (!msg) {
      RCLCPP_WARN(get_logger(), "no frame received yet (QoS mismatch or no publisher?)");
      return;
    }
    if (msg->encoding != "bgr8") {
      RCLCPP_WARN(get_logger(), "expected bgr8, got '%s' (this saver is raw-only)",
        msg->encoding.c_str());
      return;
    }

    // Wrap the message buffer as a BGR Mat (OpenCV is BGR-native -> colour as-is).
    const cv::Mat img(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<uint8_t*>(msg->data.data()), msg->step);

    const std::string path = save_dir_ + "/frame_" + next_index() + ".png";
    if (cv::imwrite(path, img)) {
      RCLCPP_INFO(get_logger(), "saved %s (%ux%u)", path.c_str(), msg->width, msg->height);
    } else {
      RCLCPP_WARN(get_logger(), "imwrite failed: %s", path.c_str());
    }
  }

  std::string next_index()
  {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%05u", ++count_);
    return std::string(buf);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic_;
  std::string save_dir_;
  sensor_msgs::msg::Image::SharedPtr latest_;
  uint32_t count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSaver>());
  rclcpp::shutdown();
  return 0;
}
