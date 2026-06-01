// image_monitor — PC-side verification subscriber for the compressed image stream.
//
// Subscribes to the Jetson's /camera/image/compressed (best_effort, to match the
// driver's publisher QoS) and reports receive rate / bandwidth / frame size once
// per second. This is how we check whether the WUXGA compressed stream actually
// survives the WiFi LAN at the operational trigger rate.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class ImageMonitor : public rclcpp::Node
{
public:
  ImageMonitor()
  : Node("image_monitor")
  {
    const std::string topic =
      declare_parameter<std::string>("topic", "/camera/image/compressed");

    // Must match the publisher (econ_ros2_driver.cpp): KeepLast(5) + best_effort.
    // A reliable subscriber would NOT match a best_effort publisher → no data.
    const rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic, qos,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        ++window_count_;
        window_bytes_ += msg->data.size();
        last_frame_bytes_ = msg->data.size();
      });

    timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { report(); });

    RCLCPP_INFO(get_logger(),
      "image_monitor subscribing to %s (best_effort, KeepLast(5))", topic.c_str());
  }

private:
  void report()
  {
    total_count_ += window_count_;
    const double mbps = static_cast<double>(window_bytes_) * 8.0 / 1.0e6;  // over 1 s window
    const double last_kb = static_cast<double>(last_frame_bytes_) / 1024.0;

    RCLCPP_INFO(get_logger(),
      "fps=%u  rate=%.1f Mbps  last_frame=%.0f KB  total=%llu",
      window_count_, mbps, last_kb,
      static_cast<unsigned long long>(total_count_));

    window_count_ = 0;
    window_bytes_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint32_t window_count_ = 0;
  uint64_t window_bytes_ = 0;
  uint64_t total_count_ = 0;
  std::size_t last_frame_bytes_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageMonitor>());
  rclcpp::shutdown();
  return 0;
}
