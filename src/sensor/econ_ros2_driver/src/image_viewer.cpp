// image_viewer — PC-side live viewer for the compressed image stream.
//
// Subscribes to /camera/image/compressed (best_effort, to match the driver's
// publisher QoS), JPEG-decodes each frame with OpenCV and shows it in a window.
// Also logs the receive rate once per second so you see both the picture and
// whether the WiFi LAN is keeping up.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer()
  : Node("image_viewer")
  {
    const std::string topic =
      declare_parameter<std::string>("topic", "/camera/image/compressed");
    window_ = declare_parameter<std::string>("window", "econ image");

    // Must match the publisher (econ_ros2_driver.cpp): KeepLast(5) + best_effort.
    const rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic, qos,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { on_image(msg); });

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

    cv::namedWindow(window_, cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(),
      "image_viewer subscribing to %s (best_effort, KeepLast(5))", topic.c_str());
  }

private:
  void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    ++window_count_;
    window_bytes_ += msg->data.size();

    // Zero-copy header over the JPEG bytes, then decode to BGR.
    const cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1, msg->data.data());
    const cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (img.empty()) {
      RCLCPP_WARN(get_logger(), "JPEG decode failed (%zu bytes)", msg->data.size());
      return;
    }
    cv::imshow(window_, img);
    cv::waitKey(1);  // pump the GUI event loop
  }

  void report()
  {
    const double mbps = static_cast<double>(window_bytes_) * 8.0 / 1.0e6;
    RCLCPP_INFO(get_logger(), "fps=%u  rate=%.1f Mbps", window_count_, mbps);
    window_count_ = 0;
    window_bytes_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string window_;
  uint32_t window_count_ = 0;
  uint64_t window_bytes_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewer>());
  rclcpp::shutdown();
  return 0;
}
