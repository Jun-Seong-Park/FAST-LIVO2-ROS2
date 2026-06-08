// image_viewer — PC-side / local live viewer for the econ image stream.
//
// Subscribes (best_effort, to match the driver's publisher QoS) to either:
//   encoding=raw        → sensor_msgs/Image (bgr8), shown directly
//   encoding=compressed → sensor_msgs/CompressedImage (JPEG), imdecode'd
// and shows each frame in a window. Also logs the receive rate once per second
// so you see both the picture and whether the LAN is keeping up.

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer()
  : Node("image_viewer")
  {
    const std::string encoding =
      declare_parameter<std::string>("encoding", "raw");

    // Default topic follows the encoding so the user only has to set one param.
    const std::string default_topic =
      (encoding == "compressed") ? "/camera/image/compressed" : "/camera/image";
    const std::string topic =
      declare_parameter<std::string>("topic", default_topic);

    window_ = declare_parameter<std::string>("window", "econ image");

    // Must match the publisher (econ_ros2_driver.cpp): KeepLast(5) + best_effort.
    const rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    if (encoding == "raw") {
      raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { on_raw(msg); });
    } else if (encoding == "compressed") {
      compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, qos,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) { on_compressed(msg); });
    } else {
      RCLCPP_FATAL(get_logger(),
        "encoding must be 'raw' or 'compressed', got '%s'", encoding.c_str());
      throw std::runtime_error("invalid encoding parameter");
    }

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

    // KEEPRATIO 로 종횡비(16:9) 고정, 시작 크기는 원본 1280x720 의 절반으로 박는다.
    cv::namedWindow(window_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(window_, 960, 540);
    RCLCPP_INFO(get_logger(),
      "image_viewer subscribing to %s (encoding=%s, best_effort, KeepLast(5))",
      topic.c_str(), encoding.c_str());
  }

private:
  // raw bgr8 path: wrap the message buffer as a cv::Mat without copying.
  void on_raw(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++window_count_;
    window_bytes_ += msg->data.size();

    if (msg->encoding != "bgr8") {
      RCLCPP_WARN(get_logger(),
        "raw viewer expects bgr8, got '%s' — showing as-is", msg->encoding.c_str());
    }

    const int rows = static_cast<int>(msg->height);
    const int cols = static_cast<int>(msg->width);
    const size_t step = static_cast<size_t>(msg->step);

    // Zero-copy header over the bgr8 buffer (no cv_bridge needed).
    const cv::Mat img(rows, cols, CV_8UC3, msg->data.data(), step);
    if (img.empty()) {
      RCLCPP_WARN(get_logger(), "empty raw frame (%ux%u)", msg->width, msg->height);
      return;
    }
    cv::imshow(window_, img);
    cv::waitKey(1);  // pump the GUI event loop
  }

  // compressed (JPEG) path: decode to BGR with OpenCV.
  void on_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
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
