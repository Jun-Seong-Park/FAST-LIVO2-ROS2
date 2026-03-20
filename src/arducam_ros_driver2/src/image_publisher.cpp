/**
 * @file image_publish_node.cpp
 * @brief ROS2 node for GStreamer NV12 camera publishing.
 *        Camera hardware params from arducam.yaml, runtime params from cam.yaml.
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/core/mat.hpp"
#include "gst.h"

using namespace std::chrono_literals;

class ImagePublishNode : public rclcpp::Node
{
public:
  ImagePublishNode() : Node("image_publish_node")
  {
    // camera_arducam.yaml params
    this->declare_parameter<std::string>("cam_port", "/dev/video2");
    this->declare_parameter<int>("cam_width", 2592);
    this->declare_parameter<int>("cam_height", 1944);
    this->declare_parameter<std::string>("cam_format", "YUY2");
    this->declare_parameter<int>("out_width", 1280);
    this->declare_parameter<int>("out_height", 720);


    std::string cam_port = this->get_parameter("cam_port").as_string();
    int cam_w  = this->get_parameter("cam_width").as_int();
    int cam_h  = this->get_parameter("cam_height").as_int();
    std::string cam_fmt = this->get_parameter("cam_format").as_string();
    out_w_ = this->get_parameter("out_width").as_int();
    out_h_ = this->get_parameter("out_height").as_int();
    double base_hz = this->get_parameter("base_hz").as_double();

    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/left_camera/image", sensor_qos);

    std::string pipeline =
        "v4l2src device=" + cam_port + " ! "
        "video/x-raw, format=" + cam_fmt +
        ", width=" + std::to_string(cam_w) +
        ", height=" + std::to_string(cam_h) +
        ", framerate=" + std::to_string(static_cast<int>(base_hz)) + "/1 ! "
        "videoscale method=0 ! "
        "video/x-raw, width=" + std::to_string(out_w_) +
        ", height=" + std::to_string(out_h_) + " ! "
        "videoconvert ! "
        "video/x-raw, format=NV12 ! "
        "appsink drop=true max-buffers=1";

    RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline.c_str());
    cap_.open(pipeline, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(),
          "GStreamer pipeline open failed. Check: gst-inspect-1.0 videoscale");
      return;
    }

    timer_ = this->create_wall_timer(1ms, std::bind(&ImagePublishNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "NV12 %dx%d at %.0fHz", out_w_, out_h_, base_hz);
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) return;
    if (frame.empty()) return;

    auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>();
    msg_ptr->header.stamp = this->now();
    msg_ptr->header.frame_id = "camera_init";
    msg_ptr->height = out_h_;
    msg_ptr->width = out_w_;
    msg_ptr->encoding = "nv12";
    msg_ptr->is_bigendian = false;
    msg_ptr->step = out_w_;

    size_t size = frame.total() * frame.elemSize();
    msg_ptr->data.assign(frame.data, frame.data + size);

    image_pub_->publish(std::move(msg_ptr));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  int out_w_;
  int out_h_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublishNode>());
  rclcpp::shutdown();
  return 0;
}
