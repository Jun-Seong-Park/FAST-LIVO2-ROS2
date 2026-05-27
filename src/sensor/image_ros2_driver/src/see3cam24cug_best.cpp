#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "image_ros_driver/config.hpp"
#include "image_ros_driver/hid_control.hpp"
#include "image_ros_driver/mmap_stamper.hpp"
#include "blackbox/blackbox.hpp"

namespace cfg = see3cam;
namespace hid = see3cam::hid;

class See3cam24cugTest : public rclcpp::Node
{
 public:
  See3cam24cugTest()
    : Node("see3cam24cug_test"),
      stamper_(resolve_shared_path()),
      hid_fd_(-1),
      pipeline_(nullptr),
      sink_(nullptr),
      stop_(false)
  {
    // 블랙박스 초기화
    blackbox::image::init(blackbox::log_dir() + "/see3cam24cug_sd_image_pub.bin");
    
    // 발행자 초기화
    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/image", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile());
    
    // GStreamer 초기화
    gst_init(nullptr, nullptr);
   
    if (!hid::open_and_init_trigger(cfg::kHidDevice, cfg::kExposureUs,
                                    cfg::kAflMode, hid_fd_, get_logger())) return;
    if (!init_pipeline()) return;

    pipeline_thread_ = std::thread(&See3cam24cugTest::pipeline_loop, this);
  }

  ~See3cam24cugTest() override {
    shutdown();
    blackbox::image::shutdown();
  }

 private:
  static std::string resolve_shared_path() {
    const char* home = std::getenv("HOME");
    return std::string(home && *home ? home : "/tmp") + "/timeshare";
  }
};
