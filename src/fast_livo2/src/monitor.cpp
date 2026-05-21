#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"

class FastLivoMonitor : public rclcpp::Node
{
public:
  FastLivoMonitor() : Node("fast_livo_monitor")
  {
    const std::string robot_name =
      declare_parameter<std::string>("robot_name", "robot");
    const auto relay_list =
      declare_parameter<std::vector<std::string>>("relay_list", std::vector<std::string>{});

    if (relay_list.empty()) {
      RCLCPP_FATAL(get_logger(), "relay_list is empty");
      throw std::runtime_error("relay_list is empty");
    }

    const auto qos = rclcpp::SensorDataQoS();

    for (const auto & entry : relay_list) {
      const auto sep = entry.find(':');
      if (sep == std::string::npos) {
        RCLCPP_FATAL(get_logger(), "invalid relay_list entry (missing ':'): %s", entry.c_str());
        throw std::runtime_error("invalid relay_list entry: " + entry);
      }
      const std::string in_topic  = entry.substr(0, sep);
      const std::string type_name = entry.substr(sep + 1);
      const std::string out_topic = "/" + robot_name + in_topic;

      auto pub = create_generic_publisher(out_topic, type_name, qos);
      auto sub = create_generic_subscription(
        in_topic, type_name, qos,
        [pub](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          pub->publish(*msg);
        });

      pubs_.push_back(pub);
      subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "relay: %s -> %s  [%s]",
        in_topic.c_str(), out_topic.c_str(), type_name.c_str());
    }

    RCLCPP_INFO(get_logger(), "FastLivoMonitor started: %zu relays", subs_.size());
  }

private:
  std::vector<rclcpp::GenericPublisher::SharedPtr>    pubs_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLivoMonitor>());
  rclcpp::shutdown();
  return 0;
}
