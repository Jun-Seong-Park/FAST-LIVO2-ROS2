#include "LIVMapper.h"
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping");
  mapper.initializeSubscribersAndPublishers(nh, it_);
  try {
    mapper.run(nh);
  } catch (const std::exception &e) {
    std::cerr << "[main] run() threw: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "[main] run() threw unknown exception" << std::endl;
  }
  try { mapper.savePCD(); } catch (const std::exception &e) {
    std::cerr << "[main] savePCD() threw: " << e.what() << std::endl;
  } catch (...) {}
  rclcpp::shutdown();
  return 0;
}