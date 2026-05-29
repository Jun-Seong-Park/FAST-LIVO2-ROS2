cmake_minimum_required(VERSION 3.14)
project(image_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(blackbox REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# trigger node — element-based GStreamer pipeline (gstream_pipeline.hpp) + pull/single-thread.
# blackbox: src-pad probe (push) → log_cbk → log_pub cascade.
add_executable(see3cam24cug_trig src/see3cam24cug_trig.cpp)
ament_target_dependencies(see3cam24cug_trig rclcpp sensor_msgs)
target_include_directories(see3cam24cug_trig PRIVATE
  include
  ${GST_INCLUDE_DIRS})
target_link_libraries(see3cam24cug_trig ${GST_LIBRARIES} blackbox::blackbox)

install(TARGETS
  see3cam24cug_trig
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
