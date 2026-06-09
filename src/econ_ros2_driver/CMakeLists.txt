cmake_minimum_required(VERSION 3.14)
project(econ_ros2_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(econ_ros2_driver
  src/econ_ros2_driver.cpp
  src/gst_backend.cpp)
ament_target_dependencies(econ_ros2_driver rclcpp sensor_msgs)
target_include_directories(econ_ros2_driver PRIVATE
  include
  ${GST_INCLUDE_DIRS})
target_link_libraries(econ_ros2_driver
  ${GST_LIBRARIES})

add_executable(econ_monitor src/image_viewer.cpp)
ament_target_dependencies(econ_monitor rclcpp sensor_msgs)
target_include_directories(econ_monitor PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(econ_monitor ${OpenCV_LIBS})

add_executable(econ_image_saver src/image_saver.cpp)
ament_target_dependencies(econ_image_saver rclcpp sensor_msgs)
target_include_directories(econ_image_saver PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(econ_image_saver ${OpenCV_LIBS})

install(TARGETS
  econ_ros2_driver
  econ_monitor
  econ_image_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
