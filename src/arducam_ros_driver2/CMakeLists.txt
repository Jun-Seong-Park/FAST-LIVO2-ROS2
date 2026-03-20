cmake_minimum_required(VERSION 3.17)
project(image)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS videoio core)
find_package(sensor_msgs REQUIRED)

add_executable(image_publish src/image_publish_node.cpp)
ament_target_dependencies(image_publish rclcpp sensor_msgs)
target_link_libraries(image_publish ${OpenCV_LIBRARIES})

install(TARGETS image_publish DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
