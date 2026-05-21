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
find_package(std_srvs REQUIRED)
find_package(blackbox REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

function(add_see3cam24cug_trig variant)
  set(exe see3cam24cug_trig_${variant})
  add_executable(${exe} src/${exe}.cpp)
  ament_target_dependencies(${exe} rclcpp sensor_msgs std_srvs)
  target_include_directories(${exe} PRIVATE
    include
    ${GST_INCLUDE_DIRS})
  target_link_libraries(${exe} ${GST_LIBRARIES} blackbox::blackbox)
endfunction()

add_see3cam24cug_trig(sd)
add_see3cam24cug_trig(hd)
add_see3cam24cug_trig(wuxga)
add_see3cam24cug_trig(wuxga_mjpg)

install(TARGETS
  see3cam24cug_trig_sd
  see3cam24cug_trig_hd
  see3cam24cug_trig_wuxga
  see3cam24cug_trig_wuxga_mjpg
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
