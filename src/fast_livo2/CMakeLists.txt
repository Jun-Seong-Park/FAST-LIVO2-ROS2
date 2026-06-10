cmake_minimum_required(VERSION 3.12)
project(fast_livo2)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find ament and required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
# vikit_common is a plain-cmake library (installed in /usr/local). Its Config provides
# only the includes and leaves the library path empty, so we find the .so directly and link it into the executable.
find_library(VIKIT_COMMON_LIBRARY vikit_common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

if(TARGET Sophus::Sophus AND NOT Sophus_INCLUDE_DIRS)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
  set(Sophus_LIBRARIES Sophus::Sophus)
endif()

find_package(Boost REQUIRED COMPONENTS thread)
find_package(fmt REQUIRED)

# Include directories for dependencies.
# The vikit_common/vikit_ros includes are handled by ament_target_dependencies.
# Only the vikit_common library link is handled separately via VIKIT_COMMON_LIBRARY.
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  vikit_common
  vikit_ros
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
)

set(COMMON_DEPENDENCIES OpenMP::OpenMP_CXX fmt::fmt)

# link_directories(${COMMON_DEPENDENCIES}
#   ${vikit_common_LIBRARIES}/libvikit_common.so
#   ${vikit_ros_LIBRARIES}/libvikit_ros.so
# )

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
add_library(utils src/utils.cpp)

ament_target_dependencies(vio ${dependencies} )
ament_target_dependencies(lio ${dependencies})
ament_target_dependencies(pre ${dependencies})
ament_target_dependencies(imu_proc ${dependencies})
ament_target_dependencies(laser_mapping ${dependencies})

target_link_libraries(laser_mapping ${COMMON_DEPENDENCIES})

target_link_libraries(vio ${COMMON_DEPENDENCIES})
target_link_libraries(lio utils ${COMMON_DEPENDENCIES})
target_link_libraries(pre ${COMMON_DEPENDENCIES})
target_link_libraries(imu_proc ${COMMON_DEPENDENCIES})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)
ament_target_dependencies(fastlivo_mapping ${dependencies})


# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${VIKIT_COMMON_LIBRARY}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Monitoring relay node (loopback -> external republisher)
add_executable(monitor src/monitor.cpp)
ament_target_dependencies(monitor rclcpp)

# Host resource logger (1 Hz /proc·/sys polling → blackbox::resource, 4 files)
add_executable(resource_logger src/resource_logger_node.cpp)
ament_target_dependencies(resource_logger rclcpp)

# Install the executable
install(TARGETS
  fastlivo_mapping
  monitor
  resource_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg scripts
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
  OpenCV 
  Sophus
)

ament_package()
