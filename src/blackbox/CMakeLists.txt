cmake_minimum_required(VERSION 3.10)
project(blackbox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ─────────────────────────────────────────────────────────────
# header-only INTERFACE library — caller includes <blackbox/blackbox.hpp>
# ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Inject package source path so log_dir()/plots_dir() resolve to src/blackbox/{log,plots}.
# Same pattern as fast_livo2's ROOT_DIR. Trailing slash matches usage in header.
target_compile_definitions(${PROJECT_NAME} INTERFACE
  BLACKBOX_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ─────────────────────────────────────────────────────────────
# resource_logger executable — 1 Hz host resource polling node
# ─────────────────────────────────────────────────────────────
add_executable(resource_logger src/resource_logger_node.cpp)
target_link_libraries(resource_logger ${PROJECT_NAME})
ament_target_dependencies(resource_logger rclcpp)

install(TARGETS resource_logger DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
