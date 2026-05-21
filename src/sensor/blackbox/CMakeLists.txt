cmake_minimum_required(VERSION 3.10)
project(blackbox)

find_package(ament_cmake REQUIRED)

# header-only INTERFACE library — GStreamer link 는 호출자가 책임
# (호출자 코드는 어차피 GStreamer 직접 사용하니 별도 link 함)
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
