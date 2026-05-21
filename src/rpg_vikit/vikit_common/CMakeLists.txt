cmake_minimum_required(VERSION 3.8)
project(vikit_common)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# CPU 아키텍처별 최적화 플래그
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mmmx -msse -msse2 -msse3 -mssse3")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-class-memaccess")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# 의존 패키지
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

if(TARGET Sophus::Sophus AND NOT Sophus_INCLUDE_DIRS)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
  set(Sophus_LIBRARIES Sophus::Sophus)
endif()

# 서드파티 헤더 경로 (디렉토리 스코프 — export 타깃에는 포함되지 않음)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
)

set(SOURCEFILES
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/equidistant_camera.cpp
  src/polynomial_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# vikit_common 라이브러리
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  fmt::fmt
)

# 테스트 실행파일
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME} ${OpenCV_LIBS})

# 설치 — colcon install 트리로 들어간다 (/usr/local, ldconfig 불필요)
install(DIRECTORY include/vikit DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  test_vk_common_camera
  test_vk_common_triangulation
  test_vk_common_patch_score
  DESTINATION lib/${PROJECT_NAME}
)

# 다운스트림(vikit_ros, fast_livo2)이 find_package / ament_target_dependencies 로
# 타깃과 include, 의존성을 그대로 받을 수 있도록 export 한다.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Eigen3 Sophus fmt)

ament_package()
