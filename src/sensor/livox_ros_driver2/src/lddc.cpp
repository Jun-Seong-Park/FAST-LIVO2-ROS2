//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <atomic>
#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

#ifdef BUILDING_ROS2
#include "blackbox/blackbox.hpp"
#endif

namespace livox_ros {

#ifdef BUILDING_ROS2
// LiDAR/IMU publish trace 는 blackbox 가 담당:
//   blackbox::lidar::log(header_stamp_ns)  →  $HOME/FAST-LIVO2-ROS2/trace/log/lidar_pubrecord.bin
//   blackbox::imu::log  (header_stamp_ns)  →  $HOME/FAST-LIVO2-ROS2/trace/log/imu_pubrecord.bin
// record schema = { seq, header_stamp, t_pub_ns } little-endian.
static inline double NsToSec(uint64_t ns) {
  return static_cast<double>(ns) * 1e-9;
}
#endif

/** Lidar Data Distribute Control--------------------------------------------*/
#ifdef BUILDING_ROS1
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      enable_lidar_bag_(lidar_bag),
      enable_imu_bag_(imu_bag) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  cur_node_ = nullptr;
  bag_ = nullptr;
}
#elif defined BUILDING_ROS2
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
#if 0
  bag_ = nullptr;
#endif
  blackbox::lidar::init(blackbox::log_dir() + "/lidar_pubrecord.bin");
  blackbox::imu::init(blackbox::log_dir() + "/imu_pubrecord.bin");
  blackbox::lidar_resource::init(blackbox::log_dir() + "/lidar_resource.bin");
}
#endif

Lddc::~Lddc() {
#ifdef BUILDING_ROS2
  blackbox::lidar::shutdown();
  blackbox::imu::shutdown();
  blackbox::lidar_resource::shutdown();
#endif
  // mmap 공유 타임스탬프 해제 (LIV_handhold inline writer)
  if (shared_stamp_ != nullptr) {
    munmap(shared_stamp_, sizeof(SharedTimestamp));
    shared_stamp_ = nullptr;
  }
  if (shared_fd_ >= 0) {
    close(shared_fd_);
    shared_fd_ = -1;
  }

#ifdef BUILDING_ROS1
  if (global_pub_) {
    delete global_pub_;
  }

  if (global_imu_pub_) {
    delete global_imu_pub_;
  }
#endif

  PrepareExit();

#ifdef BUILDING_ROS1
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i];
    }
  }
#endif
}

// LIV_handhold 방식 inline mmap writer
// (A) 지점: SDK 콜백 → 드라이버 내부 → publish 직후 같은 함수 안에서 mmap 쓰기.
// reader(image_mmap_stamper_node) 가 /home/$USER/timeshare 를 PROT_READ 로 매핑.
void Lddc::TryOpenSharedMmap() {
  if (mmap_opened_) return;
  const char* home = std::getenv("HOME");
  std::string path = std::string(home && *home ? home : "/tmp") + "/timeshare";
  shared_fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0666);
  if (shared_fd_ < 0) {
    std::cerr << "[lddc] open('" << path << "') failed — mmap writer 비활성" << std::endl;
    return;
  }
  if (ftruncate(shared_fd_, sizeof(SharedTimestamp)) != 0) {
    close(shared_fd_); shared_fd_ = -1;
    return;
  }
  void* m = mmap(nullptr, sizeof(SharedTimestamp),
                 PROT_READ | PROT_WRITE, MAP_SHARED, shared_fd_, 0);
  if (m == MAP_FAILED) {
    close(shared_fd_); shared_fd_ = -1;
    return;
  }
  shared_stamp_ = static_cast<SharedTimestamp*>(m);
  shared_stamp_->high = 0;
  shared_stamp_->low = 0;
  mmap_opened_ = true;
}

void Lddc::WriteSharedStamp(uint64_t ns) {
  if (shared_stamp_ != nullptr) {
    // 64-bit aligned single store → aarch64/x86_64 에서 원자적
    shared_stamp_->low = static_cast<int64_t>(ns);
  }
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) return;
  if (lds_->IsRequestExit()) return;

  // 최초 호출 시 mmap 공유 파일 초기화 (LIV_handhold 방식 (A) inline writer)
  TryOpenSharedMmap();

  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) return;
  if (lds_->IsRequestExit()) return;
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, index);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, index);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPclMsg(p_queue, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
#ifdef BUILDING_ROS1
  if (bag_) {
    DRIVER_INFO(*cur_node_, "Waiting to save the bag file!");
    bag_->close();
    DRIVER_INFO(*cur_node_, "Save the bag file successfully!");
    bag_ = nullptr;
  }
#endif
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) continue;

    PointCloud2 cloud;
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);
    PublishPointcloud2Data(index, timestamp, cloud);
    WriteSharedStamp(timestamp);  // mmap: 카메라가 읽을 최신 ns epoch
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) continue;

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
    WriteSharedStamp(livox_msg.timebase);  // mmap: 카메라가 읽을 최신 ns epoch
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) {
#ifdef BUILDING_ROS2
  // pcl::PointCloud type is not supported in ROS2; user must set
  // 'xfer_format' to a supported format in the launch file.
  return;
#endif
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) continue;

    PointCloud cloud;
    uint64_t timestamp = 0;
    InitPclMsg(pkg, cloud, timestamp);
    FillPointsToPclMsg(pkg, cloud);
    PublishPclData(index, timestamp, cloud);
  }
  return;
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2& cloud) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;
  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp) {
  InitPointcloud2MsgHeader(cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);

  cloud.width = pkg.points_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense     = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  #ifdef BUILDING_ROS1
      cloud.header.stamp = ros::Time( timestamp / 1000000000.0);
  #elif defined BUILDING_ROS2
      cloud.header.stamp = rclcpp::Time(timestamp);
  #endif

  std::vector<LivoxPointXyzrtlt> points;
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point;
    point.x = pkg.points[i].x;
    point.y = pkg.points[i].y;
    point.z = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag = pkg.points[i].tag;
    point.line = pkg.points[i].line;
    point.timestamp = static_cast<double>(pkg.points[i].offset_time);
    points.push_back(std::move(point));
  }
  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<PointCloud2>::SharedPtr publisher_ptr =
    std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index));
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
#ifdef BUILDING_ROS2
    blackbox::lidar::log(timestamp);
    static uint64_t pub_count = 0;
    static uint64_t prev_stamp = 0;
    pub_count++;
    if (pub_count == 3) {
      RCLCPP_INFO(cur_node_->get_logger(), "\033[34mLIDAR ... OK\033[0m");
    }
    if (prev_stamp != 0) {
      const double dt = timestamp >= prev_stamp ? NsToSec(timestamp - prev_stamp) : -1.0;
      if (timestamp < prev_stamp || dt > 0.25) {
        RCLCPP_WARN(
            cur_node_->get_logger(),
            "[PUB-GAP] lidar_pc2 pub=%lu stamp=%.9f prev=%.9f dt=%.6f threshold=0.250000",
            static_cast<unsigned long>(pub_count),
            NsToSec(timestamp),
            NsToSec(prev_stamp),
            dt);
      }
    }
    prev_stamp = timestamp;
#endif
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud);
    }
#endif
  }
}

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg.header.frame_id.assign(frame_id_);

#ifdef BUILDING_ROS1
  static uint32_t msg_seq = 0;
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
#endif

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg.timebase = timestamp;

#ifdef BUILDING_ROS1
  livox_msg.header.stamp = ros::Time(timestamp / 1000000000.0);
#elif defined BUILDING_ROS2
  livox_msg.header.stamp = rclcpp::Time(timestamp);
#endif

  livox_msg.point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    livox_msg.lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));
#endif

if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg);
#ifdef BUILDING_ROS2
    blackbox::lidar::log(livox_msg.timebase);
    static uint64_t pub_count = 0;
    static uint64_t prev_stamp = 0;
    pub_count++;
    if (pub_count == 3) {
      RCLCPP_INFO(cur_node_->get_logger(), "\033[34mLIDAR ... OK\033[0m");
    }
    if (prev_stamp != 0) {
      const double dt = livox_msg.timebase >= prev_stamp ? NsToSec(livox_msg.timebase - prev_stamp) : -1.0;
      if (livox_msg.timebase < prev_stamp || dt > 0.25) {
        RCLCPP_WARN(
            cur_node_->get_logger(),
            "[PUB-GAP] lidar_custom pub=%lu stamp=%.9f prev=%.9f dt=%.6f threshold=0.250000",
            static_cast<unsigned long>(pub_count),
            NsToSec(livox_msg.timebase),
            NsToSec(prev_stamp),
            dt);
      }
    }
    prev_stamp = livox_msg.timebase;
#endif
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(livox_msg.timebase / 1000000000.0), livox_msg);
    }
#endif
  }
}

void Lddc::InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp) {
#ifdef BUILDING_ROS1
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = pkg.points_num;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  cloud.header.stamp = timestamp / 1000.0;  // to pcl ros time stamp
#elif defined BUILDING_ROS2
  // pcl::PointCloud not supported in ROS2 path; no-op.
#endif
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg) {
#ifdef BUILDING_ROS1
  if (pkg.points.empty()) {
    return;
  }

  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    pcl::PointXYZI point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.intensity = points[i].intensity;

    pcl_msg.points.push_back(std::move(point));
  }
#elif defined BUILDING_ROS2
  // pcl::PointCloud not supported in ROS2 path; no-op.
#endif
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud);
    }
  }
#elif defined BUILDING_ROS2
  // pcl::PointCloud not supported in ROS2 path; no-op.
#endif
  return;
}

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
  imu_msg.header.frame_id = "livox_frame";

  timestamp = imu_data.time_stamp;
#ifdef BUILDING_ROS1
  imu_msg.header.stamp = ros::Time(timestamp / 1000000000.0);  // to ros time stamp
#elif defined BUILDING_ROS2
  imu_msg.header.stamp = rclcpp::Time(timestamp);  // to ros time stamp
#endif

  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp);

#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = GetCurrentImuPublisher(index);
#elif defined BUILDING_ROS2
  Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg);
#ifdef BUILDING_ROS2
    blackbox::imu::log(timestamp);
    static uint64_t pub_count = 0;
    static uint64_t prev_stamp = 0;
    pub_count++;
    if (pub_count == 3) {
      RCLCPP_INFO(cur_node_->get_logger(), "\033[34mIMU ... OK\033[0m");
    }
    if (prev_stamp != 0) {
      const double dt = timestamp >= prev_stamp ? NsToSec(timestamp - prev_stamp) : -1.0;
      if (timestamp < prev_stamp || dt > 0.05) {
        RCLCPP_WARN(
            cur_node_->get_logger(),
            "[PUB-GAP] imu pub=%lu stamp=%.9f prev=%.9f dt=%.6f threshold=0.050000",
            static_cast<unsigned long>(pub_count),
            NsToSec(timestamp),
            NsToSec(prev_stamp),
            dt);
      }
    }
    prev_stamp = timestamp;
#endif
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_imu_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), imu_msg);
    }
#endif
  }
}

#ifdef BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {
    rclcpp::QoS best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(queue_size)).best_effort();
    if (kPointCloud2Msg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format (BEST_EFFORT)", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud2>(topic_name, best_effort_qos);
    } else if (kLivoxCustomMsg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format (BEST_EFFORT)", topic_name.c_str());
      return cur_node_->create_publisher<CustomMsg>(topic_name, best_effort_qos);
    }
#if 0
    else if (kPclPxyziMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud>(topic_name, queue_size);
    }
#endif
    else if (kLivoxImuMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use imu format (BEST_EFFORT)", topic_name.c_str());
      return cur_node_->create_publisher<ImuMsg>(topic_name, best_effort_qos);
    } else {
      PublisherPtr null_publisher(nullptr);
      return null_publisher;
    }
}
#endif

#ifdef BUILDING_ROS1
PublisherPtr Lddc::GetCurrentPublisher(uint8_t index) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_pub_[index];
    queue_size = queue_size / 8; // queue size is 4 for only one lidar
  } else {
    pub = &global_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      std::string ip_string = IpNumToString(lds_->lidars_[index].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      DRIVER_INFO(*cur_node_, "Support multi topics.");
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->GetNode().advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<livox_ros_driver2::CustomMsg>(name_str,
                                                                queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<PointCloud>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format, set ROS publisher queue "
          "size %d",
          name_str, queue_size);
    }
  }

  return *pub;
}

PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_imu_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      DRIVER_INFO(*cur_node_, "Support multi topics.");
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->GetNode().advertise<sensor_msgs::Imu>(name_str, queue_size);
    DRIVER_INFO(*cur_node_, "%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}
#elif defined BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name,
          queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
}
#endif

void Lddc::CreateBagFile(const std::string &file_name) {
#ifdef BUILDING_ROS1
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str());
  }
#endif
}

}  // namespace livox_ros
