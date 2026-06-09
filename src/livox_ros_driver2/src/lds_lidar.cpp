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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

#include "blackbox/blackbox.hpp"

using namespace std;

namespace livox_ros {

/**
 * MID-360 push msg (cmd_id 0x0102, 1 Hz) → blackbox::lidar_resource
 *   Wire-verified 2026-05-26: 4 keys (core_temp / lidar_diag_status /
 *   cur_work_state / hms_code) always co-bundled per packet (30/30).
 *   SDK delivers parsed JSON string in `info`. We extract with strstr+strtol
 *   (zero-dependency, pattern from Livox-SDK2/samples/livox_lidar_temp_logger).
 */
namespace
{

// Locate the value start (first char after the colon following `"key"`)
// inside `json`. Returns nullptr if the key is absent.
const char * FindValueStart(const char * json, const char * quoted_key)
{
  const char * k = std::strstr(json, quoted_key);
  if (k == nullptr) {
    return nullptr;
  }
  const char * colon = std::strchr(k, ':');
  if (colon == nullptr) {
    return nullptr;
  }
  return colon + 1;
}

bool ExtractInt32(const char * json, const char * quoted_key, int32_t & out)
{
  const char * v = FindValueStart(json, quoted_key);
  if (v == nullptr) {
    return false;
  }
  char * end = nullptr;
  long parsed = std::strtol(v, &end, 10);
  if (end == v) {
    return false;
  }
  out = static_cast<int32_t>(parsed);
  return true;
}

bool ExtractUint32(const char * json, const char * quoted_key, uint32_t & out)
{
  const char * v = FindValueStart(json, quoted_key);
  if (v == nullptr) {
    return false;
  }
  char * end = nullptr;
  unsigned long parsed = std::strtoul(v, &end, 10);
  if (end == v) {
    return false;
  }
  out = static_cast<uint32_t>(parsed);
  return true;
}

// hms_code is a fixed 8-element array: `"hms_code":[v0,v1,...,v7]`.
// SDK writes all 8 unconditionally (parse_lidar_state_info.cpp:694-706).
bool ExtractHmsCode8(const char * json, uint32_t out[8])
{
  const char * v = FindValueStart(json, "\"hms_code\"");
  if (v == nullptr) {
    return false;
  }
  const char * lb = std::strchr(v, '[');
  if (lb == nullptr) {
    return false;
  }
  const char * p = lb + 1;
  for (int i = 0; i < 8; ++i) {
    char * end = nullptr;
    unsigned long parsed = std::strtoul(p, &end, 10);
    if (end == p) {
      return false;
    }
    out[i] = static_cast<uint32_t>(parsed);
    p = end;
    while (*p == ' ' || *p == ',') {
      ++p;
    }
  }
  return true;
}

void LidarStatusPushCallback(const uint32_t /*handle*/,
                             const uint8_t  /*dev_type*/,
                             const char *   info,
                             void *         /*client_data*/)
{
  if (info == nullptr) {
    return;
  }

  int32_t  core_temp_centi  = 0;
  uint32_t lidar_diag       = 0;
  uint32_t cur_work_state32 = 0;
  uint32_t hms_code[8]      = {0};

  const bool ok_temp = ExtractInt32 (info, "\"core_temp\"",         core_temp_centi);
  const bool ok_diag = ExtractUint32(info, "\"lidar_diag_status\"", lidar_diag);
  const bool ok_ws   = ExtractUint32(info, "\"cur_work_state\"",    cur_work_state32);
  const bool ok_hms  = ExtractHmsCode8(info, hms_code);

  if (!(ok_temp && ok_diag && ok_ws && ok_hms)) {
    return;   // partial key set → drop
  }

  blackbox::lidar_resource::log(core_temp_centi,
                                static_cast<uint16_t>(lidar_diag),
                                static_cast<uint8_t>(cur_work_state32),
                                hms_code);
}

}  // namespace

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  SetLivoxLidarInfoCallback(LidarStatusPushCallback, nullptr);
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros
