#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

static bool ExtractCoreTempCenti(const char* json, int32_t& core_temp_centi) {
  const char* key_ptr = std::strstr(json, "\"core_temp\"");
  if (key_ptr == nullptr) {
    return false;
  }
  const char* colon_ptr = std::strchr(key_ptr, ':');
  if (colon_ptr == nullptr) {
    return false;
  }
  core_temp_centi = std::strtol(colon_ptr + 1, nullptr, 10);
  return true;
}

static std::string TimestampNow() {
  auto now      = std::chrono::system_clock::now();
  auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count();
  std::time_t tt = static_cast<std::time_t>(epoch_ms / 1000);
  int         ms = static_cast<int>(epoch_ms % 1000);

  std::tm tm_local;
  localtime_r(&tt, &tm_local);

  std::ostringstream ss;
  ss << std::put_time(&tm_local, "%Y-%m-%d %H:%M:%S")
     << "." << std::setw(3) << std::setfill('0') << ms;
  return ss.str();
}

static void PushMsgCallback(const uint32_t handle,
                            const uint8_t  /*dev_type*/,
                            const char*    info,
                            void*          /*client_data*/) {
  int32_t core_temp_centi = 0;
  if (!ExtractCoreTempCenti(info, core_temp_centi)) {
    return;
  }
  in_addr     addr;
  addr.s_addr = handle;
  const char* ip_str = inet_ntoa(addr);
  const double temp_c = core_temp_centi / 100.0;
  std::printf("[%s] ip=%-15s core_temp=%6.2f C\n",
              TimestampNow().c_str(), ip_str, temp_c);
  std::fflush(stdout);
}

static void InfoChangeCallback(const uint32_t          handle,
                               const LivoxLidarInfo*   info,
                               void*                   /*client_data*/) {
  in_addr     addr;
  addr.s_addr = handle;
  std::printf("[%s] connected lidar SN=%s ip=%s\n",
              TimestampNow().c_str(), info->sn, inet_ntoa(addr));
  std::fflush(stdout);
}

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::fprintf(stderr, "Usage: %s <config.json>\n", argv[0]);
    return -1;
  }
  const std::string cfg_path = argv[1];

  if (!LivoxLidarSdkInit(cfg_path.c_str())) {
    std::fprintf(stderr, "Livox SDK init failed.\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoCallback(PushMsgCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, nullptr);

  std::printf("[%s] waiting for MID-360 push msg (core_temp)... Ctrl-C to stop.\n",
              TimestampNow().c_str());
  std::fflush(stdout);

  while (true) {
    sleep(3600);
  }

  LivoxLidarSdkUninit();
  return 0;
}
