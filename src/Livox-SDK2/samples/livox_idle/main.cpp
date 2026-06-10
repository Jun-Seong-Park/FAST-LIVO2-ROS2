// livox_idle — MID-360 lifecycle tool + HMS code dump
//
// Modes:
//   --info     : output work_state, time_sync_type, hms_code (query only)
//   --wake     : SetWorkMode(Normal=0x01) — attempt to enter SAMPLING
//   --standby  : SetWorkMode(WakeUp=0x02) — attempt to enter IDLE/Standby
//   --sleep    : SetWorkMode(Sleep=0x03)  — may be ineffective on the MID-360
//   --reboot   : LivoxLidarRequestReboot — cmd 0x0200 firmware reboot
//
// Key changes vs previous version:
//   - add parsing/output of key 0x8011 (hms_code[8]) inside QueryInfoCallback
//   - add output of response->ret_code, response->error_key to WorkModeCallback
//
// HMS code format (livox_eth_protocol_mid360.md, hms_code_mid360.html):
//   uint32 = (id << 16) | (reserved << 8) | level
//   level: 0x01 Info / 0x02 Warning / 0x03 Error / 0x04 Fatal

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <atomic>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

enum class Mode { STANDBY, WAKE, INFO, SLEEP, REBOOT, BOOT_NORMAL, BOOT_WAKEUP, FIRMWARE };
static Mode g_mode = Mode::STANDBY;
static std::atomic<bool> g_done{false};
static std::atomic<bool> g_connected{false};
static std::atomic<bool> g_ptp_synced{false};
static uint32_t g_handle = 0;

void PointCloudCallback(uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
  if (data && data->time_type != 0) {
    g_ptp_synced = true;
  }
}

static void DumpHmsCodes(const uint8_t* val, uint16_t len) {
  // 8 x uint32
  if (len < 32) {
    printf("hms_code: (unexpected length=%u)\n", len);
    return;
  }
  const uint32_t* codes = reinterpret_cast<const uint32_t*>(val);
  int active = 0;
  for (int i = 0; i < 8; ++i) {
    if (codes[i] == 0) continue;
    ++active;
    uint16_t id    = (codes[i] >> 16) & 0xFFFF;
    uint8_t  level =  codes[i]        & 0xFF;
    const char* lvl =
        (level == 0x01) ? "Info"    :
        (level == 0x02) ? "Warning" :
        (level == 0x03) ? "Error"   :
        (level == 0x04) ? "Fatal"   : "?";
    printf("hms_code[%d]=0x%08X (id=0x%04X, level=%s)\n",
           i, codes[i], id, lvl);
  }
  if (active == 0) {
    printf("hms_code: all zero (no fault reported)\n");
  }
}

void QueryInfoCallback(livox_status status, uint32_t /*handle*/,
                       LivoxLidarDiagInternalInfoResponse* response, void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess && response && response->ret_code == 0) {
    uint8_t* data = response->data;
    uint16_t param_num = response->param_num;
    uint16_t offset = 0;
    for (int i = 0; i < param_num && offset < 1000; i++) {
      uint16_t key = *(uint16_t*)(data + offset);
      uint16_t len = *(uint16_t*)(data + offset + 2);
      uint8_t* val = data + offset + 4;

      if (key == 0x8006) {
        printf("work_state=%d", *val);
        switch (*val) {
          case 1: printf(" (Normal)\n");  break;
          case 2: printf(" (Standby)\n"); break;
          case 3: printf(" (Sleep)\n");   break;
          case 4: printf(" (ERROR)\n");   break;
          case 5: printf(" (SelfCheck)\n"); break;
          case 6: printf(" (MotorStartup)\n"); break;
          case 7: printf(" (MotorStopping)\n"); break;
          case 8: printf(" (Upgrade)\n"); break;
          case 9: printf(" (Ready)\n");   break;
          default: printf("\n");          break;
        }
      } else if (key == 0x800C) {
        printf("time_sync_type=%d", *val);
        switch (*val) {
          case 0: printf(" (NoSync)\n");  break;
          case 1: printf(" (PTP)\n");     break;
          case 2: printf(" (GPS)\n");     break;
          case 3: printf(" (PPS)\n");     break;
          default: printf(" (Unknown)\n"); break;
        }
      } else if (key == 0x8011) {
        DumpHmsCodes(val, len);
      } else if (key == 0x8002) {
        // VersionApp — typically 4 bytes major.minor.patch.build
        if (len >= 4) {
          printf("version_app=%u.%u.%u.%u (raw=", val[0], val[1], val[2], val[3]);
          for (int b = 0; b < len; ++b) printf("%02X", val[b]);
          printf(")\n");
        }
      } else if (key == 0x8003) {
        if (len >= 4) {
          printf("version_loader=%u.%u.%u.%u\n", val[0], val[1], val[2], val[3]);
        }
      } else if (key == 0x8004) {
        if (len >= 4) {
          printf("version_hardware=%u.%u.%u.%u\n", val[0], val[1], val[2], val[3]);
        }
      } else {
        // for debugging unhandled keys — minimal exposure
        // printf("(key=0x%04X len=%u)\n", key, len);
      }
      offset += 4 + len;
    }
  } else {
    printf("QueryInfoCallback: status=%d, ret_code=%d\n",
           status, (response ? response->ret_code : -1));
  }
  g_done = true;
}

void WorkModeCallback(livox_status status, uint32_t /*handle*/,
                      LivoxLidarAsyncControlResponse* response, void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess) {
    if (response) {
      printf("Work mode: status=success, ret_code=%u, error_key=0x%04X\n",
             response->ret_code, response->error_key);
    } else {
      printf("Work mode: status=success, response=null\n");
    }
  } else if (status == kLivoxLidarStatusTimeout) {
    printf("Work mode: status=timeout (no ACK in 1s — LiDAR may still be transitioning)\n");
  } else {
    printf("Work mode: status=%d (error)\n", status);
  }
}

void AfterBootCallback(livox_status status, uint32_t /*handle*/,
                       LivoxLidarAsyncControlResponse* response, void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess && response) {
    printf("SetWorkModeAfterBoot ACKed: ret_code=%u, error_key=0x%04X\n",
           response->ret_code, response->error_key);
  } else if (status == kLivoxLidarStatusTimeout) {
    printf("SetWorkModeAfterBoot timeout (no ACK in 1s)\n");
  } else {
    printf("SetWorkModeAfterBoot error: status=%d\n", status);
  }
  g_done = true;
}

void RebootCallback(livox_status status, uint32_t /*handle*/,
                    LivoxLidarRebootResponse* response, void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess && response) {
    printf("Reboot ACKed, ret_code=%u\n", response->ret_code);
  } else if (status == kLivoxLidarStatusTimeout) {
    printf("Reboot sent (no ACK in 1s — LiDAR likely already rebooting)\n");
  } else {
    printf("Reboot error: status=%d\n", status);
  }
  g_done = true;
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
  if (info == nullptr) return;
  printf("LiDAR connected: %s (%s)\n", info->sn, info->lidar_ip);
  g_handle = handle;
  g_connected = true;
}

void print_usage(const char* prog) {
  printf("Usage: %s <config.json> [--wake|--standby|--sleep|--info|--reboot]\n", prog);
}

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }

  for (int i = 2; i < argc; i++) {
    if      (strcmp(argv[i], "--wake")    == 0) g_mode = Mode::WAKE;
    else if (strcmp(argv[i], "--standby") == 0) g_mode = Mode::STANDBY;
    else if (strcmp(argv[i], "--sleep")   == 0) g_mode = Mode::SLEEP;
    else if (strcmp(argv[i], "--info")    == 0) g_mode = Mode::INFO;
    else if (strcmp(argv[i], "--reboot")  == 0) g_mode = Mode::REBOOT;
    else if (strcmp(argv[i], "--boot-normal") == 0) g_mode = Mode::BOOT_NORMAL;
    else if (strcmp(argv[i], "--boot-wakeup") == 0) g_mode = Mode::BOOT_WAKEUP;
    else if (strcmp(argv[i], "--firmware")    == 0) g_mode = Mode::FIRMWARE;
  }

  if (!LivoxLidarSdkInit(argv[1])) {
    printf("SDK init failed\n");
    return 1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  if (g_mode == Mode::WAKE) {
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  }

  if (!LivoxLidarSdkStart()) {
    printf("SDK start failed\n");
    return 1;
  }

  printf("Waiting for LiDAR...\n");
  for (int i = 0; i < 100 && !g_connected; i++) usleep(100000);
  if (!g_connected) {
    printf("LiDAR not found\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  if (g_mode == Mode::STANDBY) {
    printf("Switching to Standby...\n");
    SetLivoxLidarWorkMode(g_handle, kLivoxLidarWakeUp, WorkModeCallback, nullptr);
    sleep(3);
    QueryLivoxLidarInternalInfo(g_handle, QueryInfoCallback, nullptr);

  } else if (g_mode == Mode::WAKE) {
    printf("Switching to Normal mode...\n");
    SetLivoxLidarWorkMode(g_handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
    printf("Waiting for PTP sync...\n");
    for (int i = 0; i < 300 && !g_ptp_synced; i++) usleep(100000);
    if (g_ptp_synced) printf("PTP synced!\n");
    else              printf("PTP sync timeout\n");
    g_done = true;

  } else if (g_mode == Mode::SLEEP) {
    printf("Switching to Sleep mode...\n");
    SetLivoxLidarWorkMode(g_handle, kLivoxLidarSleep, WorkModeCallback, nullptr);
    sleep(3);
    QueryLivoxLidarInternalInfo(g_handle, QueryInfoCallback, nullptr);

  } else if (g_mode == Mode::INFO) {
    QueryLivoxLidarInternalInfo(g_handle, QueryInfoCallback, nullptr);

  } else if (g_mode == Mode::REBOOT) {
    printf("Requesting firmware reboot...\n");
    LivoxLidarRequestReboot(g_handle, RebootCallback, nullptr);

  } else if (g_mode == Mode::BOOT_NORMAL) {
    printf("Setting WorkModeAfterBoot = Normal (0x01) — auto-SAMPLING after reboot\n");
    SetLivoxLidarWorkModeAfterBoot(g_handle, kLivoxLidarWorkModeAfterBootNormal,
                                   AfterBootCallback, nullptr);

  } else if (g_mode == Mode::BOOT_WAKEUP) {
    printf("Setting WorkModeAfterBoot = WakeUp (0x02) — auto-IDLE after reboot\n");
    SetLivoxLidarWorkModeAfterBoot(g_handle, kLivoxLidarWorkModeAfterBootWakeUp,
                                   AfterBootCallback, nullptr);

  } else if (g_mode == Mode::FIRMWARE) {
    QueryLivoxLidarFirmwareVer(g_handle, QueryInfoCallback, nullptr);
  }

  for (int i = 0; i < 100 && !g_done; i++) usleep(100000);

  LivoxLidarSdkUninit();
  printf("Done\n");
  return (g_mode == Mode::WAKE) ? (g_ptp_synced ? 0 : 1) : 0;
}
