# 커널 기록 오류 로그 (확정 사실만)

- 기록일: 2026-06-05
- 부팅: 2026-06-05 17:30 (이번 부팅, boot 0)
- 출처: `dmesg -T` / `journalctl -k -b 0` → `~/Downloads/usb3_recent_dmesg.txt`
- errno 매핑 출처: `/usr/include/asm-generic/errno{,-base}.h` (grep 확인 2026-06-05)
- 범위: 해석 없이, 커널에 실제 찍힌 오류 라인만 기록.

## See3CAM_24CUG (usb 2-2) 관련 오류

| 시각 | 로그 라인 | errno |
|---|---|---|
| 17:33:45 | `usb 2-2: Failed to resubmit status URB (-1).` | 1 = EPERM (Operation not permitted) |
| 17:38:32 | `uvcvideo 2-2:1.1: Non-zero status (-71) in video completion handler.` | 71 = EPROTO (Protocol error) |
| 17:51:51 | `uvcvideo 2-2:1.1: Non-zero status (-71) in video completion handler.` | 71 = EPROTO (Protocol error) |
| 18:04:53 | `usb 2-2: Failed to resubmit status URB (-1).` | 1 = EPERM (Operation not permitted) |

## 별개 디바이스 오류 (See3CAM 아님, 참고)

| 시각 | 로그 라인 | errno | 비고 |
|---|---|---|---|
| 17:30:39 | `imx219: probe of 9-0010 failed with error -121` | 121 = EREMOTEIO (Remote I/O error) | imx219 = CSI 센서. USB 카메라와 별개 디바이스 |
| 17:30:39 | `imx219: probe of 10-0010 failed with error -121` | 121 = EREMOTEIO (Remote I/O error) | 동상 |

## 비-오류 이벤트 (참고: 17:50~18:04 사이 수행한 조작 기록)

다음은 오류가 아니라 조작/재-enumerate 기록임 (참고용):
- 18:00:24 `usb 2-2: reset SuperSpeed USB device number 2` — USBDEVFS_RESET
- 18:02:01 / 18:02:01 `authorized to connect` — authorized 토글
- 18:04:01 `USB disconnect, device number 2` → 18:04:03 `new SuperSpeed USB device number 3` — 물리 replug
- 18:04:47 `USB disconnect, device number 3` → 18:04:49 `new SuperSpeed USB device number 4` — 물리 replug
