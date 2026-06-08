# See3CAM_24CUG USB3 스트리밍 에러 리포트

- 날짜: 2026-06-05
- 장비: Jetson Orin NX 16GB on CTI Hadron (NGX024), L4T R36.4.4, 5.15.148-tegra
- 카메라: e-con Systems See3CAM_24CUG (USB ID 2560:c128), uvcvideo
- USB 컨트롤러: tegra-xusb (3610000.usb), Host supports USB 3.1 Enhanced SuperSpeed

## 증상
- image_saver 로 1초당 1프레임 PNG 저장 시, **첫 프레임(frame_00001)이 수평 찢김 + 색 노이즈**로 깨짐.
- 스트리밍을 돌리면 **시작 시점에만** USB 에러가 뜨고, 이후에는 초당 반복 없이 안정화됨(사용자 관측).

## 관측 데이터 (dmesg / journalctl)
원본: `dmesg_usb3_2026-06-05.txt` (이 폴더)

핵심 라인:
- 이번 부팅(17:30): `usb 2-2: new SuperSpeed USB device number 2` — high-speed 폴백 없이 **바로 USB3(5000Mbps)로 협상됨**.
- 17:38:33: `uvcvideo 2-2:1.1: Non-zero status (-71) in video completion handler`
  - `-71 = EPROTO` (Protocol error). 검증: `grep -w EPROTO /usr/include/asm-generic/errno.h` → `#define EPROTO 71`.
  - 의미: isochronous 비디오 전송 중 USB 프로토콜/패킷 단 에러. 대역폭 부족이 아니라 **링크 신호 단 데이터 손상**.
- 직전 부팅(10:06): `usb 2-2: device descriptor read/8, error -110` → 재시도 후 `new SuperSpeed USB device`
  - `-110 = ETIMEDOUT`. enumeration 디스크립터 읽기 타임아웃 후 재시도로 USB3 성공.

현재 `lsusb -t` 상태(정상 시): `Port 2: Dev 2, Class=Video, Driver=uvcvideo, 5000M` — USB3로 정상 enumerate.

## 분석 / 결론
- **USB2.0 강등(대역폭 부족) 문제 아님.** 카메라는 항상 USB3로 협상됨.
- 실제 문제는 **SuperSpeed 링크 신호 무결성 마진 부족**:
  - enumeration 단계 — 가끔 타임아웃(-110) 후 재시도로 붙음.
  - streaming 단계 — 개시 직후 EPROTO(-71) 버스트 → 첫 프레임 찢김. 정착 후 안정.
- frame_00001 이 깨진 것과 일치: image_saver 첫 저장 프레임이 EPROTO 버스트 구간에 걸림.

## 참고 (포럼)
- NVIDIA forum 326838 (custom carrier board, 신호 품질 / compliance test) — **이 케이스와 일치**.
- NVIDIA forum 36249 (구형 TK1, USB3 boot config 비활성) — 무관 (협상 자체는 USB3로 됨).

## 미해결 / 다음 단계
- 첫 EPROTO 버스트가 끝난 뒤 프레임(frame_00002+)이 깨끗한지 시각 확인 — **미완료** (Log 디렉토리가 캡처 직후 삭제되어 PNG 미확보).
- 재현: 스트리밍 재시작 → frame_00001 vs frame_00010 비교로 "첫 N프레임만 버리면 되는가" 판정.
- 근본 대응 후보: CTI Hadron USB3 레인 신호 품질(compliance), 케이블/커넥터, 부팅 시 카메라 연결 vs 핫플러그 차이.

## 첨부물
- `dmesg_usb3_2026-06-05.txt` — dmesg/journalctl USB3 관련 발췌
- (이미지 PNG: Log 디렉토리 삭제로 미확보 — 재스트리밍으로 재생성 필요)
