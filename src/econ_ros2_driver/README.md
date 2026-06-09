# econ_ros2_driver

e-con Systems **See3CAM_24CUG** (글로벌 셔터, USB3) 용 ROS2 드라이버.
외부 트리거 기반 캡처 + **하드웨어 타임스탬프 공유**(LiDAR 와 같은 time domain)를 목표로 한다.
캡처/변환은 **GStreamer/nvvidconv HW 경로** 단일 backend (Jetson 전용).

- 캡처: HID 로 카메라를 **TRIGGER 모드** + 고정 노출로 설정 → 외부 트리거(~10 Hz)가 실효 프레임레이트 결정
- 변환: GStreamer backend (`v4l2src → nvvidconv → videoconvert → appsink`)
- 발행: `sensor_msgs/Image`(bgr8) 또는 `sensor_msgs/CompressedImage`(JPEG)
- 타임스탬프: mmap 공유 파일에서 GPS-epoch ns 를 읽어 `header.stamp` 에 박음 → LiDAR/카메라 동기
- 관측: 항상 켜져 있는 blackbox(mmap 바이너리)로 캡처→cbk→pub 구간 기록

---

## 아키텍처

노드(`econ_ros2_driver.cpp`)는 **오케스트레이션 + 발행 경로**만 소유하고, 캡처/변환의 실제 디테일은 GStreamer backend 로 위임한다. 노드는 V4L2/GStreamer 를 직접 만지지 않는다.

```
[param load] → [HID 트리거모드+노출] → [backend start] → grab_thread loop:

   grab() ─→ mmap GPS-epoch stamp ─→ publish() ─→ release()
  (backend)     (MmapStamper)       (DDS 동기복사)  (backend 가 버퍼 회수)
                                         │
                                  blackbox 타이밍 기록
```

| 파일 | 역할 |
|---|---|
| `src/econ_ros2_driver.cpp` | 노드 본체. param→HID→backend 초기화 후 grab thread 에서 grab/stamp/publish/release 반복 |
| `src/gst_backend.cpp` `.hpp` | GStreamer backend (Jetson, 유일). `v4l2src → nvvidconv(HW flip/downscale) → videoconvert → appsink`. `ImageFormat`/`Frame`(borrowed view) 도 여기 정의 |
| `include/config.hpp` | 기본값 + 해상도 프로파일 표 + ROS param 로더 |
| `include/hid_control.hpp` | hidraw 로 stream mode(TRIGGER) / exposure 설정 |
| `include/mmap_stamper.hpp` | LiDAR 프로세스와 공유하는 mmap(`~/timeshare`)에서 GPS-epoch ns 읽기 |
| `include/blackbox.hpp` | mmap 고정배열 + 1초 fdatasync writer. 발행 타이밍 블랙박스 |

---

## Prerequisites

검증 환경: **Ubuntu 22.04 / ROS2 Humble**. 드라이버는 `nvvidconv` (Tegra HW) 경로를 쓰므로 Jetson 전용이다. OpenCV 는 드라이버가 아니라 모니터링 도구(`econ_monitor`/`econ_image_saver`) 에만 필요하다.

| 항목 | 설치 | 다운로드 / 문서 |
|---|---|---|
| ROS2 Humble | `ros-humble-desktop` (rclcpp, sensor_msgs 포함) | [docs.ros.org — Humble Ubuntu install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| colcon | `python3-colcon-common-extensions` | [colcon installation](https://colcon.readthedocs.io/en/released/user/installation.html) |
| GStreamer (dev + plugins) | `libgstreamer1.0-dev` `libgstreamer-plugins-base1.0-dev` `gstreamer1.0-plugins-good` `gstreamer1.0-plugins-bad` `gstreamer1.0-tools` | [gstreamer.freedesktop.org — install on Linux](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html) |
| OpenCV | `libopencv-dev` | [opencv.org/releases](https://opencv.org/releases/) |
| v4l-utils (`v4l2-ctl`) | `v4l-utils` | — |
| **Jetson 전용** nvvidconv | JetPack/L4T 에 포함 (`nvidia-l4t-gstreamer`) — 별도 apt 불필요 | [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack) |
| 카메라 (하드웨어) | See3CAM_24CUG (AR0234, USB3 글로벌 셔터) | [제품 페이지](https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp) · [기술 문서/펌웨어](https://www.e-consystems.com/doc-2mp-global-shutter-color-camera.asp) |

> e-con 기술 문서 페이지에서 데이터시트·Trigger Mode Application Note·펌웨어를 받을 수 있다(등록 필요할 수 있음).

ROS2 와 colcon 설치는 위 공식 문서를 따르고, 나머지 apt 의존은 한 번에:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop python3-colcon-common-extensions \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-tools \
  libopencv-dev v4l-utils
```

---

## 빌드

의존: `rclcpp`, `sensor_msgs`, `gstreamer-1.0` / `gstreamer-app-1.0` (pkg-config, 드라이버). `OpenCV` 는 모니터링 도구 전용. C++17. (설치는 위 [Prerequisites](#prerequisites) 참고)

```bash
cd ~/FAST-LIVO2-ROS2
colcon build --packages-select econ_ros2_driver
source install/setup.bash
```

생성되는 실행파일 3개 (`CMakeLists.txt`):

| 실행파일 | 소스 | 용도 |
|---|---|---|
| `econ_ros2_driver` | `econ_ros2_driver.cpp` + `gst_backend.cpp` | 카메라 발행 노드 (GStreamer, OpenCV 미사용) |
| `econ_monitor` | `image_viewer.cpp` | PC 측 라이브 뷰어 (구독 → imshow, fps/Mbps 로깅) |
| `econ_image_saver` | `image_saver.cpp` | 검증용. raw 구독 → 1초당 PNG 한 장 저장 |

---

## 시스템 셋업 (최초 1회, sudo)

```bash
sudo bash scripts/setup_see3cam.sh   # udev symlink + USB autosuspend off + uvcvideo nodrop
bash scripts/setup.sh                # FastDDS 대용량 소켓버퍼용 sysctl (rmem/wmem_max)
```

`setup_see3cam.sh` 가 하는 일 (VID=2560 / PID=c128):

- `/dev/24cug` (V4L2), `/dev/24cug_hid` (HID) **udev symlink** 생성 + 권한 0666
  → 포트가 바뀌어도 카메라 식별로 잡힌다
- USB **autosuspend 영구 비활성화** (기본 2000ms → -1) — 절전 복귀 지연 레이턴시 제거
- `uvcvideo nodrop=1` — 트리거 incomplete 프레임 드롭 방지

설치 후 카메라를 한 번 뺐다 다시 꽂는다.

---

## 실행

### 발행 (Jetson)

```bash
ros2 launch econ_ros2_driver pub.launch.py
```

`pub.launch.py` 는 `config/config.yaml` 을 파라미터로 넘기고, `localhost_only` 값에 따라 FastDDS 프로파일(loopback / jetson)과 `ROS_LOCALHOST_ONLY` 를 설정한다.

### 모니터 (뷰어)

```bash
ros2 launch econ_ros2_driver monitor.launch.py client:=local encoding:=raw
```

- `client`: `local` (Jetson 내부 loopback) | `server` (외부 PC 수신) → FastDDS 프로파일 선택
- `encoding`: `raw` (`sensor_msgs/Image`) | `compressed` (`CompressedImage`)

### 이미지 저장 (검증)

```bash
ros2 launch econ_ros2_driver save.launch.py
```

`/camera/image` 를 구독해 `Log/` 에 1초당 PNG 한 장을 저장한다.

---

## 설정 (`config/config.yaml`)

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `device` | `/dev/24cug` | V4L2 캡처 디바이스 (udev symlink) |
| `hid_device` | `/dev/24cug_hid` | HID 제어 디바이스 (udev symlink) |
| `resolution` | `hd` | `sd` \| `hd` \| `fhd` \| `wuxga` (프로파일, 아래 표) |
| `exposure_us` | `10000` | 노출 [us] |
| `compressed` | `false` | `false` → raw bgr8 / `true` → MJPG CompressedImage |
| `topic_name` | `/camera/image` | 발행 토픽 (compressed 면 `/camera/image/compressed`) |
| `frame_id` | `camera_init` | header frame_id |
| `localhost_only` | `true` | FastDDS loopback vs 네트워크 프로파일 선택 (launch 가 사용) |
| `flip_method` | `0` | nvvidconv HW 회전. `0` none / `1` CCW90 / `2` 180 / `3` CW90 (**gstreamer 전용**) |

> `flip_method`, `auto_exposure` 등 일부는 `config.hpp` 의 constexpr 기본값으로 들어가며 yaml/launch 가 오버라이드한다.

### 해상도 프로파일

`resolution` 은 숫자가 아니라 프로파일 문자열이며, **캡처 포맷별로 표가 다르다**(UYVY vs MJPG — `v4l2-ctl --list-formats-ext` 메뉴의 1:1 사본, `config.hpp`).

| 프로파일 | 캡처 (UYVY/MJPG) | 출력 | 비고 |
|---|---|---|---|
| `sd` | 1280×720 | 640×360 | 유일하게 다운스케일 |
| `hd` | 1280×720 | 1280×720 | 풀프레임 |
| `fhd` | 1920×1080 | 1920×1080 | 풀프레임 |
| `wuxga` | 1920×1200 | 1920×1200 | UYVY 는 55fps 만 협상됨 |

- fps 는 caps 협상용 명목값 — 실효 프레임레이트는 외부 트리거가 결정한다.
- 90° 회전(`flip_method` 1/3) 시 발행 가로·세로가 swap 된다.
- JPEG(`compressed=true`) 는 스트림 내 리스케일 불가 → 캡처 해상도 그대로 발행(출력/다운스케일 무관).

---

## 토픽

| compressed | 토픽 | 타입 | 내용 |
|---|---|---|---|
| `false` | `/camera/image` | `sensor_msgs/Image` | bgr8, 고정 크기 |
| `true` | `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `format=jpeg`, 가변 크기 |

QoS 는 `SensorDataQoS` (best effort).

---

## 타임스탬프 동기

두 개의 서로 다른 시계를 쓴다.

- `header.stamp` ← `~/timeshare` mmap 의 **GPS-epoch ns**. LiDAR 측(`livox_ros_driver2_sync`)이 같은 파일에 기록 → 카메라/LiDAR 가 동일 time domain 을 공유한다 (SLAM 정합용). mmap 이 아직 0 이면 `now()` 로 fallback.
- `t_capture_ns` ← **CLOCK_MONOTONIC_RAW**, v4l2src src-pad probe(또는 V4L2 DQBUF) 시점. blackbox 구간 측정 전용.

레이아웃은 `LIV_handhold grab_trigger.cpp` 와 16B 고정으로 호환 유지한다.

---

## Blackbox (관측)

발행 경로 타이밍을 **항상** mmap 바이너리로 남긴다 (끄는 스위치 없음 — lock-free, ns 급 비용).

- 경로: `~/.blackbox/log/<YYYY-MM-DD-HH-MM-SS>-<pid>/econ_ros2_driver_pub.bin`
- 레코드(48B): `seq`, `header_stamp`, `t_capture_ns`, `t_cbk_ns`, `t_pub_ns`, flags
- 구간: 캡처→cbk = `t_cbk_ns - t_capture_ns`, cbk→pub = `t_pub_ns - t_cbk_ns`
- writer 스레드가 1초마다 `fdatasync` → 비정상 종료해도 직전 상태 보존

또한 노드는 grab 단위로 **drop accounting** 을 한다: backend 의 `push_count` 델타에서 방금 받은 1 을 뺀 값이 드롭이며, 발생 시 WARN 으로 누계를 찍는다.

---

## 디렉토리

```
econ_ros2_driver/
├── src/        econ_ros2_driver / gst_backend / image_viewer / image_saver
├── include/    gst_backend, config, hid_control, mmap_stamper, blackbox
├── config/     config.yaml, fastdds_{pc,jetson,loopback}.xml
├── launch/     pub / monitor / save .launch.py
├── scripts/    setup_see3cam.sh, setup.sh, usb-reset.sh
└── Document/   pipeline_timeline 다이어그램
```

---

## License

MIT
