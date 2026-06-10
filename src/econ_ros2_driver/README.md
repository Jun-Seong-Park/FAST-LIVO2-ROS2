# econ_ros2_driver

ROS2 driver for the e-con Systems **See3CAM_24CUG** (global shutter, USB3).
The goal is external-trigger-based capture + **shared hardware timestamps** (same time domain as the LiDAR).
Capture/conversion use a single backend on the **GStreamer/nvvidconv HW path** (Jetson only).

- Capture: configure the camera over HID to **TRIGGER mode** + fixed exposure → the external trigger (~10 Hz) sets the effective frame rate
- Conversion: GStreamer backend (`v4l2src → nvvidconv → videoconvert → appsink`)
- Publish: `sensor_msgs/Image` (bgr8) or `sensor_msgs/CompressedImage` (JPEG)
- Timestamp: read the GPS-epoch ns from an mmap-shared file and stamp it into `header.stamp` → LiDAR/camera sync
- Observability: an always-on blackbox (mmap binary) records the capture→cbk→pub stages

---

## Architecture

The node (`econ_ros2_driver.cpp`) only owns the **orchestration + publish path**; the actual capture/conversion details are delegated to the GStreamer backend. The node never touches V4L2/GStreamer directly.

```
[param load] → [HID trigger mode+exposure] → [backend start] → grab_thread loop:

   grab() ─→ mmap GPS-epoch stamp ─→ publish() ─→ release()
  (backend)     (MmapStamper)       (DDS sync copy)  (backend reclaims buffer)
                                         │
                                  blackbox timing record
```

| File | Role |
|---|---|
| `src/econ_ros2_driver.cpp` | Node body. After param→HID→backend init, the grab thread repeats grab/stamp/publish/release |
| `src/gst_backend.cpp` `.hpp` | GStreamer backend (Jetson, the only one). `v4l2src → nvvidconv(HW flip/downscale) → videoconvert → appsink`. `ImageFormat`/`Frame` (borrowed view) are also defined here |
| `include/config.hpp` | Defaults + resolution profile table + ROS param loader |
| `include/hid_control.hpp` | Sets stream mode (TRIGGER) / exposure over hidraw |
| `include/mmap_stamper.hpp` | Reads GPS-epoch ns from the mmap (`~/timeshare`) shared with the LiDAR process |
| `include/blackbox.hpp` | mmap fixed array + 1-second fdatasync writer. Blackbox for publish timing |

---

## Prerequisites

Validated environment: **Ubuntu 22.04 / ROS2 Humble**. The driver uses the `nvvidconv` (Tegra HW) path, so it is Jetson-only. OpenCV is required only by the monitoring tools (`econ_monitor`/`econ_image_saver`), not by the driver.

| Item | Install | Download / Docs |
|---|---|---|
| ROS2 Humble | `ros-humble-desktop` (includes rclcpp, sensor_msgs) | [docs.ros.org — Humble Ubuntu install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| colcon | `python3-colcon-common-extensions` | [colcon installation](https://colcon.readthedocs.io/en/released/user/installation.html) |
| GStreamer (dev + plugins) | `libgstreamer1.0-dev` `libgstreamer-plugins-base1.0-dev` `gstreamer1.0-plugins-good` `gstreamer1.0-plugins-bad` `gstreamer1.0-tools` | [gstreamer.freedesktop.org — install on Linux](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html) |
| OpenCV | `libopencv-dev` | [opencv.org/releases](https://opencv.org/releases/) |
| v4l-utils (`v4l2-ctl`) | `v4l-utils` | — |
| **Jetson only** nvvidconv | Included in JetPack/L4T (`nvidia-l4t-gstreamer`) — no separate apt needed | [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack) |
| Camera (hardware) | See3CAM_24CUG (AR0234, USB3 global shutter) | [Product page](https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp) · [Technical docs/firmware](https://www.e-consystems.com/doc-2mp-global-shutter-color-camera.asp) |

> The datasheet, Trigger Mode Application Note, and firmware can be downloaded from the e-con technical docs page (registration may be required).

Follow the official docs above for the ROS2 and colcon install; install the remaining apt dependencies in one go:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop python3-colcon-common-extensions \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-tools \
  libopencv-dev v4l-utils
```

---

## Build

Dependencies: `rclcpp`, `sensor_msgs`, `gstreamer-1.0` / `gstreamer-app-1.0` (pkg-config, for the driver). `OpenCV` is only for the monitoring tools. C++17. (For installation see [Prerequisites](#prerequisites) above)

```bash
cd ~/FAST-LIVO2-ROS2
colcon build --packages-select econ_ros2_driver
source install/setup.bash
```

The 3 generated executables (`CMakeLists.txt`):

| Executable | Source | Purpose |
|---|---|---|
| `econ_ros2_driver` | `econ_ros2_driver.cpp` + `gst_backend.cpp` | Camera publish node (GStreamer, no OpenCV) |
| `econ_monitor` | `image_viewer.cpp` | PC-side live viewer (subscribe → imshow, logs fps/Mbps) |
| `econ_image_saver` | `image_saver.cpp` | For validation. Subscribes to raw → saves one PNG per second |

---

## System setup (one-time, sudo)

```bash
sudo bash scripts/setup_see3cam.sh   # udev symlink + USB autosuspend off + uvcvideo nodrop
bash scripts/setup.sh                # sysctl for FastDDS large socket buffers (rmem/wmem_max)
```

What `setup_see3cam.sh` does (VID=2560 / PID=c128):

- Creates `/dev/24cug` (V4L2) and `/dev/24cug_hid` (HID) **udev symlinks** + permission 0666
  → the camera is identified by device, so it is found even if the port changes
- **Permanently disables USB autosuspend** (default 2000ms → -1) — removes the wake-from-power-save latency
- `uvcvideo nodrop=1` — prevents dropping incomplete trigger frames

After installing, unplug and replug the camera once.

---

## Run

### Publish (Jetson)

```bash
ros2 launch econ_ros2_driver pub.launch.py
```

`pub.launch.py` passes `config/config.yaml` as parameters and, depending on the `localhost_only` value, sets the FastDDS profile (loopback / jetson) and `ROS_LOCALHOST_ONLY`.

### Monitor (viewer)

```bash
ros2 launch econ_ros2_driver monitor.launch.py client:=local encoding:=raw
```

- `client`: `local` (Jetson-internal loopback) | `server` (external PC receiving) → selects the FastDDS profile
- `encoding`: `raw` (`sensor_msgs/Image`) | `compressed` (`CompressedImage`)

### Image saving (validation)

```bash
ros2 launch econ_ros2_driver save.launch.py
```

Subscribes to `/camera/image` and saves one PNG per second under `Log/`.

---

## Configuration (`config/config.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/24cug` | V4L2 capture device (udev symlink) |
| `hid_device` | `/dev/24cug_hid` | HID control device (udev symlink) |
| `resolution` | `hd` | `sd` \| `hd` \| `fhd` \| `wuxga` (profile, table below) |
| `exposure_us` | `10000` | Exposure [us] |
| `compressed` | `false` | `false` → raw bgr8 / `true` → MJPG CompressedImage |
| `topic_name` | `/camera/image` | Publish topic (when compressed, `/camera/image/compressed`) |
| `frame_id` | `camera_init` | header frame_id |
| `localhost_only` | `true` | Selects FastDDS loopback vs network profile (used by launch) |
| `flip_method` | `0` | nvvidconv HW rotation. `0` none / `1` CCW90 / `2` 180 / `3` CW90 (**gstreamer only**) |

> Some, such as `flip_method` and `auto_exposure`, take their constexpr defaults from `config.hpp` and are overridden by yaml/launch.

### Resolution profiles

`resolution` is a profile string rather than a number, and **the table differs per capture format** (UYVY vs MJPG — a 1:1 copy of the `v4l2-ctl --list-formats-ext` menu, `config.hpp`).

| Profile | Capture (UYVY/MJPG) | Output | Note |
|---|---|---|---|
| `sd` | 1280×720 | 640×360 | The only downscaled one |
| `hd` | 1280×720 | 1280×720 | Full frame |
| `fhd` | 1920×1080 | 1920×1080 | Full frame |
| `wuxga` | 1920×1200 | 1920×1200 | UYVY only negotiates 55fps |

- fps is a nominal value for caps negotiation — the effective frame rate is set by the external trigger.
- On 90° rotation (`flip_method` 1/3) the published width and height are swapped.
- JPEG (`compressed=true`) cannot be rescaled within the stream → it is published at the capture resolution as-is (regardless of output/downscale).

---

## Topics

| compressed | Topic | Type | Content |
|---|---|---|---|
| `false` | `/camera/image` | `sensor_msgs/Image` | bgr8, fixed size |
| `true` | `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `format=jpeg`, variable size |

QoS is `SensorDataQoS` (best effort).

---

## Timestamp sync

Two different clocks are used.

- `header.stamp` ← the **GPS-epoch ns** from the `~/timeshare` mmap. The LiDAR side (`livox_ros_driver2_sync`) writes to the same file → the camera/LiDAR share the same time domain (for SLAM registration). If the mmap is still 0, falls back to `now()`.
- `t_capture_ns` ← **CLOCK_MONOTONIC_RAW**, taken at the v4l2src src-pad probe (or V4L2 DQBUF). Used only for blackbox stage measurement.

The layout is kept compatible with `LIV_handhold grab_trigger.cpp` at a fixed 16B.

---

## Blackbox (observability)

Publish-path timing is **always** recorded to an mmap binary (no off switch — lock-free, ns-level cost).

- Path: `~/.blackbox/log/<YYYY-MM-DD-HH-MM-SS>-<pid>/econ_ros2_driver_pub.bin`
- Record (48B): `seq`, `header_stamp`, `t_capture_ns`, `t_cbk_ns`, `t_pub_ns`, flags
- Stages: capture→cbk = `t_cbk_ns - t_capture_ns`, cbk→pub = `t_pub_ns - t_cbk_ns`
- The writer thread runs `fdatasync` every second → the most recent state is preserved even on abnormal exit

The node also does **drop accounting** per grab: the dropped count is the backend's `push_count` delta minus the 1 just received, and on occurrence it logs the running total as WARN.

---

## Directory

```
econ_ros2_driver/
├── src/        econ_ros2_driver / gst_backend / image_viewer / image_saver
├── include/    gst_backend, config, hid_control, mmap_stamper, blackbox
├── config/     config.yaml, fastdds_{pc,jetson,loopback}.xml
├── launch/     pub / monitor / save .launch.py
├── scripts/    setup_see3cam.sh, setup.sh, usb-reset.sh
└── Document/   pipeline_timeline diagram
```

---

## License

MIT
