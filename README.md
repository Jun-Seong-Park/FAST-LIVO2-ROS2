# FAST-LIVO2 ROS2 HUMBLE
Thanks to hku mars lab [chunran zheng](https://github.com/xuankuzcr) for the open source excellent work!! [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)

# 1. Abstract
This repository is forked version with see3cam_24CUG with livox mid 360 and ROS2 humble
all in one Repository for FAST-LIVO2 

# 2. Hardware Settings
- Computer: [Orin NGX024](https://connecttech.com/product/hadron-dm-carrier-for-nvidia-jetson-orin-nx/)
- Camera: [see3cam_24CUG](https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp)
- Lidar: [Livox MID360](https://store.dji.com/product/livox-mid-360?set_region=US&from=site-nav)
- Stm: [bluepill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html)

# 3. Results

# 4. Calibration Method
**Camera Calibration**
[**CAM-CALIB-ROS2**](https://github.com/Jun-Seong-Park/CAM-CALIB)

**Lidar-Camera Calibration**
[**FAST-Calib-ROS2**](https://github.com/Jun-Seong-Park/FAST-CALIB-MULTI-ROS2)

# 5. Prerequisited

## 5.1 Ubuntu and ROS

Ubuntu 22.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

## 5.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/).

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

## 5.3 ETC
```bash
sudo apt install ros-humble-compressed-image-transport
```

# 6. Install
```bash
cd ~/
git clone 
```


## 6.1 Sophus

```bash
cd ~/FAST-LIVO2-ROS2/src/Sophus
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```
## 6.2 Vikit

```bash
cd ~/FAST-LIVO2-ROS2/src/rpg_vikit/vikit_common
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

## 6.3 Livox-SDK2
Custom made livox ros driver2!! for hardware synchro and logging

```bash
cd ~/FAST-LIVO2-ROS2/src/Livox-SDK2
mkdir build
cd build
cmake .. && make -j$(nproc)
sudo make install
```

## 6.4 Sensor Pkg
- econ_ros2_driver: See3CAM_24CUG image driver (GStreamer, hardware triggered, mmap timestamp)
- livox_ros_driver2: LiDAR ROS2 driver (mmap timestamp)

Each package carries its own vendored blackbox header (fixed-size mmap logger) — no shared logging package, so every package builds standalone.

```bash
cd ~/FAST-LIVO2-ROS2
colcon build --symlink-install 
```

# 7. Flashing STM32
Firmware source is located in `LIV_handhold/stm32_timersync-open/`. build, and flash to the Blue Pill. 

# 8. Setup Jetson Orin NGX024
Run all one-time setup scripts (udev rules, jetson_clocks, FastDDS socket buffer):

```bash
sudo bash src/scripts/setup_all.sh
```

# 9. Run with Device
```bash
source install/setup.bash
ros2 launch fast_livo2 launch.py
```
# 10. Analyze Log

Each node writes its blackbox records to a fresh per-run session directory:

```
~/.blackbox/log/<YYYY-MM-DD-HH-MM-SS>-<pid>/   # .bin records, one dir per process per run
```

Old sessions are never overwritten — delete them manually when no longer needed.

Analysis scripts live in `blackbox/` (repo root); plots are written to `blackbox/Log/` (git-ignored).

# 11. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.





