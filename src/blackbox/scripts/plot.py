#!/usr/bin/env python3
"""
Wrapper: run all blackbox analyzers in sequence.

Calls in order:
  1. analyze_sensor_publish.py    pub-side stats (lidar/imu/image) → plots/{lidar,imu,image}/
  2. analyze_sensor_subscribe.py  sub-side stats (lidar/imu/image) → plots/{lidar,imu,image}/
  3. analyze_sensor_compare.py    pub vs sub match per stream      → plots/{lidar,imu,image}/
  4. analyze_resource.py          host + LiDAR device resource     → plots/resource/

Each analyzer takes its own --log-dir / --out-dir. Defaults assume the package
layout (script_dir/../log, script_dir/../plots).

RESOLUTION (edit below) selects the camera variant and is forwarded to the
analyzers that read see3cam24cug_<resolution>_image_pub.bin (publish / compare),
overriding their per-script default. No CLI arg — run with F5 / python3 plot.py.

Usage:
  python3 plot.py        # edit RESOLUTION below to pick the variant
"""
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent

# camera variant forwarded to publish/compare — edit here (no CLI arg):
#   sd | hd | wuxga | wuxga_mjpg
RESOLUTION = 'hd'

# (script, takes_resolution) — publish/compare read see3cam24cug_<resolution>_image_pub.bin
SCRIPTS = [
    ('analyze_sensor_publish.py',   True),
    ('analyze_sensor_subscribe.py', False),
    ('analyze_sensor_compare.py',   True),
    ('analyze_resource.py',         False),
]


def main() -> int:
    for name, takes_resolution in SCRIPTS:
        path = SCRIPTS_DIR / name
        if not path.is_file():
            print(f'[plot] skip {name} (missing)')
            continue
        cmd = [sys.executable, str(path)]
        if takes_resolution:
            cmd += ['--resolution', RESOLUTION]
        print(f'\n[plot] === {name} ===')
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            print(f'[plot] {name} exited {rc}')
            return rc
    print(f'\n[plot] all analyzers done.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
