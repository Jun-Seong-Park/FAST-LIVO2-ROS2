#!/usr/bin/env python3
"""
Wrapper: run all blackbox analyzers in sequence.

Calls in order:
  1. analyze_sensor_publish.py    pub-side stats (lidar/imu/image) → plots/{lidar,imu,image}/
  2. analyze_sensor_subscribe.py  sub-side stats (lidar/imu/image) → plots/{lidar,imu,image}/
  3. compare_pub_sub.py           pub vs sub match per stream      → plots/{lidar,imu,image}/
  4. analyze_resource.py          host + LiDAR device resource     → plots/resource/

Each analyzer takes its own --log-dir / --out-dir. Defaults assume the package
layout (script_dir/../log, script_dir/../plots).

Usage:
  python3 plot.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent

SCRIPTS = [
    'analyze_sensor_publish.py',
    'analyze_sensor_subscribe.py',
    'compare_pub_sub.py',
    'analyze_resource.py',
]


def main() -> int:
    for name in SCRIPTS:
        path = SCRIPTS_DIR / name
        if not path.is_file():
            print(f'[plot] skip {name} (missing)')
            continue
        print(f'\n[plot] === {name} ===')
        rc = subprocess.run([sys.executable, str(path)]).returncode
        if rc != 0:
            print(f'[plot] {name} exited {rc}')
            return rc
    print(f'\n[plot] all analyzers done.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
