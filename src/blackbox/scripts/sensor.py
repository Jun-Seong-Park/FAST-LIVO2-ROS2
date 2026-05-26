#!/usr/bin/env python3
"""
Wrapper: sensor-only analyzers (no SLAM session).

Calls in order:
  1. analyze_sensor_publish.py    pub-side stats → plots/{lidar,imu,image}/
  2. analyze_resource.py          host + LiDAR resource → plots/resource/

Skips analyze_sensor_subscribe.py and compare_pub_sub.py since sub_*.bin
files are absent when fast_livo2 wasn't running.

Usage:
  python3 sensor.py
"""
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent

SCRIPTS = [
    'analyze_sensor_publish.py',
    'analyze_resource.py',
]


def main() -> int:
    for name in SCRIPTS:
        path = SCRIPTS_DIR / name
        if not path.is_file():
            print(f'[sensor] skip {name} (missing)')
            continue
        print(f'\n[sensor] === {name} ===')
        rc = subprocess.run([sys.executable, str(path)]).returncode
        if rc != 0:
            print(f'[sensor] {name} exited {rc}')
            return rc
    print(f'\n[sensor] sensor analyzers done.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
