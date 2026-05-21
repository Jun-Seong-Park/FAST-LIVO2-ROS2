#!/usr/bin/env python3
"""Run all three trace analyzers in sequence."""
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent

SCRIPTS = [
    'analyze_sensor_publish.py',
    'analyze_sensor_subscribe.py',
    'compare_pub_sub.py',
]

for script in SCRIPTS:
    path = SCRIPTS_DIR / script
    print(f'\n=== {script} ===')
    result = subprocess.run([sys.executable, str(path)], check=False)
    if result.returncode != 0:
        print(f'[WARN] {script} exited with code {result.returncode}')
