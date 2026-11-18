#!/usr/bin/env python3
"""Manual experiment annotation only; never changes the vessel's controls."""
import argparse
import json
import os
from pathlib import Path
import time

from logger_config import load_logger_config

parser = argparse.ArgumentParser()
parser.add_argument('label', help='e.g. water_idle, motor_step_left, coast_down')
parser.add_argument('--water', choices=['yes', 'no', 'unknown'], default='unknown')
parser.add_argument('--hull-stationary', choices=['yes', 'no', 'unknown'], default='unknown')
parser.add_argument('--directory', help='Override the configured log directory')
args = parser.parse_args()
root = Path(args.directory if args.directory is not None else load_logger_config()['directory'])
root.mkdir(parents=True, exist_ok=True)
marker = {'experiment_label': args.label, 'water_context': args.water,
          'hull_stationary_observer': args.hull_stationary,
          'wall_ns': time.time_ns(), 'mono_ns': time.monotonic_ns()}
temp = root / 'marker.tmp'
temp.write_text(json.dumps(marker, ensure_ascii=False)+'\n', encoding='utf-8')
os.replace(temp, root / 'marker.json')
print(json.dumps(marker, ensure_ascii=False))
