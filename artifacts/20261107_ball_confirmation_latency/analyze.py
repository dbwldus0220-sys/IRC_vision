#!/usr/bin/env python3
"""Read-only offline analysis of recorded STEP vision logs and screen evidence.

Run from any directory. No ROS nodes, model inference, or hardware access.
Video hit counts are manual observations, not a raw detection replay.
"""
import csv
import json
import math
import re
from decimal import Decimal
from pathlib import Path

OUT = Path(__file__).resolve().parent
LOG = Path('/home/jet/.ros/log')
RUNS = {
    '2026-11-07_18-36': ('python3_10200_1794044172359.log', 'python3_10206_1794044171300.log'),
    '2026-11-06_15-31': ('python3_10172_1793946702920.log', 'python3_10176_1793946701820.log'),
}

def stamp_ns(line):
    return int(Decimal(re.search(r'\[(\d+\.\d+)\]', line)[1]) * 1_000_000_000)

def stats(values):
    values = sorted(values)
    if not values:
        return {'count': 0}
    return {'count': len(values), 'mean': sum(values)/len(values),
            'p50': values[math.ceil(len(values)*.5)-1],
            'p95': values[math.ceil(len(values)*.95)-1],
            'min': values[0], 'max': values[-1]}

def write_csv(name, rows):
    if not rows:
        return
    with (OUT/name).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

all_windows, ages, result = [], [], {}
for run, (detector, decision) in RUNS.items():
    windows = []
    for number, line in enumerate((LOG/detector).read_text().splitlines(), 1):
        if '[YOLO_INFERENCE_TIMING]' not in line:
            continue
        pairs = dict(re.findall(r'(\w+)=([\d.]+)', line))
        row = {'run': run, 'log': detector, 'line': number,
               'end_epoch_ns': stamp_ns(line),
               **{k: float(v) for k,v in pairs.items()}}
        row['samples'] = int(row['samples'])
        row['mean_inference_completion_interval_ms'] = row['window_sec']*1000/row['samples']
        windows.append(row)
    all_windows.extend(windows)
    run_ages = []
    for number, line in enumerate((LOG/decision).read_text().splitlines(), 1):
        capture, category = None, None
        received = stamp_ns(line)
        if '[GRASP_VERIFY_FRAME]' in line:
            obj = json.loads(line.split('[GRASP_VERIFY_FRAME] ',1)[1])
            capture = obj['stamp_ns']
            received = obj['callback_receive_ros_ns']
            category = 'grasp_detection_capture_to_receive'
        elif 'BALL entry Line alignment requested:' in line:
            match = re.search(r'ball_rgb_stamp_ns=(\d+)',line)
            if match:
                capture = int(match[1])
                category = 'ball_approach_capture_to_command_log'
        elif 'Pickup alignment decision:' in line:
            match = re.search(r'rgb_stamp_ns=(\d+)',line)
            if match:
                capture = int(match[1])
                category = 'pickup_capture_to_decision_log'
        elif 'Line corner remembered:' in line:
            match = re.search(r"'last_observation_id': (\d+)",line)
            if match:
                capture = int(match[1])
                category = 'line_corner_capture_to_log'
        if capture:
            row = {'run': run, 'log': decision, 'line': number, 'category': category,
                   'capture_epoch_ns': capture, 'log_or_receive_epoch_ns': received,
                   'age_ms': (received-capture)/1e6}
            run_ages.append(row)
    ages.extend(run_ages)
    selected_end = (1794044257 if run.startswith('2026-11-07') else 1793946747)*10**9
    entry_windows = [r for r in windows if selected_end-6*10**9 <= r['end_epoch_ns'] <= selected_end+2*10**9]
    result[run] = {
        'whole_run_inference_count': sum(r['samples'] for r in windows),
        'whole_run_effective_fps': sum(r['samples'] for r in windows)/sum(r['window_sec'] for r in windows),
        'whole_run_weighted_inference_ms': sum(r['samples']*r['avg_ms'] for r in windows)/sum(r['samples'] for r in windows),
        'first_ball_entry_windows': entry_windows,
        'capture_age_ms_by_category': {cat:stats([r['age_ms'] for r in run_ages if r['category']==cat]) for cat in sorted({r['category'] for r in run_ages})},
    }

# Exact event timestamps from the November 7 decision log, checked by literal text.
lines = (LOG/RUNS['2026-11-07_18-36'][1]).read_text().splitlines()
def first_stamp(text):
    return next(stamp_ns(line) for line in lines if text in line)
pending = first_stamp('BALL near/unknown candidate;')
end = first_stamp('Line completed without prequeue: action=STRAIGHT, command_id=5,')
confirmed = first_stamp('BALL mode consumed at confirmation: index=1,')
command = first_stamp('BALL entry Line alignment requested: command_id=15,')
capture = next(r['capture_epoch_ns'] for r in ages if r['run']=='2026-11-07_18-36' and r['category']=='ball_approach_capture_to_command_log')
window = next(r for r in all_windows if r['run']=='2026-11-07_18-36' and 1794044256*10**9 < r['end_epoch_ns'] < 1794044257*10**9)
assert pending < end < capture < confirmed < command
result['first_entry_events'] = {
    'pending_epoch_ns': pending, 'line_complete_epoch_ns': end,
    'confirmed_epoch_ns': confirmed, 'confirmed_frame_capture_epoch_ns': capture,
    'approach_command_epoch_ns': command,
    'candidate_hold_to_confirmation_ms': (confirmed-pending)/1e6,
    'line_end_to_confirmed_frame_capture_ms': (capture-end)/1e6,
    'confirmed_frame_capture_to_confirmation_ms': (confirmed-capture)/1e6,
    'confirmation_to_command_log_ms': (command-confirmed)/1e6,
    'line_end_to_command_log_ms': (command-end)/1e6,
}
result['ten_vs_twelve_estimate_not_replay'] = {
    'assumption': 'Two consecutive successful detections removed, unchanged timing and track; window mean is not individual frame timing.',
    'effective_fps': window['effective_fps'],
    'estimated_gain_ms': 2/window['effective_fps']*1000,
    'estimated_remaining_command_gap_ms': (command-end)/1e6-2/window['effective_fps']*1000,
    'exact_tenth_hit_capture_and_receipt_available': False,
    'full_raw_detection_replay_available': False,
}
write_csv('inference_windows.csv', all_windows)
write_csv('capture_ages.csv', ages)
(OUT/'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(result, ensure_ascii=False, indent=2))
