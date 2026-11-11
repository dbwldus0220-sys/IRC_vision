"""Compare against backup 125497e; source /opt/ros/humble/setup.bash first.

Run with OPENBLAS_NUM_THREADS=1 for the recorded benchmark configuration.
No ROS nodes, model inference, camera or robot hardware are started.
"""

import importlib.util
import json
import statistics
import subprocess
from pathlib import Path
import sys
import timeit
from dataclasses import asdict
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src/step"), str(ROOT / "src/step/test")]

import cv2
import numpy as np
from step import yolo26_detector as current_yolo
from step import goal_analyzer as current_goal
from test_goal_backboard_selection import make_analyzer, detection


def baseline(name):
    module_name = 'step._baseline_' + name
    source_path = 'src/step/step/' + name + '.py'
    source = subprocess.check_output(
        ['git', 'show', '125497e:' + source_path], cwd=ROOT, text=True,
    )
    spec = importlib.util.spec_from_loader(module_name, loader=None)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    module.__file__ = str(ROOT / source_path)
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module


old_yolo = baseline('yolo26_detector')
old_goal = baseline('goal_analyzer')
rng = np.random.default_rng(861)
images = []
for index in range(160):
    h, w = (720, 1280) if index % 8 == 0 else (240, 320)
    image = np.zeros((h, w, 3), dtype=np.uint8)
    for _ in range(index % 17):
        p1 = tuple(int(v) for v in rng.integers([0, 0], [w, h]))
        p2 = tuple(int(v) for v in rng.integers([0, 0], [w, h]))
        cv2.line(image, p1, p2, (0, 220, 255), int(rng.integers(1, 25)))
    left, top = rng.integers([-30, -20], [max(1, w // 3), max(1, h // 3)])
    right, bottom = rng.integers([w // 2, h // 2], [w + 30, h + 20])
    bbox = [int(v) for v in (left, top, right, bottom)]
    before = old_yolo.extract_hurdle_bar_centerline(image, bbox)
    after = current_yolo.extract_hurdle_bar_centerline(image, bbox)
    assert before == after, (index, before, after)
    images.append((image, bbox))

comparisons = 0
for depth in (None, 0.4, 0.66, 2.0):
    for board_depth in (None, 0.41, 0.60):
        for offset in (-350, 0, 350):
            for calibration in (0., 96.):
                for missing in (None, 'fx', 'fy', 'cx', 'cy'):
                    analyzer, _ = make_analyzer(depth, robot_center_offset_px=calibration)
                    analyzer._sample_backboard_line_depths = lambda *_: (board_depth, board_depth, board_depth, 3, [[550, 222], [640, 222], [730, 222]])
                    if missing:
                        setattr(analyzer, missing, None)
                    item = detection(center_x=640 + offset)
                    before = asdict(old_goal.GoalAnalyzer._build_candidate(analyzer, item, 1280, 720))
                    after = asdict(current_goal.GoalAnalyzer._build_candidate(analyzer, item, 1280, 720))
                    assert before == after, (depth, board_depth, offset, calibration, missing)
                    comparisons += 1

header = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=20), frame_id='camera')
message = SimpleNamespace(header=header, width=1280, height=720)
items = [current_yolo.Detection(i % 6, current_yolo.DEFAULT_CLASS_NAMES[i % 6], 0.85, [20, 30, 70, 80], [45, 55]) for i in range(30)]
node = object.__new__(current_yolo.Yolo26Detector)
outputs = []
node.detections_publisher = SimpleNamespace(publish=lambda message: outputs.append(message.data))
old_yolo.Yolo26Detector._publish_detections(node, message, items)
current_yolo.Yolo26Detector._publish_detections(node, message, items)
assert outputs[0] == outputs[1]
node.detections_publisher = SimpleNamespace(publish=lambda message: None)


def benchmark(before, after, number):
    results = [[], []]
    for turn in range(7):
        for i in ((0, 1) if turn % 2 == 0 else (1, 0)):
            results[i].append(timeit.timeit((before, after)[i], number=number) / number * 1000)
    b, a = map(statistics.median, results)
    return {'before_ms': b, 'after_ms': a, 'reduction_percent': 100 * (b-a)/b}

# A large loose bbox with one long bar and separate yellow distractors.
image = np.zeros((720, 1280, 3), dtype=np.uint8)
cv2.line(image, (180, 490), (1100, 340), (0, 220, 255), 24)
for x in range(40, 1240, 120):
    cv2.rectangle(image, (x, 100), (x + 25, 125), (0, 220, 255), -1)
bbox = [0, 0, 1280, 720]
assert old_yolo.extract_hurdle_bar_centerline(image, bbox) == current_yolo.extract_hurdle_bar_centerline(image, bbox)
analyzer, _ = make_analyzer(.66, robot_center_offset_px=96.)
item = detection(center_x=740)
result = {'identical_hurdle_cases': len(images) + 1, 'identical_goal_candidates': comparisons,
          'identical_detection_json': True,
          'benchmark_hurdle_loose_bbox': benchmark(lambda: old_yolo.extract_hurdle_bar_centerline(image, bbox), lambda: current_yolo.extract_hurdle_bar_centerline(image, bbox), 20),
          'benchmark_detection_json_30_boxes': benchmark(lambda: old_yolo.Yolo26Detector._publish_detections(node, message, items), lambda: current_yolo.Yolo26Detector._publish_detections(node, message, items), 500),
          'benchmark_goal_candidate': benchmark(lambda: old_goal.GoalAnalyzer._build_candidate(analyzer, item, 1280, 720), lambda: current_goal.GoalAnalyzer._build_candidate(analyzer, item, 1280, 720), 500)}
print(json.dumps(result, indent=2))
