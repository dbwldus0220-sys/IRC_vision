"""Offline comparison with commit 3b90819; no robot I/O.

Source /opt/ros/humble/setup.bash, then run with OPENBLAS_NUM_THREADS=1.
Only callback processing_ms is excluded from output equality checks.
"""
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import sys
import timeit
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src/step"), str(ROOT / "src/step/test")]

import numpy as np
from std_msgs.msg import String
from step import yolo_line_analyzer as current
from step import yolo26_detector as detector_module
from step.line_navigation_planner import LineNavigationPlanner, NavigationConfig
from test_line_work_reduction import make_offline_analyzer



def baseline(name):
    module_name = 'step._baseline_' + name
    source_path = 'src/step/step/' + name + '.py'
    source = subprocess.check_output(['git', 'show', '3b90819:' + source_path], cwd=ROOT, text=True)
    spec = importlib.util.spec_from_loader(module_name, loader=None)
    module = importlib.util.module_from_spec(spec)
    module.__file__ = str(ROOT / source_path)
    sys.modules[module_name] = module
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module


old = baseline('yolo_line_analyzer')
old_detector = baseline('yolo26_detector')
rng = np.random.default_rng(2841)


def message(count, kind='straight', noise=False):
    ys = np.linspace(680., 180., count)
    xs = 710. + (680. - ys) * .05
    if kind == 'curve':
        xs += ((680. - ys) / 35.) ** 2
    elif kind == 'corner':
        xs += np.maximum(450. - ys, 0.) * .8
    elif kind == 'left':
        xs -= np.maximum(450. - ys, 0.) * .8
    if noise:
        xs += rng.normal(0, 30, count)
    detections = [{'class_name':'line','confidence':.9,'center':[x,y],
                   'bbox':[x-5,y-5,x+5,y+5]} for x,y in zip(xs,ys)]
    if noise and count:
        detections += [{'class_name':'line','confidence':.4,'center':[xs[0]+2,ys[0]+2],
                        'bbox':[xs[0]-3,ys[0]-3,xs[0]+7,ys[0]+7]}]
    return String(data=json.dumps({'image_width':1280,'image_height':720,
        'stamp':{'sec':123,'nanosec':456}, 'detections':detections}))


before, before_messages = make_offline_analyzer(old.YoloLineAnalyzer)
after, after_messages = make_offline_analyzer()
planners = [[LineNavigationPlanner(NavigationConfig(heading_source=source)) for _ in range(2)]
            for source in ('image', 'ground')]
clock = [10.]
last_info = None
with patch.object(current.time, 'monotonic', side_effect=lambda: clock[0]):
    for frame in range(600):
        clock[0] += 1./30.
        count = (0, 1, 2, 3, 6, 10, 15, 30)[(frame // 12) % 8]
        kind = ('straight', 'curve', 'corner', 'left')[(frame // 96) % 4]
        msg = message(count, kind, frame % 7 == 0)
        for node in (before, after):
            node.latest_depth_image = None if frame % 41 < 4 else np.full((720,1280),700,dtype=np.uint16)
            node.latest_depth_time = clock[0] - (1. if frame % 29 == 0 else .01)
            node.fx = node.fy = 600.
            node.cx, node.cy = 640., 360.
            node._detections_callback(msg)
        left = json.loads(before_messages.pop().data)
        right = json.loads(after_messages.pop().data)
        left.pop('processing_ms')
        right.pop('processing_ms')
        assert left == right, (frame, {k:(left[k], right[k]) for k in left if left[k] != right[k]})
        for first, second in planners:
            assert first.plan(left, 1./30.).to_dict() == second.plan(right, 1./30.).to_dict(), frame
        if right['detected']:
            last_info = right


def benchmark(before, after, number=50):
    times = [[], []]
    for turn in range(5):
        for index in ((0,1) if turn % 2 == 0 else (1,0)):
            times[index].append(timeit.timeit((before,after)[index], number=number) / number * 1000.)
    a,b = [statistics.median(values) for values in times]
    return {'before_ms':a,'after_ms':b,'reduction_percent':100*(a-b)/a}


results = {'identical_line_frames':600, 'identical_planner_commands':1200,
           'normalization':'Only processing_ms removed; simulated monotonic timestamps identical.',
           'line_callbacks':{}}
with patch.object(current.time, 'monotonic', return_value=100.):
    for count in (6,15,30):
        a, am = make_offline_analyzer(old.YoloLineAnalyzer)
        b, bm = make_offline_analyzer()
        sample = message(count)
        def call_a():
            a._detections_callback(sample)
            am.clear()
        def call_b():
            b._detections_callback(sample)
            bm.clear()
        for _ in range(5):
            call_a(); call_b()
        results['line_callbacks'][str(count)] = benchmark(call_a,call_b)

node = object.__new__(detector_module.Yolo26Detector)
node.input_width = node.input_height = 640
image = rng.integers(0,256,(720,1280,3),dtype=np.uint8)
results['preprocess_1280x720_to_640x640'] = benchmark(
    lambda: old_detector.Yolo26Detector._preprocess(node,image),
    lambda: node._preprocess(image))
node.show_line_metrics = True
node._fresh_motion_command = lambda: None
node._fresh_line_info = lambda: last_info
before_image, after_image = image.copy(), image.copy()
old_detector.Yolo26Detector._draw_line_metrics(node,before_image)
node._draw_line_metrics(after_image)
assert np.array_equal(before_image,after_image)
results['identical_full_line_overlay'] = True
results['line_overlay_1280x720'] = benchmark(
    lambda: old_detector.Yolo26Detector._draw_line_metrics(node, image.copy()),
    lambda: node._draw_line_metrics(image.copy()))
print(json.dumps(results,indent=2))
