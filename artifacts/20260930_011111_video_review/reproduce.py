"""Reproduce decision branches without ROS or robot hardware."""
from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'step'))
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from step.line_navigation_planner import LineNavigationPlanner
from step.temporal_confirmation import TemporalConfirmationFilter

for depth in (0.9, 0.6, 0.536):
    info = dict(
        detected=True, confidence=0.9, depth_valid=True, depth_m=depth,
        hurdle_angle_deg=30.0, bottom_distance_px=200,
        camera_center_offset_x_px=200 * math.tan(math.radians(-42.52)),
        go_now=False, path_reference_valid=True, path_offset_x_norm=-0.4,
    )
    result = HurdleNavigationPlanner().plan(info, positioning=True)
    print('synthetic hurdle:', depth, result.action, result.reason,
          'parallel=', result.is_parallel, 'center=', result.center_steering_deg)

# Illustrate the discontinuity; these are synthetic inputs, not recorded topics.
for heading in (9.9, 10.0, 11.9, 12.0, 14.36):
    planner = LineNavigationPlanner()
    print('synthetic line:', heading,
          planner._classify_recovery(-0.349, heading, False))

confirmation = TemporalConfirmationFilter(
    window_size=40, required_hits=18, max_missed_frames=10,
)
box = dict(bbox=[10, 10, 40, 40], image_width=1280, image_height=720)
for _ in range(18):
    result = confirmation.update(True, **box)
assert result.confirmed and result.hit_count == 18
assert not confirmation.update(False).confirmed
assert confirmation.update(True, **box).confirmed
print('confirmation: 18 hits confirm; missing frame is false; matching reacquisition restores')
