import json
import math
import time
import numpy as np
from step.ball_analyzer import ball_ground_geometry
from step.ball_navigation_planner import BallNavigationPlanner
from step.yolo_line_analyzer import GROUND_PROJECTION_DEFAULTS
from step.line_navigation_planner import LineNavigationPlanner
from step.yolo26_detector import Yolo26Detector
from mission_control.motion_decision_planner import MotionDecisionPlanner

# Approximate box read from the video, not a recorded ROS message.
bbox = [624, 122, 664, 155]
h = np.asarray(GROUND_PROJECTION_DEFAULTS['ground_homography']).reshape(3, 3)
p = h @ np.asarray([644, 155, 1])
geometry = ball_ground_geometry(bbox, 1280, 720, GROUND_PROJECTION_DEFAULTS)
ball = dict(detected=True, confidence=.83, distance_m=1.4, depth_m=1.4,
            depth_valid=True, depth_age_sec=0.0, offset_x_norm=-.144,
            bearing_deg=-.3, steering_angle_deg=-9, **geometry)
b = BallNavigationPlanner().plan(ball, .1)
assert b.reason == 'invalid_ball_alignment'
print(json.dumps({'ball_case': 'approximate video box', 'unfiltered_ground_z_m': round(p[1]/p[2], 3),
                  'max_ground_z_m': GROUND_PROJECTION_DEFAULTS['ground_fit_max_forward_m'],
                  'ground_projection_valid': geometry['ground_projection_valid'], 'reason': b.reason}))
line = dict(detected=True, ground_projection_valid=True, ground_heading_error_deg=29.45,
            filtered_lateral_offset_norm=-.148, lateral_offset_norm=.368,
            heading_quality=.864, geometry_quality=.864, detection_quality=.864,
            turn_angle_deg=59.3, turn_consistency=1., corner_direction='RIGHT',
            center_points_px=[[550,700],[1000,400]], image_width=1280)
lp = LineNavigationPlanner()
commands = [lp.plan(line,.1) for _ in range(3)]
assert [c.motion for c in commands] == ['STRAIGHT','STRAIGHT','RIGHT']
print(json.dumps({'synthetic_reliable_right_curve': [(c.motion,c.reason) for c in commands]}))
planner=MotionDecisionPlanner()
planner.observe_line_for_search(line)
lost=planner._plan_source('line',dict(detected=False,corner_direction='RIGHT'),.1)
assert lost['motion']=='LINE_LOST_TURN_LEFT'
print(json.dumps({'synthetic_near_point_left_and_corner_right': lost}))
for action,alias,expected in [
 ('RIGHT','line_recovery_right_4','RIGHT'),
 ('LEFT','line_recovery_left_4','LEFT'),
 ('RECOVER_RIGHT_TURN_RIGHT_4','line_recovery_right_4','LINE RETURN RIGHT 4'),
 ('BALL_APPROACH_RECOVER_LEFT_4','line_recovery_left_4','LINE RETURN LEFT 4'),
 ('LINE_LOST_TURN_LEFT','line_search_left_2','LINE SEARCH / IN-PLACE TURN LEFT 2'),
 ('RIGHT','line_search_left_2','LINE SEARCH / IN-PLACE TURN LEFT 2'),
]:
 detector=object.__new__(Yolo26Detector)
 detector.latest_running_motion=dict(action=action,motion_id=alias,source='line')
 detector.latest_running_motion_time=time.monotonic()
 label=detector._running_motion_banner()[0]
 assert label==expected, (action,label,expected)
 print(json.dumps({'action':action,'alias':alias,'banner':label}))
for side in ('LEFT','RIGHT'):
 assert Yolo26Detector._hurdle_line_tracking_label(
  dict(detected=True,confirmation_confirmed=True,depth_valid=True,depth_m=.7),'line',side) is None
print('Corner banner checks passed; normal recovery and lost-search labels preserved.')
