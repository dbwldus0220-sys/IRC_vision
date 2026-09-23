"""Backboard target selection without a required goal detection."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from step.goal_analyzer import GoalAnalyzer
from step.goal_navigation_planner import GoalNavigationPlanner
from step.temporal_confirmation import TemporalConfirmationFilter


def make_analyzer(depth_m=0.4, required_hits=1, robot_center_offset_px=0.0):
    analyzer = object.__new__(GoalAnalyzer)
    analyzer.goal_class_name = "goal"
    analyzer.backboard_class_name = "backboard"
    analyzer.prefer_backboard_center = True
    analyzer.robot_center_offset_px = robot_center_offset_px
    analyzer.min_confidence = 0.55
    analyzer.max_valid_depth_m = 6.0
    analyzer.detect_depth_m = 2.0
    analyzer.approach_depth_m = 0.5
    analyzer.score_target_depth_m = 0.795
    analyzer.score_depth_tolerance_m = 0.025
    analyzer.score_left_bound_px = -40.0
    analyzer.score_right_bound_px = 100.0
    analyzer.direction_deadband_norm = 0.04
    analyzer.publish_empty_when_missing = True
    analyzer.fx = analyzer.fy = 600.0
    analyzer.cx = 640.0
    analyzer.cy = 360.0
    analyzer.latest_image_width = 1280
    analyzer.latest_image_height = 720
    analyzer._sample_goal_depth_m = lambda bbox: (
        depth_m, depth_m is not None, 5 if depth_m is not None else 0
    )
    analyzer._depth_age_sec = lambda: 0.0
    analyzer.confirmation_filter = TemporalConfirmationFilter(
        window_size=40, required_hits=required_hits, max_missed_frames=2
    )
    analyzer.score_confirmation_filter = TemporalConfirmationFilter(
        window_size=5, required_hits=3, max_missed_frames=0,
        spatial_matching=False,
    )
    analyzer.confirmation_fields = {}
    analyzer.score_confirmation_fields = {}
    published = []
    analyzer.publisher = SimpleNamespace(publish=published.append)
    return analyzer, published


def detection(class_name="backboard", center_x=640, confidence=0.9):
    return {
        "class_name": class_name,
        "confidence": confidence,
        "bbox": [center_x - 80, 200, center_x + 80, 320],
        "center": [center_x, 260],
    }


def send(analyzer, published, detections):
    analyzer._detections_callback(String(data=json.dumps({
        "image_width": 1280, "image_height": 720,
        "detections": detections,
    })))
    return json.loads(published[-1].data)


@pytest.mark.parametrize(
    "center_x,direction,action",
    [(450, "LEFT", "GOAL_CAMERA90_CRAB_LEFT"), (830, "RIGHT", "GOAL_CAMERA90_CRAB_RIGHT")],
)
def test_standalone_backboard_center_reaches_navigation(center_x, direction, action):
    analyzer, published = make_analyzer()
    board = detection(center_x=center_x)
    info = send(analyzer, published, [board])

    assert info["detected"] is True
    assert info["aim_source"] == "backboard"
    assert info["center_x"] == center_x
    assert info["aim_bbox"] == board["bbox"]
    assert info["horizontal_direction"] == direction
    assert info["confidence"] == board["confidence"]
    assert GoalNavigationPlanner().plan(info).action == action


def test_goal_alone_is_not_a_target():
    analyzer, published = make_analyzer()
    info = send(analyzer, published, [detection("goal")])
    assert info["detected"] is False
    assert info["candidate_count"] == 0
    assert info["score_now"] is False


def test_goal_associated_board_is_preferred_and_uses_board_geometry():
    analyzer, published = make_analyzer()
    goal = detection("goal", center_x=400, confidence=0.95)
    goal["bbox"] = [250, 100, 600, 650]
    associated = detection(center_x=450, confidence=0.60)
    standalone = detection(center_x=640, confidence=0.99)
    sampled = []

    def sample_depth(bbox):
        sampled.append(bbox)
        return 0.4, True, 5

    analyzer._sample_goal_depth_m = sample_depth
    info = send(analyzer, published, [goal, standalone, associated])

    assert info["center_x"] == 450
    assert info["bbox"] == associated["bbox"]
    assert info["confidence"] == 0.60
    assert info["candidate_count"] == 2
    assert goal["bbox"] not in sampled
    assert associated["bbox"] in sampled


def test_unusable_associated_board_does_not_block_valid_standalone_board():
    analyzer, published = make_analyzer()
    associated = detection(center_x=400)
    analyzer._sample_goal_depth_m = lambda bbox: (
        (None, False, 0) if bbox == associated["bbox"] else (0.4, True, 5)
    )
    info = send(analyzer, published, [
        detection("goal", center_x=400), associated, detection(),
    ])
    assert info["detected"] is True
    assert info["center_x"] == 640
    assert info["candidate_count"] == 1


@pytest.mark.parametrize("confidence", [0.54, float("nan"), float("inf"), 1.1])
def test_invalid_backboard_confidence_is_not_rescued_by_goal(confidence):
    analyzer, published = make_analyzer()
    info = send(analyzer, published, [
        detection("goal"), detection(confidence=confidence),
    ])
    assert info["detected"] is False
    assert info["score_now"] is False


@pytest.mark.parametrize("depth_m", [None, 2.01, 0.0])
def test_standalone_board_keeps_depth_gate(depth_m):
    analyzer, published = make_analyzer(depth_m=depth_m)
    info = send(analyzer, published, [detection()])
    assert info["detected"] is False
    assert info["score_now"] is False
    assert GoalNavigationPlanner().plan(info).valid is False


@pytest.mark.parametrize("depth_m", [0.795, 1.5, 2.0])
def test_backboard_depth_is_published_up_to_two_meters(depth_m):
    analyzer, published = make_analyzer(depth_m=depth_m)
    for _ in range(3):
        info = send(analyzer, published, [detection()])
    assert info["detected"] is True
    assert info["depth_valid"] is True
    assert info["depth_m"] == depth_m
    assert info["score_now"] is (depth_m == 0.795)
    if depth_m > 1.0:
        assert GoalNavigationPlanner().plan(info).valid is False


def test_goal_dropout_preserves_backboard_confirmation():
    analyzer, published = make_analyzer(required_hits=30)
    board = detection()
    for index in range(29):
        detections = [detection("goal"), board] if index < 15 else [board]
        info = send(analyzer, published, detections)
        assert info["detected"] is False
        assert info["note"] == "goal_confirmation_pending"
    info = send(analyzer, published, [board])
    assert info["detected"] is True
    assert info["confirmation_hits"] == 30
    assert info["candidate_count"] == 1


def test_goal_cannot_keep_lost_backboard_active():
    analyzer, published = make_analyzer()
    assert send(analyzer, published, [detection()])["detected"] is True
    info = send(analyzer, published, [detection("goal")])
    assert info["detected"] is False
    assert info["score_now"] is False


def test_standalone_board_keeps_separate_score_confirmation():
    analyzer, published = make_analyzer(depth_m=0.795)
    for _ in range(2):
        info = send(analyzer, published, [detection()])
        assert info["detected"] is True
        assert info["score_now"] is False
        assert GoalNavigationPlanner().plan(info).action == "WAIT_SCORE_CONFIRMATION"
    info = send(analyzer, published, [detection()])
    assert info["score_now"] is True
    assert GoalNavigationPlanner().plan(info).action == "SHOT"


def test_disabling_pair_preference_still_targets_backboards_only():
    analyzer, published = make_analyzer()
    analyzer.prefer_backboard_center = False
    info = send(analyzer, published, [
        detection("goal", center_x=400),
        detection(center_x=400, confidence=0.60),
        detection(confidence=0.99),
    ])
    assert info["center_x"] == 640
    assert info["aim_source"] == "backboard"
    assert send(analyzer, published, [detection("goal")])["detected"] is False


@pytest.mark.parametrize("goal_confidence", [0.54, float("nan")])
def test_unreliable_goal_does_not_give_board_priority(goal_confidence):
    analyzer, published = make_analyzer()
    info = send(analyzer, published, [
        detection("goal", center_x=400, confidence=goal_confidence),
        detection(center_x=400, confidence=0.60),
        detection(confidence=0.99),
    ])
    assert info["center_x"] == 640


def test_calibrated_backboard_center_agrees_with_yaw_and_scoring():
    analyzer, published = make_analyzer(depth_m=0.795, robot_center_offset_px=96.0)
    analyzer.cx = 650.0
    for _ in range(3):
        info = send(analyzer, published, [detection(center_x=736)])
    assert info["robot_center_x_px"] == 736.0
    assert info["robot_center_offset_px"] == 96.0
    assert info["offset_x_px"] == 0
    assert info["offset_x_norm"] == 0.0
    assert info["bearing_deg"] == 0.0
    assert info["lateral_offset_m"] == 0.0
    assert info["depth_m"] == 0.795
    assert info["score_now"] is True
    assert GoalNavigationPlanner().plan(info).action == "SHOT"


def test_optical_image_center_is_left_of_calibrated_goal_axis():
    analyzer, published = make_analyzer(robot_center_offset_px=96.0)
    info = send(analyzer, published, [detection(center_x=640)])
    assert info["offset_x_px"] == -96
    assert info["offset_x_norm"] == -0.15
    assert info["bearing_deg"] < 0.0
    assert info["horizontal_direction"] == "LEFT"
    assert info["is_centered"] is False
    assert GoalNavigationPlanner().plan(info).action == "GOAL_CAMERA90_CRAB_LEFT"


@pytest.mark.parametrize('width,offset,expected', [
    (1280, 70.0, 710.0), (640, 96.0, 416.0),
    (640, 1000.0, 639.0), (640, -1000.0, 0.0),
])
def test_goal_reference_matches_ball_pixel_calibration(width, offset, expected):
    analyzer, published = make_analyzer(robot_center_offset_px=offset)
    board = detection(center_x=int(expected))
    candidate = analyzer._build_candidate(board, width, 720)
    assert candidate.offset_x_px == 0
    assert candidate.bearing_deg == 0.0


def test_pending_goal_publishes_calibrated_axis_before_confirmation():
    analyzer, published = make_analyzer(required_hits=30, robot_center_offset_px=96.0)
    info = send(analyzer, published, [detection(center_x=736)])
    assert info["detected"] is False
    assert info["robot_center_x_px"] == 736.0
    assert info["robot_center_offset_px"] == 96.0


@pytest.mark.parametrize('depth', [0.77, 0.795, 0.82])
@pytest.mark.parametrize('offset', [-40, 0, 100])
def test_analyzer_confirms_new_scoring_rectangle(depth, offset):
    analyzer, published = make_analyzer(depth_m=depth, robot_center_offset_px=96.0)
    for index in range(3):
        info = send(analyzer, published, [detection(center_x=736 + offset)])
        assert info['score_now'] is (index == 2)
    assert info['offset_x_px'] == offset
    assert info['depth_in_score_range'] is True
    assert GoalNavigationPlanner().plan(info).action == 'SHOT'


@pytest.mark.parametrize('depth,offset', [
    (0.769, 0), (0.821, 0), (0.795, -41), (0.795, 101),
])
def test_analyzer_rejects_outside_new_scoring_rectangle(depth, offset):
    analyzer, published = make_analyzer(depth_m=depth, robot_center_offset_px=96.0)
    for _ in range(5):
        info = send(analyzer, published, [detection(center_x=736 + offset)])
        assert info['score_now'] is False
