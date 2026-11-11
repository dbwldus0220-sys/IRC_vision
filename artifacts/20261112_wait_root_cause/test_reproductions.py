"""Characterize current WAIT paths; passing tests reproduce issues, not fixes.

Run with ROS Python libraries and src/{step,mission_control} plus their test
directories on PYTHONPATH. No ROS graph, camera, or motor node is constructed.
"""

import ast
from pathlib import Path

import numpy as np

from mission_control.ball_loss_confirmation import BallLossConfirmation
from mission_control.goal_initial_search import GoalInitialSearch
from mission_control.motion_decision_planner import MotionDecision
from step.depth_frame_cache import DepthFrameCache
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from step.temporal_confirmation import TemporalConfirmationFilter
from test_goal_backboard_selection import detection, make_analyzer, send


ROOT = Path(__file__).resolve().parents[2]


def confirmation(name):
    tree = ast.parse((ROOT / "src/step/step" / f"{name}_analyzer.py").read_text())
    defaults = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter" and len(node.args) == 2
                and all(isinstance(arg, ast.Constant) for arg in node.args)):
            defaults[node.args[0].value] = node.args[1].value
    return TemporalConfirmationFilter(**{
        key: defaults[f"confirmation_{key}"] for key in (
            "window_size", "required_hits", "max_missed_frames",
            "max_center_shift_norm", "min_area_ratio")})


def test_goal_visible_rgb_with_missing_depth_is_reported_as_no_raw_detection():
    analyzer, messages = make_analyzer(depth_m=None, required_hits=10)
    for _ in range(30):
        result = send(analyzer, messages, [detection(confidence=.99)])
    assert result["note"] == "no_goal_detection"
    assert result["raw_detected"] is False
    assert result["confirmation_hits"] == 0


def test_goal_three_missing_depth_frames_erase_confirmed_track():
    analyzer, messages = make_analyzer(depth_m=1.0, required_hits=10)
    analyzer.confirmation_filter = confirmation("goal")
    for _ in range(10):
        result = send(analyzer, messages, [detection()])
    assert result["detected"] is True
    analyzer._sample_goal_depth_m = lambda _bbox: (None, False, 0)
    for _ in range(3):
        result = send(analyzer, messages, [detection()])
    assert result["confirmation_hits"] == 0
    analyzer._sample_goal_depth_m = lambda _bbox: (1.0, True, 5)
    for _ in range(9):
        result = send(analyzer, messages, [detection()])
        assert result["detected"] is False
    assert send(analyzer, messages, [detection()])["detected"] is True


def test_goal_repeated_nine_good_three_missing_depth_frames_never_confirm():
    analyzer, messages = make_analyzer(depth_m=1.0, required_hits=10)
    analyzer.confirmation_filter = confirmation("goal")
    for _ in range(20):
        for valid, count in ((True, 9), (False, 3)):
            analyzer._sample_goal_depth_m = lambda _bbox, good=valid: (
                (1.0, True, 5) if good else (None, False, 0))
            for _ in range(count):
                assert send(analyzer, messages, [detection()])["detected"] is False


def test_one_missing_frame_immediately_suppresses_an_already_confirmed_target():
    for name in ("ball", "goal", "hurdle"):
        filt = confirmation(name)
        box = dict(bbox=[500, 200, 600, 300], image_width=1280, image_height=720)
        for _ in range(filt.required_hits):
            result = filt.update(True, **box)
        assert result.confirmed
        assert filt.update(False).confirmed is False
        # Track memory survives, but the current motion candidate did not.
        assert filt.update(True, **box).confirmed is True


def test_area_change_resets_confirmed_track_even_with_unchanged_center():
    for name in ("ball", "goal", "hurdle"):
        filt = confirmation(name)
        for _ in range(filt.required_hits):
            result = filt.update(True, bbox=[500, 200, 600, 300],
                                 image_width=1280, image_height=720)
        assert result.confirmed
        result = filt.update(True, bbox=[520, 220, 580, 280],
                             image_width=1280, image_height=720)
        assert result.hit_count == 1 and not result.confirmed


def test_ball_raw_flicker_prevents_loss_confirmation_for_ten_seconds():
    loss = BallLossConfirmation()
    for index in range(100):
        loss.update(dict(rgb_stamp_ns=1_000_000_000 + index * 100_000_000,
                         detected=False, raw_detected=index % 7 == 6),
                    received_at=index / 10, max_gap_sec=.5)
        assert not loss.confirmed
    # Continuous, fresh misses eventually release the gate.
    for index in range(100, 111):
        loss.update(dict(rgb_stamp_ns=1_000_000_000 + index * 100_000_000,
                         detected=False, raw_detected=False),
                    received_at=index / 10, max_gap_sec=.5)
    assert loss.confirmed


def test_hurdle_interior_observation_erases_edge_then_loss_has_no_exit():
    planner = HurdleNavigationPlanner()
    planner.begin_near("RIGHT")
    visible = dict(detected=True, confirmation_confirmed=True, confidence=.9,
                   center_y=300, bottom_distance_px=50)
    planner.observe(visible)
    assert planner.last_seen_side == "BOTTOM"
    planner.observe({**visible, "bottom_distance_px": 300})
    assert planner.last_seen_side is None
    missing = dict(detected=False, raw_detected=False,
                   hurdle_camera_ready=True, hurdle_camera_pose="0")
    for _ in range(300):
        assert planner.plan(missing, positioning=True).reason == "hurdle_loss_direction_unavailable"
    # An entry line direction alone does not release this branch: TOP is required.
    assert planner.entry_line_side == "RIGHT"


def test_goal_initial_search_candidate_has_no_elapsed_time_exit():
    search = GoalInitialSearch()
    search.arm()
    decision = MotionDecision(phase="GOAL_APPROACH", source="goal", action="WAIT",
        valid=False, reason="goal_not_detected", sdk_motion_requested=False,
        requires_ack=False, source_command={})
    for _ in range(300):
        result = search.select(decision, dict(detected=False, raw_detected=True))
        assert result.reason == "goal_initial_search_waiting_for_confirmation"
    assert search.step == 0


def test_goal_samples_newest_depth_instead_of_depth_matching_rgb():
    analyzer, _ = make_analyzer()
    cache = DepthFrameCache()
    # A target moves between capture and processing. New depth at the old RGB
    # target location is background, despite a matching RGB-time frame existing.
    old = np.full((720, 1280), 1000, dtype=np.uint16)
    new = np.full((720, 1280), 3000, dtype=np.uint16)
    cache.update(old, width=1280, height=720, stamp_ns=1_000_000_000)
    cache.update(new, width=1280, height=720, stamp_ns=1_100_000_000)
    analyzer._depth_cache = cache
    analyzer._active_rgb_stamp_ns = 1_000_000_000
    analyzer.depth_window_px = 5
    # Use the actual sampler, not the fixture's depth stub.
    patch = analyzer._valid_patch_m(640, 260)
    assert np.allclose(patch, 3.0)
    matching, delta = cache.nearest(analyzer._active_rgb_stamp_ns)
    assert delta == 0 and matching.image[260, 640] == 1000
