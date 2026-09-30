"""Ground contact projection, distance-based steering, and display tests."""

from dataclasses import fields
import json
from types import SimpleNamespace

import numpy as np
import pytest

from step.ball_analyzer import BallAnalyzer, BallInfo, ball_ground_geometry
from step.ball_navigation_planner import BallNavigationPlanner
from step.yolo26_detector import Yolo26Detector
from step.yolo_line_analyzer import GROUND_PROJECTION_DEFAULTS
from step.yolo_line_analyzer import LinePoint, project_line_points_to_ground


def ground_info(angle=20.0, **overrides):
    sample = {
        "detected": True, "confidence": 0.95,
        "ground_projection_enabled": True, "ground_projection_valid": True,
        "ground_projection_scope": "ball_approach_only",
        "ground_coordinate_frame": "robot_x_right_z_forward",
        "ground_steering_angle_deg": angle,
        "ground_forward_distance_m": 0.9,
        "steering_angle_deg": -45.0, "bearing_deg": -30.0,
        "offset_x_norm": 0.0, "offset_x_px": 0,
        "depth_valid": True, "depth_age_sec": 0.01,
        "depth_m": 0.9, "distance_m": 0.9,
    }
    sample.update(overrides)
    return sample


@pytest.mark.parametrize("x", [-0.2, 0.0, 0.2])
def test_contact_projection_and_resolution_scaling(x):
    h = np.asarray(GROUND_PROJECTION_DEFAULTS["ground_homography"]).reshape(3, 3)
    pixel = np.linalg.inv(h) @ [x, 0.6, 1.0]
    u, v = pixel[:2] / pixel[2]
    bbox = [u - 20, v - 40, u + 20, v]
    full = ball_ground_geometry(bbox, 1280, 720, GROUND_PROJECTION_DEFAULTS)
    half = ball_ground_geometry(
        [value / 2 for value in bbox], 640, 360, GROUND_PROJECTION_DEFAULTS,
    )
    assert full["ground_projection_valid"] is True
    assert full["ground_lateral_offset_m"] == pytest.approx(x, abs=1e-6)
    assert full["ground_forward_distance_m"] == pytest.approx(0.6)
    assert full["ground_steering_angle_deg"] == pytest.approx(
        np.rad2deg(np.arctan2(x, 0.6)), abs=0.001,
    )
    assert half["ground_steering_angle_deg"] == full["ground_steering_angle_deg"]
    assert half["ground_lateral_offset_m"] == full["ground_lateral_offset_m"]


@pytest.mark.parametrize("bbox", [
    None, [], [600, 690, 660, 720], [600, 690, 660, 719],
    [-1, 400, 20, 440], [1260, 400, 1280, 440],
    [600, 450, 650, 400],
])
def test_missing_clipped_or_reversed_contact_is_invalid(bbox):
    result = ball_ground_geometry(bbox, 1280, 720, GROUND_PROJECTION_DEFAULTS)
    assert result["ground_projection_valid"] is False
    assert result["ground_steering_angle_deg"] is None
    assert result["ground_forward_distance_m"] is None
    json.dumps(result, allow_nan=False)


def test_publish_adds_ground_fields_without_replacing_depth_or_image_angle():
    analyzer = object.__new__(BallAnalyzer)
    analyzer.confirmation_fields = {}
    analyzer.pickup_confirmation_fields = {}
    published = []
    analyzer.publisher = SimpleNamespace(publish=published.append)
    values = {field.name: None for field in fields(BallInfo)}
    values.update(
        detected=True, bbox=[689, 340, 729, 380],
        image_width=1280, image_height=720,
        depth_m=0.7, distance_m=0.7, steering_angle_deg=12.0,
    )
    analyzer._publish(BallInfo(**values))
    payload = json.loads(published[-1].data)
    assert payload["ground_projection_valid"] is True
    assert payload["depth_m"] == payload["distance_m"] == 0.7
    assert payload["steering_angle_deg"] == 12.0
    values["detected"] = False
    analyzer._publish(BallInfo(**values))
    lost = json.loads(published[-1].data)
    assert lost["ground_projection_valid"] is False
    assert lost["ground_steering_angle_deg"] is None


def test_ground_angle_overrides_image_angle_and_pixel_deadband():
    planner = BallNavigationPlanner()
    info = ground_info(20.0, distance_m=0.60)
    assert planner.approach_steering_error(info) == 20.0
    command = planner.plan(info, 0.1)
    assert command.valid is True
    assert command.angular_speed_rad_s > 0.0
    assert command.to_dict()["steering_error_deg"] == 20.0
    assert command.to_dict()["steering_source"] == "ground_steering_angle_deg"


@pytest.mark.parametrize("overrides", [
    {"ground_projection_valid": False},
    {"ground_projection_enabled": False},
    {"ground_projection_scope": "head_down"},
    {"ground_coordinate_frame": "camera"},
    {"ground_steering_angle_deg": None},
    {"ground_steering_angle_deg": float("nan")},
    {"ground_steering_angle_deg": float("inf")},
    {"ground_steering_angle_deg": True},
    {"ground_steering_angle_deg": 90.0},
])
def test_invalid_ground_metadata_never_falls_back_to_pixel_angle(overrides):
    planner = BallNavigationPlanner()
    info = ground_info(**overrides)
    assert planner.approach_steering_error(info) is None
    assert planner.plan(info, 0.1).valid is False


def test_invalid_ground_contact_cannot_choose_steering_at_pickup_entry():
    command = BallNavigationPlanner().plan(
        ground_info(distance_m=0.5, ground_projection_valid=False), 0.1,
    )
    assert command.valid is False
    assert command.reason == "invalid_ball_alignment"


@pytest.mark.parametrize("pickup", [False, True])
@pytest.mark.parametrize("near", [False, True])
@pytest.mark.parametrize("width,height", [(1280, 720), (640, 360)])
def test_display_reports_distance_based_source_in_all_phases(
    monkeypatch, pickup, near, width, height,
):
    detector = object.__new__(Yolo26Detector)
    detector.show_ball_metrics = True
    info = ground_info(ground_forward_distance_m=0.35 if near else 0.9)
    command = {
        "source": "ball", "action": "WAIT",
        "phase": "BALL_PICKUP_INITIAL_ALIGN" if pickup else "BALL_APPROACH_ALIGN",
        "source_command": {
            "steering_error_deg": -45.0 if near else 20.0,
            "steering_source": (
                "near_image_angle" if near else "ground_steering_angle_deg"
            ),
        },
    }
    monkeypatch.setattr(detector, "_fresh_ball_info", lambda: info)
    monkeypatch.setattr(detector, "_recent_ball_info", lambda: info)
    monkeypatch.setattr(detector, "_fresh_motion_command", lambda: command)
    texts = []
    monkeypatch.setattr(
        "step.yolo26_detector.cv2.putText",
        lambda image, text, origin, *args: texts.append((text, origin)),
    )
    detector._draw_ball_metrics(np.zeros((height, width, 3), dtype=np.uint8))
    rows = dict(text.split(":", 1) for text, _ in texts if ":" in text)
    assert rows["Image angle "].strip() == "-45.0deg"
    assert rows["Ground steer"].strip() == "+20.00deg"
    assert rows["Steering    "].strip() == ("-45.00deg" if near else "+20.00deg")
    assert rows["Angle source"].strip() == (
        "IMAGE / <=35cm" if near else "GROUND / >35cm"
    )
    assert all(0 <= origin[1] < height for _, origin in texts)


@pytest.mark.parametrize("x", [-0.2, 0.0, 0.2])
@pytest.mark.parametrize("z", [1.2, 1.5, 1.8])
def test_far_ball_extrapolates_while_line_range_stays_limited(x, z):
    parameters = dict(GROUND_PROJECTION_DEFAULTS)
    h = np.asarray(parameters["ground_homography"]).reshape(3, 3)
    pixel = np.linalg.inv(h) @ [x, z, 1.0]
    u, v = pixel[:2] / pixel[2]
    bbox = [u - 10, v - 20, u + 10, v]
    result = ball_ground_geometry(bbox, 1280, 720, parameters)
    assert result["ground_projection_valid"] is True
    assert result["ground_forward_distance_m"] == pytest.approx(z)
    assert result["ground_steering_angle_deg"] == pytest.approx(
        np.rad2deg(np.arctan2(x, z)), abs=0.001,
    )
    assert parameters == GROUND_PROJECTION_DEFAULTS
    assert len(project_line_points_to_ground(
        [LinePoint(u, v, 1.0)], 1280, 720, parameters,
    )) == 0
    half = ball_ground_geometry(
        [value / 2 for value in bbox], 640, 360, parameters,
    )
    assert half["ground_steering_angle_deg"] == result["ground_steering_angle_deg"]
    json.dumps(result, allow_nan=False)


def test_video_ball_contact_no_longer_waits_on_line_range_limit():
    # Approximate contact read from the 02:16:24 recording at 1:55.
    result = ball_ground_geometry(
        [624, 122, 664, 155], 1280, 720, GROUND_PROJECTION_DEFAULTS,
    )
    assert result["ground_projection_valid"] is True
    assert result["ground_forward_distance_m"] == pytest.approx(1.141, abs=.001)
    assert result["ground_steering_angle_deg"] == pytest.approx(-5.98, abs=.01)
    info = ground_info(depth_m=1.4, distance_m=1.4, **result)
    command = BallNavigationPlanner().plan(info, .1)
    assert command.valid is True
    assert command.steering_source == "ground_steering_angle_deg"
    assert command.steering_error_deg == result["ground_steering_angle_deg"]
    assert command.depth_m == command.distance_m == 1.4


@pytest.mark.parametrize("homography", [
    [0.] * 9,
    [float("nan")] * 9,
    [1., 0., 0., 0., 0., -1., 0., 1., 0.],  # Behind the robot.
    [1., 0., 0., 0., 0., 1., 0., 1., -400.],  # Zero divisor.
])
def test_extrapolation_still_rejects_invalid_projection(homography):
    result = ball_ground_geometry(
        [600, 350, 650, 400], 1280, 720,
        {**GROUND_PROJECTION_DEFAULTS, "ground_homography": homography},
    )
    assert result["ground_projection_valid"] is False
    assert result["ground_steering_angle_deg"] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("forward,expected,source", [
    (0.20, -45.0, "near_image_angle"),
    (0.349999, -45.0, "near_image_angle"),
    (0.35, -45.0, "near_image_angle"),
    (0.350001, 20.0, "ground_steering_angle_deg"),
    (1.05, 20.0, "ground_steering_angle_deg"),
    (1.8, 20.0, "ground_steering_angle_deg"),
])
@pytest.mark.parametrize("depth", [0.30, 0.9])
def test_switch_uses_ground_forward_not_depth(forward, expected, source, depth):
    info = ground_info(
        ground_forward_distance_m=forward, depth_m=depth, distance_m=depth,
        offset_x_norm=-0.3,
    )
    planner = BallNavigationPlanner()
    assert planner.select_steering(info) == (expected, source)
    assert planner.approach_steering_error(info) == expected
    command = planner.plan(info, 0.1)
    assert command.valid
    if depth > planner.config.pickup_sequence_start_distance_m:
        assert command.steering_error_deg == expected
        assert command.steering_source == source
    else:
        assert command.motion == "PICKUP_NOW"


@pytest.mark.parametrize("forward", [None, True, 0.0, -0.1, float("nan"), float("inf")])
def test_invalid_ground_forward_never_selects_a_distance_branch(forward):
    planner = BallNavigationPlanner()
    info = ground_info(ground_forward_distance_m=forward)
    assert planner.approach_steering_error(info) is None
    assert not planner.plan(info, 0.1).valid


def test_near_image_keeps_original_pixel_deadband_without_requiring_ground_angle():
    planner = BallNavigationPlanner()
    info = ground_info(
        ground_forward_distance_m=0.35, distance_m=0.5,
        ground_steering_angle_deg=None,
    )
    assert planner.select_steering(info) == (0.0, "near_image_angle")
    info["offset_x_norm"] = -0.3
    assert planner.select_steering(info) == (-45.0, "near_image_angle")


def test_crossing_boundary_reselects_angle_each_frame():
    planner = BallNavigationPlanner()
    info = ground_info(offset_x_norm=-0.3)
    for forward, expected in [(0.36, 20.0), (0.35, -45.0), (0.34, -45.0), (0.36, 20.0)]:
        info["ground_forward_distance_m"] = forward
        assert planner.approach_steering_error(info) == expected
