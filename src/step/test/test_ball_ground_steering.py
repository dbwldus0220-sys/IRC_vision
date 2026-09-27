"""Ground contact steering, legacy head-pose separation, and display tests."""

from dataclasses import fields
import json
from types import SimpleNamespace

import numpy as np
import pytest

from step.ball_analyzer import BallAnalyzer, BallInfo, ball_ground_geometry
from step.ball_navigation_planner import BallNavigationPlanner
from step.yolo26_detector import Yolo26Detector
from step.yolo_line_analyzer import GROUND_PROJECTION_DEFAULTS


def ground_info(angle=20.0, **overrides):
    sample = {
        "detected": True, "confidence": 0.95,
        "ground_projection_enabled": True, "ground_projection_valid": True,
        "ground_projection_scope": "ball_approach_only",
        "ground_coordinate_frame": "robot_x_right_z_forward",
        "ground_steering_angle_deg": angle,
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
    [600, 50, 650, 100], [600, 450, 650, 400],
])
def test_missing_clipped_or_out_of_range_contact_is_invalid(bbox):
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


def test_invalid_ground_contact_does_not_change_close_pickup_entry():
    command = BallNavigationPlanner().plan(
        ground_info(distance_m=0.5, ground_projection_valid=False), 0.1,
    )
    assert command.motion == "PICKUP_NOW"


@pytest.mark.parametrize("head_down", [False, True])
@pytest.mark.parametrize("width,height", [(1280, 720), (640, 360)])
def test_display_distinguishes_ground_approach_from_head_down(
    monkeypatch, head_down, width, height,
):
    detector = object.__new__(Yolo26Detector)
    detector.show_ball_metrics = True
    info = ground_info()
    command = {
        "source": "ball", "action": "WAIT",
        "phase": "BALL_PICKUP_INITIAL_ALIGN" if head_down else "BALL_APPROACH_ALIGN",
        "source_command": {
            "steering_error_deg": -45.0 if head_down else 20.0,
            "steering_source": (
                "head_down_image_angle" if head_down else "ground_steering_angle_deg"
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
    assert rows["Ground steer"].strip() == ("N/A" if head_down else "+20.00deg")
    assert rows["Steering    "].strip() == ("-45.00deg" if head_down else "+20.00deg")
    assert rows["Angle source"].strip() == (
        "IMAGE / HEAD DOWN" if head_down else "GROUND / APPROACH"
    )
    assert all(0 <= origin[1] < height for _, origin in texts)
