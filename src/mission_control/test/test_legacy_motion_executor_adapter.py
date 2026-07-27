import copy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from legacy_motion_executor_adapter import (
    LegacyCommandValidationError,
    build_executor_request,
    map_action_to_motion_id,
    parse_legacy_motion_command,
    timeout_for_motion,
)


@pytest.mark.parametrize(
    ("action", "motion_id"),
    [
        ("STRAIGHT", "forward"),
        ("TURN_LEFT", "turn_left"),
        ("PICKUP_NOW", "pick_ball"),
        ("SHOT", "shoot"),
        ("CROSS_FINISH", "hurdle"),
    ],
)
def test_action_mapping(action, motion_id):
    assert map_action_to_motion_id(action) == motion_id


def test_timeout_mapping():
    assert timeout_for_motion("forward") == 5000
    assert timeout_for_motion("pick_ball") == 10000
    assert timeout_for_motion("shoot") == 10000
    assert timeout_for_motion("hurdle") == 12000
    assert timeout_for_motion("recover") == 8000


def test_build_request_preserves_request_id():
    assert build_executor_request(37, "shoot") == {
        "request_id": 37,
        "motion_id": "shoot",
        "timeout_ms": 10000,
    }


def test_invalid_json():
    with pytest.raises(LegacyCommandValidationError):
        parse_legacy_motion_command("{invalid")


def test_missing_action():
    with pytest.raises(LegacyCommandValidationError):
        parse_legacy_motion_command({"angle_deg": 0.0})


def test_unsupported_action():
    assert map_action_to_motion_id("FLY") is None


def test_parse_does_not_modify_input_object():
    source = {"action": "STRAIGHT", "angle_deg": 12.5}
    original = copy.deepcopy(source)
    command = parse_legacy_motion_command(source)
    assert source == original
    assert command.action == "STRAIGHT"
    assert command.angle_deg == 12.5
