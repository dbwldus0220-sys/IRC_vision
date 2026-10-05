"""Verify the archived catalog6 import before the catalog7 replacement."""

import copy
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261004_catalog6_geon_motion_update"
SNAPSHOT = ROOT / "artifacts/20261004_catalog7_left_turn_update"
CATALOG = json.loads((SNAPSHOT / "robot_motions_runtime.before.json").read_text())
MOTIONS = {m["name"]: m for m in CATALOG["motions"]}
SOURCE = json.loads((UPDATE / "source_requested_motions.json").read_text())
ALIASES = yaml.safe_load((SNAPSHOT / "motion_aliases.before.yaml").read_text())["motion_aliases"]


@pytest.mark.parametrize("supplied", SOURCE["motions"], ids=lambda m: m["name"])
def test_supplied_motions_preserve_all_data_except_runtime_policy(supplied):
    expected = copy.deepcopy(supplied)
    expected["completion"]["position_tolerance_deg"] = 5.0
    for frame in expected["frames"]:
        frame["angles"].update({"4": 18.0, "5": -18.0})
    assert MOTIONS[expected["name"]] == expected


@pytest.mark.parametrize("camera", [45, 90])
@pytest.mark.parametrize("count", [2, 4, 6, 8])
def test_forward_repeats_count_one_cycle_four_frame_gait(camera, count):
    motion = MOTIONS[f"건전진{camera}도({count}회)"]
    assert len(motion["frames"]) == 4
    assert motion["repeat_count"] == count
    assert motion["playback_speed"] == 1.0


@pytest.mark.parametrize("alias,name", [
    ("post_ball_camera_90", "건오뒤카메라90도"),
    ("sdk_forward_4", "건전진45도(4회)"),
    ("line_forward_2", "건전진45도(2회)"),
    ("line_forward_4", "건전진45도(4회)"),
    ("line_forward_6", "건전진45도(6회)"),
    ("line_forward_8", "건전진45도(8회)"),
    ("forward", "건전진45도(8회)"),
    ("ball_camera_down_forward_2", "건전진45도(2회)"),
    ("ball_camera_down_forward_4", "건전진45도(4회)"),
    ("post_ball_forward_4", "건전진45도(4회)"),
    ("post_ball_forward_6", "건전진45도(6회)"),
    ("post_ball_forward_8", "건전진45도(8회)"),
    ("goal_camera_90_forward_2", "건전진90도(2회)"),
    ("goal_camera_90_forward_4", "건전진90도(4회)"),
    ("goal_camera_90_forward_6", "건전진90도(6회)"),
    ("pickup_crab_right_0", "유미세오옆꽃게0도"),
    ("pickup_crab_left_0", "유미세왼옆꽃게0도"),
    ("goal_camera_90_crab_right", "유미세오옆꽃게90도"),
    ("goal_camera_90_crab_left", "유미세왼옆꽃게90도"),
    ("line_recovery_left_4", "건라인복귀좌회전45도(4회)"),
    ("line_recovery_left_6", "건라인복귀좌회전45도(6회)"),
    ("line_recovery_right_4", "건라인복귀우회전45도(4회)"),
])
def test_existing_command_aliases_resolve_to_new_motions(alias, name):
    assert ALIASES[alias] == name
    assert name in MOTIONS


def test_left_six_cycle_derivative_changes_only_name_and_repeat_count():
    expected = copy.deepcopy(MOTIONS["건라인복귀좌회전45도(4회)"])
    assert expected["repeat_count"] == 4
    expected.update(name="건라인복귀좌회전45도(6회)", repeat_count=6)
    assert MOTIONS[expected["name"]] == expected


def test_partial_replacement_preserves_unrelated_motions_and_alias_ids():
    before = json.loads((UPDATE / "robot_motions_runtime.before.json").read_text())
    manifest = json.loads((UPDATE / "manifest.json").read_text())
    renames = manifest["renames"]
    assert len(before["motions"]) == 83
    assert len(MOTIONS) == len(CATALOG["motions"]) == 84
    assert manifest["added_motions"] == ["건라인복귀좌회전45도(1회)"]
    assert not set(renames) & MOTIONS.keys()
    for motion in before["motions"]:
        if motion["name"] not in renames:
            assert MOTIONS[motion["name"]] == motion
    before_aliases = yaml.safe_load((UPDATE / "motion_aliases.before.yaml").read_text())["motion_aliases"]
    assert ALIASES == {alias: renames.get(name, name) for alias, name in before_aliases.items()}
    assert set(ALIASES.values()) <= MOTIONS.keys()
