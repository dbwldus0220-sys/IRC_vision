"""Verify catalog8 and its transition wiring before the catalog9 import."""

import copy
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261005_catalog8_geon_motion_update"
SNAPSHOT = ROOT / "artifacts/20261005_catalog9_right_turn_update"
CATALOG = json.loads((SNAPSHOT / "robot_motions_runtime.before.json").read_text())
MOTIONS = {m["name"]: m for m in CATALOG["motions"]}
SOURCE = json.loads((UPDATE / "source_requested_motions.json").read_text())
MANIFEST = json.loads((UPDATE / "manifest.json").read_text())
ALIASES = yaml.safe_load(
    (SNAPSHOT / "motion_aliases.before.yaml").read_text()
)["motion_aliases"]


def runtime_copy(supplied):
    expected = copy.deepcopy(supplied)
    expected["completion"]["position_tolerance_deg"] = 5.0
    for frame in expected["frames"]:
        frame["angles"].update({"4": 18.0, "5": -18.0})
    return expected


@pytest.mark.parametrize("supplied", SOURCE["motions"], ids=lambda m: m["name"])
def test_all_geon_sources_preserved_except_runtime_policy_and_goal_turns(supplied):
    assert supplied["name"].startswith("건")
    expected = runtime_copy(supplied)
    actual = copy.deepcopy(MOTIONS[expected["name"]])
    if expected["name"] == "건기본자세에서제자리좌회전(골대)":
        assert len(actual["frames"]) == 14
        assert actual["repeat_count"] == 1
        for cycle in (1, 2):
            for index, frame in enumerate(expected["frames"][-2:]):
                added = copy.deepcopy(frame)
                added["start_ms"] += 339 * cycle
                assert actual["frames"][10 + (cycle - 1) * 2 + index] == added
        assert actual["max_seq_ms"] == 2656
        actual["frames"] = actual["frames"][:10]
        actual["max_seq_ms"] = expected["max_seq_ms"]
    assert actual == expected


@pytest.mark.parametrize("count", range(1, 5))
def test_goal_fine_commands_keep_requested_cycle_count(count):
    name = ALIASES[f"goal_camera_90_fine_forward_{count}"]
    expected = copy.deepcopy(MOTIONS["건미세90도"])
    expected.update(name=f"건미세90도({count}회)", repeat_count=count)
    assert name == expected["name"]
    assert MOTIONS[name] == expected


def test_hurdle_fine_command_keeps_ten_cycles():
    expected = copy.deepcopy(MOTIONS["건미세0도"])
    assert expected["repeat_count"] == 5
    expected.update(name="건미세0도(허들10회)", repeat_count=10)
    assert ALIASES["hurdle_fine_forward_10"] == expected["name"]
    assert MOTIONS[expected["name"]] == expected


def test_alias_ids_and_unrelated_motion_data_are_preserved():
    before = json.loads((UPDATE / "robot_motions_runtime.before.json").read_text())
    old_aliases = yaml.safe_load((UPDATE / "motion_aliases.before.yaml").read_text())["motion_aliases"]
    expected_aliases = {
        alias: MANIFEST["renames"].get(name, name)
        for alias, name in old_aliases.items()
    }
    expected_aliases.update(pickup="건공잡기", sdk_pickup="건공잡기")
    # The subsequent six-transition wiring reuses these catalog8 poses.
    expected_aliases.update(
        pickup_forward_to_crab_0="건오뒤에서 기본자세(0도)",
        pickup_fine_to_crab_right_0="건미세오뒤에서 기본자세45도",
        goal_forward_to_crab_right_90="건오뒤에서 기본자세(카메라45도)",
        goal_fine_to_crab_right_90="건미세오뒤에서 기본자세90도",
    )
    assert ALIASES == expected_aliases
    assert set(ALIASES.values()) <= MOTIONS.keys()
    assert len(SOURCE["motions"]) == 44
    assert len(MOTIONS) == len(CATALOG["motions"]) == 86
    changed = set(MANIFEST["renames"]) | set(MANIFEST["replaced_same_name"])
    for motion in before["motions"]:
        if motion["name"] not in changed:
            assert MOTIONS[motion["name"]] == motion
    assert not set(MANIFEST["renames"]) & MOTIONS.keys()
    assert all(m["completion"]["position_tolerance_deg"] == 5.0 for m in MOTIONS.values())


def test_startup_pose_remains_unambiguous_after_new_transitions():
    canonical = MOTIONS["건김오뒤3"]["frames"][0]["angles"]
    for motion in MOTIONS.values():
        for frame in motion["frames"]:
            if frame["name"] == "김오뒤3":
                assert frame["angles"] == canonical, motion["name"]
