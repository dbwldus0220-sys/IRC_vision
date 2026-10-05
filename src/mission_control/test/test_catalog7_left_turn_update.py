"""Verify the archived catalog7 import before the catalog8 replacement."""

import copy
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261004_catalog7_left_turn_update"
SNAPSHOT = ROOT / "artifacts/20261005_catalog8_geon_motion_update"
CATALOG = json.loads((SNAPSHOT / "robot_motions_runtime.before.json").read_text())
MOTIONS = {motion["name"]: motion for motion in CATALOG["motions"]}
SOURCE = json.loads((UPDATE / "source_requested_motions.json").read_text())
MANIFEST = json.loads((UPDATE / "manifest.json").read_text())
ALIASES = yaml.safe_load(
    (SNAPSHOT / "motion_aliases.before.yaml").read_text()
)["motion_aliases"]


@pytest.mark.parametrize("supplied", SOURCE["motions"], ids=lambda m: m["name"])
def test_import_preserves_source_except_runtime_policy_and_added_goal_turns(supplied):
    expected = copy.deepcopy(supplied)
    expected["completion"]["position_tolerance_deg"] = 5.0
    for frame in expected["frames"]:
        frame["angles"].update({"4": 18.0, "5": -18.0})
    actual = copy.deepcopy(MOTIONS[expected["name"]])
    if expected["name"] == "건기본자세에서제자리좌회전(골대)":
        # The six-turn runtime must preserve the entire supplied four-turn prefix.
        actual["frames"] = actual["frames"][:len(expected["frames"])]
        actual["max_seq_ms"] = expected["max_seq_ms"]
    assert actual == expected


@pytest.mark.parametrize("camera,alias_prefix", [
    (0, "pickup_camera_down_turn_left"),
    (45, "post_ball_line_turn_left"),
    (90, "goal_camera_90_turn_left"),
])
@pytest.mark.parametrize("count", range(1, 7))
def test_left_turn_aliases_preserve_camera_and_repeat_count(camera, alias_prefix, count):
    name = f"건제자리좌회전{camera}도({count}회)"
    assert ALIASES[f"{alias_prefix}_{count}"] == name
    assert MOTIONS[name]["repeat_count"] == count
    assert len(MOTIONS[name]["frames"]) == 2


def test_goal_exit_plays_six_internal_turns_once():
    name = "건기본자세에서제자리좌회전(골대)"
    assert ALIASES["post_shot_default_turn_left"] == name
    motion = MOTIONS[name]
    assert motion["repeat_count"] == 1
    assert [frame["name"] for frame in motion["frames"]] == (
        ["김왼들(앞먼저닿음)", "김오뒤3"] + ["제좌왼들25", "김오뒤3"] * 6
    )
    assert motion["max_seq_ms"] == 2656
    assert motion["frames"][-1]["start_ms"] + motion["frames"][-1]["time_ms"] == 2656
    for cycle in (1, 2):
        for index, template in enumerate(motion["frames"][8:10]):
            expected = copy.deepcopy(template)
            expected["start_ms"] += 339 * cycle
            assert motion["frames"][10 + (cycle - 1) * 2 + index] == expected


def test_partial_update_preserves_other_motions_and_alias_ids():
    before = json.loads((UPDATE / "robot_motions_runtime.before.json").read_text())
    changed = set(MANIFEST["renames"]) | set(MANIFEST["replaced_same_name"])
    assert len(changed) == 20
    assert len(MOTIONS) == len(CATALOG["motions"]) == len(before["motions"]) == 84
    assert not set(MANIFEST["renames"]) & MOTIONS.keys()
    for motion in before["motions"]:
        if motion["name"] not in changed:
            assert MOTIONS[motion["name"]] == motion
    old_aliases = yaml.safe_load((UPDATE / "motion_aliases.before.yaml").read_text())["motion_aliases"]
    assert ALIASES == {
        alias: MANIFEST["renames"].get(name, name) for alias, name in old_aliases.items()
    }
    assert set(ALIASES.values()) <= MOTIONS.keys()
    assert all(m["completion"]["position_tolerance_deg"] == 5.0 for m in MOTIONS.values())


def test_new_turn_end_poses_keep_startup_pose_unambiguous():
    canonical = MOTIONS["건김오뒤3"]["frames"][0]["angles"]
    for motion in MOTIONS.values():
        for frame in motion["frames"]:
            if frame["name"] == "김오뒤3":
                assert frame["angles"] == canonical, motion["name"]
