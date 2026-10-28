"""Check catalog9 right-turn data and mappings without touching other gaits."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261005_catalog9_right_turn_update"
CATALOG = json.loads((ROOT / "artifacts/robot_motions_runtime.json").read_text())
MOTIONS = {m["name"]: m for m in CATALOG["motions"]}
SOURCE = json.loads((UPDATE / "source_requested_motions.json").read_text())
ALIASES = yaml.safe_load(
    (ROOT / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text()
)["motion_aliases"]


@pytest.mark.parametrize("supplied", SOURCE["motions"], ids=lambda m: m["name"])
def test_import_preserves_source_except_runtime_policy(supplied):
    expected = copy.deepcopy(supplied)
    expected["completion"]["position_tolerance_deg"] = 5.0
    for frame in expected["frames"]:
        frame["angles"].update({"4": 18.0, "5": -18.0})
    assert MOTIONS[expected["name"]] == expected


@pytest.mark.parametrize("camera,alias_prefix", [
    (0, "pickup_camera_down_turn_right"),
    (45, "post_ball_line_turn_right"),
    (90, "goal_camera_90_turn_right"),
])
@pytest.mark.parametrize("count", [2, 3, 5, 7, 9])
def test_right_turn_aliases_keep_camera_and_repeat_count(camera, alias_prefix, count):
    name = ("찐제자리우회전45도(9회)" if camera == 45 and count == 9
            else f"찐제자리우회전{camera}도-1({count}회)")
    assert ALIASES[f"{alias_prefix}_{count}"] == name
    motion = MOTIONS[name]
    assert motion["repeat_count"] == count
    assert motion["playback_speed"] == 0.9
    assert len(motion["frames"]) == 2
    assert motion["end_pose"] == ("김오뒤3" if camera == 45 else f"김오뒤3({camera}도)")
    assert [(f["start_ms"], f["time_ms"]) for f in motion["frames"]] == [(120, 60), (180, 140)]


def test_goal_exit_plays_new_seven_turn_composite_once():
    alias = MotionCommandBridgeNode.motion_id_for_action("POST_SHOT_TURN_RIGHT_9")
    assert alias == "post_shot_default_turn_right"
    assert ALIASES[alias] == "건기본자세에서제자리우회전(골대)"
    motion = MOTIONS[ALIASES[alias]]
    assert motion["repeat_count"] == 1
    assert [f["name"] for f in motion["frames"]] == (
        ["김왼들(앞먼저닿음)", "김오뒤3"] + ["제우오들25", "김오뒤3"] * 7
    )
    assert motion["max_seq_ms"] == 3591
    assert motion["frames"][-1]["start_ms"] + motion["frames"][-1]["time_ms"] == 3591


def test_only_requested_motions_and_goal_exit_mapping_change():
    before = json.loads((UPDATE / "robot_motions_runtime.before.json").read_text())
    before_aliases = yaml.safe_load((UPDATE / "motion_aliases.before.yaml").read_text())["motion_aliases"]
    manifest = json.loads((UPDATE / "manifest.json").read_text())
    assert len(SOURCE["motions"]) == 16
    assert len(MOTIONS) == len(CATALOG["motions"]) == len(before["motions"]) == 86
    changed = set(manifest["renames"]) | set(manifest["replaced_same_name"])
    assert len(changed) == 16
    assert [m["name"] for m in CATALOG["motions"]] == [
        manifest["renames"].get(m["name"], m["name"]) for m in before["motions"]
    ]
    for motion in before["motions"]:
        if motion["name"] not in changed:
            assert MOTIONS[motion["name"]] == motion
    assert ALIASES == {
        **before_aliases,
        "post_shot_default_turn_right": "건기본자세에서제자리우회전(골대)",
    }
    assert not set(manifest["renames"]) & MOTIONS.keys()
    assert set(ALIASES.values()) <= MOTIONS.keys()
    assert all(m["completion"]["position_tolerance_deg"] == 5.0 for m in MOTIONS.values())


def test_new_right_turn_end_poses_do_not_make_startup_pose_ambiguous():
    canonical = MOTIONS["건김오뒤3"]["frames"][0]["angles"]
    for motion in MOTIONS.values():
        for frame in motion["frames"]:
            if frame["name"] == "김오뒤3":
                assert frame["angles"] == canonical, motion["name"]
