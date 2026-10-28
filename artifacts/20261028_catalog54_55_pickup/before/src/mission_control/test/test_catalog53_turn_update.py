"""Preserve supplied turn frames and all unrelated deployed motion data."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261028_catalog53_turn_queue"
MANIFEST = json.loads((UPDATE / "manifest.json").read_text())
SPEED1_MOTIONS = set(json.loads((ROOT / "artifacts/20261028_forward_speed1_boundary/speed_update.json").read_text())["motions"])
SOURCE_PATH = ROOT / MANIFEST["source_file"]
SOURCE = {m["name"]: m for m in json.loads(SOURCE_PATH.read_text())["motions"]}


@pytest.mark.parametrize("filename", ["robot_motions_runtime.json", "robot_motions_pc.json"])
def test_turn_import_preserves_source_and_unrelated_motions(filename):
    motions = json.loads((ROOT / "artifacts" / filename).read_text())["motions"]
    actual = {m["name"]: m for m in motions}
    assert len(actual) == len(motions) == 86
    expected = {n: copy.deepcopy(SOURCE[n]) for n in MANIFEST["source_turn_motions"]}
    for name, spec in MANIFEST["derived"].items():
        motion = copy.deepcopy(SOURCE[spec["source"]])
        motion.update(name=name, repeat_count=spec["repeat_count"])
        expected[name] = motion
    for name in ("R골대후제자리좌회전", "R골대후제자리우회전"):
        assert SOURCE[name]["repeat_count"] == 5
        expected[name]["repeat_count"] = 1  # User-approved whole-composition count.
    for name, motion in expected.items():
        if "runtime" in filename:
            motion["completion"]["position_tolerance_deg"] = 5.0
        assert actual[name] == motion
    before = json.loads((UPDATE / "before/artifacts" / filename).read_text())["motions"]
    for motion in before:
        name = motion["name"]
        if name not in MANIFEST["renames"] and name not in expected:
            if name in SPEED1_MOTIONS:
                motion["playback_speed"] = 1.0
            assert actual[name] == motion  # Every forward hold and angle is preserved.
    assert not (actual.keys() & MANIFEST["renames"].keys())


def test_aliases_preserve_commands_and_select_new_turns():
    path = "src/irc_step_motion_executor/config/motion_aliases.yaml"
    before = yaml.safe_load((UPDATE / "before" / path).read_text())["motion_aliases"]
    aliases = yaml.safe_load((ROOT / path).read_text())["motion_aliases"]
    motions = {m["name"]: m for m in json.loads(
        (ROOT / "artifacts/robot_motions_runtime.json").read_text())["motions"]}
    assert aliases == {key: MANIFEST["renames"].get(value, value) for key, value in before.items()}
    assert set(aliases.values()) <= motions.keys()
    assert aliases["pickup_second_backward_turn_left"] == "R후진에서 제자리 좌회전"
    assert aliases["post_shot_default_turn_left"] == "R골대후제자리좌회전"
    assert aliases["post_shot_default_turn_right"] == "R골대후제자리우회전"
    for prefix, camera in (("post_ball_line", ""), ("pickup_camera_down", "0도"),
                           ("goal_camera_90", "90도")):
        for count in range(1, 7):
            name = aliases[f"{prefix}_turn_left_{count}"]
            assert name == f"R제자리좌회전{camera}({count}회)"
            assert motions[name]["repeat_count"] == count


def test_source_snapshot_is_unmodified():
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == MANIFEST["source_sha256"]
