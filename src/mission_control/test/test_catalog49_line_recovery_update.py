"""Verify recovery source fidelity, action routing, and partial import boundaries."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20261009_catalog49_line_recovery_update"
MANIFEST = json.loads((UPDATE / "manifest.json").read_text())
SOURCE_PATH = ROOT / MANIFEST["source_file"]
SOURCE = {m["name"]: m for m in json.loads(SOURCE_PATH.read_text())["motions"]}
ALIASES = yaml.safe_load((ROOT / "src/irc_step_motion_executor/config/motion_aliases.yaml")
                         .read_text())["motion_aliases"]


@pytest.mark.parametrize("filename", ["robot_motions_runtime.json", "robot_motions_pc.json"])
def test_only_requested_recoveries_change_and_preserve_source_motion(filename):
    before = json.loads((UPDATE / filename.replace(".json", ".before.json")).read_text())
    # Later catalog51 changes only pickup/startup; its own test checks that boundary.
    after = json.loads((ROOT / 'artifacts/20261009_catalog51_pickup_startup_update'
                       / filename.replace('.json', '.before.json')).read_text())
    assert len(after["motions"]) == len(before["motions"])
    assert {k: v for k, v in after.items() if k != "motions"} == {
        k: v for k, v in before.items() if k != "motions"}
    for old, actual in zip(before["motions"], after["motions"], strict=True):
        assert actual["name"] == old["name"]
        if filename == "robot_motions_runtime.json" and old["name"] == "건공잡기":
            # The later source-arm restoration has its own contract tests.
            actual = json.loads((ROOT / "artifacts/20261009_pickup_source_arms_update/pickup.before.json").read_text())
        if old["name"] not in MANIFEST["updated_motions"]:
            assert actual == old
            continue
        expected = copy.deepcopy(SOURCE[old["name"]])
        for frame in expected["frames"]:
            frame["name"] = MANIFEST["pose_renames"].get(frame["name"], frame["name"])
            if filename == "robot_motions_runtime.json":
                frame["angles"].update({"4": 18.0, "5": -18.0})
        for key in ("start_pose", "end_pose"):
            expected[key] = MANIFEST["pose_renames"].get(expected[key], expected[key])
        if filename == "robot_motions_runtime.json":
            expected["completion"]["position_tolerance_deg"] = 5.0
        assert actual == expected
        assert actual["repeat_count"] == 4
        assert len(actual["frames"]) == 4
        assert actual["frames"][-1]["angles"]["21"] == 2.833984375


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("line_side", ["LEFT", "RIGHT"])
def test_recovery_actions_resolve_by_turn_direction(direction, line_side):
    action = f"RECOVER_{line_side}_TURN_{direction}_4"
    alias = f"line_recovery_{direction.lower()}_4"
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID[action] == alias
    assert ALIASES[alias] == MANIFEST["aliases"][alias]
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID[direction] == alias
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID[f"BALL_APPROACH_RECOVER_{direction}_4"] == alias


def test_source_snapshot_is_unchanged():
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == MANIFEST["source_sha256"]
