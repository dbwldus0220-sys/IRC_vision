"""Verify the supplied catalog import, removed motions, and runtime routing."""

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode


ROOT = Path(__file__).resolve().parents[3]
UPDATE = json.loads((ROOT / "artifacts/catalog26_shot_update_manifest.json").read_text())
MANIFEST = json.loads((ROOT / "artifacts/catalog25_migration_manifest.json").read_text())
FORWARD_UPDATE = json.loads((ROOT / "artifacts/catalog30_forward_update_manifest.json").read_text())
CRAB_ALIASES = {
    "pickup_crab_right_0": "찐미세오옆꽃게0-1(1회)",
    "goal_camera_90_crab_right": "찐미세오옆꽃게90-1(1회)",
    "pickup_fine_to_crab_right_0": "찐미세오뒤에서오옆꽃게0도",
    "goal_fine_to_crab_right_90": "찐미세오뒤에서 오옆꽃게90도",
}


@pytest.fixture(scope="module")
def catalog():
    motions = json.loads((ROOT / "artifacts/robot_motions_runtime.json").read_text())["motions"]
    by_name = {motion["name"]: motion for motion in motions}
    assert len(by_name) == len(motions)
    return by_name


@pytest.fixture(scope="module")
def aliases():
    return yaml.safe_load((ROOT / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text())["motion_aliases"]


@pytest.mark.parametrize("name,digest", [
    *((name, digest) for name, digest in MANIFEST["imported_sha256"].items()
      if name not in FORWARD_UPDATE["renames"]),
    *UPDATE["imported_sha256"].items(),
    *FORWARD_UPDATE["imported_sha256"].items(),
])
def test_motion_matches_import_snapshot(catalog, name, digest):
    if name in {"찐후진실전-2(2회)", "찐후진실전(1회)"}:
        source = json.loads((ROOT / "artifacts/20260929_catalog34_retreat_update/source_requested_motions.json").read_text())
        expected = next(m for m in source["motions"] if m["name"] == name)
        expected["completion"]["position_tolerance_deg"] = 5.0
        assert catalog[name] == expected
    else:
        encoded = json.dumps(catalog[name], sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        assert hashlib.sha256(encoded.encode()).hexdigest() == digest


def test_renamed_motions_and_aliases_are_consistent(catalog, aliases):
    assert not set(MANIFEST["renames"]) & set(catalog)
    assert set(aliases.values()) <= set(catalog)
    expected_aliases = {alias: change["after"] for alias, change in MANIFEST["changed_aliases"].items() if alias not in UPDATE["removed_aliases"]}
    expected_aliases.update(UPDATE["aliases"])
    expected_aliases.update(CRAB_ALIASES)
    expected_aliases.update({alias: change["after"] for alias, change in FORWARD_UPDATE["changed_aliases"].items()})
    for alias, name in expected_aliases.items():
        assert aliases[alias] == name
    assert not set(UPDATE["removed_aliases"]) & set(aliases)
    assert set(catalog) == (
        (set(MANIFEST["imported_sha256"]) - set(FORWARD_UPDATE["renames"]))
        | set(FORWARD_UPDATE["imported_sha256"]) | set(UPDATE["imported_sha256"])
        | set(CRAB_ALIASES.values())
    )


@pytest.mark.parametrize("action,name", [
    ("SHOT", "찐골넣기"),
    ("LINE_LOST_TURN_LEFT", "찐제자리좌회전45도-1(2회)"),
    ("LINE_LOST_TURN_RIGHT", "찐제자리우회전45도-1(5회)"),
    ("STRAIGHT", "찐전진45(6회)"),
    ("STRAIGHT_2", "찐전진45(4회)"),
    ("GOAL_CAMERA_90_FORWARD_2", "찐전진90(4회)"),
    ("GOAL_CAMERA_90_FORWARD", "찐전진90(6회)"),
    ("RECOVER_LEFT_TURN_LEFT_4", "찐라인복귀좌회전45도(4회)"),
    ("RECOVER_RIGHT_TURN_RIGHT_4", "찐라인복귀우회전45 도(4회)"),
    ("RECOVER_RIGHT_TURN_LEFT_4", "찐라인복귀좌회전45도(4회)"),
    ("RECOVER_LEFT_TURN_RIGHT_4", "찐라인복귀우회전45 도(4회)"),
])
def test_navigation_reaches_supplied_motion(catalog, aliases, action, name):
    assert aliases[MotionCommandBridgeNode.motion_id_for_action(action)] == name
    assert catalog[name]["completion"]["position_tolerance_deg"] == 5.0


def test_startup_pose_still_has_one_unambiguous_set_of_angles(catalog):
    poses = [frame["angles"] for motion in catalog.values() for frame in motion["frames"] if frame["name"] == "오뒤412"]
    assert poses
    assert all(set(pose) == {str(motor) for motor in range(23)} for pose in poses)
    assert all(pose == poses[0] for pose in poses)


def test_removed_legacy_motions_and_aliases_stay_absent(catalog, aliases):
    assert len(catalog) == 78
    assert len(MANIFEST["removed_sha256"]) == 45
    assert not set(MANIFEST["removed_sha256"]) & set(catalog)
    assert not set(MANIFEST["removed_aliases"]) & set(aliases)


def test_unused_six_repeat_recovery_stays_in_catalog_without_navigation_route(catalog):
    assert "찐라인복귀좌회전45도(6회)" in catalog
    assert MotionCommandBridgeNode.motion_id_for_action("RECOVER_LEFT_TURN_LEFT_6") is None


def test_new_shot_preserves_catalog26_frames_and_runtime_tolerance(catalog):
    original = json.loads((ROOT / "artifacts/catalog26_shot_update_backup/source_shot.json").read_text())
    original["completion"]["position_tolerance_deg"] = 5.0
    assert catalog["찐골넣기"] == original
    assert catalog["찐골넣기"]["start_pose"] == "골기본11"


def test_post_grasp_retreat_preserves_catalog34_motion(catalog, aliases):
    source = json.loads((ROOT / "artifacts/20260929_catalog34_retreat_update/source_requested_motions.json").read_text())
    original = next(m for m in source["motions"] if m["name"] == "찐후진실전(1회)")
    original["completion"]["position_tolerance_deg"] = 5.0
    retreat = catalog[aliases["pickup_retreat_2"]]
    assert retreat == original
    assert retreat["name"] == "찐후진실전(1회)"
    assert retreat["repeat_count"] == 1
