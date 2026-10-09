"""Check the catalog49 gait, camera variants, and partial import boundaries."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
MANIFEST = json.loads(
    (ROOT / "artifacts/20261009_catalog49_forward_update/manifest.json").read_text()
)
SOURCE_PATH = ROOT / MANIFEST["source_file"]
SOURCE = next(
    m for m in json.loads(SOURCE_PATH.read_text())["motions"] if m["name"] == "전."
)
# Catalog51 supersedes these forward motions; verify this import's saved result.
FORWARD51 = ROOT / "artifacts/20261009_catalog51_forward_update"
RUNTIME = json.loads((FORWARD51 / "robot_motions_runtime.before.json").read_text())
PC = json.loads((FORWARD51 / "robot_motions_pc.before.json").read_text())
ALIASES = yaml.safe_load(
    (FORWARD51 / "motion_aliases.before.yaml").read_text()
)["motion_aliases"]


@pytest.mark.parametrize("previous", MANIFEST["previous_forward_motions"],
                         ids=lambda m: m["name"])
@pytest.mark.parametrize("runtime", [False, True])
def test_forward_preserves_source_gait_and_existing_camera_and_repeats(previous, runtime):
    expected = copy.deepcopy(SOURCE)
    expected["name"] = MANIFEST["renames"][previous["name"]]
    expected["repeat_count"] = previous["repeat_count"]
    for frame, old_frame in zip(expected["frames"], previous["frames"], strict=True):
        frame["angles"]["0"] = old_frame["angles"]["0"]
        if "90도" in expected["name"]:
            frame["name"] += "(90도)"
        if runtime:
            frame["angles"].update({"4": 18.0, "5": -18.0})
    expected["start_pose"] = expected["frames"][0]["name"]
    expected["end_pose"] = expected["frames"][-1]["name"]
    if runtime:
        expected["completion"]["position_tolerance_deg"] = 5.0
    catalog = RUNTIME if runtime else PC
    actual = next(m for m in catalog["motions"] if m["name"] == expected["name"])
    assert actual == expected
    assert actual["repeat_count"] == int(actual["name"].split("(")[1][0])
    assert [(f["start_ms"], f["time_ms"]) for f in actual["frames"]] == [
        (90, 55), (145, 105), (334, 62), (396, 110),
    ]
    assert actual["playback_speed"] == 1.0


def test_source_snapshot_is_byte_identical():
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == MANIFEST["source_sha256"]


def test_pickup_uses_catalog51_regrasp_and_preserves_source_arm_targets():
    source = next(m for m in json.loads((ROOT / 'motion_imports/robot_motions(51).json').read_text())["motions"]
                  if m["name"] == "건공잡기")
    assert ALIASES["pickup"] == ALIASES["sdk_pickup"] == source["name"]
    expected = copy.deepcopy(source)
    expected["completion"]["position_tolerance_deg"] = 5.0
    renames = json.loads((ROOT / "artifacts/20261009_pickup_source_arms_update/manifest.json").read_text())["pose_renames"]
    for frame in expected["frames"]:
        frame["name"] = renames.get(frame["name"], frame["name"])
    expected["end_pose"] = renames.get(expected["end_pose"], expected["end_pose"])
    actual = next(m for m in RUNTIME["motions"] if m["name"] == source["name"])
    assert actual == expected
    assert len(actual["frames"]) == 11
    assert [(f["name"], f["start_ms"], f["time_ms"]) for f in actual["frames"]
            if "리그랩" in f["name"]] == [
                ("건리그랩1", 4700, 100), ("건리그랩2", 5100, 50), ("건리그랩1", 5150, 100)]
    assert next(m for m in PC["motions"] if m["name"] == source["name"]) == source


def test_only_forward_alias_targets_change():
    assert ALIASES == {
        alias: MANIFEST["renames"].get(name, name)
        for alias, name in MANIFEST["aliases_before"].items()
    }
    assert sum(name in MANIFEST["renames"].values() for name in ALIASES.values()) == 14
    assert set(ALIASES.values()) <= {m["name"] for m in RUNTIME["motions"]}


@pytest.mark.parametrize("catalog,filename", [
    (RUNTIME, "robot_motions_runtime.json"), (PC, "robot_motions_pc.json"),
])
def test_non_forward_motions_and_catalog_size_are_unchanged(catalog, filename):
    # The later pickup/startup import is checked against the live catalog separately.
    catalog = json.loads((ROOT / 'artifacts/20261009_catalog51_pickup_startup_update'
                          / filename.replace('.json', '.before.json')).read_text())
    motions = {m["name"]: m for m in catalog["motions"]}
    assert len(motions) == len(catalog["motions"]) == 82
    assert not motions.keys() & MANIFEST["renames"].keys()
    unchanged = MANIFEST["unchanged_sha256"][filename]
    assert motions.keys() == unchanged.keys() | set(MANIFEST["renames"].values())
    # Recovery was updated later; check its pre-update snapshot here.
    recovery_update = ROOT / "artifacts/20261009_catalog49_line_recovery_update"
    recovery_names = json.loads((recovery_update / "manifest.json").read_text())["updated_motions"]
    previous = {m["name"]: m for m in json.loads(
        (recovery_update / filename.replace(".json", ".before.json")).read_text()
    )["motions"]}
    for name, digest in unchanged.items():
        motion = previous[name] if name in recovery_names else motions[name]
        if filename == "robot_motions_runtime.json" and name == "건공잡기":
            motion = json.loads((ROOT / "artifacts/20261009_pickup_source_arms_update/pickup.before.json").read_text())
        encoded = json.dumps(motion, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":")).encode()
        assert hashlib.sha256(encoded).hexdigest() == digest, name


def test_policy_reference_matches_every_new_frame_when_overrides_are_disabled():
    reference = {m["name"]: m for m in PC["motions"]}
    for motion in RUNTIME["motions"]:
        if motion["name"] not in MANIFEST["renames"].values():
            continue
        frames = reference[motion["name"]]["frames"]
        for frame in motion["frames"]:
            matches = [f for f in frames if f["start_ms"] == frame["start_ms"]
                       and (f["frame_id"] == frame["frame_id"] or f["name"] == frame["name"])]
            assert len(matches) == 1
            assert matches[0]["angles"]["0"] == frame["angles"]["0"]


@pytest.mark.parametrize("catalog", [RUNTIME, PC])
def test_legacy_back_pose_remains_unambiguous(catalog):
    poses = [f["angles"] for m in catalog["motions"] for f in m["frames"]
             if f["name"] == "김오뒤3"]
    assert poses
    assert all(pose == poses[0] for pose in poses)
