"""Protect the approved startup-pose alignment and motion exceptions."""

import copy
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SNAPSHOT = ROOT / "artifacts/20260929_startup_pose_alignment/runtime_before.json"
EXCLUDED = {"찐공잡기리그랩까지 실전", "찐골넣기", "찐허들"}
CATALOG36_UPDATE = ROOT / "artifacts/20260929_catalog36_pickup_update"
REMOVED = set(json.loads((CATALOG36_UPDATE / "manifest.json").read_text())["removed_motion_names"])


@pytest.fixture(scope="module")
def catalogs():
    before = json.loads(SNAPSHOT.read_text())["motions"]
    after = json.loads((ROOT / "artifacts/robot_motions_runtime.json").read_text())["motions"]
    return ({m["name"]: m for m in before}, {m["name"]: m for m in after})


def test_startup_pose_matches_double_jjin_six_repeat_forward(catalogs):
    _, current = catalogs
    reference = next(f["angles"] for f in current["찐찐전진(6회)"]["frames"]
                     if f["name"] == "오뒤412")
    poses = [f["angles"] for m in current.values() for f in m["frames"]
             if f["name"] == "오뒤412"]
    assert len(poses) == 30
    assert reference["4"] == 18 and reference["5"] == -18
    assert reference["13"] == -41.59375
    assert reference["14"] == 55.568359375
    assert all(angles == reference for angles in poses)


def test_back_pose_motor_override_excludes_pickup_shot_and_hurdle(catalogs):
    _, current = catalogs
    for name, motion in current.items():
        if name in EXCLUDED:
            continue
        for frame in motion["frames"]:
            if "오뒤" in frame["name"]:
                assert frame["angles"]["4"] == 18
                assert frame["angles"]["5"] == -18


@pytest.mark.parametrize("name", sorted(EXCLUDED))
def test_exception_motion_is_preserved_except_hurdle_pose_label(catalogs, name):
    before, current = catalogs
    expected = copy.deepcopy(before[name])
    if name == "찐공잡기리그랩까지 실전":
        expected = json.loads((CATALOG36_UPDATE / "source_pickup.json").read_text())
        expected["completion"]["position_tolerance_deg"] = 5.0
    if name == "찐허들":
        source = ROOT / "artifacts/20260929_catalog37_hurdle_update/source_hurdle.json"
        expected = json.loads(source.read_text())
        expected["completion"]["position_tolerance_deg"] = 5.0
        assert current[name]["end_pose"] == "오뒤412(허들)"
        for frame in expected["frames"]:
            if frame["name"] == "오뒤412":
                frame["name"] = "오뒤412(허들)"
        expected["end_pose"] = "오뒤412(허들)"
    assert current[name] == expected


def test_other_frames_and_motion_timing_are_unchanged(catalogs):
    before, current = catalogs
    assert set(before) - REMOVED == set(current)
    for name, original in before.items():
        if name in EXCLUDED or name in REMOVED:
            continue
        changed = copy.deepcopy(current[name])
        for old, new in zip(original["frames"], changed["frames"]):
            if old["name"] == "오뒤412":
                new["angles"] = old["angles"]
            elif "오뒤" in old["name"]:
                for motor in ("4", "5"):
                    new["angles"][motor] = old["angles"][motor]
        assert changed == original
