"""Protect the requested pruning and source-exact pickup replacement."""

import copy
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / "artifacts/20260929_catalog36_pickup_update"
PICKUP = "찐공잡기리그랩까지 실전"
REMOVED = {
    "찐골넣기실전",
    "찐미세오옆꽃게0도",
    "찐미세오옆꽃게90도-1",
    "찐오뒤에서기본자세(골대 카메라90도)",
    "찐미세오뒤에서기본자세(골대 카메라90도)",
    "미세오옆에서오옆꽃게0도",
    "미세오뒤에서 오옆꽃게90도",
    "후진실전-2(1회, 픽업 카메라0도)",
}


def test_catalog36_changes_only_requested_motions():
    before = json.loads((UPDATE / "runtime_before.json").read_text())
    after = json.loads((ROOT / "artifacts/robot_motions_runtime.json").read_text())
    source = json.loads((UPDATE / "source_pickup.json").read_text())
    expected_pickup = copy.deepcopy(source)
    expected_pickup["completion"]["position_tolerance_deg"] = 5.0
    expected = copy.deepcopy(before)
    expected["motions"] = [
        expected_pickup if m["name"] == PICKUP else m
        for m in before["motions"] if m["name"] not in REMOVED
    ]
    assert len(after["motions"]) == 81
    # Catalog37 supersedes the hurdle; its full source is checked separately.
    for index, motion in enumerate(after["motions"]):
        if motion["name"] == "찐허들":
            after["motions"][index] = next(
                m for m in expected["motions"] if m["name"] == "찐허들"
            )
    assert after == expected
    assert source["completion"]["position_tolerance_deg"] == 2.0


def test_catalog36_preserves_active_aliases_and_similarly_named_motions():
    before = yaml.safe_load((UPDATE / "motion_aliases_before.yaml").read_text())["motion_aliases"]
    after = yaml.safe_load((ROOT / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text())["motion_aliases"]
    assert after == {k: v for k, v in before.items() if v not in REMOVED}
    assert len(before) - len(after) == 2
    names = {m["name"] for m in json.loads((ROOT / "artifacts/robot_motions_runtime.json").read_text())["motions"]}
    assert set(after.values()) <= names
    assert after["sdk_pickup"] == after["pickup"] == PICKUP
    assert after["goal_fine_to_crab_right_90"] == "찐미세오뒤에서 오옆꽃게90도"
    assert after["fine_to_turn_ready_90"] == "찐미세오뒤에서 오뒤(골대 카메라90도)"
    assert after["goal_fine_to_default"] == "찐미세오뒤에서기본자세"
