"""Verify runtime import policies without changing archived source motions."""

import json
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("source_tolerance", [2.0, 5.0, 8.0])
def test_upsert_normalizes_new_replaced_and_unreferenced_motions(tmp_path, source_tolerance):
    source = tmp_path / "source.json"
    runtime = tmp_path / "runtime.json"
    aliases = tmp_path / "aliases.yaml"
    backup_dir = tmp_path / "backups"
    new_motion = {
        "name": "new", "frames": [{"angles": {"0": 42.0}}],
        "completion": {"position_tolerance_deg": source_tolerance, "settle_timeout_ms": 3000},
    }
    source_payload = {"motions": [new_motion, {"name": "replaced"}]}
    original = {"motions": [
        {"name": "replaced", "completion": {"position_tolerance_deg": 3.0}},
        {"name": "unreferenced", "completion": {"position_tolerance_deg": 2.0}},
    ]}
    source.write_text(json.dumps(source_payload))
    runtime.write_text(json.dumps(original))
    aliases.write_text("motion_aliases:\n  first: new\n  second: replaced\n")

    subprocess.run([
        sys.executable, str(Path(__file__).with_name("upsert_motion_catalog.py")),
        "--source", str(source), "--runtime", str(runtime),
        "--aliases", str(aliases), "--backup-dir", str(backup_dir),
    ], check=True, capture_output=True, text=True)

    motions = {m["name"]: m for m in json.loads(runtime.read_text())["motions"]}
    assert set(motions) == {"new", "replaced", "unreferenced"}
    assert all(m["completion"]["position_tolerance_deg"] == 5.0
               for m in motions.values())
    assert motions["new"]["frames"] == [
        {"angles": {"0": 42.0}}
    ]
    assert motions["new"]["completion"]["settle_timeout_ms"] == 3000
    assert json.loads(source.read_text()) == source_payload
    assert json.loads(next(backup_dir.glob("*.json")).read_text()) == original


def test_import_preserves_special_motions_and_other_joint_data(tmp_path):
    import copy

    source = tmp_path / "source.json"
    runtime = tmp_path / "runtime.json"
    aliases = tmp_path / "aliases.yaml"
    backup_dir = tmp_path / "backups"
    exceptions = {"찐공잡기리그랩까지 실전", "건공잡기", "찐골넣기", "찐허들"}

    def motion(name):
        return {
            "name": name, "repeat_count": 4, "playback_speed": 1.1,
            "completion": {"position_tolerance_deg": 2.0, "settle_duration_ms": 80},
            "frames": [{
                "name": "pose", "start_ms": 80, "time_ms": 72,
                "angles": {"0": -64.0, "4": 11.25, "5": -13.271484375},
                "torques": {"4": False, "5": True},
                "lift_early_arrival": True, "playback_cycle_end": True,
            }],
        }

    # Preserve shoulders for ordinary and special motions alike.
    source_payload = {"motions": [motion(name) for name in sorted(exceptions)] + [
        motion("찐공잡기전후진-2(2회)"), motion("찐기본자세에서 제자리 우회전(골대)"),
    ]}
    original = {"motions": [motion("찐공잡기전후진-2(2회)"), motion("unreferenced")]}
    source.write_text(json.dumps(source_payload))
    runtime.write_text(json.dumps(original))
    aliases.write_text("motion_aliases:\n" + "".join(
        f"  motion_{i}: {m['name']}\n" for i, m in enumerate(source_payload["motions"])
    ))
    subprocess.run([
        sys.executable, str(Path(__file__).with_name("upsert_motion_catalog.py")),
        "--source", str(source), "--runtime", str(runtime),
        "--aliases", str(aliases), "--backup-dir", str(backup_dir),
    ], check=True, capture_output=True, text=True)

    result = {m["name"]: m for m in json.loads(runtime.read_text())["motions"]}
    for supplied in source_payload["motions"] + [original["motions"][1]]:
        expected = copy.deepcopy(supplied)
        expected["completion"]["position_tolerance_deg"] = 5.0
        assert result[supplied["name"]] == expected
    assert json.loads(source.read_text()) == source_payload
    assert json.loads(next(backup_dir.glob("*.json")).read_text()) == original


def test_production_runtime_preserves_original_shoulders_and_other_fields():
    root = Path(__file__).resolve().parents[1]
    path = root / "artifacts/robot_motions_runtime.json"
    motions = json.loads(path.read_text())["motions"]
    original = {m["name"]: m for m in json.loads(
        (root / "artifacts/robot_motions_pc.json").read_text())["motions"]}
    # Compare against the current PC catalog; motion families can be replaced.
    from upsert_motion_catalog import _with_runtime_policy
    assert {m["name"] for m in motions} == original.keys()
    for motion in motions:
        assert motion == _with_runtime_policy(original[motion["name"]])
    for motion in motions:
        for index, frame in enumerate(motion["frames"]):
            source = original[motion["name"]]["frames"][index]
            assert all(frame["angles"][joint] == source["angles"][joint] for joint in ("4", "5"))
