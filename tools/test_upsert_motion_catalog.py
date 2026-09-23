"""Verify that catalog imports cannot restore a tighter completion tolerance."""

import json
from pathlib import Path
import subprocess
import sys


def test_upsert_normalizes_new_replaced_and_unreferenced_motions(tmp_path):
    source = tmp_path / "source.json"
    runtime = tmp_path / "runtime.json"
    aliases = tmp_path / "aliases.yaml"
    backup_dir = tmp_path / "backups"
    new_motion = {
        "name": "new", "frames": [{"angles": {"0": 42.0}}],
        "completion": {"position_tolerance_deg": 2.0, "settle_timeout_ms": 3000},
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
    assert motions["new"]["frames"] == new_motion["frames"]
    assert motions["new"]["completion"]["settle_timeout_ms"] == 3000
    assert json.loads(source.read_text()) == source_payload
    assert json.loads(next(backup_dir.glob("*.json")).read_text()) == original
