#!/usr/bin/env python3
"""Read-only input audit; writes evidence beside this script, never motion data."""
import collections
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
CATALOGS = {
    "runtime_source": "artifacts/robot_motions_runtime.json",
    "pc_source": "artifacts/robot_motions_pc.json",
    "runtime_installed": "install/irc_step_motion_executor/share/irc_step_motion_executor/config/robot_motions_runtime.json",
    "gui_reference": "artifacts/20261006_gui_alignment/reference/robot_motions(10).json",
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def differences(left, right, path=""):
    if isinstance(left, dict) and isinstance(right, dict):
        for key in sorted(left.keys() | right.keys()):
            if key not in left or key not in right:
                yield {"path": path + "/" + key, "left": left.get(key),
                       "right": right.get(key), "missing": "left" if key not in left else "right"}
            else:
                yield from differences(left[key], right[key], path + "/" + key)
    elif isinstance(left, list) and isinstance(right, list):
        for index in range(max(len(left), len(right))):
            if index >= len(left) or index >= len(right):
                yield {"path": path + "/" + str(index),
                       "left": left[index] if index < len(left) else None,
                       "right": right[index] if index < len(right) else None,
                       "missing": "left" if index >= len(left) else "right"}
            else:
                yield from differences(left[index], right[index], path + "/" + str(index))
    elif left != right:
        yield {"path": path, "left": left, "right": right}


def timing(motion):
    frames = sorted(motion["frames"], key=lambda frame: frame["start_ms"])
    previous_end = 0
    gaps = []
    for frame in frames:
        gaps.append(frame["start_ms"] - previous_end)
        previous_end = frame["start_ms"] + frame["time_ms"]
    return {
        "repeat_count": motion.get("repeat_count", 1),
        "playback_speed": motion.get("playback_speed", 1),
        "max_seq_ms": motion["max_seq_ms"],
        "frames": [{key: frame.get(key) for key in
                    ("start_ms", "time_ms", "lift_early_arrival", "playback_cycle_end")}
                   for frame in frames],
        "gaps_before_frames_ms": gaps,
        "sequence_end_ms": previous_end,
        "nominal_total_ms": previous_end * motion.get("repeat_count", 1) / motion.get("playback_speed", 1),
    }


def main():
    catalogs = {}
    files = {}
    for label, name in CATALOGS.items():
        path = ROOT / name
        motions = json.loads(path.read_text())["motions"]
        catalogs[label] = {motion["name"]: motion for motion in motions}
        if len(catalogs[label]) != len(motions):
            raise ValueError(f"duplicate motion names: {path}")
        files[label] = {"path": str(path), "resolved_path": str(path.resolve()),
                        "sha256": sha(path), "motion_count": len(motions)}
    reference = catalogs["runtime_source"]
    comparisons = {}
    target = "건전진45도(6회)"
    for label, other in catalogs.items():
        if label == "runtime_source":
            continue
        common = sorted(reference.keys() & other.keys())
        diffs = [{"motion": name, **row} for name in common
                 for row in differences(reference[name], other[name])]
        fields = collections.Counter()
        for row in diffs:
            parts = row["path"].strip("/").split("/")
            if parts[0] == "frames" and len(parts) > 1:
                parts[1] = "*"
            fields["/".join(parts)] += 1
        comparisons[label] = {
            "left": "runtime_source", "right": label,
            "left_only": sorted(reference.keys() - other.keys()),
            "right_only": sorted(other.keys() - reference.keys()),
            "changed_motion_count": len({row["motion"] for row in diffs}),
            "timing_changed_motions": [name for name in common if timing(reference[name]) != timing(other[name])],
            "field_difference_counts": dict(fields),
            "selected_motion_differences": [row for row in diffs if row["motion"] == target],
            "differences": diffs,
        }
    provenance_paths = [
        "artifacts/20261006_gui_alignment/reference/sdk_gui.py",
        "artifacts/20261006_gui_alignment/reference/sdk_gui_state.json",
        "src/irc_step_motion_executor/config/motion_aliases.yaml",
        "src/mission_control/launch/full_system_robot.launch.py",
        "src/mission_control/mission_control/motion_command_bridge_node.py",
        "src/irc_step_motion_executor/src/sdk_executor_node.cpp",
        "tools/verify_latest_gui_execution.py",
        "tools/trace_sdk_gui.py",
        "install/irc_step_motion_executor/lib/irc_step_motion_executor/sdk_motion_executor",
    ]
    provenance_paths += [str(path.relative_to(ROOT)) for path in
                         sorted((ROOT / "external_sdk/gui_aligned").iterdir()) if path.is_file()]
    aliases = yaml.safe_load((ROOT / provenance_paths[2]).read_text())["motion_aliases"]
    report = {
        "scope": "local files only; Jetson deployment, running GUI memory and physical motion unverified",
        "files": files,
        "provenance_sha256": {name: sha(ROOT / name) for name in provenance_paths},
        "comparisons": comparisons,
        "selected_motion": {"name": target, "timing": timing(reference[target]),
                            "aliases": [key for key, value in aliases.items() if value == target]},
        "aliases_missing_runtime": {key: value for key, value in aliases.items() if value not in reference},
        "physical_hardware_accessed": False,
    }
    (OUT / "input_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"catalog_counts": {k: v["motion_count"] for k, v in files.items()},
                      "timing_changed": {k: v["timing_changed_motions"] for k, v in comparisons.items()},
                      "aliases_missing_runtime": report["aliases_missing_runtime"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
