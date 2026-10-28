"""Audit saved playback traces offline; no ROS or motor access."""

import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return {"mean": statistics.mean(values),
            "p99_nearest_rank": values[math.ceil(0.99 * len(values)) - 1],
            "max": values[-1], "over_7_5": sum(v > 7.5 for v in values),
            "over_10": sum(v > 10 for v in values)}


def parse(path):
    runs = []
    with path.open() as stream:
        for row in csv.reader(stream):
            if row[0] == "summary":
                runs.append({"summary": row, "ticks": []})
            elif row[0] == "tick":
                runs[-1]["ticks"].append(row)
    return runs


def sample(frames, base, timeline):
    pose = base.copy()
    end = max(f["start_ms"] + f["time_ms"] for f in frames)
    active = next((f for f in frames if
                   f["start_ms"] <= timeline < f["start_ms"] + f["time_ms"] or
                   timeline == end == f["start_ms"] + f["time_ms"]), None)
    before = active["start_ms"] if active else timeline
    for f in frames:
        if f["start_ms"] + f["time_ms"] <= before:
            pose.update({int(k): v for k, v in f["angles"].items()})
    if active:
        early = active.get("lift_early_arrival", False) and any(
            word in active["name"] for word in ("발들", "들기", "오들", "왼들"))
        duration = max(1, active["time_ms"] * (0.8 if early else 1.0))
        u = min(1, max(0, (timeline - active["start_ms"]) / duration))
        progress = 0.5 - 0.5 * math.cos(math.pi * u)
        for key, target in active["angles"].items():
            joint = int(key)
            start = pose.get(joint, target)
            delta = (target - start + 180) % 360 - 180
            pose[joint] = start + delta * progress
    return pose


def audit(path, catalog):
    runs = parse(path)
    origin = float(runs[0]["ticks"][0][2])
    results = []
    previous_sent = None
    for number, run in enumerate(runs, 1):
        s, rows = run["summary"], run["ticks"]
        sent = [r for r in rows if r[14] == "1"]
        # Exclude first interval: it may contain idle time, initial reads or a transition.
        following = sent[1:]
        item = {
            "number": number, "motion": s[1],
            "start_relative_s": (float(rows[0][2]) - origin) / 1000,
            "duration_ms": float(s[12]), "ticks": int(s[2]),
            "endpoints": int(s[9]), "write_failures": int(s[10]),
            "dropped_rows": int(s[13]), "queue_delay_ms": float(s[11]),
            "goal_interval_ms": stats([float(r[18]) for r in following]),
            "write_call_ms": stats([float(r[16]) for r in following]),
            "update_to_write_ms": stats([float(r[17])-float(r[2]) for r in following]),
            "head_flags": sorted({int(r[20]) for r in sent}),
            "shoulder_flags": sorted({int(r[21]) for r in sent}),
            "motor_counts": sorted({int(r[19]) for r in sent}),
        }
        if not sent:
            results.append(item)
            continue
        item["gap_from_previous_last_write_ms"] = (
            float(sent[0][17])-float(previous_sent[17]) if previous_sent else None)
        previous_sent = sent[-1]
        changes = []
        for p, r in zip(sent, following):
            deltas = [float(b)-float(a) for a, b in zip(p[22:], r[22:])]
            joint = max(range(23), key=lambda j: abs(deltas[j]))
            changes.append({"repeat": int(r[5]), "frame": int(r[6]),
                            "timeline_from_ms": float(p[8]), "timeline_to_ms": float(r[8]),
                            "interval_ms": float(r[18]), "tick_interval_ms": float(r[4]),
                            "update_to_write_ms": float(r[17])-float(r[2]),
                            "write_ms": float(r[16]), "largest_step_motor_id": joint,
                            "largest_step_deg": deltas[joint]})
        item["largest_command_step"] = max(changes, key=lambda v: abs(v["largest_step_deg"]))
        item["slow_intervals"] = [v for v in changes if v["interval_ms"] > 7.5]
        if s[1].startswith("전45도"):
            motion = catalog[s[1]]
            frames = motion["frames"]
            assert float(sent[0][8]) < frames[0]["start_ms"]
            base = dict(enumerate(map(float, sent[0][22:])))
            repeat, max_error, comparisons = int(sent[0][5]), 0.0, 0
            skipped_dynamic_head = 0
            endpoints = []
            previous_angles = base
            for r in sent:
                current_repeat = int(r[5])
                if current_repeat != repeat:
                    base, repeat = previous_angles, current_repeat
                timeline = float(r[8])
                if r[10] == "1":
                    frame = frames[int(r[6])]
                    timeline = frame["start_ms"] + frame["time_ms"]
                    endpoints.append([repeat, int(r[6])])
                expected = sample(frames, base, timeline)
                actual = dict(enumerate(map(float, r[22:])))
                # Dynamic motor-0 targets are not contained in the static catalog.
                checked = [j for j in actual if not (j == 0 and r[20] == "1")]
                skipped_dynamic_head += len(actual) - len(checked)
                max_error = max(max_error, max(abs(actual[j]-expected[j]) for j in checked))
                comparisons += len(checked)
                previous_angles = actual
            expected_endpoints = [[r, f] for r in range(1, motion["repeat_count"]+1)
                                  for f in range(len(frames))]
            item["catalog_validation"] = {
                "max_error_deg": max_error, "compared_angles": comparisons,
                "dynamic_head_angles_excluded": skipped_dynamic_head,
                "all_endpoints_in_order": endpoints == expected_endpoints,
            }
        results.append(item)
    return {"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "runs": results}


if __name__ == "__main__":
    catalog_path = ROOT / "artifacts/robot_motions_runtime.json"
    catalog = {m["name"]: m for m in json.loads(catalog_path.read_text())["motions"]}
    report = {
        "scope": "Host write calls and commanded angles; no physical yaw or gait-time actual positions.",
        "catalog_sha256": hashlib.sha256(catalog_path.read_bytes()).hexdigest(),
        "full_system": audit(OUT / "full_system_original_shoulders.csv", catalog),
        "isolated_original_shoulders": audit(
            ROOT / "artifacts/20261027_restore_original_shoulders/original_shoulders_hardware_trace.csv", catalog),
    }
    path = OUT / "full_system_analysis.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    for condition in ("full_system", "isolated_original_shoulders"):
        for r in report[condition]["runs"]:
            if r["motion"].startswith("전45도"):
                print(condition, r["number"], r["motion"], r["goal_interval_ms"],
                      r.get("catalog_validation"), r.get("largest_command_step"))
    print(path)
