"""Compare saved host-side traces; never connects to robot hardware."""

import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


def distribution(values):
    ordered = sorted(values)
    return {
        "mean": statistics.mean(ordered),
        "p99_nearest_rank": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max": ordered[-1],
    }


def analyze(path):
    runs = []
    with path.open() as stream:
        for row in csv.reader(stream):
            if row[0] == "summary":
                runs.append({"summary": row, "ticks": []})
            elif row[0] == "tick":
                runs[-1]["ticks"].append(row)
    results = []
    for run in runs:
        summary = run["summary"]
        sent = [row for row in run["ticks"] if row[14] == "1"]
        # The first update reads actual positions; its interval can include idle time.
        following = sent[1:]
        endpoints = [row for row in sent if row[10] == "1"]
        results.append({
            "motion": summary[1],
            "elapsed_ms": float(summary[12]),
            "ticks": int(summary[2]),
            "endpoints": int(summary[9]),
            "write_failures": int(summary[10]),
            "dropped_rows": int(summary[13]),
            "goal_interval_ms": distribution([float(r[18]) for r in following]),
            "write_call_ms": distribution([float(r[16]) for r in following]),
            "update_to_write_ms": distribution([
                float(r[17]) - float(r[2]) for r in following
            ]),
            "initial_read_and_compute_ms": float(sent[0][17]) - float(sent[0][2]),
            "head_override_flags": sorted({int(r[20]) for r in sent}),
            "shoulder_override_flags": sorted({int(r[21]) for r in sent}),
            "motor_counts": sorted({int(r[19]) for r in sent}),
            "initial_pose_deg": [float(v) for v in sent[0][22:]],
            "endpoint_goals": [
                {"repeat": int(r[5]), "frame": int(r[6]),
                 "angles": [float(v) for v in r[22:]]} for r in endpoints
            ],
        })
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "runs": results}


if __name__ == "__main__":
    folder = Path(__file__).resolve().parent
    on = analyze(folder.parent / "20261027_forward_execution_diagnosis/isolated_hardware_trace.csv")
    off = analyze(folder / "original_shoulders_hardware_trace.csv")
    baseline = on["runs"][0]["endpoint_goals"]
    max_other_error = 0.0
    for run in on["runs"] + off["runs"]:
        assert len(run["endpoint_goals"]) == len(baseline) == 24
        for actual, expected in zip(run["endpoint_goals"], baseline):
            assert (actual["repeat"], actual["frame"]) == (expected["repeat"], expected["frame"])
            max_other_error = max(max_other_error, max(
                abs(a - b) for joint, (a, b) in enumerate(zip(actual["angles"], expected["angles"]))
                if joint not in (4, 5)
            ))
    report = {
        "scope": "Host command timing and target angles; no gait-time measured joint positions or yaw.",
        "limitations": "Two runs per condition; initial actual poses differ. No dynamic head override in these traces. update_to_write excludes node work before SDK update.",
        "max_endpoint_difference_excluding_shoulders_deg": max_other_error,
        "shoulder_override_on": on,
        "shoulder_override_off": off,
    }
    destination = folder / "override_timing_comparison.json"
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(destination)
    print("Max non-shoulder endpoint difference (deg):", max_other_error)
