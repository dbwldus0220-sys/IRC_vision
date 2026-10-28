#!/usr/bin/env python3
"""Summarize executor callback traces without contacting ROS or robot hardware."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

TOP_LEVEL = {"poll_timer", "request", "cancel", "ball_info", "navigation", "hurdle_info", "heartbeat"}


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return {"count": len(values), "mean": statistics.mean(values),
            "p95": values[math.ceil(0.95 * len(values)) - 1],
            "p99": values[math.ceil(0.99 * len(values)) - 1], "max": values[-1]}


def read_trace(path):
    records, sessions = [], []
    with path.open() as stream:
        for row in csv.reader(stream):
            if not row:
                continue
            if row[0].startswith("# session"):
                sessions.append({"records": int(row[2]), "dropped": int(row[4])})
            elif row[0] != "callback":
                records.append({"callback": row[0], "start_ms": float(row[1]),
                                "end_ms": float(row[2]), "wall_ms": float(row[3]),
                                "cpu_ms": float(row[4]), "active": row[5] == "1",
                                "session": len(sessions)})
    return sorted(records, key=lambda row: row["start_ms"]), sessions


def summarize(records):
    active = [r for r in records if r["active"]]
    groups = {}
    for name in sorted({r["callback"] for r in active}):
        rows = [r for r in active if r["callback"] == name]
        groups[name] = {"wall_ms": stats([r["wall_ms"] for r in rows]),
                        "thread_cpu_ms": stats([r["cpu_ms"] for r in rows if r["cpu_ms"] >= 0])}
    polls = [r for r in records if r["callback"] == "poll_timer"]
    intervals, slow = [], []
    for previous, current in zip(polls, polls[1:]):
        if not previous["active"] or not current["active"] or previous["session"] != current["session"]:
            continue
        gap = current["start_ms"] - previous["start_ms"]
        intervals.append(gap)
        if gap <= 7.5:
            continue
        between = [r for r in records if r["callback"] in TOP_LEVEL - {"poll_timer"}
                   and r["session"] == current["session"]
                   and previous["end_ms"] <= r["start_ms"] < current["start_ms"]]
        other_wall = sum(min(r["end_ms"], current["start_ms"]) - r["start_ms"] for r in between)
        slow.append({"start_ms": previous["start_ms"], "next_start_ms": current["start_ms"],
                     "interval_ms": gap, "previous_poll_wall_ms": previous["wall_ms"],
                     "previous_poll_cpu_ms": previous["cpu_ms"],
                     "other_callback_wall_ms": other_wall,
                     "unmeasured_between_callbacks_ms": max(0, current["start_ms"] - previous["end_ms"] - other_wall),
                     "other_callbacks": between})
    return {"callbacks": groups, "active_poll_interval_ms": stats(intervals),
            "over_7_5_count": len(slow), "slow_intervals": slow}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--phases", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    records, sessions = read_trace(args.trace)
    report = {"trace": str(args.trace), "sessions": sessions,
              "limitations": "Nested scopes overlap. Unmeasured time may include ROS dispatch and OS scheduling; it is not proof of CPU starvation. Thread CPU excludes blocked/descheduled time. Abrupt termination may lose buffered data.",
              "all": summarize(records)}
    if args.phases:
        metadata = json.loads(args.phases.read_text())
        report["scope"] = metadata["scope"]
        report["phases"] = {p["name"]: summarize([
            r for r in records if p["start_ms"] <= r["start_ms"] and r["end_ms"] <= p["end_ms"]
        ]) for p in metadata["phases"]}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    for name, result in report.get("phases", {"all": report["all"]}).items():
        print(name, "poll interval ms:", result["active_poll_interval_ms"],
              "over 7.5ms:", result["over_7_5_count"])
        for key in ("head_update", "ball_info", "hurdle_info", "navigation"):
            if key in result["callbacks"]:
                print(" ", key, result["callbacks"][key])


if __name__ == "__main__":
    main()
