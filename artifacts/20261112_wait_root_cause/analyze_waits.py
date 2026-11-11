"""Audit recorded waits without starting ROS nodes or touching robot hardware."""

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
from statistics import median
from zoneinfo import ZoneInfo


LINE = re.compile(r"\[(\w+)\] \[([\d.]+)\] \[([^]]+)\]: (.*)")
KST = ZoneInfo("Asia/Seoul")


def records(path):
    for number, line in enumerate(path.read_text().splitlines(), 1):
        match = LINE.match(line)
        if match:
            yield dict(time=float(match[2]), node=match[3], level=match[1],
                       message=match[4], line=number, file=str(path))


def local_time(epoch):
    return datetime.fromtimestamp(epoch, KST).isoformat(timespec="milliseconds")


def timing(rows, tag):
    samples = []
    for row in rows:
        prefix = f"[{tag}] "
        if row["message"].startswith(prefix):
            samples.append(json.loads(row["message"][len(prefix):]))
    result = {}
    for key in sorted({k for sample in samples for k in sample["summary_ms"]}):
        groups = [s["summary_ms"][key] for s in samples if key in s["summary_ms"]]
        count = sum(g["n"] for g in groups)
        result[key] = dict(n=count, weighted_mean_ms=round(
            sum(g["n"] * g["avg"] for g in groups) / count, 3),
            maximum_ms=max(g["max"] for g in groups))
    return result


def summarize(bridge_path, run_rows):
    bridge_rows = list(records(bridge_path))
    episodes = []
    counts = Counter()
    current = None
    for row in bridge_rows:
        match = re.search(r"Command ignored: .*?reason=([^,\n]+)", row["message"])
        if not match:
            current = None
            continue
        reason = match[1]
        counts[reason] += 1
        if (current is None or current["reason"] != reason
                or row["time"] - current["last_epoch"] > .35):
            current = dict(reason=reason, first_epoch=row["time"],
                           last_epoch=row["time"], count=0,
                           first_line=row["line"], last_line=row["line"])
            episodes.append(current)
        current.update(last_epoch=row["time"], last_line=row["line"],
                       count=current["count"] + 1)
    for episode in episodes:
        episode["observed_span_sec"] = round(episode["last_epoch"] - episode["first_epoch"], 3)
        episode["start_kst"] = local_time(episode["first_epoch"])
        episode["end_kst"] = local_time(episode["last_epoch"])
    fps = []
    for row in run_rows:
        match = re.search(r"effective_fps=([\d.]+)", row["message"])
        if match:
            fps.append(float(match[1]))
    no_prequeue = [r for r in run_rows if "Line completed without prequeue:" in r["message"]]
    prequeue_counts = Counter()
    for row in no_prequeue:
        match = re.search(r"reason=(.*?), usable_frames", row["message"])
        if match:
            prequeue_counts[match[1]] += 1
    diagnostics = []
    for row in run_rows:
        if "GOAL prequeue: " in row["message"]:
            data = json.loads(row["message"].split("GOAL prequeue: ", 1)[1])
            if data.get("reason") == "goal_not_detected":
                diagnostics.append(dict(time_kst=local_time(row["time"]),
                    file=row["file"], line=row["line"], **data))
    return dict(bridge_file=str(bridge_path), start_kst=local_time(bridge_rows[0]["time"]),
        files=sorted({r["file"] for r in run_rows}), ignored_reason_counts=dict(counts),
        wait_episodes=sorted(episodes, key=lambda e:e["observed_span_sec"], reverse=True),
        detector_ms=timing(run_rows, "DETECTOR_PIPELINE_TIMING"),
        ball_pipeline_ms=timing(run_rows, "BALL_PIPELINE_TIMING"),
        inference_fps_windows=(dict(min=min(fps), median=median(fps), max=max(fps)) if fps else None),
        no_prequeue_reasons=dict(prequeue_counts), no_prequeue_events=no_prequeue,
        goal_not_detected_diagnostics=diagnostics,
        faults=[r for r in run_rows if r["level"] in {"WARN", "ERROR", "FATAL"}])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, default=Path("/home/jet/.ros/log"))
    parser.add_argument("--since", default="2026-11-11T00:00:00+09:00")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("wait_summary.json"))
    args = parser.parse_args()
    since = datetime.fromisoformat(args.since).timestamp()
    files = []
    for path in args.log_dir.glob("*.log"):
        if path.stat().st_mtime < since:
            continue
        rows = list(records(path))
        if rows and rows[0]["time"] >= since:
            files.append((path, rows))
    runs = []
    for path, rows in files:
        if rows[0]["node"] != "motion_command_bridge_node":
            continue
        if not any("Command ignored:" in row["message"] for row in rows):
            continue
        # Startup logs identify companion processes; no ROS graph is launched.
        companions = [r for _, other in files if abs(other[0]["time"] - rows[0]["time"]) < 5
                      for r in other]
        runs.append(summarize(path, sorted(companions, key=lambda r:r["time"])))
    runs.sort(key=lambda r:r["start_kst"], reverse=True)
    report = dict(limitations=[
        "Ignored WAIT commands do not prove physical standstill: an existing motion may still run.",
        "Episode span is first-to-last logged identical reason with gaps <=350ms; not precise stop duration.",
        "Timing means are weighted by logged sample counts; maxima are recorded sample maxima.",
        "FPS statistics describe emitted inference windows, not camera sensor FPS or usable object FPS.",
        "Logs contain no full RGB/depth/IMU replay; blur, depth holes, and timing mismatch cannot be apportioned.",
        "Current source may differ from a historical process; historical policy is not inferred solely from current code.",
        "Companion files are grouped by startup timestamps within five seconds; filenames are preserved for review.",
    ], runs=runs)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"Saved {len(runs)} runs to {args.output}")
    for run in runs[:8]:
        print(run["start_kst"], Path(run["bridge_file"]).name)
        for episode in [e for e in run["wait_episodes"]
                        if e["reason"] != "mission_locked_waiting_for_motion_status"][:4]:
            print(" ", episode["start_kst"], episode["observed_span_sec"], episode["reason"])


if __name__ == "__main__":
    main()
