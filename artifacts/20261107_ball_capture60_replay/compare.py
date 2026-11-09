#!/usr/bin/env python3
"""Compare the same synthetic inputs with only the capture-window change toggled."""
import csv
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "entry_replay", ROOT / "artifacts/20261107_ball_entry_replay/replay.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
import pytest
import mission_control.motion_decision_node as decision_module

namespace = dict(vars(decision_module))
exec(compile((OUT / "before_methods.py").read_text(), "before_methods.py", "exec"), namespace)
METHODS = ("_prepare_pending_ball_entry_walk", "_walking_prequeue_allowed")
scenarios = [(f"earlier_{advance}_hits_{hits}", hits, {"earlier_sec": advance})
             for advance in (0., .15, .30, .50) for hits in (12, 10)]
scenarios += [("missing_tenth", 10, {"missing_tenth": True}),
              ("stale_capture", 10, {"capture_age": .6})]
# Extra sensitivity cases: confirmation can now arrive in the 60--70% window.
scenarios += [(f"earlier_{advance}_hits_10", 10, {"earlier_sec": advance})
              for advance in (.7, .8, .9, 1.0)]
results = []
for name, hits, kwargs in scenarios:
    for version in ("before", "after"):
        with pytest.MonkeyPatch.context() as patch:
            if version == "before":
                for method in METHODS:
                    patch.setattr(replay.Node, method, namespace[method])
            result = replay.run(hits, **kwargs)
        results.append({"scenario": name, "version": version, **result})

previous = json.loads((ROOT / "artifacts/20261107_ball_entry_replay/results.json").read_text())
for index, expected in enumerate(previous):
    result = results[index * 2]
    assert result["command_relative_ms"] == expected["command_relative_ms"]
    assert result.get("queued_before_boundary") == expected.get("queued_before_boundary")
for before, after in zip(results[::2], results[1::2]):
    assert before["trace"][:1] == after["trace"][:1]
    assert before.get("confirmed_relative_ms") == after.get("confirmed_relative_ms")
    assert before.get("action") == after.get("action")
    assert before.get("motion_id") == after.get("motion_id")
    if after["command_relative_ms"] is not None:
        assert after["command_relative_ms"] <= before["command_relative_ms"]
    else:
        assert before["command_relative_ms"] is None

(OUT / "results.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")
rows = [{k: v for k, v in r.items() if k not in ("trace", "last_decision")}
        for r in results]
with (OUT / "results.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=sorted({k for row in rows for k in row}))
    writer.writeheader()
    writer.writerows(rows)
for before, after in zip(results[::2], results[1::2]):
    print(json.dumps({"scenario": before["scenario"],
                      "confirmed_ms": after.get("confirmed_relative_ms"),
                      "before_command_ms": before["command_relative_ms"],
                      "after_command_ms": after["command_relative_ms"],
                      "before_queued": before.get("queued_before_boundary"),
                      "after_queued": after.get("queued_before_boundary")}, ensure_ascii=False))
