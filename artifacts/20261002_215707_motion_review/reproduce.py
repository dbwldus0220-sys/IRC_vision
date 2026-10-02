"""Reproduce recorded decision boundaries without accessing robot hardware."""

from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / path) for path in (
    "src/mission_control", "src/step", "src/mission_control/test",
)]

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from step.goal_navigation_planner import GoalNavigationPlanner
from test_mission_phase_flow import MissionFlowHarness
from test_motion_decision_planner import goal_info, line_info, observations


planner = MotionDecisionPlanner(MotionDecisionConfig(line_heading_source="ground"))
weak_line = line_info(
    ground_projection_valid=False, ground_heading_error_deg=None,
    heading_error_deg=25.0, filtered_heading_error_deg=25.0,
    filtered_lateral_offset_norm=.833, heading_quality=.630,
    ground_fit_reason="too_few_segment_points",
)
line_wait = planner.plan("POST_BALL_LINE_ALIGN", observations(line=weak_line), .1)
assert line_wait.reason == "invalid_post_ball_line_alignment_input"
assert not line_wait.valid
line_search = planner.plan(
    "POST_BALL_LINE_ALIGN", observations(line=line_info(detected=False)), .1,
)
assert line_search.action == "POST_BALL_LINE_TURN_RIGHT_5"

goal_planner = GoalNavigationPlanner()
boundary_cases = []
for depth, offset, confirmed, expected in (
    (.46, 162, False, "GOAL_CAMERA90_CRAB_RIGHT"),
    (.46, 90, False, "GOAL_CAMERA90_CRAB_RIGHT"),
    (.46, 70, True, "SHOT"),
    (.47, -28, False, "WAIT_SCORE_CONFIRMATION"),
    (.47, -28, True, "SHOT"),
    (.471, -28, False, "GOAL_CAMERA90_FINE_FORWARD_1"),
):
    command = goal_planner.plan(goal_info(
        depth_m=depth, offset_x_px=offset, score_now=confirmed,
    ))
    assert command.action == expected
    boundary_cases.append({
        "depth_m": depth, "offset_x_px": offset,
        "score_confirmed": confirmed, "action": command.action,
    })

node = MissionFlowHarness(phase="GOAL_APPROACH")
node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC = 1.0
shot = planner.plan("GOAL_APPROACH", observations(goal=goal_info(
    depth_m=.47, offset_x_px=-28, score_now=True,
)), .1)
fine = planner.plan("GOAL_APPROACH", observations(goal=goal_info(
    depth_m=.471, offset_x_px=-28, score_now=False,
)), .1)
assert shot.action == "SHOT" and fine.action == "GOAL_CAMERA90_FINE_FORWARD_1"
assert not node._pre_motion_settle_ready(shot, 10.0)
assert not node._pre_motion_settle_ready(fine, 12.0)
assert node.pre_motion_settle_action == fine.action
assert not node._pre_motion_settle_ready(shot, 12.1)
assert not node._pre_motion_settle_ready(shot, 13.1)
assert node._pre_motion_settle_ready(shot, 15.1)

log_root = Path("/home/jet/.ros/log")
decision_log = log_root / "python3_6473_1790945836541.log"
bridge_log = log_root / "python3_6475_1790945835740.log"
events = []
for path in (decision_log, bridge_log):
    for line in path.read_text().splitlines():
        if not any(key in line for key in (
            "Post-ball line turn requested:", "General motion lock released:",
            "Pre-motion settle started:", "Pre-motion settle reset:",
            "Pre-motion settle complete:", "Atomic sequence advanced to goal_camera_90_crab",
            "invalid_post_ball_line_alignment_input", "action=SHOT, command_id=562",
        )):
            continue
        timestamp = float(re.search(r"\[(\d+\.\d+)\]", line)[1])
        events.append({
            "epoch_sec": timestamp,
            "local_time": datetime.fromtimestamp(timestamp, ZoneInfo("Asia/Seoul")).isoformat(),
            "log": path.name, "message": line.split("]: ", 1)[-1],
        })
events.sort(key=lambda event: event["epoch_sec"])
crab_counts = Counter(
    event["message"].split("action=")[-1]
    for event in events
    if "General motion lock released: status=SUCCEEDED, action=GOAL_CAMERA90_CRAB_" in event["message"]
)
waits = [e for e in events if "invalid_post_ball_line_alignment_input" in e["message"]]
starts = [e for e in events if "Pre-motion settle started: action=SHOT" in e["message"]]
resets = [e for e in events if "Pre-motion settle reset: previous=SHOT" in e["message"]]
shot_sent = next(e for e in events if "Special motion lock enabled: action=SHOT" in e["message"])
report = {
    "scope": "Screen recording, matching ROS logs, and synthetic boundary reproduction; no RGB-D replay or hardware execution.",
    "line_wait": {"action": line_wait.action, "reason": line_wait.reason,
                  "first_log": waits[0]["local_time"], "last_log": waits[-1]["local_time"]},
    "missing_line_search_action": line_search.action,
    "goal_boundaries": boundary_cases,
    "goal_crab_success_counts": dict(crab_counts),
    "shot_settle_resets": len(resets),
    "first_shot_settle_to_request_sec": round(shot_sent["epoch_sec"] - starts[0]["epoch_sec"], 3),
    "synthetic_1mm_depth_change_restarts_shot_settle": True,
    "events": events,
}
Path(__file__).with_name("verification.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
)
print(json.dumps({key: value for key, value in report.items() if key != "events"}, ensure_ascii=False, indent=2))
