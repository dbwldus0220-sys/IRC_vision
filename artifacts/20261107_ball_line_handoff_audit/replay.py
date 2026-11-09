"""Read-only planner reproduction; no ROS initialization or motor connection."""

import json

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_entry_line_alignment import ground_ball_info
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info


planner = MotionDecisionPlanner()
cases = [
    ("no_ball_ground_line_right", line_info(30, .3),
     ball_info(bearing_deg=-30), "RECOVER_RIGHT_TURN_RIGHT_4"),
    ("no_ball_ground_line_straight", line_info(),
     ball_info(bearing_deg=-30), "STRAIGHT_2"),
    ("no_ball_ground_no_line", None,
     ball_info(bearing_deg=-30), "STRAIGHT_2"),
    ("ball_ground_left_line_right", line_info(30, .3),
     ground_ball_info(ground_steering_angle_deg=-30,
                      ground_forward_distance_m=1.2),
     "RECOVER_LEFT_TURN_LEFT_4"),
]
results = []
for name, line, ball, expected in cases:
    entry = BallEntryLineAlignment()
    entry.arm()
    decision = entry.select(planner, line, 0., ball)
    assert decision.action == expected and decision.valid
    results.append({"case": name, "action": decision.action,
                    "valid": decision.valid, "reason": decision.reason,
                    "heading_source": decision.source_command.get("heading_source"),
                    "heading": decision.source_command.get("heading_error_deg")})

# The last case acquired Ball ground geometry; losing it must not resume Line.
decision = entry.select(planner, line_info(30, .3), 0., ball_info(bearing_deg=-30))
assert decision.action == "STOP" and not decision.valid
results.append({"case": "ball_ground_lost_after_acquisition",
                "action": decision.action, "valid": decision.valid,
                "reason": decision.reason})
print(json.dumps(results, indent=2))
