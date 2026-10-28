"""Replay scoring-depth pixel alignment through real input and motion callbacks."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from test_line_corner_memory import CornerHarness
from test_mission_phase_flow import mark_next_ball_grabbed, release_general, score_ready_goal


@pytest.mark.parametrize("direction,sign,count", [("RIGHT", 1, 2), ("LEFT", -1, 1)])
@pytest.mark.parametrize("final_offset,next_action", [(140, "CRAB"), (80, "SHOT")])
def test_large_offset_repeats_only_after_one_second_and_new_capture(
    monkeypatch, direction, sign, count, final_offset, next_action,
):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns",
                        staticmethod(lambda _: round(now[0] * 1e9)))
    node = CornerHarness(phase="GOAL_APPROACH")
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    mark_next_ball_grabbed(node)

    def sample(offset, captured_at=None):
        return {
            **score_ready_goal(), "offset_x_px": offset,
            "offset_x_norm": offset / 640,
            "rgb_stamp_ns": round((now[0] if captured_at is None else captured_at) * 1e9),
        }

    expected_turn = f"GOAL_CAMERA90_TURN_{direction}_{count}"
    for _ in range(3):
        command = node.publish_vision(goal=sample(sign * 200))[-1]
        assert command["action"] == expected_turn
        now[0] += .1
        assert node.publish_vision(goal=sample(sign * 200)) == []
        now[0] += .9
        release_general(node, command)
        deadline = now[0] + 1.0
        assert node.goal_post_motion_dwell_until == pytest.approx(deadline)
        now[0] = deadline - .001
        assert node.publish_vision(goal=sample(sign * 200)) == []
        now[0] = deadline
        assert node.publish_vision(goal=sample(sign * 200)) == []
        assert node.latest_info["goal"] is None
        now[0] += .01
        # A delayed image captured during the pause cannot repeat the turn.
        node.publish_vision(goal=sample(sign * 200, deadline - .01))
        assert node.latest_info["goal"] is None
        assert not node.general_motion_gate.locked
        now[0] += .01

    commands = node.publish_vision(goal=sample(sign * final_offset))
    expected = f"GOAL_CAMERA90_CRAB_{direction}" if next_action == "CRAB" else "SHOT"
    assert commands[-1]["action"] == expected
    turns = [m for m in node.publisher.messages if m["action"] == expected_turn]
    assert len(turns) == 3
    assert len({m["command_id"] for m in turns}) == 3
