"""A changing line observation must not starve an equivalent physical turn."""

import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision
from test_mission_phase_flow import MissionFlowHarness


def decision(action, *, valid=True, source="line"):
    return MotionDecision("LINE_TRACK_AFTER_PICKUP", source, action, valid,
                          "test", False, False, {})


def harness():
    node = MissionFlowHarness(phase="LINE_TRACK_AFTER_PICKUP")
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    return node


@pytest.mark.parametrize("suffix", ["LEFT_1", "LEFT_3", "RIGHT_2", "RIGHT_5"])
@pytest.mark.parametrize("first", ["OFFSET", "LOST"])
def test_alternating_search_and_alignment_execute_current_decision_after_one_second(suffix, first):
    node = harness()
    actions = [f"LINE_{first}_TURN_{suffix}",
               f"LINE_{'LOST' if first == 'OFFSET' else 'OFFSET'}_TURN_{suffix}"]
    # Keep this equivalence tied to the actual executor mapping.
    bridge = MotionCommandBridgeNode
    assert bridge.ACTION_TO_MOTION_ID[actions[0]] == bridge.ACTION_TO_MOTION_ID[actions[1]]
    for i, elapsed in enumerate((0., .1, .2, .3, .4, .5, .6, .7, .8, .999)):
        assert not node._pre_motion_settle_ready(decision(actions[i % 2]), 10. + elapsed)
        assert node.pre_motion_settle_started_at == 10.
    assert node._pre_motion_settle_ready(decision(actions[1]), 11.)
    assert node.pre_motion_settle_started_at is None


@pytest.mark.parametrize("changed", ["LINE_LOST_TURN_LEFT_1", "LINE_LOST_TURN_RIGHT_5"])
def test_different_direction_or_amount_restarts_pause(changed):
    node = harness()
    assert not node._pre_motion_settle_ready(decision("LINE_OFFSET_TURN_RIGHT_2"), 10.)
    assert not node._pre_motion_settle_ready(decision(changed), 10.5)
    assert node.pre_motion_settle_started_at == 10.5
    assert not node._pre_motion_settle_ready(decision(changed), 11.)
    assert node._pre_motion_settle_ready(decision(changed), 11.5)


@pytest.mark.parametrize("interruption", ["invalid", "walking", "corner", "locked"])
def test_interruption_discards_pending_turn_pause(interruption):
    node = harness()
    turn = decision("LINE_OFFSET_TURN_RIGHT_2")
    assert not node._pre_motion_settle_ready(turn, 10.)
    if interruption == "locked":
        assert node.phase_manager.start_special_action("PICKUP_NOW", 1)
        assert not node._pre_motion_settle_ready(turn, 10.5)
    else:
        sample = (decision("WAIT", valid=False) if interruption == "invalid"
                  else decision("RIGHT" if interruption == "corner" else "STRAIGHT_1"))
        assert node._pre_motion_settle_ready(sample, 10.5)
    assert node.pre_motion_settle_started_at is None
    if interruption != "locked":
        assert not node._pre_motion_settle_ready(decision("LINE_LOST_TURN_RIGHT_2"), 11.)
        assert node.pre_motion_settle_started_at == 11.
