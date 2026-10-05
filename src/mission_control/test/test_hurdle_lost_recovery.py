"""Exercise hurdle loss recovery through planning, gating and motion mapping."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_hurdle_positioning import HurdleHarness, clock, hurdle
from test_mission_phase_flow import release_general
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


def visible(offset=400, depth=.6, **updates):
    return hurdle(depth, camera_center_offset_x_px=offset, **updates)


@pytest.mark.parametrize("offset,direction,count", [(400, "RIGHT", 2), (-400, "LEFT", 1)])
def test_loss_repeats_small_stationary_turn_and_maps_to_catalog(offset, direction, count):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": visible(offset)}, .1)
    for _ in range(3):
        decision = planner.plan("AUTO", {"hurdle": {"detected": False}}, .1)
        assert decision.valid and decision.action == f"ALIGN_{direction}"
        assert decision.source_command["turn_count"] == count
        assert decision.source_command["recovery_active"]
        assert not decision.requires_ack
        bridge = FakeBridge()
        bridge.navigation_command_callback(navigation_message(
            source="hurdle", action=decision.action, source_command=decision.source_command,
        ))
        requests = decoded_messages(bridge.executor_request_publisher)
        assert requests[-1]["motion_id"] == f"post_ball_line_turn_{direction.lower()}_{count}"


@pytest.mark.parametrize("angle,direction,count", [(12, "LEFT", 1), (-12, "RIGHT", 2)])
def test_reacquisition_aligns_parallel_before_near_go(angle, direction, count):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": visible()}, .1)
    planner.plan("AUTO", {"hurdle": {"detected": False}}, .1)
    decision = planner.plan("AUTO", {
        "hurdle": visible(depth=.5, hurdle_angle_deg=angle),
    }, .1)
    assert decision.action == f"ALIGN_{direction}"
    assert decision.source_command["turn_count"] == count
    assert not decision.source_command["fine_sequence_requested"]
    ready = planner.plan("AUTO", {
        "hurdle": visible(depth=.5, hurdle_angle_deg=0),
    }, .1)
    assert ready.action == "GO"
    assert not planner.hurdle_planner.recovery_active


@pytest.mark.parametrize("angle", [-8, 0, 8])
def test_parallel_tolerance_resumes_fine_approach(angle):
    planner = HurdleNavigationPlanner()
    planner.plan(visible(), positioning=True)
    planner.plan({"detected": False}, positioning=True)
    decision = planner.plan(visible(hurdle_angle_deg=angle), positioning=True)
    assert decision.action == "STRAIGHT_0"
    assert not planner.recovery_active


@pytest.mark.parametrize("updates", [
    {"hurdle_angle_deg": None}, {"depth_valid": False}, {"confidence": .1},
    {"confirmation_confirmed": False}, {"bottom_distance_px": None},
])
def test_invalid_reacquisition_cannot_resume_forward_or_go(updates):
    planner = HurdleNavigationPlanner()
    planner.plan(visible(), positioning=True)
    planner.plan({"detected": False}, positioning=True)
    decision = planner.plan(visible(depth=.5, **updates), positioning=True)
    assert decision.action == "WAIT" and not decision.valid
    assert planner.recovery_active


@pytest.mark.parametrize("updates", [
    {"camera_center_offset_x_px": 0}, {"camera_center_offset_x_px": 64},
    {"camera_center_offset_x_px": -64}, {"bottom_distance_px": None},
    {"image_width": None},
])
def test_unknown_side_loss_stays_wait_and_clears_old_side(updates):
    planner = HurdleNavigationPlanner()
    planner.plan(visible(), positioning=True)
    sample = visible()
    sample.update(updates)
    planner.observe(sample)
    decision = planner.plan({"detected": False}, positioning=True)
    assert decision.action == "WAIT" and not decision.valid


@pytest.mark.parametrize("bottom", [0, 20, 100])
def test_bottom_loss_repeats_one_cycle_retreat_until_visible(bottom):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": visible(bottom_distance_px=bottom)}, .1)
    for _ in range(3):
        decision = planner.plan("AUTO", {"hurdle": {"detected": False}}, .1)
        assert decision.action == "HURDLE_LOST_BACKWARD_1" and decision.valid
        assert not decision.requires_ack
        bridge = FakeBridge()
        bridge.navigation_command_callback(navigation_message(
            source="hurdle", action=decision.action, source_command=decision.source_command,
        ))
        requests = decoded_messages(bridge.executor_request_publisher)
        assert len(requests) == 1
        assert requests[0]["motion_id"] == "pickup_lost_ball_backward_1"
    ready = planner.plan("AUTO", {"hurdle": visible()}, .1)
    assert ready.action == "STRAIGHT_0"


@pytest.mark.parametrize("info", [None, {}, {"detected": False, "raw_detected": True},
                                  visible(depth_valid=False), visible(confidence=.1)])
def test_bottom_recovery_does_not_retreat_on_stale_pending_or_visible_input(info):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": visible(bottom_distance_px=50)}, .1)
    decision = planner.plan("AUTO", {"hurdle": info}, .1)
    assert decision.action == "WAIT" and not decision.valid


def test_bottom_recovery_resets_with_mission():
    planner = HurdleNavigationPlanner()
    planner.observe(visible(bottom_distance_px=50))
    planner.reset()
    assert planner.plan({"detected": False}).action == "WAIT"


def test_bottom_retreat_alias_is_requested_runtime_motion():
    import json
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text())
    name = aliases["motion_aliases"]["pickup_lost_ball_backward_1"]
    assert name == "찐후진실전(1회)"
    motions = json.loads((root / "artifacts/robot_motions_runtime.json").read_text())["motions"]
    motion = next(m for m in motions if m["name"] == name)
    assert motion["repeat_count"] == 1


def test_bottom_retreat_waits_for_completion_pause_and_fresh_reacquisition(clock):
    node = HurdleHarness()
    node.receive(visible())
    fine = node.publish()[-1]
    node.receive(visible(bottom_distance_px=50))
    node.receive({"detected": False})
    assert node.publish() == []
    release_general(node, fine)
    clock[0] += 1.1
    node.publish()
    node.receive({"detected": False})
    backward = node.publish()[-1]
    assert backward["action"] == "HURDLE_LOST_BACKWARD_1"
    assert node.publish() == []
    node.send_status(backward["action"], backward["command_id"], "RUNNING")
    node.send_status(backward["action"], backward["command_id"], "SUCCEEDED")
    clock[0] += .5
    node.receive({"detected": False})
    assert node.publish() == []
    clock[0] += .6
    node.publish()
    assert not any(m.get("valid") for m in node.publish())
    node.receive({"detected": False})
    repeated = node.publish()[-1]
    assert repeated["action"] == "HURDLE_LOST_BACKWARD_1"
    node.receive(visible())
    assert node.publish() == []
    node.send_status(repeated["action"], repeated["command_id"], "RUNNING")
    node.send_status(repeated["action"], repeated["command_id"], "SUCCEEDED")
    clock[0] += 1.1
    node.publish()
    node.receive(visible())
    resumed = node.publish()[-1]
    assert resumed["action"] == "STRAIGHT_0"


def test_screen_center_takes_precedence_over_calibrated_robot_offset():
    planner = HurdleNavigationPlanner()
    planner.plan(visible(-400, center_x=1100), positioning=True)
    assert planner.plan({"detected": False}, positioning=True).action == "ALIGN_RIGHT"


def test_timeout_malformed_input_and_pending_confirmation_do_not_turn():
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": visible()}, .1)
    planner.plan("AUTO", {"hurdle": {"detected": False}}, .1)
    for info in (None, {}, {"detected": False, "raw_detected": True}):
        decision = planner.plan("AUTO", {"hurdle": info}, .1)
        assert decision.action == "WAIT" and not decision.valid


def test_reset_discards_search_direction_and_alignment_state():
    planner = HurdleNavigationPlanner()
    planner.plan(visible(), positioning=True)
    planner.plan({"detected": False}, positioning=True)
    planner.reset()
    assert not planner.recovery_active and planner.last_seen_side is None
    assert planner.plan({"detected": False}, positioning=True).action == "WAIT"


def test_callback_remembers_side_during_motion_and_blocks_early_go(clock):
    node = HurdleHarness()
    node.receive(visible(0))
    fine = node.publish()[-1]
    assert fine["action"] == "STRAIGHT_0"
    node.receive(visible(-400))
    node.receive({"detected": False})
    assert node.publish() == []
    assert node.planner.hurdle_planner.recovery_active
    # A close reacquisition must not latch the atomic GO before alignment.
    node.receive(visible(-400, depth=.5, hurdle_angle_deg=20))
    assert getattr(node, "pending_hurdle_final_sequence", None) is None
    release_general(node, fine)
    clock[0] += 1.1
    node.publish()  # Discard images received before the post-motion pause.
    node.receive(visible(-400, depth=.5, hurdle_angle_deg=20))
    turn = node.publish()[-1]
    assert turn["action"] == "ALIGN_LEFT"
    assert turn["source_command"]["turn_count"] == 1
    assert node.publish() == []


def test_search_waits_for_completion_pause_and_new_vision(clock):
    node = HurdleHarness()
    node.receive(visible())
    fine = node.publish()[-1]
    node.receive({"detected": False})
    release_general(node, fine)
    clock[0] += 1.1
    node.publish()
    node.receive({"detected": False})
    turn = node.publish()[-1]
    assert turn["action"] == "ALIGN_RIGHT"
    node.receive({"detected": False})
    assert node.publish() == []
    release_general(node, turn)
    clock[0] += .5
    node.receive({"detected": False})
    assert node.publish() == []
    clock[0] += .6
    node.publish()
    node.publish()  # Both general correction and hurdle pause invalidate old input.
    assert not any(m.get("valid") for m in node.publish())
    node.receive({"detected": False})
    repeated = node.publish()[-1]
    assert repeated["action"] == "ALIGN_RIGHT" and repeated["valid"]
