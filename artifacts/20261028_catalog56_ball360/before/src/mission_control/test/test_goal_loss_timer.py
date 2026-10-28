"""Goal loss uses new post-motion camera frames instead of planner call intervals."""

import pytest

from mission_control.goal_loss_timer import GoalLossTimer
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_line_corner_memory import CornerHarness
from test_mission_phase_flow import mark_next_ball_grabbed, release_general
from test_motion_decision_planner import goal_info


def missing(t, **changes):
    return {'detected': False, 'rgb_stamp_ns': round(t * 1e9), **changes}


@pytest.mark.parametrize('invalid', [None, {}, {'detected': False},
                                     missing(10, raw_detected=True),
                                     missing(10, rgb_stamp_ns=True)])
def test_no_loss_evidence_from_missing_or_unconfirmed_input(invalid):
    timer = GoalLossTimer()
    timer.observe(missing(9.8), 9.8)
    timer.observe(invalid, 10.)
    assert timer.started_at is None
    assert timer.elapsed_sec == 0.


def test_duplicate_old_frames_and_delayed_burst_cannot_advance_loss():
    timer = GoalLossTimer()
    timer.observe(missing(10), 10)
    timer.observe(missing(10), 10.4)
    timer.observe(missing(9.9), 10.4)
    assert timer.elapsed_sec == 0.
    timer.observe(missing(10.4), 10.01)
    assert timer.elapsed_sec == pytest.approx(.01)


@pytest.mark.parametrize('next_capture,next_receive', [(10.6, 10.6), (10.4, 11.)])
def test_camera_or_receive_gap_starts_a_new_loss(next_capture, next_receive):
    timer = GoalLossTimer()
    timer.observe(missing(10), 10)
    timer.observe(missing(next_capture), next_receive)
    assert timer.elapsed_sec == 0.


def test_first_miss_does_not_inherit_a_large_planner_dt(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_planner.time.monotonic', lambda: now[0])
    planner = MotionDecisionPlanner()
    planner.plan('GOAL_APPROACH', {'goal': goal_info(depth_m=1., bearing_deg=3.8)}, .1)
    now[0] = 13.
    stopped = planner.plan('GOAL_APPROACH', {'goal': missing(13)}, 3.)
    assert stopped.action == 'GOAL_LOST_STOP' and not stopped.valid
    now[0] = 13.4
    repeated = planner.plan('GOAL_APPROACH', {'goal': missing(13)}, 3.)
    assert repeated.action == 'GOAL_LOST_STOP'
    search = planner.plan('GOAL_APPROACH', {'goal': missing(13.4)}, 3.)
    assert search.action == 'GOAL_CAMERA90_TURN_RIGHT_2'
    assert search.source_command['lost_elapsed_sec'] == .4
    planner.plan('GOAL_APPROACH', {'goal': goal_info(depth_m=1.)}, .1)
    now[0] = 13.5
    assert planner.plan('GOAL_APPROACH', {'goal': missing(13.5)}, 3.).action == 'GOAL_LOST_STOP'


def test_node_waits_for_post_motion_frames_and_continuous_loss(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', staticmethod(lambda node: round(now[0] * 1e9)))
    node = CornerHarness(phase='GOAL_APPROACH')
    mark_next_ball_grabbed(node)
    forward = node.publish_vision(goal=goal_info(depth_m=1., bearing_deg=0., rgb_stamp_ns=10_000_000_000))[-1]
    assert forward['action'] == 'GOAL_CAMERA_90_FORWARD_2'
    now[0] = 12.
    assert node.publish_vision(goal=missing(12)) == []
    assert node.planner.goal_loss_timer.started_at is None
    now[0] = 13.
    release_general(node, forward)
    now[0] = 13.01
    node.publish_vision(goal=missing(12.99))
    assert node.latest_info['goal'] is None
    assert node.planner.goal_loss_timer.started_at is None
    assert node.publish_vision(goal=missing(now[0]))[-1]['action'] == 'GOAL_LOST_STOP'
    now[0] = 13.2
    assert node.publish_vision(goal=missing(now[0]))[-1]['action'] == 'GOAL_LOST_STOP'
    now[0] = 13.4
    search = node.publish_vision(goal=missing(now[0]))[-1]
    assert search['action'] == 'GOAL_CAMERA90_TURN_RIGHT_5'
    now[0] = 14.
    release_general(node, search)
    assert node.planner.goal_lost_elapsed_sec == 0.
    now[0] = node.goal_post_motion_dwell_until
    assert node.publish_vision(goal=missing(now[0])) == []
    now[0] += .01
    node.publish_vision(goal=missing(now[0] - .02))
    assert node.latest_info['goal'] is None
    assert node.publish_vision(goal=missing(now[0]))[-1]['action'] == 'GOAL_LOST_STOP'


def test_expired_goal_input_restarts_loss_instead_of_search(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_planner.time.monotonic', lambda: now[0])
    planner = MotionDecisionPlanner()
    planner.plan('GOAL_APPROACH', {'goal': goal_info(depth_m=1.)}, .1)
    planner.plan('GOAL_APPROACH', {'goal': missing(10)}, .1)
    now[0] = 12.
    assert not planner.plan('GOAL_APPROACH', {}, 2.).valid
    assert planner.plan('GOAL_APPROACH', {'goal': missing(12)}, 2.).action == 'GOAL_LOST_STOP'
