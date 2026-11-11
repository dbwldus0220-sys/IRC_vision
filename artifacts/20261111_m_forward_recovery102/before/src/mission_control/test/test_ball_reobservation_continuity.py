"""Replay measured Ball latency and bounded motion between candidate frames."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_approach_observation import BallApproachObservation
from mission_control.motion_decision_node import MotionDecisionNode as Node
from step.ball_navigation_planner import BallNavigationConfig
from test_ball_approach_latency import sample
from test_ball_entry_line_alignment import node_with_entry, start_line
from test_ball_entry_reobservation import observation
from test_continuous_approach import clock
from test_motion_command_bridge import FakeBridge


def move_box(info, *, shift=0., scale=1.):
    half = 25 * scale
    center = 625 + shift
    info.update(bbox=[center - half, 265 - half, center + half, 265 + half],
                center_x=center, center_y=265, camera_center_offset_x_px=center - 640)
    return info


@pytest.mark.parametrize('delay', [.115, .23, .24, .269, .331, .35])
@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('moving', [False, True])
def test_measured_delays_and_progressive_motion_reserve_from_third_current_frame(clock, delay, index, moving):
    node, bridge = node_with_entry(), FakeBridge()
    start_line(node, bridge, clock)
    start, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    callback = Node._info_callback(node, 'ball')

    def receive(captured, confirmed, frame):
        clock[0] = captured + delay
        info = observation([captured], confirmed=confirmed, angle=-25.)
        if moving:
            move_box(info, shift=30 * frame, scale=1.1 ** frame)
        callback(String(data=json.dumps(info)))

    receive(start + duration * .69, True, 0)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    trusted_stamp = node.ball_approach_observation.reference['rgb_stamp_ns']
    for frame in range(1, 4):
        receive(start + duration * .72 + (frame - 1) * .04, False, frame)
        if frame < 3:
            assert node.queued_general_command_id is None
            assert node.ball_approach_observation.reference['rgb_stamp_ns'] == trusted_stamp
    assert clock[0] < start + duration
    assert node.queued_general_action == 'RECOVER_LEFT_TURN_LEFT_4'
    command = node.publisher.messages[-1]
    assert command['source_command']['ball_entry_ground_tracking']
    assert node.latest_info['ball']['detected'] is False
    assert not node.ball_pickup_entry_pending


@pytest.mark.parametrize('failure', ['stale_rgb', 'stale_depth', 'missing_depth', 'future_rgb'])
def test_invalid_current_observations_cannot_seed_or_extend_reconfirmation(failure):
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10.23, config, preserve_gaps=True, capture_now_ns=10_230_000_000)
    for i in range(1, 4):
        captured = 10. + .04 * i
        raw = sample(captured, detected=False)
        delay = .351 if failure == 'stale_rgb' else -.01 if failure == 'future_rgb' else .23
        if failure == 'stale_depth':
            raw['approach_candidate']['depth_age_sec'] = .201
        elif failure == 'missing_depth':
            raw['approach_candidate']['distance_m'] = None
        observer.observe(raw, captured + delay, config, preserve_gaps=True,
                         capture_now_ns=round((captured + delay) * 1e9))
        assert observer.observation is None
        assert observer.hits == 0
        assert observer.tracking_reference is None


@pytest.mark.parametrize('change', ['center', 'area', 'distance'])
def test_locally_matching_candidates_cannot_exceed_trusted_track_drift(change):
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10., config, preserve_gaps=True)
    for i in range(1, 4):
        raw = sample(10. + .04 * i, detected=False)
        if change == 'center':
            move_box(raw, shift=1280 * .04 * i)
        elif change == 'area':
            move_box(raw, scale=1.15 ** i)
        else:
            raw['approach_candidate']['distance_m'] = 1.2 - .06 * i
        observer.observe(raw, 10. + .04 * i, config, preserve_gaps=True)
        assert observer.observation is None
    assert observer.hits == 0
    assert observer.reference is None


def test_first_candidate_still_needs_to_match_confirmed_target():
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10., config, preserve_gaps=True)
    for i in range(1, 5):
        raw = move_box(sample(10. + .04 * i, detected=False), shift=100)
        observer.observe(raw, 10. + .04 * i, config, preserve_gaps=True)
        assert observer.observation is None and observer.candidate is None


def test_short_miss_keeps_candidate_chain_without_renewing_reference():
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10., config, preserve_gaps=True)
    for i in (1, 2):
        observer.observe(move_box(sample(10. + .04 * i, detected=False), shift=30 * i),
                         10. + .04 * i, config, preserve_gaps=True)
    observer.observe(dict(detected=False, raw_detected=False, rgb_stamp_ns=10_120_000_000),
                     10.12, config, preserve_gaps=True)
    assert observer.observation is None and observer.candidate is None
    assert observer.reference_received_at == 10.
    assert observer.hits == 2
    observer.observe(move_box(sample(10.16, detected=False), shift=90), 10.16, config, preserve_gaps=True)
    assert observer.is_current(10.16)
    assert observer.observation['bbox'] == [690., 240., 740., 290.]


def test_provisional_motion_cannot_extend_expired_trusted_reference():
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10.), 10., config, preserve_gaps=True)
    for captured, shift in [(10.2, 30), (10.4, 60), (10.51, 90)]:
        observer.observe(move_box(sample(captured, detected=False), shift=shift),
                         captured, config, preserve_gaps=True)
        assert observer.observation is None
    assert observer.reference_received_at == 10.
    assert observer.last_rejection_reason == 'reference_expired'


@pytest.mark.parametrize('delay', [.351, -.01])
def test_latest_capture_gate_blocks_even_a_previously_verified_candidate(clock, delay):
    node, bridge = node_with_entry(), FakeBridge()
    start_line(node, bridge, clock)
    anchor = observation([clock[0] - .23], confirmed=True)
    callback = Node._info_callback(node, 'ball')
    callback(String(data=json.dumps(anchor)))
    for i in range(1, 4):
        clock[0] += .04
        callback(String(data=json.dumps(observation([clock[0] - .23]))))
    packet = node.latest_info['ball']
    assert node.ball_approach_observation.observation is not None
    captured = packet['rgb_stamp_ns'] / 1e9
    # Test the publication gate separately from the receive-time observer gate.
    clock[0] = captured + delay
    if delay > 0:
        assert node.ball_approach_observation.is_current(clock[0])
    assert Node._ball_entry_observations(node, {'ball': packet}, clock[0])['ball'] is packet
