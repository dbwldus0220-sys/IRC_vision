"""Reproduce the existing raw-packet checkpoint limitation without changing it."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from step.hurdle_camera_pose import HurdleCameraPose
from test_continuous_approach import clock
from test_hurdle_forward2 import start, finish_walk
from test_hurdle_positioning import hurdle


@pytest.mark.parametrize('depth', [.50, .47, .45])
@pytest.mark.parametrize('bottom', [97, 100, 101, 200])
def test_raw_analyzer_packet_checkpoint_vs_ready_planner(clock, depth, bottom):
    node, bridge, request, started = start(clock)
    node.hurdle_camera_pose = HurdleCameraPose()
    clock[0] = started + .7
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 1, 'auto_ready': True, 'active': True,
        'hurdle_head_override_active': True,
        'hurdle_head_override_current_deg': -64.,
    })))
    clock[0] += .1
    stamp = round(clock[0] * 1e9)
    packet = hurdle(depth, bottom_distance_px=bottom,
                    stamp={'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                    rgb_stamp_ns=stamp)
    # The analyzer publishes angle hypotheses; the decision node adds posture.
    packet.pop('hurdle_camera_pose')
    packet.pop('hurdle_camera_ready')
    node.receive(packet)
    latched = node.pending_hurdle_final_sequence is not None
    # Existing limitation: raw checkpoint inspection precedes pose annotation.
    assert latched is (depth <= .45 or bottom <= 100)
    observations, _ = node._fresh_observations(clock[0])
    assert observations['hurdle']['hurdle_camera_ready']
    decision = node.planner.plan('HURDLE_POSITIONING', observations, .1)
    assert decision.action == 'GO'
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('depth', [.50, .47])
def test_video_bottom_occlusion_keeps_final_checkpoint(clock, depth):
    node, bridge, request, started = start(clock)
    clock[0] = started + .8
    packet = hurdle(depth, bottom_distance_px=97,
                    rgb_stamp_ns=round(clock[0] * 1e9))
    packet.pop('hurdle_camera_pose')
    packet.pop('hurdle_camera_ready')
    node.receive(packet)
    assert node.pending_hurdle_final_sequence is not None
    clock[0] += .01
    node.receive({**packet, 'detected': False, 'raw_detected': False,
                  'rgb_stamp_ns': round(clock[0] * 1e9)})
    assert node.publish() == []
    finish_walk(node, bridge, request, clock, started)
    command = node.publish()[-1]
    assert command['action'] == 'GO'
    assert command['source_command']['hurdle_final_sequence_latched']
