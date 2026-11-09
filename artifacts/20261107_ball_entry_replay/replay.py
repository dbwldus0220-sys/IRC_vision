#!/usr/bin/env python3
"""Synthetic replay anchored to recorded times, using production decision/filter code.
No ROS graph, camera, executor process, or hardware is started.
"""
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
for path in ('src/step', 'src/mission_control', 'src/mission_control/test'):
    sys.path.insert(0, str(ROOT / path))
import pytest
from std_msgs.msg import String
from step.temporal_confirmation import TemporalConfirmationFilter
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import node_with_entry, start_line, send, status, ground_ball_info
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge

OUT = Path(__file__).resolve().parent
RECORDED = json.loads((ROOT/'artifacts/20261107_ball_confirmation_latency/metrics.json').read_text())
EVENTS = RECORDED['first_entry_events']
INTERVAL = 1 / 10.305
AGE = EVENTS['confirmed_frame_capture_to_confirmation_ms']/1000
CONFIRM_RELATIVE = (EVENTS['confirmed_epoch_ns']-EVENTS['line_complete_epoch_ns'])/1e9
FIRST_RELATIVE = (EVENTS['pending_epoch_ns']-EVENTS['line_complete_epoch_ns'])/1e9


def run(required_hits, *, earlier_sec=0., capture_age=AGE, missing_tenth=False):
    clock = [10.]
    trace = []
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr('mission_control.motion_decision_node.time.monotonic', lambda:clock[0])
        patch.setattr(Node, '_current_ros_time_ns', lambda _:round(clock[0]*1e9))
        node = node_with_entry()
        bridge = FakeBridge(advance_transition_dwells=False)
        _, active = start_line(node, bridge, clock)
        end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
        filter_ = TemporalConfirmationFilter(window_size=30, required_hits=required_hits,
                    max_missed_frames=10, max_center_shift_norm=.18, min_area_ratio=.4)
        # First and twelfth receipts match the logs. Intermediate detections are
        # modeled from the local mean interval, not claimed to be recorded data.
        hits = [end+FIRST_RELATIVE-earlier_sec]
        hits += [end+CONFIRM_RELATIVE-(11-i)*INTERVAL-earlier_sec for i in range(1, 18)]
        events = [(t, 'ball', i+1) for i,t in enumerate(hits)]
        events += [(end, 'end', None)]
        # Provide valid Line input at 15 Hz; reserve eligibility still comes
        # from the actual node, including its capture and confirmation gates.
        t = clock[0] + 1/15
        while t < end+1.1:
            events.append((t,'line',None)); t += 1/15
        t = clock[0] + .05
        while t < end+1.1:
            events.append((t,'timer',None)); t += .05
        published = len(node.publisher.messages)
        first_confirmed = None
        result = None
        for t, kind, index in sorted(events):
            if t < clock[0]:
                raise AssertionError('Synthetic time moved backwards')
            clock[0] = t
            if kind == 'end':
                status(node, bridge, active, 'SUCCEEDED')
            elif kind == 'timer':
                Node._publish_decision(node)
            elif kind == 'line':
                info = {**line_info(), 'rgb_stamp_ns':round((t-capture_age)*1e9)}
                Node._info_callback(node,'line')(String(data=json.dumps(info)))
            else:
                detected = not (missing_tenth and index==10)
                confirmation = filter_.update(detected, bbox=[800,350,870,450] if detected else None,
                                              image_width=1280,image_height=720)
                info = ground_ball_info(depth_m=.86, distance_m=.86, confidence=.9,
                         ground_steering_angle_deg=-28.57, ground_forward_distance_m=.75,
                         depth_age_sec=.15, rgb_stamp_ns=round((t-capture_age)*1e9),
                         bbox=[800,350,870,450], image_width=1280,image_height=720,
                         center_x=835, center_y=400, bottom_distance_px=319,
                         head_down_requested=False, pickup_now=False, pickup_ready=False)
                info.update(confirmation.as_dict())
                info['detected'] = confirmation.confirmed
                if not confirmation.confirmed:
                    info['approach_candidate'] = dict(info)
                    info.update(depth_valid=False, distance_m=None, depth_m=None,
                                note='ball_confirmation_pending' if detected else 'no_ball_detection')
                if confirmation.confirmed and first_confirmed is None:
                    first_confirmed = t
                Node._info_callback(node,'ball',dispatch_on_ball=True)(String(data=json.dumps(info)))
                trace.append({'relative_ms':round((t-end)*1000,3),'hit':confirmation.hit_count,
                              'confirmed':confirmation.confirmed,'raw':detected})
            for command in node.publisher.messages[published:]:
                if not command.get('valid'):
                    continue
                if command.get('source') != 'ball':
                    raise AssertionError(f'Unexpected extra motion: {command}')
                request = send(bridge, command)
                result = {'required_hits': required_hits, 'candidate_earlier_ms':earlier_sec*1000,
                          'capture_age_ms':capture_age*1000,'missing_tenth':missing_tenth,
                          'confirmed_relative_ms':round((first_confirmed-end)*1000,3),
                          'command_relative_ms':round((t-end)*1000,3),
                          'command_wait_ms':round(max(0,t-end)*1000,3),
                          'queued_before_boundary':t<end and not bridge.queued_request_deferred,
                          'action':command['action'],'motion_id':request['motion_id'],
                          'trace':trace}
                return result
            published = len(node.publisher.messages)
        return {'required_hits':required_hits,'candidate_earlier_ms':earlier_sec*1000,
                'capture_age_ms':capture_age*1000,'missing_tenth':missing_tenth,
                'command_relative_ms':None,'trace':trace,
                'last_decision':str(node.last_selected_decision)}


def main():
    results = []
    for advance in (0., .15, .30, .50):
        for threshold in (12,10):
            results.append(run(threshold, earlier_sec=advance))
    results.append(run(10,missing_tenth=True))
    results.append(run(10,capture_age=.6))
    # Validate the actual state-machine outputs, including no command from stale data.
    base_old, base_new = results[:2]
    assert abs(base_old['command_wait_ms'] - 328.338) < .01
    assert 120 < base_new['command_wait_ms'] < 150
    assert not base_new['queued_before_boundary']
    assert results[3]['command_wait_ms'] == 0 and not results[3]['queued_before_boundary']
    assert results[7]['queued_before_boundary'] and results[7]['command_relative_ms'] < 0
    assert results[8]['command_wait_ms'] > base_new['command_wait_ms']
    assert results[9]['command_relative_ms'] is None
    assert all(r.get('action') == 'RECOVER_LEFT_TURN_LEFT_4' for r in results[:9])
    (OUT/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    rows = [{k:v for k,v in r.items() if k not in ('trace','last_decision')} for r in results]
    with (OUT/'results.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=sorted({k for r in rows for k in r}))
        writer.writeheader();writer.writerows(rows)
    print(json.dumps(rows,ensure_ascii=False,indent=2))

if __name__=='__main__':
    main()
