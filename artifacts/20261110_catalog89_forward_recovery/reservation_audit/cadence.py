import json
from pathlib import Path
from unittest.mock import patch
from std_msgs.msg import String
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_line_turn_prequeue import start
from test_mission_phase_flow import line_info
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status

results=[]
for action in ('STRAIGHT_1','STRAIGHT','RECOVER_LEFT_TURN_LEFT_4','RECOVER_RIGHT_TURN_RIGHT_4'):
 for target in ('forward','right_recovery'):
  for fps in (10,20,25):
   for delay in (.05,.15,.25):
    clock=[10.]
    with patch('mission_control.motion_decision_node.time.monotonic',side_effect=lambda:clock[0]), patch.object(Node,'_current_ros_time_ns',lambda _:round(clock[0]*1e9)):
     node,bridge,running=start(clock,action,'AUTO')
     started=node.active_line_motion_started_at; duration=node.active_line_motion_duration_sec
     sample=line_info() if target=='forward' else line_info(20.,.4)
     consumed=len(node.publisher.messages); seen={running['command_id']}; queued_at=None
     def dispatch():
      global consumed,queued_at
      for command in node.publisher.messages[consumed:]:
       if command.get('valid') and command['command_id'] not in seen:
        seen.add(command['command_id'])
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        if queued_at is None and node.general_motion_gate.active_command_id==running['command_id']:
         queued_at=clock[0]
      consumed=len(node.publisher.messages)
     i=1
     while started+i/fps+delay < started+duration:
      captured=started+i/fps;clock[0]=captured+delay
      Node._info_callback(node,'line')(String(data=json.dumps({**sample,'rgb_stamp_ns':round(captured*1e9)})))
      dispatch();i+=1
     queued_before_finish=node.queued_general_command_id
     queued_reason=getattr(node,'line_prequeue_block_reason',None)
     clock[0]=started+duration
     forward_status(node,bridge,running,'SUCCEEDED');dispatch()
     requests=decoded_messages(bridge.executor_request_publisher)
     if len(requests)>1:
      forward_status(node,bridge,requests[-1],'RUNNING')
     # If no prequeue was possible, supply one current frame after completion.
     if node.general_motion_gate.active_command_id in (None,running['command_id']):
      clock[0]+=.05
      Node._info_callback(node,'line')(String(data=json.dumps({**sample,'rgb_stamp_ns':round(clock[0]*1e9)})))
      Node._publish_decision(node);dispatch()
      requests=decoded_messages(bridge.executor_request_publisher)
      if len(requests)>1:forward_status(node,bridge,requests[-1],'RUNNING')
     results.append(dict(action=action,target=target,fps=fps,delay=delay,duration=duration,
      queued_before_finish=queued_before_finish is not None,
      lead_sec=None if queued_at is None else round(started+duration-queued_at,4),
      block_reason=queued_reason,requests=len(requests),
      resumed=len(requests)>1 and node.general_motion_gate.active_command_id!=running['command_id'],
      active_id=node.general_motion_gate.active_command_id,
      last_action=node.publisher.messages[-1]['action']))
Path(__file__).resolve().parent.joinpath('cadence.json').write_text(json.dumps(results,indent=2))
print('cases',len(results),'prequeued',sum(r['queued_before_finish'] for r in results),'resumed',sum(r['resumed'] for r in results))
print('not resumed',json.dumps([r for r in results if not r['resumed']],indent=2))

assert all(r["resumed"] and r["requests"] == 2 for r in results)
assert all(r["queued_before_finish"] for r in results)
