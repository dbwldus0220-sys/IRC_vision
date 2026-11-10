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
for action in ('STRAIGHT','RECOVER_LEFT_TURN_LEFT_4','RECOVER_RIGHT_TURN_RIGHT_4'):
 clock=[10.]
 with patch('mission_control.motion_decision_node.time.monotonic',side_effect=lambda:clock[0]),patch.object(Node,'_current_ros_time_ns',lambda _:round(clock[0]*1e9)):
  node,bridge,first=start(clock,action,'AUTO')
  started=node.active_line_motion_started_at;duration=node.active_line_motion_duration_sec
  for i in range(1,46):
   clock[0]=started+duration*.95*i/45
   Node._info_callback(node,'line')(String(data=json.dumps({**line_info(),'rgb_stamp_ns':round(clock[0]*1e9)})))
   if node.queued_general_command_id is not None:break
  command=node.publisher.messages[-1]
  assert command['command_id']!=first['command_id']
  bridge.navigation_command_callback(String(data=json.dumps(command)))
  queued=decoded_messages(bridge.executor_request_publisher)[-1]
  forward_status(node,bridge,queued,'REJECTED',error_code='INCOMPATIBLE_TRANSITION')
  assert node.queued_general_command_id is None and bridge.queued_request_id is None
  assert node.general_motion_gate.active_command_id==first['command_id']
  clock[0]=started+duration
  forward_status(node,bridge,first,'SUCCEEDED')
  clock[0]+=.05
  Node._info_callback(node,'line')(String(data=json.dumps({**line_info(),'rgb_stamp_ns':round(clock[0]*1e9)})))
  Node._publish_decision(node)
  resumed=node.publisher.messages[-1]
  assert resumed['valid'] and resumed['command_id'] not in {first['command_id'],queued['command_id']},resumed
  bridge.navigation_command_callback(String(data=json.dumps(resumed)))
  req=decoded_messages(bridge.executor_request_publisher)[-1]
  forward_status(node,bridge,req,'RUNNING')
  assert node.general_motion_gate.active_command_id==resumed['command_id']
  results.append({'action':action,'rejection_cleared':True,'resumed':resumed['action']})
Path(__file__).resolve().parent.joinpath('rejection.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
