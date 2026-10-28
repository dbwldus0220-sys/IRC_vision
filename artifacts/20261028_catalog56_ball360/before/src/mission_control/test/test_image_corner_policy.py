"""Validate ordinary tracking and the restored corner distance gate."""
import pytest
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecision, MotionDecisionPlanner, MotionDecisionConfig
from test_line_corner_memory import CornerHarness


def info(**updates):
    data=dict(detected=True,filtered_heading_error_deg=1.9,lateral_offset_px=0.,
              filtered_lateral_offset_norm=-.194,turn_angle_deg=79.5,turn_consistency=1.,
              heading_quality=.95,geometry_quality=.95,detection_quality=.95,
              ground_projection_valid=False,corner_preview_confirmed=True,
              corner_direction='RIGHT',corner_start_distance_m=.89,
              corner_start_depth_valid=True,corner_preview_held=False,rgb_stamp_ns=10_000_000_000)
    data.update(updates)
    return data


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic',lambda:10.)
    harness = CornerHarness(phase='LINE_TRACK')
    harness.planner = MotionDecisionPlanner(MotionDecisionConfig(line_heading_source='image'))
    return harness


def candidate(action):
    return MotionDecision('LINE_TRACK','line',action,True,'line_tracking',False,False,{})


def apply(node,action,**updates):
    node.latest_time['line']=10.
    return MotionDecisionNode._apply_pending_line_corner(node,candidate(action),info(**updates))


def test_recorded_wait_scene_now_publishes_normal_forward(node):
    command=node.publish_vision(line=info())[-1]
    assert command['valid'] and command['action']=='STRAIGHT'
    assert node.pending_line_corner['corner_direction']=='RIGHT'


@pytest.mark.parametrize('action',['STRAIGHT','RECOVER_LEFT_TURN_LEFT_4','RECOVER_RIGHT_TURN_RIGHT_4'])
@pytest.mark.parametrize('distance',[None,.1501,.89])
def test_far_confirmed_corner_preserves_normal_tracking(node,action,distance):
    apply(node, 'STRAIGHT')
    decision=apply(node,action,corner_start_distance_m=distance,corner_start_depth_valid=distance is not None)
    assert decision.action == action and decision.valid


@pytest.mark.parametrize('side',['LEFT','RIGHT'])
def test_distance_delays_only_an_already_selected_turn(node,side):
    assert apply(node,side,corner_direction=side).action=='STRAIGHT_1'
    result=apply(node,side,corner_direction=side,corner_start_distance_m=.15)
    assert result.action==side and result.valid and result.reason=='line_tracking'
    assert apply(node,'STRAIGHT',corner_direction=side,corner_start_distance_m=.1).action=='STRAIGHT'


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('action', [
    'STRAIGHT', 'RECOVER_LEFT_TURN_LEFT_4', 'RECOVER_RIGHT_TURN_RIGHT_4',
    'LINE_HEADING_TURN_RIGHT_7', 'LINE_OFFSET_TURN_LEFT_5',
])
def test_near_corner_preserves_tracking_action_in_image_mode(node, side, action):
    result = apply(node, action, corner_direction=side, corner_start_distance_m=.15)
    assert result.valid and result.action == action
    assert result.source_command == candidate(action).source_command


@pytest.mark.parametrize('updates',[
    {'corner_start_distance_m':None}, {'corner_start_depth_valid':False},
    {'corner_preview_held':True}, {'corner_preview_confirmed':False},
])
def test_turn_candidate_needs_fresh_confirmed_distance(node,updates):
    apply(node,'STRAIGHT')
    decision=apply(node,'RIGHT',**updates)
    assert not decision.valid and decision.reason=='line_corner_waiting_for_fresh_distance'


def test_stale_or_invalid_image_input_cannot_execute(node):
    assert not apply(node,'STRAIGHT',filtered_heading_error_deg=None).valid
    node.pending_line_corner['minimum_rgb_stamp_ns']=9_000_000_000
    assert not apply(node,'STRAIGHT',rgb_stamp_ns=1).valid
    node.latest_time['line']=9.
    decision=MotionDecisionNode._apply_pending_line_corner(node,candidate('STRAIGHT'),info())
    assert not decision.valid


def test_post_pickup_accepts_image_heading_without_ground_geometry():
    decision=MotionDecisionPlanner(MotionDecisionConfig(line_heading_source='image')).plan('POST_BALL_LINE_ALIGN',{'line':info()},.1)
    assert decision.action=='POST_BALL_LINE_ALIGNED' and decision.valid
    ground=MotionDecisionPlanner(MotionDecisionConfig(line_heading_source='ground'))
    assert not ground.plan('POST_BALL_LINE_ALIGN',{'line':info()},.1).valid


@pytest.mark.parametrize('robot',[False,True])
@pytest.mark.parametrize('mode',['image','ground'])
def test_launch_exposes_fixed_heading_source(monkeypatch,tmp_path,robot,mode):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description=launch_description(monkeypatch,tmp_path)
    context=default_context(description)
    assert context.launch_configurations['line_heading_source']=='ground'
    context.launch_configurations['line_heading_source']=mode
    node=next(e for e in description.entities if isinstance(e,Node) and e.node_executable=='motion_decision_node')
    assert node_parameters(node,context)['line_heading_source']==mode
