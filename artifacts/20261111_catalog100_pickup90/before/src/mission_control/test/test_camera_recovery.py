"""Exercise time faults, camera ownership, and mission recovery without hardware."""

from concurrent.futures import Future
import json
import signal
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_control.camera_time_health import CameraRecoveryGate, CameraTimeHealth
from mission_control.camera_supervisor_node import CameraProcess, CameraSupervisor
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_vision_time_guard import clocks, deliver
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info


EPOCH = 1_794_154_000_000_000_000


def frames(health, base=EPOCH, start=1.1, count=3, depth=True):
    for i in range(count):
        now = start + i * .05
        ros = base + round(now * 1e9)
        health.observe('rgb', ros - 20_000_000, ros, now)
        if depth:
            health.observe('depth', ros - 20_000_000, ros, now)
    return ros, now


def test_startup_rejects_1970_even_when_camera_matches_and_needs_both_streams():
    health = CameraTimeHealth(0.)
    ros, now = frames(health, base=0)
    assert health.status(ros, now) == ('WAIT_SYSTEM_TIME', False)
    health = CameraTimeHealth(0.)
    ros, now = frames(health, depth=False)
    assert health.status(ros, now)[0] == 'WAIT_FRAMES'
    ros, now = frames(health, start=1.3)
    assert health.status(ros, now) == ('READY', False)


@pytest.mark.parametrize('jump', [371., -371., 1_794_153_891.])
def test_clock_step_blocks_old_epoch_until_restarted_frames_arrive(jump):
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    assert health.status(ros, now)[0] == 'READY'
    shifted = EPOCH + round(jump * 1e9)
    # The camera keeps its old origin while the host changes time.
    for i in range(70):
        now = 1.3 + i * .05
        ros = shifted + round(now * 1e9)
        old = EPOCH + round(now * 1e9)
        for source in ('rgb', 'depth'):
            health.observe(source, old, ros, now)
        state, needed = health.status(ros, now)
        assert state != 'READY'
    assert needed
    health.restarted(ros, now)
    ros, now = frames(health, shifted, start=now + .1)
    assert health.status(ros, now) == ('READY', False)


def test_duplicate_frames_and_one_fresh_stream_cannot_keep_camera_ready():
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    assert health.status(ros, now)[0] == 'READY'
    for i in range(20):
        current = now + (i + 1) * .1
        clock = EPOCH + round(current * 1e9)
        health.observe('rgb', clock - 20_000_000, clock, current)
        health.observe('depth', ros - 20_000_000, clock, current)
    assert health.status(clock, current)[0] == 'WAIT_FRAMES'


class Child:
    def __init__(self):
        self.returncode = None
        self.signals = []

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)

    def kill(self):
        self.signals.append(signal.SIGKILL)


def test_supervisor_keeps_rgb_depth_and_disables_unused_infrared(monkeypatch):
    import mission_control.camera_supervisor_node as module

    commands = []
    monkeypatch.setattr(module.Node, '__init__', lambda *args: None)
    monkeypatch.setattr(CameraSupervisor, 'declare_parameter', lambda *args: None)
    monkeypatch.setattr(CameraSupervisor, 'get_parameter',
                        lambda *args: SimpleNamespace(value='/camera/camera'))
    for method in ('create_publisher', 'create_subscription', 'create_service', 'create_timer'):
        monkeypatch.setattr(CameraSupervisor, method, lambda *args, **kwargs: None)
    monkeypatch.setattr(module, 'get_package_prefix', lambda name: '/camera-driver')
    monkeypatch.setattr(module, 'CameraProcess', lambda command, now: commands.append(command))

    CameraSupervisor()
    command = commands[0]
    parameters = dict(command[i + 1].split(':=', 1)
                      for i, argument in enumerate(command) if argument == '-p')
    assert parameters['enable_color'] == parameters['enable_depth'] == 'true'
    assert all(parameters[name] == 'false'
               for name in ('enable_infra', 'enable_infra1', 'enable_infra2'))
    assert parameters['align_depth.enable'] == 'true'
    assert parameters['rgb_camera.color_profile'] == '1280,720,30'
    assert parameters['depth_module.depth_profile'] == '848,480,30'


def test_restart_never_opens_second_camera_before_child_exits_and_is_bounded():
    children = []

    def spawn(*args, **kwargs):
        children.append(Child())
        return children[-1]

    process = CameraProcess(['camera-test'], 0., popen=spawn)
    for attempt in range(2):
        start = attempt * 10.
        assert process.request_restart(start)
        assert not process.request_restart(start)
        assert not process.tick(start + 1.)
        assert len(children) == attempt + 1
        assert children[-1].signals == [signal.SIGINT]
        children[-1].returncode = 0
        assert process.tick(start + 2.)
    assert not process.request_restart(30.)
    assert process.failed == 'camera_restart_limit'
    assert len(children) == 3


def test_camera_that_will_not_exit_latches_failure_without_duplicate_process():
    child = Child()
    process = CameraProcess(['camera-test'], 0., popen=lambda *a, **k: child)
    assert process.request_restart(1.)
    assert not process.tick(6.)
    assert child.signals == [signal.SIGINT, signal.SIGKILL]
    assert not process.tick(9.)
    assert process.failed == 'camera_stop_timeout'
    assert process.generation == 1


def test_camera_self_recovery_during_service_request_does_not_latch_failure():
    process = CameraProcess(['camera-test'], 0., popen=lambda *a, **k: Child())
    supervisor = SimpleNamespace(
        process=process,
        health=SimpleNamespace(status=lambda *a: ('READY', False)),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=EPOCH)),
    )
    response = CameraSupervisor.restart(supervisor, None, SimpleNamespace())
    assert response.success
    assert response.message == 'camera_already_ready'
    assert process.attempts == 0


class RestartClient:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future


def supervised_node(clocks, phase='LINE_TRACK'):
    node = LiveInputHarness(phase=phase)
    node.camera_recovery = CameraRecoveryGate()
    node.camera_restart_future = None
    node.camera_restart_requested_at = None
    node.camera_restart_requested_generation = None
    node.camera_restart_failure = None
    node.camera_restart_client = RestartClient()
    node.executor_active = False
    node.executor_auto_ready = True
    node.executor_heartbeat_watchdog.observe(sequence=1, observed_at=clocks[0])
    return node


def camera_status(node, state='READY', generation=1, needed=False):
    Node._camera_time_status_callback(node, String(data=json.dumps({
        'state': state, 'session': 'test-camera', 'generation': generation,
        'restart_needed': needed,
    })))


def advance(node, clocks, source='line', info=None):
    clocks[0] += .1
    clocks[1] += .1
    node.executor_heartbeat_watchdog.observe(sequence=2, observed_at=clocks[0])
    deliver(node, clocks, source, info=info)


def test_supervised_start_blocks_until_raw_health_and_required_source_are_ready(clocks):
    node = supervised_node(clocks)
    for _ in range(3):
        advance(node, clocks)
    Node._publish_decision(node)
    assert not node.publisher.messages
    camera_status(node)
    for _ in range(3):
        advance(node, clocks, 'goal', info={'detected': False})
    Node._publish_decision(node)
    assert not node.publisher.messages
    for _ in range(3):
        advance(node, clocks)
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['valid']


def test_restart_waits_for_submitted_queue_and_keeps_its_identity(clocks):
    node = supervised_node(clocks)
    node.queued_general_command_id = 42
    node.queued_general_action = 'STRAIGHT'
    camera_status(node, 'WAIT_FRAMES', needed=True)
    Node._publish_decision(node)
    assert not node.camera_restart_client.calls
    assert node.queued_general_command_id == 42
    assert node.queued_general_action == 'STRAIGHT'
    assert node.camera_recovery.reason == 'WAIT_MOTION_IDLE'
    node.queued_general_command_id = None
    Node._publish_decision(node)
    assert len(node.camera_restart_client.calls) == 1
    Node._publish_decision(node)
    assert len(node.camera_restart_client.calls) == 1


def test_post_pickup_recovery_keeps_phase_and_requires_fresh_line_geometry(clocks):
    node = supervised_node(clocks, phase='POST_BALL_LINE_ALIGN')
    camera_status(node, 'WAIT_FRAMES', needed=True)
    Node._publish_decision(node)
    node.camera_restart_client.calls[0].set_result(SimpleNamespace(success=True))
    camera_status(node, generation=2)
    for _ in range(3):
        advance(node, clocks, 'goal', info={'detected': False})
    Node._publish_decision(node)
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'
    assert not node.publisher.messages
    for _ in range(3):
        advance(node, clocks, info={**line_info(), 'ground_fit_point_count': 2})
    Node._publish_decision(node)
    assert not node.publisher.messages
    camera_status(node, generation=2)
    for _ in range(3):
        advance(node, clocks, info={**line_info(heading=20., offset=.2), 'ground_fit_point_count': 3})
    Node._publish_decision(node)
    assert not node.camera_recovery.reobserve
    assert node.publisher.messages[-1]['valid']
    assert node.publisher.messages[-1]['source'] == 'line'


def test_lost_supervisor_and_late_restart_response_cannot_resume_motion(clocks):
    node = supervised_node(clocks)
    camera_status(node, 'WAIT_FRAMES', needed=True)
    Node._publish_decision(node)
    clocks[0] += 3.1
    clocks[1] += 3.1
    Node._publish_decision(node)
    assert node.camera_restart_failure == 'CAMERA_RESTART_SERVICE_TIMEOUT'
    node.camera_restart_client.calls[0].set_result(SimpleNamespace(success=True))
    camera_status(node)
    for _ in range(3):
        advance(node, clocks)
    Node._publish_decision(node)
    assert not node.publisher.messages
    assert len(node.camera_restart_client.calls) == 1


def test_pickup_waiting_checkpoint_can_recover_without_releasing_mission(clocks):
    node = supervised_node(clocks, phase='BALL_APPROACH')
    node.phase_manager.active_special_action = 'PICKUP_NOW'
    node.phase_manager.active_special_command_id = 92
    node.pickup_initial_align_waiting = True
    node.pickup_positioning_motion_running = False
    camera_status(node, 'WAIT_FRAMES', needed=True)
    Node._publish_decision(node)
    assert len(node.camera_restart_client.calls) == 1
    assert node.active_special_command_id == 92
    assert node.mission_phase == 'BALL_APPROACH'


def test_active_executor_and_stale_heartbeat_never_authorize_restart(clocks):
    node = supervised_node(clocks)
    camera_status(node, 'WAIT_FRAMES', needed=True)
    node.executor_active = True
    Node._publish_decision(node)
    assert not node.camera_restart_client.calls
    node.executor_active = False
    node.executor_heartbeat_watchdog.last_heartbeat_at = clocks[0] - 10.
    Node._publish_decision(node)
    assert not node.camera_restart_client.calls


def test_recovered_pickup_runs_three_line_commands_then_goal_without_repeating_pickup(clocks):
    node = supervised_node(clocks, phase='POST_BALL_LINE_ALIGN')
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    camera_status(node, 'WAIT_FRAMES', needed=True)
    Node._publish_decision(node)
    node.camera_restart_client.calls[0].set_result(SimpleNamespace(success=True))
    for ordinal in (1, 2, 3):
        camera_status(node, generation=2)
        for _ in range(3):
            advance(node, clocks, info={**line_info(), 'ground_fit_point_count': 3})
        Node._publish_decision(node)
        command = node.publisher.messages[-1]
        assert command['source_command']['post_pickup_line_index'] == ordinal
        node.send_status(command['action'], command['command_id'], 'RUNNING')
        clocks[0] += 4.
        clocks[1] += 4.
        camera_status(node, generation=2)
        node.executor_heartbeat_watchdog.observe(sequence=ordinal + 3, observed_at=clocks[0])
        node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.phase_manager.pickups_completed == 1
    assert node.phase_manager.ball_grasp_results[1] == 'GRABBED'
    assert not any(message['action'] == 'PICKUP_NOW' for message in node.publisher.messages)


def test_lost_supervisor_invalidates_ready_phase_and_old_observations(clocks):
    node = supervised_node(clocks)
    camera_status(node)
    for _ in range(3):
        advance(node, clocks)
    assert Node._camera_recovery_ready(node, clocks[0])
    clocks[0] += 1.1
    clocks[1] += 1.1
    assert not Node._camera_recovery_ready(node, clocks[0])
    assert node.camera_recovery.reason == 'WAIT_CAMERA_STATUS'
    assert all(info is None for info in node.latest_info.values())
    assert node.camera_recovery.reobserve


@pytest.mark.parametrize('robot', [False, True])
@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_full_launch_uses_original_camera_launch_without_supervisor_gate(monkeypatch, tmp_path, robot, enabled):
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node as LaunchNode
    from test_full_system_launch import launch_description, launch_context, node_parameters
    from test_full_system_robot_launch import launch_description as robot_description
    description = (robot_description if robot else launch_description)(monkeypatch, tmp_path)
    nodes = [item for item in description.entities if isinstance(item, LaunchNode)]
    camera = next(item for item in description.entities if isinstance(item, IncludeLaunchDescription))
    decision = next(item for item in nodes if item.node_executable == 'motion_decision_node')
    context = launch_context(description, {'enable_camera': enabled})
    assert camera.condition.evaluate(context) == (enabled == 'true')
    assert not node_parameters(decision, context).get('camera_time_supervision_enabled', False)
    assert not any(item.node_executable == 'camera_supervisor' for item in nodes)
    arguments = dict(camera.launch_arguments)
    assert arguments['align_depth.enable'] == 'true'
    assert arguments['rgb_camera.color_profile'] == '1280,720,30'
    assert arguments['depth_module.depth_profile'] == '848,480,30'
    assert arguments['enable_gyro'] == ('true' if robot else 'false')
    assert arguments['enable_accel'] == ('true' if robot else 'false')
    assert not any(item.node_executable == 'realsense2_camera_node' for item in nodes)
