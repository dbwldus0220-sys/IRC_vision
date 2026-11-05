"""Motor-free executor test: only accepted third RUNNING may raise the head."""
import json
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.util
from launch_ros.actions import Node
import rclpy
from std_msgs.msg import String

ROOT = '/third_line_camera_test'
TOPICS = (
    '/motion/executor/request', '/motion/executor/cancel', '/motion/executor/status',
    '/motion/executor/heartbeat', '/vision/ball_info', '/navigation/motion_command',
    '/vision/hurdle_info', '/navigation/goal_camera_prepare',
)


def generate_test_description():
    executor = Node(
        package='irc_step_motion_executor', executable='sdk_motion_executor',
        name='third_line_camera_executor', output='screen',
        parameters=[{'backend_type': 'simulated', 'enable_robot_hardware': False,
                     'poll_period_ms': 20, 'running_polls': 35, 'settling_polls': 1,
                     'heartbeat_period_ms': 20}],
        remappings=[(topic, ROOT + topic) for topic in TOPICS])
    return launch.LaunchDescription([
        executor, launch_testing.actions.ReadyToTest(), launch_testing.util.KeepAliveProc(),
        launch.actions.TimerAction(period=20., actions=[launch.actions.EmitEvent(
            event=launch.events.Shutdown(reason='test timeout'))]),
    ]), {'executor': executor}


class TestThirdLineCamera(unittest.TestCase):
    def test_camera_starts_only_with_third_running(self):
        rclpy.init()
        node = rclpy.create_node('third_line_camera_client')
        statuses, heartbeats = [], []
        request = node.create_publisher(String, ROOT + TOPICS[0], 10)
        node.create_subscription(String, ROOT + TOPICS[2], lambda m: statuses.append(json.loads(m.data)), 10)
        node.create_subscription(String, ROOT + TOPICS[3], lambda m: heartbeats.append(json.loads(m.data)), 10)

        def wait(predicate):
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(node, timeout_sec=.02)
            self.fail(f'timeout: {statuses[-4:]}, {heartbeats[-1:]}')

        def has_status(command_id, status):
            return any(s['request_id'] == command_id and s['status'] == status for s in statuses)

        def send(command_id, camera=False):
            payload = dict(command_id=command_id, request_id=command_id, event_id=None,
                           action='STRAIGHT', motion_id='line_forward_6', timeout_ms=5000)
            if camera:
                payload['goal_head_prepare_id'] = command_id
            request.publish(String(data=json.dumps(payload)))

        try:
            wait(lambda: request.get_subscription_count() and heartbeats)
            send(1)
            wait(lambda: has_status(1, 'RUNNING'))
            wait(lambda: has_status(1, 'SUCCEEDED'))
            send(2)
            wait(lambda: has_status(2, 'RUNNING'))
            self.assertFalse(heartbeats[-1]['goal_head_override_active'])
            send(3, camera=True)
            # The simulated backend rejects queuing; the core unit test covers
            # QUEUED metadata and promotion separately with a queue-capable fake.
            wait(lambda: has_status(3, 'REJECTED'))
            seq = heartbeats[-1]['sequence']
            wait(lambda: heartbeats[-1]['sequence'] > seq)
            self.assertFalse(has_status(3, 'RUNNING'))
            self.assertFalse(heartbeats[-1]['goal_head_override_active'])
            wait(lambda: has_status(2, 'SUCCEEDED'))
            send(3, camera=True)
            wait(lambda: has_status(3, 'RUNNING'))
            wait(lambda: heartbeats[-1]['goal_head_override_active'])
            self.assertEqual(heartbeats[-1]['goal_head_prepare_id'], 3)
            wait(lambda: heartbeats[-1]['goal_head_transition_complete'])
            wait(lambda: has_status(3, 'SUCCEEDED'))
        finally:
            node.destroy_node()
            rclpy.shutdown()
