"""Exercise the camera hold lifecycle on isolated topics without hardware."""

import json
import time
import unittest

import launch
import launch.actions
import launch.events
import launch_testing
import launch_testing.actions
import launch_testing.util
from launch_ros.actions import Node
import rclpy
from std_msgs.msg import String


TOPIC_ROOT = "/goal_head_override_test"
TOPICS = (
    "/motion/executor/request", "/motion/executor/cancel",
    "/motion/executor/status", "/motion/executor/heartbeat", "/vision/ball_info",
)


def generate_test_description():
    executor = Node(
        package="irc_step_motion_executor", executable="sdk_motion_executor",
        name="goal_head_override_test_executor", output="screen",
        parameters=[{
            "backend_type": "simulated", "enable_robot_hardware": False,
            "poll_period_ms": 20, "running_polls": 12, "settling_polls": 1,
            "heartbeat_period_ms": 20,
        }],
        remappings=[(topic, TOPIC_ROOT + topic) for topic in TOPICS],
    )
    return launch.LaunchDescription([
        executor, launch_testing.actions.ReadyToTest(),
        launch_testing.util.KeepAliveProc(),
        launch.actions.TimerAction(
            period=15.0,
            actions=[launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="camera hold test timeout guard",
            ))],
        ),
    ]), {"executor": executor}


class TestGoalHeadOverride(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("goal_head_override_test_client")
        cls.statuses = []
        cls.heartbeats = []
        cls.request = cls.node.create_publisher(String, TOPIC_ROOT + TOPICS[0], 10)
        cls.cancel = cls.node.create_publisher(String, TOPIC_ROOT + TOPICS[1], 10)
        cls.ball = cls.node.create_publisher(String, TOPIC_ROOT + TOPICS[4], 10)
        cls.status_sub = cls.node.create_subscription(
            String, TOPIC_ROOT + TOPICS[2],
            lambda msg: cls.statuses.append(json.loads(msg.data)), 10,
        )
        cls.heartbeat_sub = cls.node.create_subscription(
            String, TOPIC_ROOT + TOPICS[3],
            lambda msg: cls.heartbeats.append(json.loads(msg.data)), 10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.fail(f"Timed out; statuses={self.statuses[-5:]}; heartbeats={self.heartbeats[-2:]}")

    def publish(self, publisher, payload):
        publisher.publish(String(data=json.dumps(payload)))

    def send(self, request_id, motion_id, timeout_ms=2000):
        self.publish(self.request, {
            "request_id": request_id, "motion_id": motion_id,
            "action": "SHOT" if motion_id == "goal_shot" else "POST_BALL_GOAL_TRANSITION",
            "command_id": request_id, "event_id": None, "timeout_ms": timeout_ms,
        })

    def status(self, request_id, status):
        self.wait(lambda: any(
            value["request_id"] == request_id and value["status"] == status
            for value in self.statuses
        ))

    def hold(self, enabled):
        sequence = self.heartbeats[-1]["sequence"] if self.heartbeats else -1
        self.wait(lambda: any(
            value["sequence"] > sequence
            and value.get("goal_head_override_active") is enabled
            for value in self.heartbeats
        ))
        self.assertEqual(self.heartbeats[-1]["goal_head_override_deg"], 1.0)
        if enabled:
            self.assertFalse(self.heartbeats[-1]["ball_head_override_active"])

    def test_hold_starts_after_camera_success_and_lasts_through_shot(self):
        self.wait(lambda: all(pub.get_subscription_count() for pub in (
            self.request, self.cancel, self.ball,
        )))
        self.hold(False)
        self.send(1, "line_forward_6")
        self.status(1, "SUCCEEDED")
        self.hold(False)

        self.send(4, "post_ball_camera_90")
        self.status(4, "RUNNING")
        self.hold(False)
        self.status(4, "SUCCEEDED")
        self.hold(True)

        # Hold applies to every motion, including preparation with a camera45 frame.
        for request_id, motion in (
            (5, "goal_camera_90_fine_forward_1"), (6, "goal_camera_90_turn_left_2"),
            (7, "goal_camera_90_crab_right"), (8, "goal_fine_to_default"),
            (9, "line_forward_4"),
        ):
            self.send(request_id, motion)
            self.status(request_id, "RUNNING")
            self.publish(self.ball, {"detected": True, "head_down_requested": True})
            self.hold(True)
            self.status(request_id, "SUCCEEDED")
            self.hold(True)

        self.publish(self.request, {"request_id": 10, "motion_id": "goal_shot"})
        self.wait(lambda: any(s["status"] == "REJECTED" for s in self.statuses))
        self.hold(True)
        self.send(12, "goal_shot")
        self.status(12, "RUNNING")
        self.hold(True)
        self.status(12, "SUCCEEDED")
        self.hold(False)

        # A second pickup/goal cycle can enable the hold again; cancellation resets it.
        self.send(13, "post_ball_camera_90")
        self.status(13, "SUCCEEDED")
        self.hold(True)
        self.send(14, "goal_camera_90_crab_left")
        self.status(14, "RUNNING")
        self.publish(self.cancel, {"request_id": 999})
        self.status(14, "REJECTED")
        self.hold(True)
        self.publish(self.cancel, {"request_id": 14})
        self.status(14, "CANCELLED")
        self.hold(False)

        self.send(15, "post_ball_camera_90")
        self.status(15, "SUCCEEDED")
        self.hold(True)
        self.send(16, "pickup")
        self.status(16, "RUNNING")
        self.hold(False)
        self.status(16, "SUCCEEDED")

        self.send(17, "post_ball_camera_90")
        self.status(17, "SUCCEEDED")
        self.hold(True)
        # Keep timeout last: the simulated backend awaits an extra cancellation poll.
        self.send(18, "goal_shot", timeout_ms=1)
        self.status(18, "FAILED")
        self.hold(True)
