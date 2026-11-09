"""Verify BALL look-down during walking corrections, turns, and idle waits."""

import json
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
from launch_ros.actions import Node
import rclpy
from std_msgs.msg import String


ROOT = "/ball_head_override_test"
TOPICS = (
    "/motion/executor/request", "/motion/executor/cancel",
    "/motion/executor/status", "/motion/executor/heartbeat",
    "/navigation/motion_command", "/vision/ball_info", "/vision/hurdle_info",
    "/navigation/goal_camera_prepare",
)


def generate_test_description():
    executor = Node(
        package="irc_step_motion_executor", executable="sdk_motion_executor",
        name="ball_head_override_test_executor", output="screen",
        parameters=[{
            "backend_type": "simulated", "enable_robot_hardware": False,
            "poll_period_ms": 20, "running_polls": 25, "settling_polls": 1,
            "heartbeat_period_ms": 20,
        }],
        remappings=[(topic, ROOT + topic) for topic in TOPICS],
    )
    return launch.LaunchDescription([
        executor, launch_testing.actions.ReadyToTest(), launch_testing.util.KeepAliveProc(),
        launch.actions.TimerAction(
            period=15.0,
            actions=[launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="camera hold test timeout guard",
            ))],
        ),
    ]), {"executor": executor}


class TestBallHeadOverride(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("ball_head_override_test_client")
        cls.statuses, cls.heartbeats = [], []
        cls.publishers = {
            topic: cls.node.create_publisher(String, ROOT + topic, 10)
            for topic in (TOPICS[0], TOPICS[1], TOPICS[4], TOPICS[5])
        }
        cls.status_sub = cls.node.create_subscription(
            String, ROOT + TOPICS[2], lambda msg: cls.statuses.append(json.loads(msg.data)), 10,
        )
        cls.heartbeat_sub = cls.node.create_subscription(
            String, ROOT + TOPICS[3], lambda msg: cls.heartbeats.append(json.loads(msg.data)), 10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate):
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.fail(f"Timed out: {self.statuses[-3:]}, {self.heartbeats[-2:]}")

    def publish(self, topic, data):
        self.publishers[topic].publish(String(data=json.dumps(data)))

    def fresh_heartbeat(self):
        sequence = self.heartbeats[-1]["sequence"] if self.heartbeats else -1
        self.wait(lambda: self.heartbeats and self.heartbeats[-1]["sequence"] > sequence + 2)

    def ball(self, **updates):
        self.publish(TOPICS[5], {"detected": True, "head_down_requested": True, **updates})
        self.fresh_heartbeat()

    def navigation(self, source):
        self.publish(TOPICS[4], {"source": source, "valid": True, "phase": "AUTO"})
        self.fresh_heartbeat()

    def request(self, request_id, motion_id):
        self.publish(TOPICS[0], {
            "request_id": request_id, "command_id": request_id, "event_id": None,
            "motion_id": motion_id, "action": "PICKUP_NOW" if motion_id in {"pickup", "pickup_400", "pickup_500"} else "STRAIGHT",
            "timeout_ms": 3000,
        })

    def status(self, request_id, status):
        self.wait(lambda: any(s["request_id"] == request_id and s["status"] == status
                             for s in self.statuses))

    def hold(self, enabled):
        self.fresh_heartbeat()
        self.assertIs(self.heartbeats[-1]["ball_head_override_active"], enabled)
        self.assertEqual(self.heartbeats[-1]["ball_head_override_deg"], -64.0)

    def test_ball_camera_hold_across_motion_boundaries(self):
        self.wait(lambda: all(p.get_subscription_count() for p in self.publishers.values()))
        from step.ball_analyzer import should_request_head_down
        self.navigation("ball")
        self.request(89, "ball_approach_fine_forward_45_5")
        self.status(89, "RUNNING")
        self.ball(head_down_requested=should_request_head_down(.500001, True, .05))
        self.hold(False)
        self.ball(head_down_requested=should_request_head_down(.500, True, .05))
        self.hold(True)
        self.assertFalse(any(s["request_id"] == 89 and s["status"] == "SUCCEEDED"
                             for s in self.statuses))
        self.publish(TOPICS[1], {"request_id": 89})
        self.status(89, "CANCELLED")
        self.hold(False)
        # Interior loss must lower the camera without any detected Ball message.
        for request_id, action in ((90, "BALL_LOST_BACKWARD_2"), (91, "PICKUP_NOW")):
            self.publish(TOPICS[0], {
                "request_id": request_id, "command_id": request_id, "event_id": None,
                "motion_id": "ball_lost_camera_down_backward_2", "action": action,
                "timeout_ms": 3000,
            })
            self.status(request_id, "RUNNING")
            self.hold(True)
            self.publish(TOPICS[1], {"request_id": request_id})
            self.status(request_id, "CANCELLED")
            self.hold(False)
        # Both moving corrections work even before a BALL navigation message arrives.
        for request_id, motion in ((1, "line_recovery_left_4"), (2, "line_recovery_right_4")):
            self.request(request_id, motion)
            self.status(request_id, "RUNNING")
            self.ball()
            self.hold(True)
            self.publish(TOPICS[1], {"request_id": request_id})
            self.status(request_id, "CANCELLED")
            self.hold(False)

        # BALL owns the camera while waiting with no motor motion active.
        self.navigation("ball")
        for invalid in ({"head_down_requested": False}, {"head_down_requested": "true"},
                        {"detected": False}):
            self.ball(**invalid)
            self.hold(False)
        self.ball()
        self.hold(True)
        self.assertFalse(self.heartbeats[-1]['ball_head_transition_complete'])
        self.assertIsNone(self.heartbeats[-1]['ball_head_transition_complete_stamp_ns'])
        transition_id = self.heartbeats[-1]['ball_head_transition_id']
        before_complete = self.node.get_clock().now().nanoseconds
        self.wait(lambda: self.heartbeats[-1]['ball_head_transition_complete'])
        completed = self.heartbeats[-1]
        self.assertEqual(completed['ball_head_transition_id'], transition_id)
        self.assertGreater(completed['ball_head_transition_complete_stamp_ns'], before_complete)
        self.assertLessEqual(completed['ball_head_transition_complete_stamp_ns'],
                             self.node.get_clock().now().nanoseconds)
        for request_id, motion in ((3, "stationary_turn_left"),
                                   (4, "ball_camera_down_forward_2"),
                                   (5, "ball_camera_down_forward_4")):
            self.request(request_id, motion)
            self.status(request_id, "RUNNING")
            self.hold(True)
            self.status(request_id, "SUCCEEDED")
            self.hold(True)
        # Invalid cancellations must not release camera ownership.
        self.request(6, "stationary_turn_right")
        self.status(6, "RUNNING")
        self.publish(TOPICS[1], {"request_id": 999})
        self.status(6, "REJECTED")
        self.hold(True)
        self.publish(TOPICS[1], {"request_id": 6})
        self.status(6, "CANCELLED")
        self.hold(False)

        # A newly requested BALL turn can acquire the hold during the turn itself.
        self.navigation("ball")
        self.request(7, "stationary_turn_right")
        self.status(7, "RUNNING")
        self.ball()
        self.hold(True)
        self.status(7, "SUCCEEDED")
        # Grasp motion retains its catalog camera frames and cannot re-latch.
        for request_id, motion in ((8, "pickup"), (9, "pickup_400"), (10, "pickup_500")):
            self.navigation("line")
            self.navigation("ball")
            self.ball()
            self.hold(True)
            self.request(request_id, motion)
            self.status(request_id, "RUNNING")
            self.ball()
            self.hold(False)
            self.status(request_id, "SUCCEEDED")
        self.navigation("line")
        self.navigation("ball")
        self.ball()
        self.hold(True)
