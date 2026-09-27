"""Check close-hurdle camera ownership on isolated simulated executor topics."""

import json
from pathlib import Path
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
from launch_ros.actions import Node
import rclpy
from std_msgs.msg import String


ROOT = "/hurdle_head_override_test"
TOPICS = (
    "/motion/executor/request", "/motion/executor/cancel",
    "/motion/executor/status", "/motion/executor/heartbeat",
    "/navigation/motion_command", "/vision/hurdle_info", "/vision/ball_info",
)


def generate_test_description():
    executor = Node(
        package="irc_step_motion_executor", executable="sdk_motion_executor",
        name="hurdle_head_override_test_executor", output="screen",
        parameters=[{
            "backend_type": "simulated", "enable_robot_hardware": False,
            "motion_aliases_file": str(Path(__file__).parent / "fixtures" / "hurdle_head_override_aliases.yaml"),
            "poll_period_ms": 20, "running_polls": 40, "settling_polls": 1,
            "heartbeat_period_ms": 20,
        }],
        remappings=[(topic, ROOT + topic) for topic in TOPICS],
    )
    return launch.LaunchDescription([
        executor, launch_testing.actions.ReadyToTest(),
        launch_testing.util.KeepAliveProc(),
    ]), {"executor": executor}


class TestHurdleHeadOverride(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("hurdle_head_override_test_client")
        cls.statuses = []
        cls.heartbeats = []
        cls.publishers = {
            topic: cls.node.create_publisher(String, ROOT + topic, 10)
            for topic in (TOPICS[0], TOPICS[1], TOPICS[4], TOPICS[5], TOPICS[6])
        }
        cls.status_sub = cls.node.create_subscription(
            String, ROOT + TOPICS[2],
            lambda msg: cls.statuses.append(json.loads(msg.data)), 10,
        )
        cls.heartbeat_sub = cls.node.create_subscription(
            String, ROOT + TOPICS[3],
            lambda msg: cls.heartbeats.append(json.loads(msg.data)), 10,
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

    def navigation(self, source, valid=True, phase="AUTO"):
        self.publish(TOPICS[4], {"source": source, "valid": valid, "phase": phase})

    def observation(self, **overrides):
        self.publish(TOPICS[5], {
            "detected": True, "confirmation_confirmed": True,
            "depth_valid": True, "depth_m": 0.66,
            "bottom_distance_px": 120, "head_down_requested": True, **overrides,
        })

    def hold(self, enabled):
        sequence = self.heartbeats[-1]["sequence"] if self.heartbeats else -1
        self.wait(lambda: self.heartbeats and self.heartbeats[-1]["sequence"] > sequence + 2)
        self.assertIs(self.heartbeats[-1]["hurdle_head_override_active"], enabled)
        self.assertEqual(self.heartbeats[-1]["hurdle_head_override_deg"], -60.0)

    def request(self, request_id, motion_id):
        self.publish(TOPICS[0], {
            "request_id": request_id, "command_id": request_id,
            "event_id": None, "motion_id": motion_id,
            "action": "GO" if motion_id == "hurdle" else "STRAIGHT",
            "timeout_ms": 3000,
        })

    def status(self, request_id, status):
        self.wait(lambda: any(
            s["request_id"] == request_id and s["status"] == status
            for s in self.statuses
        ))

    def test_bottom_boundary_hold_release_and_other_camera_ownership(self):
        self.wait(lambda: all(p.get_subscription_count() for p in self.publishers.values()))
        self.observation()
        self.hold(False)
        self.navigation("ball")
        self.observation()
        self.hold(False)
        self.navigation("hurdle", valid=False)
        self.observation()
        self.hold(False)
        for phase in ("LINE_TRACK_AFTER_PICKUP", "POST_SHOT_FORWARD"):
            self.navigation("line", phase=phase)
            self.observation()
            self.hold(False)
        self.navigation("line")
        for invalid in (
            {"depth_m": 1.001}, {"depth_valid": False}, {"depth_m": None},
            {"depth_m": True}, {"depth_m": float("nan")}, {"depth_m": 0},
            {"bottom_distance_px": 120.001}, {"confirmation_confirmed": False},
        ):
            self.observation(**invalid)
            self.hold(False)
        self.request(1, "line_forward_4")
        self.status(1, "RUNNING")
        self.observation(depth_m=1.0)
        self.hold(True)
        self.navigation("line")
        self.hold(True)
        self.status(1, "SUCCEEDED")
        self.navigation("ball")
        self.hold(False)
        self.navigation("hurdle", valid=False, phase="HURDLE_POSITIONING")
        self.hold(False)
        for invalid in (
            {"bottom_distance_px": 120.001}, {"bottom_distance_px": None},
            {"bottom_distance_px": -1}, {"bottom_distance_px": True},
            {"bottom_distance_px": "120"}, {"bottom_distance_px": float("nan")},
            {"bottom_distance_px": float("inf")},
            {"head_down_requested": False}, {"head_down_requested": "true"},
            {"detected": False}, {"detected": "true"},
            {"confirmation_confirmed": False},
            {"depth_m": 0.1, "bottom_distance_px": 200, "head_down_requested": False},
        ):
            self.observation(**invalid)
            self.hold(False)

        self.observation(depth_valid=False, depth_m=None)
        self.hold(True)
        self.assertGreater(self.heartbeats[-1]["hurdle_head_override_current_deg"], -60.0)
        self.wait(lambda: self.heartbeats[-1]["hurdle_head_override_current_deg"] == -60.0)
        self.status(1, "SUCCEEDED")
        self.navigation("none", valid=False)
        self.observation(detected=False, depth_valid=False, depth_m=None)
        self.hold(True)
        self.observation(depth_m=0.7)
        self.hold(True)
        self.request(2, "missing_motion")
        self.status(2, "REJECTED")
        self.hold(True)
        self.request(3, "hurdle")
        self.status(3, "RUNNING")
        self.hold(True)
        self.status(3, "SUCCEEDED")
        self.hold(False)
        self.observation()
        self.hold(False)
        self.navigation("line")
        self.observation()
        self.hold(False)
        self.observation(detected=False)
        self.hold(False)

        self.navigation("hurdle")
        self.hold(False)
        self.observation(depth_m=0.4)
        self.hold(True)
        self.request(4, "line_forward_4")
        self.status(4, "RUNNING")
        self.publish(TOPICS[1], {"request_id": 999})
        self.status(4, "REJECTED")
        self.hold(True)
        self.publish(TOPICS[1], {"request_id": 4})
        self.status(4, "CANCELLED")
        self.hold(False)

        self.request(5, "post_ball_camera_90")
        self.status(5, "SUCCEEDED")
        self.wait(lambda: self.heartbeats[-1]["goal_head_override_active"])
        self.navigation("hurdle")
        self.hold(False)
        self.observation()
        self.hold(True)
        self.publish(TOPICS[6], {"detected": True, "head_down_requested": True})
        self.hold(True)
        self.assertFalse(self.heartbeats[-1]["ball_head_override_active"])
        self.navigation("goal")
        self.hold(False)
        self.assertTrue(self.heartbeats[-1]["goal_head_override_active"])
