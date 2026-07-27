"""Mock-backed ROS 2 Motion Executor node and ROS-free helper functions."""

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    from .mock_motion_player import MockRobotMotionPlayer
    from .motion_executor_core import MotionExecutionResult, MotionExecutorCore
except ImportError:  # Allows direct, ROS-free unit-test imports.
    from mock_motion_player import MockRobotMotionPlayer
    from motion_executor_core import MotionExecutionResult, MotionExecutorCore

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Pure helpers remain importable without a ROS installation.
    rclpy = None
    Node = object
    String = None


DEFAULT_TICK_PERIOD_MS = 10


@dataclass(frozen=True)
class MotionRequest:
    request_id: int
    motion_id: str
    timeout_ms: int


class RequestValidationError(ValueError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.message = message


def parse_motion_request(payload: str) -> MotionRequest:
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise RequestValidationError(
            "INVALID_REQUEST", f"invalid JSON: {exc}"
        ) from exc

    if not isinstance(data, dict):
        raise RequestValidationError(
            "INVALID_REQUEST", "request must be a JSON object"
        )

    missing = [
        field
        for field in ("request_id", "motion_id", "timeout_ms")
        if field not in data
    ]
    if missing:
        raise RequestValidationError(
            "INVALID_REQUEST",
            f"missing required field: {', '.join(missing)}",
        )

    request_id = data["request_id"]
    motion_id = data["motion_id"]
    timeout_ms = data["timeout_ms"]

    if isinstance(request_id, bool) or not isinstance(request_id, int):
        raise RequestValidationError(
            "INVALID_REQUEST", "request_id must be an integer"
        )
    if not isinstance(motion_id, str) or not motion_id:
        raise RequestValidationError(
            "INVALID_REQUEST", "motion_id must be a non-empty string"
        )
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int):
        raise RequestValidationError(
            "INVALID_REQUEST", "timeout_ms must be an integer"
        )
    if timeout_ms <= 0:
        raise RequestValidationError(
            "INVALID_REQUEST", "timeout_ms must be greater than zero"
        )

    return MotionRequest(request_id, motion_id, timeout_ms)


def build_status_payload(
    request_id: int,
    motion_id: str,
    status: str,
    error_code: str = "",
    message: str = "",
) -> Dict[str, Any]:
    return {
        "request_id": request_id,
        "motion_id": motion_id,
        "status": status,
        "error_code": error_code,
        "message": message,
    }


class ExecutionPublicationState:
    """Preserves the accepted request and emits its terminal payload once."""

    def __init__(self) -> None:
        self._request: Optional[MotionRequest] = None
        self._terminal_published = False

    def begin(self, request: MotionRequest) -> None:
        self._request = request
        self._terminal_published = False

    def running_payload(self) -> Dict[str, Any]:
        if self._request is None:
            raise RuntimeError("no active request")
        return build_status_payload(
            self._request.request_id,
            self._request.motion_id,
            "RUNNING",
        )

    def terminal_payload(
        self, result: MotionExecutionResult
    ) -> Optional[Dict[str, Any]]:
        if self._request is None or self._terminal_published:
            return None

        self._terminal_published = True
        return build_status_payload(
            self._request.request_id,
            self._request.motion_id,
            result.final_status.name,
            result.error_code,
            result.message,
        )

    def clear(self) -> None:
        self._request = None


class MotionExecutorNode(Node):
    def __init__(self) -> None:
        if rclpy is None or String is None:
            raise RuntimeError("rclpy and std_msgs are required to run the node")

        super().__init__("motion_executor_node")
        self.declare_parameter("tick_period_ms", DEFAULT_TICK_PERIOD_MS)
        configured_period = self.get_parameter(
            "tick_period_ms"
        ).get_parameter_value().integer_value
        self._tick_period_ms = (
            configured_period
            if configured_period > 0
            else DEFAULT_TICK_PERIOD_MS
        )

        self._core = MotionExecutorCore(MockRobotMotionPlayer())
        self._publication_state = ExecutionPublicationState()
        self._status_publisher = self.create_publisher(
            String, "/motion/executor/status", 10
        )
        self._request_subscription = self.create_subscription(
            String,
            "/motion/executor/request",
            self._on_request,
            10,
        )
        self._timer = self.create_timer(
            self._tick_period_ms / 1000.0, self._on_tick
        )

    def _publish_payload(self, payload: Dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _on_request(self, message: Any) -> None:
        try:
            request = parse_motion_request(message.data)
        except RequestValidationError as exc:
            self._publish_payload(
                build_status_payload(
                    0,
                    "",
                    "REJECTED",
                    exc.error_code,
                    exc.message,
                )
            )
            return

        if self._core.busy():
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    "REJECTED_BUSY",
                    "another motion is already running",
                )
            )
            return

        if request.motion_id not in MotionExecutorCore.SUPPORTED_MOTION_IDS:
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    "INVALID_MOTION",
                    "unsupported motion_id",
                )
            )
            return

        rejection = self._core.start_motion(
            request.motion_id, request.timeout_ms
        )
        if rejection is not None:
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    rejection.error_code,
                    rejection.message,
                )
            )
            return

        self._publication_state.begin(request)
        self._publish_payload(self._publication_state.running_payload())

    def _on_tick(self) -> None:
        result = self._core.tick(self._tick_period_ms)
        if result is None:
            return

        payload = self._publication_state.terminal_payload(result)
        if payload is None:
            return

        self._publish_payload(payload)
        self._core.reset()
        self._publication_state.clear()


def main(args: Optional[list] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run motion_executor_node")

    rclpy.init(args=args)
    node = MotionExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
