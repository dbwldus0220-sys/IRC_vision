#!/usr/bin/env python3
"""Run existing line, ball, goal, and hurdle analyzers in one process."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .ball_analyzer import BallAnalyzer
from .goal_analyzer import GoalAnalyzer
from .hurdle_analyzer import HurdleAnalyzer
from .yolo_line_analyzer import YoloLineAnalyzer


def _create_analyzers() -> list[Node]:
    """Construct the existing analyzers without duplicating their code."""
    return [
        YoloLineAnalyzer(),
        BallAnalyzer(),
        GoalAnalyzer(),
        HurdleAnalyzer(),
    ]


def main(args: list[str] | None = None) -> None:
    """Compose all vision analyzers into one executable process."""
    rclpy.init(args=args)
    nodes: list[Node] = []
    # Ball/goal/hurdle nodes have independent sensor and detection callback
    # groups. Extra workers prevent high-rate RGB analysis from starving the
    # latest aligned-depth cache used for distance control.
    executor = MultiThreadedExecutor(num_threads=8)
    try:
        nodes = _create_analyzers()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
