"""Tests for the simulated C++ full-system launch topology."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml


LEGACY_EXECUTABLES = {
    "legacy_motion_executor_adapter",
    "motion_executor_node",
    "legacy_motion_status_adapter",
    "sdk_motion_stub_node",
}


def load_launch_module():
    """Load full_system.launch.py as a Python module."""
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "full_system.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "full_system_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_description(monkeypatch, tmp_path):
    """Create the launch description without starting any node."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    return load_launch_module().generate_launch_description()


def node_parameters(node):
    """Return normalized parameters using their textual names."""
    parameters = node._Node__parameters[0]
    return {
        "".join(part.text for part in name): parameter_value(value)
        for name, value in parameters.items()
    }


def parameter_value(value):
    """Resolve static values normalized by launch_ros."""
    if isinstance(value, tuple):
        if not value:
            return []
        return yaml.safe_load("".join(part.text for part in value))
    return value


def test_full_system_uses_bridge_and_cpp_simulated_executor(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    nodes = [
        entity for entity in description.entities if isinstance(entity, Node)
    ]
    executables = {node.node_executable for node in nodes}

    assert {
        "yolo26_detector",
        "unified_vision_node",
        "motion_decision_node",
        "motion_command_bridge_node",
        "sdk_motion_executor",
    }.issubset(executables)
    assert executables.isdisjoint(LEGACY_EXECUTABLES)


def test_cpp_executor_is_forced_to_safe_simulated_parameters(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    executor = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "sdk_motion_executor"
    )
    parameters = node_parameters(executor)

    assert parameters == {
        "backend_type": "simulated",
        "enable_robot_hardware": False,
        "poll_period_ms": 20,
        "running_polls": 2,
        "settling_polls": 1,
        "explicit_torque_approval": False,
        "motion_json_path": "",
        "robot_device_path": "",
        "robot_baud_rate": 0,
        "robot_motor_ids": [],
    }


def test_obsolete_motion_launch_arguments_are_removed(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert "execution_mode" not in arguments
    assert "player_backend" not in arguments
    assert "mock_fail_after_updates" not in arguments
    assert "mock_failure_code" not in arguments
