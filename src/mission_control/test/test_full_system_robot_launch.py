"""Tests for the hardware-disabled robot full-system launch defaults."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


LEGACY_EXECUTABLES = {
    "legacy_motion_executor_adapter",
    "motion_executor_node",
    "legacy_motion_status_adapter",
    "sdk_motion_stub_node",
}


def launch_description(monkeypatch, tmp_path):
    """Load the robot launch description without executing it."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "full_system_robot.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "full_system_robot_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def default_context(description):
    """Populate a launch context with declared default arguments."""
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def executor_parameters(executor, context):
    """Evaluate sdk_motion_executor parameters in one launch context."""
    parameters = executor._Node__parameters[0]
    output = {}
    for name, value in parameters.items():
        key = "".join(part.text for part in name)
        output[key] = value.evaluate(context) if hasattr(value, "evaluate") else value
    return output


def test_robot_launch_has_one_cpp_executor_and_no_legacy_nodes(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    executables = [
        entity.node_executable
        for entity in description.entities
        if isinstance(entity, Node)
    ]

    assert executables.count("sdk_motion_executor") == 1
    assert "motion_command_bridge_node" in executables
    assert set(executables).isdisjoint(LEGACY_EXECUTABLES)


def test_robot_launch_defaults_keep_hardware_and_camera_disabled(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    executor = next(
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_executable == "sdk_motion_executor"
    )
    parameters = executor_parameters(executor, context)

    assert context.launch_configurations["enable_camera"] == "false"
    assert parameters == {
        "backend_type": "robot_motion_player",
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
