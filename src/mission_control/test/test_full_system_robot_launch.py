"""Tests for the hardware-disabled robot full-system launch defaults."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
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


def node_remappings(node, context):
    """Evaluate a Node action's configured remapping rules."""
    return {
        (
            perform_substitutions(context, source),
            perform_substitutions(context, destination),
        )
        for source, destination in node._Node__remappings
    }


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


def test_robot_launch_defaults_are_production_ready(
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

    assert context.launch_configurations["enable_camera"] == "true"
    assert context.launch_configurations["device"] == "tensorrt"
    assert context.launch_configurations["display"] == "true"
    assert context.launch_configurations["initial_mission_phase"] == "AUTO"
    assert parameters == {
        "backend_type": "robot_motion_player",
        "enable_robot_hardware": True,
        "poll_period_ms": 5,
        "running_polls": 2,
        "settling_polls": 1,
        "explicit_torque_approval": True,
        "motion_json_path": (
            "/home/jet/IRC/external_sdk/"
            "robot_motion_player_sdk_work_20260801/"
            "final step/robot_motions.json"
        ),
        "robot_device_path": "/dev/ttyUSB1",
        "robot_baud_rate": 4000000,
        "robot_motor_ids": list(range(23)),
        "startup_pose_enabled": True,
        "startup_pose_name": "오뒤307",
        "startup_pose_duration_ms": 1800,
    }


def test_robot_launch_keeps_realsense_topic_remappings(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    nodes = {
        entity.node_executable: entity
        for entity in description.entities
        if isinstance(entity, Node)
    }

    assert node_remappings(nodes["yolo26_detector"], context) == {
        (
            "/camera/color/image_raw",
            "/camera/camera/color/image_raw",
        ),
    }
    assert node_remappings(nodes["unified_vision_node"], context) == {
        (
            "/camera/color/image_raw",
            "/camera/camera/color/image_raw",
        ),
        (
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/camera/aligned_depth_to_color/image_raw",
        ),
        (
            "/camera/color/camera_info",
            "/camera/camera/color/camera_info",
        ),
    }
