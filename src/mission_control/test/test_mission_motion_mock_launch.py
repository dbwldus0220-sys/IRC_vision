import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "mission_motion_mock.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "mission_motion_mock_launch", launch_path
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mission_motion_mock_launch(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    description = load_launch_module().generate_launch_description()
    nodes = [
        action
        for action in description.entities
        if isinstance(action, Node)
    ]
    mission_nodes = [
        node for node in nodes if node.node_package == "mission_control"
    ]
    executables = {node.node_executable for node in mission_nodes}

    assert len(mission_nodes) == 5
    assert executables == {
        "motion_decision_node",
        "mock_mission_input_node",
        "motion_executor_node",
        "legacy_motion_executor_adapter",
        "legacy_motion_status_adapter",
    }
    assert "motion_command_bridge_node" not in executables

    forbidden = {
        "robot_motion_player",
        "sdk_motion_stub_node",
        "dynamixel",
        "dynamixel_node",
        "realsense2_camera",
        "yolo26_detector",
        "yolo",
    }
    assert executables.isdisjoint(forbidden)

    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    default_backend = "".join(
        substitution.text
        for substitution in arguments["player_backend"].default_value
    )
    assert default_backend == "mock"
