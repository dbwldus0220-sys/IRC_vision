import importlib.util
from pathlib import Path

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


EXECUTOR_NODES = {
    "legacy_motion_executor_adapter",
    "motion_executor_node",
    "legacy_motion_status_adapter",
}


def load_launch_module():
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
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    return load_launch_module().generate_launch_description()


def launch_arguments(description):
    return {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def default_value(argument):
    return "".join(
        substitution.text for substitution in argument.default_value
    )


def enabled_executables(description, execution_mode):
    context = LaunchContext()
    context.launch_configurations["execution_mode"] = execution_mode
    return {
        node.node_executable
        for node in description.entities
        if isinstance(node, Node)
        and (
            node.condition is None
            or node.condition.evaluate(context)
        )
    }


def executor_backend_value(description, backend):
    executor = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "motion_executor_node"
    )
    parameters = executor._Node__parameters[0]
    backend_parameter = next(
        value
        for name, value in parameters.items()
        if "".join(part.text for part in name) == "player_backend"
    )
    context = LaunchContext()
    context.launch_configurations["player_backend"] = backend
    return backend_parameter.evaluate(context)


def test_default_mode_uses_executor_chain(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    arguments = launch_arguments(description)

    assert default_value(arguments["execution_mode"]) == "executor"
    assert default_value(arguments["player_backend"]) == "mock"

    executables = enabled_executables(description, "executor")
    assert {
        "yolo26_detector",
        "unified_vision_node",
        "motion_decision_node",
    }.issubset(executables)
    assert EXECUTOR_NODES.issubset(executables)
    assert "sdk_motion_stub_node" not in executables
    assert executor_backend_value(description, "mock") == "mock"


def test_stub_mode_excludes_executor_chain(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    executables = enabled_executables(description, "stub")

    assert "motion_decision_node" in executables
    assert "sdk_motion_stub_node" in executables
    assert executables.isdisjoint(EXECUTOR_NODES)


def test_sdk_placeholder_backend_is_forwarded(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)

    assert EXECUTOR_NODES.issubset(
        enabled_executables(description, "executor")
    )
    assert executor_backend_value(description, "sdk") == "sdk"


@pytest.mark.parametrize("execution_mode", ["executor", "stub"])
def test_execution_modes_are_mutually_exclusive(
    monkeypatch,
    tmp_path,
    execution_mode,
):
    description = launch_description(monkeypatch, tmp_path)
    executables = enabled_executables(description, execution_mode)

    stub_enabled = "sdk_motion_stub_node" in executables
    executor_enabled = EXECUTOR_NODES.issubset(executables)
    assert stub_enabled != executor_enabled


def test_invalid_execution_mode_is_rejected(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    execution_mode = launch_arguments(description)["execution_mode"]
    context = LaunchContext()
    context.launch_configurations["execution_mode"] = "invalid"

    with pytest.raises(RuntimeError, match="not valid"):
        execution_mode.execute(context)
