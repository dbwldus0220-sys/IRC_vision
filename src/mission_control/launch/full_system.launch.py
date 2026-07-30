#!/usr/bin/env python3
"""Launch the complete STEP vision and motion-decision pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the complete camera-to-navigation ROS graph."""
    enable_camera = LaunchConfiguration("enable_camera")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    metrics_mode = LaunchConfiguration("metrics_mode")
    max_fps = LaunchConfiguration("max_fps")
    initial_mission_phase = LaunchConfiguration("initial_mission_phase")
    player_backend = LaunchConfiguration("player_backend")
    mock_fail_after_updates = LaunchConfiguration(
        "mock_fail_after_updates"
    )
    mock_failure_code = LaunchConfiguration("mock_failure_code")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
        }.items(),
    )

    detector = Node(
        package="step",
        executable="yolo26_detector",
        name="yolo26_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "device": device,
                "display": ParameterValue(display, value_type=bool),
                "metrics_mode": metrics_mode,
                "max_fps": ParameterValue(max_fps, value_type=float),
            }
        ],
    )

    analyzers = Node(
        package="step",
        executable="unified_vision_node",
        output="screen",
        emulate_tty=True,
    )

    motion_decision = Node(
        package="mission_control",
        executable="motion_decision_node",
        name="motion_decision_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"initial_mission_phase": initial_mission_phase}],
    )

    sdk_motion_stub = Node(
        package="mission_control",
        executable="sdk_motion_stub_node",
        name="sdk_motion_stub_node",
        output="screen",
        emulate_tty=True,
        condition=LaunchConfigurationEquals("execution_mode", "stub"),
    )

    command_adapter = Node(
        package="mission_control",
        executable="legacy_motion_executor_adapter",
        name="legacy_motion_executor_adapter",
        output="screen",
        emulate_tty=True,
        condition=LaunchConfigurationEquals("execution_mode", "executor"),
    )

    motion_executor = Node(
        package="mission_control",
        executable="motion_executor_node",
        name="motion_executor_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "player_backend": ParameterValue(
                    player_backend,
                    value_type=str,
                ),
                "mock_fail_after_updates": ParameterValue(
                    mock_fail_after_updates,
                    value_type=int,
                ),
                "mock_failure_code": ParameterValue(
                    mock_failure_code,
                    value_type=str,
                ),
            }
        ],
        condition=LaunchConfigurationEquals("execution_mode", "executor"),
    )

    status_adapter = Node(
        package="mission_control",
        executable="legacy_motion_status_adapter",
        name="legacy_motion_status_adapter",
        output="screen",
        emulate_tty=True,
        condition=LaunchConfigurationEquals("execution_mode", "executor"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Launch the RealSense camera and aligned depth stream.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cpu",
                description="ONNX Runtime device used by yolo26_detector.",
            ),
            DeclareLaunchArgument(
                "display",
                default_value="true",
                description="Show the detector visualization window.",
            ),
            DeclareLaunchArgument(
                "metrics_mode",
                default_value="auto",
                description="Metrics overlay selected by yolo26_detector.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="30.0",
                description="Maximum detector processing rate.",
            ),
            DeclareLaunchArgument(
                "initial_mission_phase",
                default_value="AUTO",
                description="Initial planner phase before /mission/phase is received.",
            ),
            DeclareLaunchArgument(
                "execution_mode",
                default_value="executor",
                choices=["executor", "stub"],
                description="Select the mutually exclusive motion topology.",
            ),
            DeclareLaunchArgument(
                "player_backend",
                default_value="mock",
                choices=["mock", "sdk"],
                description=(
                    "Motion Executor backend; sdk is currently a safe "
                    "disconnected placeholder."
                ),
            ),
            DeclareLaunchArgument(
                "mock_fail_after_updates",
                default_value="-1",
                description="Mock failure update count; -1 disables failure.",
            ),
            DeclareLaunchArgument(
                "mock_failure_code",
                default_value="COMMUNICATION_ERROR",
                description="MotionError enum name used for mock failure.",
            ),
            camera,
            detector,
            analyzers,
            motion_decision,
            sdk_motion_stub,
            command_adapter,
            motion_executor,
            status_adapter,
        ]
    )
