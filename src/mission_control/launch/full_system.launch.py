#!/usr/bin/env python3
"""Launch the complete STEP vision and motion-decision pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
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
            camera,
            detector,
            analyzers,
            motion_decision,
            sdk_motion_stub,
        ]
    )
