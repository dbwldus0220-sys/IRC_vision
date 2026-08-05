#!/usr/bin/env python3
"""Launch the STEP pipeline with an opt-in robot motion backend."""

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
    """Build the hardware-capable graph with hardware disabled by default."""
    enable_camera = LaunchConfiguration("enable_camera")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    metrics_mode = LaunchConfiguration("metrics_mode")
    max_fps = LaunchConfiguration("max_fps")
    initial_mission_phase = LaunchConfiguration("initial_mission_phase")
    backend_type = LaunchConfiguration("backend_type")
    enable_robot_hardware = LaunchConfiguration("enable_robot_hardware")
    explicit_torque_approval = LaunchConfiguration(
        "explicit_torque_approval"
    )
    motion_json_path = LaunchConfiguration("motion_json_path")
    robot_device_path = LaunchConfiguration("robot_device_path")
    robot_baud_rate = LaunchConfiguration("robot_baud_rate")
    robot_motor_ids = LaunchConfiguration("robot_motor_ids")

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

    unified_vision = Node(
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
    motion_command_bridge = Node(
        package="mission_control",
        executable="motion_command_bridge_node",
        name="motion_command_bridge_node",
        output="screen",
        emulate_tty=True,
    )
    sdk_motion_executor = Node(
        package="irc_step_motion_executor",
        executable="sdk_motion_executor",
        name="sdk_motion_executor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "backend_type": ParameterValue(backend_type, value_type=str),
                "enable_robot_hardware": ParameterValue(
                    enable_robot_hardware,
                    value_type=bool,
                ),
                "poll_period_ms": 20,
                "running_polls": 2,
                "settling_polls": 1,
                "explicit_torque_approval": ParameterValue(
                    explicit_torque_approval,
                    value_type=bool,
                ),
                "motion_json_path": ParameterValue(
                    motion_json_path,
                    value_type=str,
                ),
                "robot_device_path": ParameterValue(
                    robot_device_path,
                    value_type=str,
                ),
                "robot_baud_rate": ParameterValue(
                    robot_baud_rate,
                    value_type=int,
                ),
                "robot_motor_ids": ParameterValue(robot_motor_ids),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Camera is opt-in for robot startup safety.",
            ),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("display", default_value="false"),
            DeclareLaunchArgument("metrics_mode", default_value="auto"),
            DeclareLaunchArgument("max_fps", default_value="30.0"),
            DeclareLaunchArgument("initial_mission_phase", default_value="AUTO"),
            DeclareLaunchArgument(
                "backend_type",
                default_value="robot_motion_player",
                choices=["robot_motion_player"],
                description="The only supported backend for this robot launch.",
            ),
            DeclareLaunchArgument(
                "enable_robot_hardware",
                default_value="false",
                description="Must remain false until the hardware procedure is approved.",
            ),
            DeclareLaunchArgument(
                "explicit_torque_approval",
                default_value="false",
                description="Independent explicit approval for torque enable.",
            ),
            DeclareLaunchArgument("motion_json_path", default_value=""),
            DeclareLaunchArgument("robot_device_path", default_value=""),
            DeclareLaunchArgument("robot_baud_rate", default_value="0"),
            DeclareLaunchArgument(
                "robot_motor_ids",
                default_value="[]",
                description="Integer array; empty by default and therefore unsafe to run.",
            ),
            camera,
            detector,
            unified_vision,
            motion_decision,
            motion_command_bridge,
            sdk_motion_executor,
        ]
    )
