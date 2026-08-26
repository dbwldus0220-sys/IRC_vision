#!/usr/bin/env python3
"""Launch the STEP vision pipeline with a selectable C++ motion backend."""

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
    """Build the camera-to-motion ROS graph."""
    enable_camera = LaunchConfiguration("enable_camera")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    metrics_mode = LaunchConfiguration("metrics_mode")
    max_fps = LaunchConfiguration("max_fps")
    initial_mission_phase = LaunchConfiguration("initial_mission_phase")
    recovery_heading_turn_deg = LaunchConfiguration(
        "recovery_heading_turn_deg"
    )
    recovery_away_heading_turn_deg = LaunchConfiguration(
        "recovery_away_heading_turn_deg"
    )
    curve_follow_max_offset_norm = LaunchConfiguration(
        "curve_follow_max_offset_norm"
    )
    robot_center_offset_px = LaunchConfiguration("robot_center_offset_px")
    camera_pitch_down_deg = LaunchConfiguration("camera_pitch_down_deg")
    camera_forward_offset_m = LaunchConfiguration("camera_forward_offset_m")
    line_roi_x_min_ratio = LaunchConfiguration("line_roi_x_min_ratio")
    line_roi_x_max_ratio = LaunchConfiguration("line_roi_x_max_ratio")
    corner_min_turn_delta_deg = LaunchConfiguration(
        "corner_min_turn_delta_deg"
    )
    corner_straight_max_turn_delta_deg = LaunchConfiguration(
        "corner_straight_max_turn_delta_deg"
    )
    corner_straight_motion_distance_m = LaunchConfiguration(
        "corner_straight_motion_distance_m"
    )
    corner_turn_margin_m = LaunchConfiguration("corner_turn_margin_m")
    ball_tracking_range_m = LaunchConfiguration("ball_tracking_range_m")
    ball_control_range_m = LaunchConfiguration("ball_control_range_m")
    goal_tracking_range_m = LaunchConfiguration("goal_tracking_range_m")
    goal_control_range_m = LaunchConfiguration("goal_control_range_m")
    hurdle_control_range_m = LaunchConfiguration("hurdle_control_range_m")
    backend_type = LaunchConfiguration("backend_type")
    enable_robot_hardware = LaunchConfiguration("enable_robot_hardware")
    motion_json_path = LaunchConfiguration("motion_json_path")
    explicit_torque_approval = LaunchConfiguration(
        "explicit_torque_approval"
    )
    robot_device_path = LaunchConfiguration("robot_device_path")
    robot_baud_rate = LaunchConfiguration("robot_baud_rate")

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
            "rgb_camera.color_profile": "1280,720,30",
            "depth_module.depth_profile": "848,480,30",
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
                "ball_tracking_range_m": ParameterValue(
                    ball_tracking_range_m, value_type=float
                ),
                "ball_control_range_m": ParameterValue(
                    ball_control_range_m, value_type=float
                ),
                "goal_tracking_range_m": ParameterValue(
                    goal_tracking_range_m, value_type=float
                ),
                "goal_control_range_m": ParameterValue(
                    goal_control_range_m, value_type=float
                ),
                "hurdle_tracking_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
                "hurdle_control_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
            }
        ],
    )

    analyzers = Node(
        package="step",
        executable="unified_vision_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "robot_center_offset_px": ParameterValue(
                    robot_center_offset_px, value_type=float
                ),
                "camera_pitch_down_deg": ParameterValue(
                    camera_pitch_down_deg, value_type=float
                ),
                "camera_forward_offset_m": ParameterValue(
                    camera_forward_offset_m, value_type=float
                ),
                "roi_x_min_ratio": ParameterValue(
                    line_roi_x_min_ratio, value_type=float
                ),
                "roi_x_max_ratio": ParameterValue(
                    line_roi_x_max_ratio, value_type=float
                ),
                "corner_min_turn_delta_deg": ParameterValue(
                    corner_min_turn_delta_deg, value_type=float
                ),
                "corner_straight_max_turn_delta_deg": ParameterValue(
                    corner_straight_max_turn_delta_deg, value_type=float
                ),
                "corner_straight_motion_distance_m": ParameterValue(
                    corner_straight_motion_distance_m, value_type=float
                ),
                "corner_turn_margin_m": ParameterValue(
                    corner_turn_margin_m, value_type=float
                ),
            }
        ],
    )

    motion_decision = Node(
        package="mission_control",
        executable="motion_decision_node",
        name="motion_decision_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "initial_mission_phase": initial_mission_phase,
                "recovery_heading_turn_deg": ParameterValue(
                    recovery_heading_turn_deg, value_type=float
                ),
                "recovery_away_heading_turn_deg": ParameterValue(
                    recovery_away_heading_turn_deg, value_type=float
                ),
                "curve_follow_max_offset_norm": ParameterValue(
                    curve_follow_max_offset_norm, value_type=float
                ),
                "ball_tracking_range_m": ParameterValue(
                    ball_tracking_range_m, value_type=float
                ),
                "ball_control_range_m": ParameterValue(
                    ball_control_range_m, value_type=float
                ),
                "goal_tracking_range_m": ParameterValue(
                    goal_tracking_range_m, value_type=float
                ),
                "goal_control_range_m": ParameterValue(
                    goal_control_range_m, value_type=float
                ),
                "hurdle_control_range_m": ParameterValue(
                    hurdle_control_range_m, value_type=float
                ),
            }
        ],
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
                "backend_type": backend_type,
                "enable_robot_hardware": ParameterValue(
                    enable_robot_hardware,
                    value_type=bool,
                ),
                "poll_period_ms": 5,
                "running_polls": 2,
                "settling_polls": 1,
                "explicit_torque_approval": ParameterValue(
                    explicit_torque_approval,
                    value_type=bool,
                ),
                "motion_json_path": motion_json_path,
                "robot_device_path": robot_device_path,
                "robot_baud_rate": ParameterValue(
                    robot_baud_rate,
                    value_type=int,
                ),
                "robot_motor_ids": list(range(23)),
            }
        ],
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
                description=(
                    "Initial planner phase before /mission/phase is received."
                ),
            ),
            DeclareLaunchArgument("recovery_heading_turn_deg", default_value="10.0"),
            DeclareLaunchArgument(
                "recovery_away_heading_turn_deg", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "curve_follow_max_offset_norm", default_value="0.55"
            ),
            DeclareLaunchArgument("robot_center_offset_px", default_value="70.0"),
            DeclareLaunchArgument("camera_pitch_down_deg", default_value="45.0"),
            DeclareLaunchArgument("camera_forward_offset_m", default_value="0.0"),
            DeclareLaunchArgument("line_roi_x_min_ratio", default_value="0.15"),
            DeclareLaunchArgument("line_roi_x_max_ratio", default_value="0.85"),
            DeclareLaunchArgument(
                "corner_min_turn_delta_deg", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "corner_straight_max_turn_delta_deg", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "corner_straight_motion_distance_m", default_value="0.05"
            ),
            DeclareLaunchArgument("corner_turn_margin_m", default_value="0.15"),
            DeclareLaunchArgument("ball_tracking_range_m", default_value="1.5"),
            DeclareLaunchArgument("ball_control_range_m", default_value="0.9"),
            DeclareLaunchArgument("goal_tracking_range_m", default_value="1.0"),
            DeclareLaunchArgument("goal_control_range_m", default_value="0.5"),
            DeclareLaunchArgument("hurdle_control_range_m", default_value="1.0"),
            DeclareLaunchArgument(
                "backend_type",
                default_value="simulated",
                choices=["simulated", "robot_motion_player"],
                description=(
                    "Select the simulated or robot_motion_player backend."
                ),
            ),
            DeclareLaunchArgument(
                "enable_robot_hardware",
                default_value="false",
                description="Allow access to physical robot hardware.",
            ),
            DeclareLaunchArgument(
                "motion_json_path",
                default_value="",
                description=(
                    "RobotMotionPlayer motion JSON path; empty is allowed "
                    "for the simulated backend."
                ),
            ),
            DeclareLaunchArgument(
                "explicit_torque_approval",
                default_value="false",
                description=(
                    "Explicitly approve torque enable during hardware "
                    "initialization."
                ),
            ),
            DeclareLaunchArgument(
                "robot_device_path",
                default_value="/dev/ttyUSB0",
                description="Dynamixel serial device for the fixed SDK profile.",
            ),
            DeclareLaunchArgument(
                "robot_baud_rate",
                default_value="4000000",
                description="Dynamixel baud rate for the fixed SDK profile.",
            ),
            camera,
            detector,
            analyzers,
            motion_decision,
            motion_command_bridge,
            sdk_motion_executor,
        ]
    )
