"""Isolated playback comparison using the original shoulder targets."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('irc_step_motion_executor')
    return LaunchDescription([
        DeclareLaunchArgument('continuous_interpolation', default_value='true'),
        DeclareLaunchArgument('exclude_motor_5', default_value='false'),
        DeclareLaunchArgument('backend_type', default_value='simulated'),
        DeclareLaunchArgument('enable_robot_hardware', default_value='false'),
        DeclareLaunchArgument('explicit_torque_approval', default_value='false'),
        DeclareLaunchArgument('robot_device_path', default_value=''),
        DeclareLaunchArgument('motion_json_path', default_value=PathJoinSubstitution([share, 'config', 'robot_motions_runtime.json'])),
        DeclareLaunchArgument('enable_shoulder_override', default_value='false'),
        DeclareLaunchArgument('motion_trace_path', default_value='/tmp/step_gui_alignment.csv'),
        Node(package='irc_step_motion_executor', executable='sdk_motion_executor',
             name='sdk_motion_executor', output='screen', parameters=[{
                 'backend_type': LaunchConfiguration('backend_type'),
                 'enable_robot_hardware': ParameterValue(LaunchConfiguration('enable_robot_hardware'), value_type=bool),
                 'explicit_torque_approval': ParameterValue(LaunchConfiguration('explicit_torque_approval'), value_type=bool),
                 'robot_device_path': LaunchConfiguration('robot_device_path'),
                 'continuous_interpolation': ParameterValue(LaunchConfiguration('continuous_interpolation'), value_type=bool),
                 'exclude_motor_5': ParameterValue(LaunchConfiguration('exclude_motor_5'), value_type=bool),
                 'robot_baud_rate': 4000000,
                 'robot_motor_ids': list(range(23)),
                 'motion_json_path': LaunchConfiguration('motion_json_path'),
                 'policy_reference_json_path': LaunchConfiguration('motion_json_path'),
                 'enable_head_override': False,
                 'enable_shoulder_override': ParameterValue(LaunchConfiguration('enable_shoulder_override'), value_type=bool),
                 'position_tolerance_enabled': False,
                 'startup_pose_enabled': False,
                 'poll_period_ms': 5,
                 'queued_transition_hold_ms': 0,
                 'motion_trace_enabled': True,
                 'motion_trace_goals': True,
                 'motion_trace_path': LaunchConfiguration('motion_trace_path'),
             }]),
    ])
