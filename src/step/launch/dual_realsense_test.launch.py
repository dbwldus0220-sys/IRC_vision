#!/usr/bin/env python3
"""Launch a test-only dual RealSense RGB selector and one YOLO detector."""

from __future__ import annotations

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _boolean_argument(context: LaunchContext, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"{name} must be true or false; got {value!r}")


def _serial_argument(context: LaunchContext, name: str) -> str:
    value = LaunchConfiguration(name).perform(context).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1].strip()
    if not value:
        raise RuntimeError(f"{name} is required for every enabled camera")
    return value


def _camera_launch_actions(context: LaunchContext) -> list:
    enable_front = _boolean_argument(context, "enable_front_camera")
    enable_down = _boolean_argument(context, "enable_down_camera")
    if not enable_front and not enable_down:
        raise RuntimeError("at least one RealSense camera must be enabled")

    initial_camera = (
        LaunchConfiguration("initial_active_camera")
        .perform(context)
        .strip()
        .lower()
    )
    if initial_camera not in {"front", "down"}:
        raise RuntimeError(
            "initial_active_camera must be front or down; "
            f"got {initial_camera!r}"
        )
    if initial_camera == "front" and not enable_front:
        raise RuntimeError("initial front camera is disabled")
    if initial_camera == "down" and not enable_down:
        raise RuntimeError("initial down camera is disabled")

    camera_names = {
        "front": LaunchConfiguration("front_camera_name")
        .perform(context)
        .strip(),
        "down": LaunchConfiguration("down_camera_name")
        .perform(context)
        .strip(),
    }
    if not all(camera_names.values()):
        raise RuntimeError(
            "front_camera_name and down_camera_name are required"
        )
    if camera_names["front"] == camera_names["down"]:
        raise RuntimeError("front and down camera names must be different")

    serials = {}
    if enable_front:
        serials["front"] = _serial_argument(context, "front_serial_no")
    if enable_down:
        serials["down"] = _serial_argument(context, "down_serial_no")
    if enable_front and enable_down and serials["front"] == serials["down"]:
        raise RuntimeError("front and down serial numbers must be different")

    common_parameters = {
        "camera_namespace": LaunchConfiguration("test_namespace"),
        "enable_color": True,
        "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
        "enable_depth": False,
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "enable_motion": False,
        "enable_rgbd": False,
        "pointcloud.enable": False,
        "align_depth.enable": False,
        "publish_tf": False,
    }

    actions = []
    for role, enabled in (("front", enable_front), ("down", enable_down)):
        if not enabled:
            continue
        parameters = dict(common_parameters)
        parameters["camera_name"] = camera_names[role]
        parameters["serial_no"] = serials[role]
        actions.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=LaunchConfiguration("test_namespace"),
                name=camera_names[role],
                output="screen",
                emulate_tty=True,
                parameters=[parameters],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the isolated two-camera, one-detector Stage 1 graph."""
    test_namespace = LaunchConfiguration("test_namespace")
    front_camera_name = LaunchConfiguration("front_camera_name")
    down_camera_name = LaunchConfiguration("down_camera_name")

    front_image_topic = PathJoinSubstitution(
        [test_namespace, front_camera_name, "color", "image_raw"]
    )
    down_image_topic = PathJoinSubstitution(
        [test_namespace, down_camera_name, "color", "image_raw"]
    )
    active_image_topic = PathJoinSubstitution(
        [test_namespace, "active", "color", "image_raw"]
    )
    detections_topic = PathJoinSubstitution(
        [test_namespace, "detections"]
    )
    annotated_image_topic = PathJoinSubstitution(
        [test_namespace, "detections", "image"]
    )
    camera_select_topic = PathJoinSubstitution(
        [test_namespace, "camera_select"]
    )
    active_camera_topic = PathJoinSubstitution(
        [test_namespace, "active_camera"]
    )

    selector = Node(
        package="step",
        executable="dual_realsense_selector_test",
        namespace=test_namespace,
        name="dual_realsense_selector_test",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "front_image_topic": ParameterValue(
                    front_image_topic,
                    value_type=str,
                ),
                "down_image_topic": ParameterValue(
                    down_image_topic,
                    value_type=str,
                ),
                "active_image_topic": ParameterValue(
                    active_image_topic,
                    value_type=str,
                ),
                "camera_select_topic": ParameterValue(
                    camera_select_topic,
                    value_type=str,
                ),
                "active_camera_topic": ParameterValue(
                    active_camera_topic,
                    value_type=str,
                ),
                "initial_active_camera": LaunchConfiguration(
                    "initial_active_camera"
                ),
            }
        ],
    )

    detector = Node(
        package="step",
        executable="yolo26_detector",
        namespace=test_namespace,
        name="yolo26_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "display": ParameterValue(
                    LaunchConfiguration("display"),
                    value_type=bool,
                ),
                "publish_annotated_image": ParameterValue(
                    LaunchConfiguration("publish_annotated_image"),
                    value_type=bool,
                ),
                "show_camera_controls": False,
                "show_line_metrics": False,
                "show_ball_metrics": False,
                "show_goal_metrics": False,
                "show_hurdle_metrics": False,
                "max_fps": ParameterValue(
                    LaunchConfiguration("max_fps"),
                    value_type=float,
                ),
                "image_topic": ParameterValue(
                    active_image_topic,
                    value_type=str,
                ),
                "detections_topic": ParameterValue(
                    detections_topic,
                    value_type=str,
                ),
                "annotated_image_topic": ParameterValue(
                    annotated_image_topic,
                    value_type=str,
                ),
                "line_info_topic": ParameterValue(
                    PathJoinSubstitution([test_namespace, "line_info"]),
                    value_type=str,
                ),
                "ball_info_topic": ParameterValue(
                    PathJoinSubstitution([test_namespace, "ball_info"]),
                    value_type=str,
                ),
                "goal_info_topic": ParameterValue(
                    PathJoinSubstitution([test_namespace, "goal_info"]),
                    value_type=str,
                ),
                "hurdle_info_topic": ParameterValue(
                    PathJoinSubstitution([test_namespace, "hurdle_info"]),
                    value_type=str,
                ),
                "motion_command_topic": ParameterValue(
                    PathJoinSubstitution(
                        [test_namespace, "motion_command"]
                    ),
                    value_type=str,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_serial_no",
                default_value="",
                description="Required serial number of the front RealSense.",
            ),
            DeclareLaunchArgument(
                "down_serial_no",
                default_value="",
                description="Required serial number of the down RealSense.",
            ),
            DeclareLaunchArgument(
                "front_camera_name",
                default_value="front_camera",
            ),
            DeclareLaunchArgument(
                "down_camera_name",
                default_value="down_camera",
            ),
            DeclareLaunchArgument(
                "test_namespace",
                default_value="/vision_test",
                description="Namespace isolating every experimental topic.",
            ),
            DeclareLaunchArgument(
                "initial_active_camera",
                default_value="front",
                choices=["front", "down"],
            ),
            DeclareLaunchArgument(
                "enable_front_camera",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "enable_down_camera",
                default_value="true",
                choices=["true", "false"],
                description="Set false for a comparable one-camera baseline.",
            ),
            DeclareLaunchArgument(
                "color_profile",
                default_value="1280,720,30",
                description="RealSense color profile: width,height,fps.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("step"), "models", "best.engine"]
                ),
            ),
            DeclareLaunchArgument(
                "device",
                default_value="tensorrt",
                choices=["auto", "tensorrt", "cuda", "cpu"],
            ),
            DeclareLaunchArgument(
                "display",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "publish_annotated_image",
                default_value="false",
                choices=["true", "false"],
                description="Enable only when visual output is needed.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="30.0",
                description="Maximum rate for the single YOLO instance.",
            ),
            OpaqueFunction(function=_camera_launch_actions),
            selector,
            detector,
        ]
    )
