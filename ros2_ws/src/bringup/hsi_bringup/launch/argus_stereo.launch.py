"""Launch Isaac Argus stereo (or mono left) with topics remapped to /sensors/stereo/*.

Run inside the Isaac ROS Dev container after building:
  source /workspaces/isaac_ros-dev/install/setup.bash
  ros2 launch hsi_bringup argus_stereo.launch.py
  ros2 launch hsi_bringup argus_stereo.launch.py single_stereo_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    single_stereo_mode = LaunchConfiguration("single_stereo_mode").perform(context).lower() in (
        "true",
        "1",
    )
    module_id = int(LaunchConfiguration("module_id").perform(context))
    mode = int(LaunchConfiguration("mode").perform(context))
    camera_id = int(LaunchConfiguration("camera_id").perform(context))
    mono_camera_id = int(LaunchConfiguration("mono_camera_id").perform(context))
    left_camera_info_url = LaunchConfiguration("left_camera_info_url").perform(context)
    right_camera_info_url = LaunchConfiguration("right_camera_info_url").perform(context)

    if single_stereo_mode:
        # ArgusMonoNode publishes left/*; Waveshare IMX219-83 left is sensor-id/camera_id=1
        # (same mapping as legacy stereo_camera).
        argus_node = ComposableNode(
            name="argus_mono",
            package="isaac_ros_argus_camera",
            plugin="nvidia::isaac_ros::argus::ArgusMonoNode",
            namespace="",
            parameters=[
                {
                    "module_id": module_id,
                    "mode": mode,
                    "camera_id": mono_camera_id,
                    "camera_info_url": left_camera_info_url,
                }
            ],
            remappings=[
                ("left/image_raw", "/sensors/stereo/left"),
                ("left/camera_info", "/sensors/stereo/left/camera_info"),
            ],
        )
        container_name = "argus_mono_container"
    else:
        argus_node = ComposableNode(
            name="argus_stereo",
            package="isaac_ros_argus_camera",
            plugin="nvidia::isaac_ros::argus::ArgusStereoNode",
            namespace="",
            parameters=[
                {
                    "module_id": module_id,
                    "mode": mode,
                    "camera_id": camera_id,
                    "left_camera_info_url": left_camera_info_url,
                    "right_camera_info_url": right_camera_info_url,
                }
            ],
            remappings=[
                ("left/image_raw", "/sensors/stereo/left"),
                ("right/image_raw", "/sensors/stereo/right"),
                ("left/camera_info", "/sensors/stereo/left/camera_info"),
                ("right/camera_info", "/sensors/stereo/right/camera_info"),
            ],
        )
        container_name = "argus_stereo_container"

    return [
        ComposableNodeContainer(
            name=container_name,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[argus_node],
            namespace="",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
        )
    ]


def generate_launch_description():
    module_id_arg = DeclareLaunchArgument(
        "module_id",
        default_value="-1",
        description="Index of the stereo camera module (-1 = auto).",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="0",
        description="Argus sensor mode index.",
    )
    camera_id_arg = DeclareLaunchArgument(
        "camera_id",
        default_value="0",
        description="Argus camera_id parameter for stereo mode.",
    )
    mono_camera_id_arg = DeclareLaunchArgument(
        "mono_camera_id",
        default_value="1",
        description=(
            "Argus camera_id used when single_stereo_mode:=true. "
            "Default 1 = left on Waveshare IMX219-83."
        ),
    )
    left_camera_info_url_arg = DeclareLaunchArgument(
        "left_camera_info_url",
        default_value="",
        description="Optional file:// URL for left camera_info.",
    )
    right_camera_info_url_arg = DeclareLaunchArgument(
        "right_camera_info_url",
        default_value="",
        description="Optional file:// URL for right camera_info.",
    )
    single_stereo_mode_arg = DeclareLaunchArgument(
        "single_stereo_mode",
        default_value="false",
        description=(
            "If true, launch ArgusMonoNode for left only (/sensors/stereo/left). "
            "If false, launch ArgusStereoNode for left+right."
        ),
    )

    return LaunchDescription(
        [
            module_id_arg,
            mode_arg,
            camera_id_arg,
            mono_camera_id_arg,
            left_camera_info_url_arg,
            right_camera_info_url_arg,
            single_stereo_mode_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
