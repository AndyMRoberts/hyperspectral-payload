from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = {
        "publish_rate_hz": 5.0,
        "extra_bin": 1,
        "mission_state_topic": "/mission_state",
    }

    vis = Node(
        package="hsi_binned_preview_cpp",
        executable="hsi_binned_preview_node",
        name="hsi_binned_preview_vis",
        output="screen",
        parameters=[{**common, "camera": "vis"}],
    )

    nir = Node(
        package="hsi_binned_preview_cpp",
        executable="hsi_binned_preview_node",
        name="hsi_binned_preview_nir",
        output="screen",
        parameters=[{**common, "camera": "nir"}],
    )

    return LaunchDescription([vis, nir])
