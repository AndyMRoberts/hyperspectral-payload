from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stereo_binned_preview_cpp",
            executable="stereo_binned_preview_node",
            name="stereo_binned_preview",
            output="screen",
            parameters=[{
                "publish_rate_hz": 10.0,
                "bin_size": 4,
                "bin_mode": "average",
                "camera_width": 1280,
                "camera_height": 720,
                "left_topic": "/sensors/stereo/left",
                "right_topic": "/sensors/stereo/right",
                "topic_prefix": "/sensors/stereo",
            }],
        ),
    ])
