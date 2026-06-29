from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from hsi_python_utils import create_run_name_path


def _launch_setup(context, *args, **kwargs):
    run_name_path = create_run_name_path("40ms_exposure")

    vis_node = Node(
        package="hsi_camera_driver_cpp",
        executable="hsi_camera_node",
        name="hsi_camera_acquisition_node_vis",
        output="screen",
        parameters=[
            {
                "free_run": True,
                "acquisition_rate_hz": 0.0,
                "throttled_rate_hz": 1.0,
                "device_name": "vis",
                "exposure_time_ms": 40.0,
                "gain": 1.0,
                "test_frame_count": 1000,
                "camera_start_delay_ms": 1,
                "run_name_path": run_name_path,
            }
        ],
    )

    nir_node = Node(
        package="hsi_camera_driver_cpp",
        executable="hsi_camera_node",
        name="hsi_camera_acquisition_node_nir",
        output="screen",
        parameters=[
            {
                "free_run": True,
                "acquisition_rate_hz": 0.0,
                "throttled_rate_hz": 1.0,
                "device_name": "nir",
                "exposure_time_ms": 40.0,
                "gain": 1.0,
                "test_frame_count": 1000,
                "camera_start_delay_ms": 2501,
                "run_name_path": run_name_path,
            }
        ],
    )

    commands_controller = Node(
        package="hsi_system_controller",
        executable="camera_commands_controller",
        name="hsi_camera_commands_controller",
        output="screen",
        parameters=[
            {
                "cameras": ["vis", "nir"],
                "fanout_timeout_sec": 3.0,
                "wait_for_service_sec": 0.75,
                "run_name_path": run_name_path,
            }
        ],
    )

    return [vis_node, nir_node, commands_controller]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
