from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from hsi_python_utils import create_run_name_path

_FOXGLOVE_TOPIC_WHITELIST = (
    "['/hsi/vis/preview', "
    "'/hsi/nir/preview', "
    "'/sensors/stereo/preview/left', "
    "'/sensors/stereo/preview/right', "
    "'/sensors/stereo/preview/stereo', "
    "'/hsi/vis/exposure_time_actual', "
    "'/hsi/nir/exposure_time_actual', "
    "'/hsi/nir/set_preview_type', "
    "'/system_profile', "
    "'/hsi/vis/temp', "
    "'/hsi/nir/temp', "
    "'/sensors/gps', "
    "'/sensors/spectrometer']"
)
_FOXGLOVE_SERVICE_WHITELIST = "['/hsi/set_preview_type', '/hsi/camera_commands']"


def _launch_setup(context, *args, **kwargs):
    run_name_suffix = LaunchConfiguration("run_name").perform(context)
    run_name_path = create_run_name_path(run_name_suffix)
    acquisition_rate_hz = float(LaunchConfiguration("acquisition_rate_hz").perform(context))
    throttled_rate_hz = float(LaunchConfiguration("throttled_rate_hz").perform(context))

    foxglove_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [
                get_package_share_directory("foxglove_bridge"),
                "/launch/foxglove_bridge_launch.xml",
            ]
        ),
        launch_arguments={
            "max_qos_depth": "1",
            "min_qos_depth": "1",
            "topic_whitelist": _FOXGLOVE_TOPIC_WHITELIST,
            "service_whitelist": _FOXGLOVE_SERVICE_WHITELIST,
        }.items(),
    )

    vis_camera = Node(
        package="hsi_camera_driver_cpp",
        executable="hsi_camera_node",
        name="hsi_camera_acquisition_node_vis",
        output="screen",
        parameters=[
            {
                "free_run": True,
                "acquisition_rate_hz": acquisition_rate_hz,
                "throttled_rate_hz": throttled_rate_hz,
                "device_name": "vis",
                "exposure_time_ms": 20.0,
                "gain": 1.0,
                "test_frame_count": -1,
                "camera_start_delay_ms": 0,
                "run_name_path": run_name_path,
            }
        ],
    )

    nir_camera = Node(
        package="hsi_camera_driver_cpp",
        executable="hsi_camera_node",
        name="hsi_camera_acquisition_node_nir",
        output="screen",
        parameters=[
            {
                "free_run": True,
                "acquisition_rate_hz": acquisition_rate_hz,
                "throttled_rate_hz": throttled_rate_hz,
                "device_name": "nir",
                "exposure_time_ms": 30.0,
                "gain": 1.0,
                "test_frame_count": -1,
                "camera_start_delay_ms": 2500,
                "run_name_path": run_name_path,
            }
        ],
    )

    preview_params = {
        "publish_rate_hz": 10.0,
        "extra_bin": 2,  # increase to compress image stream
        "preview_type": "rgb",  # one of: "bin_max", "bin_average", "rgb"
        "raw_image_topic": "/hsi/vis/raw",
    }

    binned_preview_vis = Node(
        package="hsi_binned_preview_cpp",
        executable="hsi_binned_preview_node",
        name="hsi_binned_preview_vis",
        output="screen",
        parameters=[{**preview_params, "camera": "vis"}],
    )

    binned_preview_nir = Node(
        package="hsi_binned_preview_cpp",
        executable="hsi_binned_preview_node",
        name="hsi_binned_preview_nir",
        output="screen",
        parameters=[{**preview_params, "camera": "nir", "raw_image_topic": "/hsi/nir/raw"}],
    )

    stereo_size = {
        "camera_width": 640,
        "camera_height": 480,
    }

    stereo_camera = Node(
        package="stereo_camera",
        executable="stereo_camera",
        name="stereo_camera",
        output="screen",
        parameters=[{
            "acquisition_rate_hz": acquisition_rate_hz,
            "throttled_rate_hz": throttled_rate_hz,
            **stereo_size,
        }],
    )

    stereo_preview_params = {
        "publish_rate_hz": 10.0,
        "bin_size": 4,
        "bin_mode": "average",
        "left_topic": "/sensors/stereo/left",
        "right_topic": "/sensors/stereo/right",
        "topic_prefix": "/sensors/stereo",
        **stereo_size,
    }

    stereo_binned_preview = Node(
        package="stereo_binned_preview_cpp",
        executable="stereo_binned_preview_node",
        name="stereo_binned_preview",
        output="screen",
        parameters=[stereo_preview_params],
    )

    system_profiler = Node(
        package="system_profiler",
        executable="system_profiler",
        name="system_profiler",
        output="screen",
        parameters=[{"run_name_path": run_name_path}],
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

    gps = Node(
        package="gps_monitor_cpp", 
        executable="gps_monitor",
        name="gps_monitor",
        output="screen",
    )

    spectrometer = Node(
        package="spectrometer",
        executable="spectrometer",
        name="spectrometer",
        output="screen",
        parameters=[{"rate_hz": 1.0}],
    )

    return [
        foxglove_launch,
        vis_camera,
        nir_camera,
        binned_preview_vis,
        binned_preview_nir,
        stereo_camera,
        stereo_binned_preview,
        commands_controller,
        system_profiler,
        gps,
        spectrometer,
    ]


def generate_launch_description():
    run_name_arg = DeclareLaunchArgument(
        "run_name",
        default_value="default",
        description=(
            "Suffix for run folder; launch builds run_name_path under "
            "/mnt/data/timelapses/YYYYMMDD_HHMMSS_<run_name> and passes it to all nodes."
        ),
    )
    acquisition_rate_hz_arg = DeclareLaunchArgument(
        "acquisition_rate_hz",
        default_value="20.0",
        description="Camera acquisition rate in Hz for HSI and stereo nodes.",
    )
    throttled_rate_hz_arg = DeclareLaunchArgument(
        "throttled_rate_hz",
        default_value="1.0",
        description="Throttled publish/save rate in Hz for HSI timelapse and stereo throttled topics.",
    )

    return LaunchDescription(
        [
            run_name_arg,
            acquisition_rate_hz_arg,
            throttled_rate_hz_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
