#!/usr/bin/env python3

import os

import numpy as np
import rclpy
from custom_msgs.msg import HyperspectralImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from hsi_python_utils.fast_radiometric_processing import resolve_context, FastProcessPipeline

class RadiometricProcessingNode(Node):
    """Online radiometric processing"""

    def __init__(self) -> None:
        super().__init__('radiometric_processing')

        self.declare_parameter('device_name', '')
        self.declare_parameter('run_name_path', '')
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_cube', True)
        self.declare_parameter('throttled_rate_hz', 1.0)
        self.declare_parameter('mission_state', 'Idle')

        self.device_name = str(self.get_parameter('device_name').value)
        run_name_path = str(self.get_parameter('run_name_path').value)
        raw_image_topic = f'/hsi/{self.device_name}/raw'
        output_topic = f'/hsi/{self.device_name}/reflectance'
        mission_state_topic = str(self.get_parameter('mission_state_topic').value)
        self.mission_state = str(self.get_parameter('mission_state').value)
        self.publish_rgb = bool(self.get_parameter('publish_rgb').value)
        self.publish_cube = bool(self.get_parameter('publish_cube').value)
        self.throttled_rate_hz = float(self.get_parameter('throttled_rate_hz').value)
        if self.throttled_rate_hz <= 0.0:
            raise ValueError('throttled_rate_hz must be > 0')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            )

        self._publish_reflectance_image = self.create_publisher(
            HyperspectralImage, output_topic, 10)
        self._publish_reflectance_image_throttled = self.create_publisher(
            HyperspectralImage, f'{output_topic}/throttled', 10)
        self._publish_rgb_preview = self.create_publisher(Image, f'{output_topic}/rgb', 10)
        self._sub_mission = self.create_subscription(String, mission_state_topic, self._on_mission_input, 10)
        self._sub_raw_image = self.create_subscription(Image, raw_image_topic, self._on_raw_input, qos)

        self._throttled_tick = False
        self._throttled_timer = self.create_timer(
            1.0 / self.throttled_rate_hz, self._throttled_metronome)
        self.context_path = os.path.join(run_name_path, self.device_name, 'raw', 'context')
        self.fpp = None
        self.pipeline_active = False
        self.context_complete = False
        self.binning_factor = 4
        self.nir_calibration_filename = 'CMV2K-SSM5x5-665_975-18.2.11.9.xml'
        self.vis_calibration_filename = 'CMV2K-SSM4x4-460_600-15.7.18.12.xml'
 
        
        self.get_logger().info(
            f'{self.device_name} radiometric processing node ready '
            f'(throttled_rate_hz={self.throttled_rate_hz:.3f})')

    def _throttled_metronome(self) -> None:
        # Same pattern as hsi_camera_node::timelapse_metronome — only arm a tick.
        self._throttled_tick = True

    def _on_raw_input(self, msg: Image) -> None:
        # check if pipeline initiated
        if self.publish_cube or self.publish_rgb:
            if self.pipeline_active:
                raw_frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32)
                reflectance_cube = self.fpp.fast_process_to_reflectance(raw_frame)
                self.publish_rgb_preview(reflectance_cube)
                # Full-rate publish on every frame (like /hsi/*/raw).
                # self.publish_hypercube(reflectance_cube)

            else:
                # check if context is complete
                if self.context_complete:
                    # if complete then initialise pipeline
                    self.fpp = FastProcessPipeline(self.context_path, matrix_type='hsi_irradiance')
                    self.pipeline_active=True
                    self.get_logger().info(f"{self.device_name} Pipeline Active")

                else:
                    # try to build context
                    try:
                        self.get_logger().info(f"{self.device_name} checking {self.context_path}")
                        context = resolve_context(self.context_path)
                        self.context_complete = True
                        self.get_logger().info(f"{self.device_name} Context Ready")
                    except Exception as e:
                        self.get_logger().info(f"Context Not Ready: {e}")


    def _on_mission_input(self, msg: String) -> None:
        self.mission_state = msg.data
        # not sure yet if this affects the behaviour, may need to manually save during timelapse if rosbag method doesn't work.
    

    def publish_rgb_preview(self, cube):
        # preview rgb for foxglove interface
        if self.publish_rgb:
            msg = Image()
            if self.device_name == 'nir':
                rgb_preview = cube[:: self.binning_factor, :: self.binning_factor, [20, 7, 2]]
            else:
                rgb_preview = cube[::self.binning_factor, ::self.binning_factor, [14, 7, 2]]
            rgb_u8 = (np.clip(rgb_preview, 0.0, 1.0) * 255.0).astype(np.uint8)
            msg.height = rgb_u8.shape[0]
            msg.width = rgb_u8.shape[1]
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = rgb_u8.tobytes()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'hsi_{self.device_name}_reflectance_rgb'
            self._publish_rgb_preview.publish(msg)

    def publish_hypercube(self, cube: np.ndarray) -> None:
        # Custom cube message so that rosbag can do the saving.
        if not self.publish_cube:
            return

        msg = HyperspectralImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'hsi_{self.device_name}_reflectance'
        msg.height = cube.shape[0]
        msg.width = cube.shape[1]
        msg.bands = cube.shape[2]
        msg.wavelengths_nm = [float(w) for w in self.fpp.wavelengths_nm]
        msg.data = (np.clip(cube, 0.0, 1.0) * 65535.0).astype(np.uint16).flatten().tolist()

        self._publish_reflectance_image.publish(msg)
        if self._throttled_tick:
            self._publish_reflectance_image_throttled.publish(msg)
            self._throttled_tick = False
        

    def destroy_node(self) -> bool:
        # delete pipeline on shutdown
        if self.pipeline_active:
            self.fpp = None
            self.pipeline_active = False
            self.get_logger().info(f"{self.device_name} pipeline released")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadiometricProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
