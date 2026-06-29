#!/usr/bin/env python3

import os

# Jetson.GPIO (used by Blinka) may fail board detection on newer JetPack unless
# this is set before `board` is imported. An existing shell export takes precedence.
os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")

import board
import busio
import rclpy
from adafruit_as7341 import AS7341
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def _read_channel(getter) -> float:
    try:
        return float(getter())
    except Exception:
        return float("nan")


class SpectrometerNode(Node):
    def __init__(self) -> None:
        super().__init__("spectrometer")

        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("output_topic", "/sensors/spectrometer")

        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            rate_hz = 1.0
            self.get_logger().warn("rate_hz must be > 0; using 10.0 Hz")

        output_topic = str(self.get_parameter("output_topic").value)
        mission_state_topic = str(self.get_parameter("mission_state_topic").value)

        self._mission_shutdown_pending = False

        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self._sensor = AS7341(i2c)

        self._publisher = self.create_publisher(Float32MultiArray, output_topic, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self._mission_sub = self.create_subscription(
            String, mission_state_topic, self._on_mission_state, 10
        )

        self.get_logger().info(f"Publishing spectrometer readings on {output_topic}")

    def _read_spectrum(self) -> list[float]:
        sensor = self._sensor
        return [
            _read_channel(lambda: sensor.channel_415nm),
            _read_channel(lambda: sensor.channel_445nm),
            _read_channel(lambda: sensor.channel_480nm),
            _read_channel(lambda: sensor.channel_515nm),
            _read_channel(lambda: sensor.channel_555nm),
            _read_channel(lambda: sensor.channel_590nm),
            _read_channel(lambda: sensor.channel_630nm),
            _read_channel(lambda: sensor.channel_680nm),
            _read_channel(lambda: sensor.channel_nir),
            _read_channel(lambda: sensor.channel_clear),
        ]

    def _on_timer(self) -> None:
        if self._mission_shutdown_pending:
            return

        msg = Float32MultiArray()
        msg.data = self._read_spectrum()
        self._publisher.publish(msg)

    def _on_mission_state(self, msg: String) -> None:
        if msg.data.strip().lower() != "shutdown":
            return
        if self._mission_shutdown_pending:
            return

        self._mission_shutdown_pending = True
        self.get_logger().info("mission_state shutdown - exiting spectrometer cleanly")
        try:
            self.destroy_timer(self._timer)
        except (RuntimeError, ValueError):
            pass
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpectrometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
