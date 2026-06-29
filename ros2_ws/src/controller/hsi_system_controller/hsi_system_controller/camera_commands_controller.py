#!/usr/bin/env python3

import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from hsi_camera_driver_cpp.srv import CameraCommands
from std_msgs.msg import String

_TIMELAPSE_COMMANDS = frozenset({"timelapse", "time_lapse"})


def _normalize_camera_command(command: str) -> str:
    normalized = command.strip().lower()
    if normalized in _TIMELAPSE_COMMANDS:
        return "timelapse"
    return normalized


def _is_timelapse_command(command: str) -> bool:
    return _normalize_camera_command(command) == "timelapse" or _normalize_camera_command(command) == "time_lapse"


def _build_bag_exclude_regex(topics: List[str]) -> str:
    """Combine topic names into one -x regex (ros2 bag only accepts a single --exclude)."""
    if not topics:
        return ""
    # Anchor each topic so e.g. excluding /hsi/vis/raw does not drop /hsi/vis/raw/throttled.
    parts = [re.escape(topic.strip()) + r"$" for topic in topics if topic.strip()]
    return f"({'|'.join(parts)})" if parts else ""


class HsiCameraCommandsController(Node):
    def __init__(self) -> None:
        super().__init__("hsi_camera_commands_controller")

        self.declare_parameter("cameras", ["vis", "nir"])
        self.declare_parameter("fanout_timeout_sec", 3.0)
        self.declare_parameter("wait_for_service_sec", 0.75)
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("run_name_path", "")
        self.declare_parameter(
            "bag_exclude_topics",
            [
                "/hsi/vis/preview",
                "/hsi/nir/preview",
                "/hsi/nir/raw",
                "/hsi/vis/raw",
                "/sensors/stereo/left",
                "/sensors/stereo/right",
                "/sensors/stereo/preview/left",
                "/sensors/stereo/preview/right",
                "/sensors/stereo/preview/stereo",
            ],
        )

        self._cameras: List[str] = list(self.get_parameter("cameras").value)
        self._fanout_timeout_sec = float(self.get_parameter("fanout_timeout_sec").value)
        self._wait_for_service_sec = float(self.get_parameter("wait_for_service_sec").value)
        self._mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        run_name_path = str(self.get_parameter("run_name_path").value).strip()
        if not run_name_path:
            raise ValueError("run_name_path parameter must be set to an absolute output directory")
        self._run_name_path = Path(run_name_path)
        self._bag_exclude_topics: List[str] = list(self.get_parameter("bag_exclude_topics").value)

        # Clients must not share the service's default mutually-exclusive group;
        # otherwise async responses cannot run while _handle_camera_command is waiting.
        self._client_cb_group = ReentrantCallbackGroup()
        self._camera_clients: Dict[str, Client] = {}
        for camera in self._cameras:
            service_name = f"/hsi/{camera}/camera_commands"
            self._camera_clients[camera] = self.create_client(
                CameraCommands, service_name, callback_group=self._client_cb_group
            )

        self._server = self.create_service(
            CameraCommands, "/hsi/camera_commands", self._handle_camera_command
        )
        self._mission_sub = self.create_subscription(
            String, self._mission_state_topic, self._on_mission_state, 10
        )

        self._bag_process: Optional[subprocess.Popen] = None
        self._bag_output_path: Optional[Path] = None
        self._shutdown_pending = False
        self._shutdown_timer = None

        self.get_logger().info(
            f"Controller ready on /hsi/camera_commands, fanout targets={','.join(self._cameras)}, "
            f"run_name_path={self._run_name_path}"
        )

    def _make_bag_output_path(self) -> Path:
        # ros2 bag record -o expects a new directory name; it creates the folder itself.
        return self._run_name_path / "rosbag"

    def _start_rosbag_recording(self) -> None:
        if self._bag_process is not None and self._bag_process.poll() is None:
            self.get_logger().warn(
                f"rosbag recording already active at {self._bag_output_path}"
            )
            return

        output = self._make_bag_output_path()
        if output.exists():
            self.get_logger().error(
                f"rosbag output path already exists; not recording: {output}"
            )
            return

        cmd = ["ros2", "bag", "record", "-a", "-o", str(output)]
        exclude_regex = _build_bag_exclude_regex(self._bag_exclude_topics)
        if exclude_regex:
            cmd.extend(["-x", exclude_regex])

        try:
            self._bag_process = subprocess.Popen(cmd)
        except OSError as exc:
            self.get_logger().error(f"failed to start rosbag recording: {exc}")
            self._bag_process = None
            self._bag_output_path = None
            return

        self._bag_output_path = output
        self.get_logger().info(f"Started rosbag recording to {output} (exclude regex: {exclude_regex!r})")

    def _stop_rosbag_recording(self) -> None:
        if self._bag_process is None:
            return

        process = self._bag_process
        output = self._bag_output_path
        self._bag_process = None
        self._bag_output_path = None

        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("rosbag recorder did not exit cleanly; sending SIGKILL")
                process.kill()
                process.wait(timeout=5.0)

        self.get_logger().info(f"Stopped rosbag recording at {output}")

    def _schedule_shutdown(self) -> None:
        if self._shutdown_pending:
            return
        self._shutdown_pending = True
        self._shutdown_timer = self.create_timer(0.01, self._on_shutdown_timer)

    def _on_shutdown_timer(self) -> None:
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        self._stop_rosbag_recording()
        self.get_logger().info("shutdown - exiting controller cleanly")
        rclpy.shutdown()

    def _on_mission_state(self, msg: String) -> None:
        if msg.data.strip().lower() == "shutdown":
            self.get_logger().info(
                "mission_state shutdown - stopping rosbag and exiting controller cleanly"
            )
            self._schedule_shutdown()

    def _handle_camera_command(
        self, request: CameraCommands.Request, response: CameraCommands.Response
    ) -> CameraCommands.Response:
        command = _normalize_camera_command(request.command)
        self.get_logger().info(f"Received camera command '{request.command}' (normalized='{command}')")

        if command == "shutdown":
            self._stop_rosbag_recording()
            response.success = True
            response.message = "ok"
            self._schedule_shutdown()
            return response

        failures: List[str] = []
        futures: List[Tuple[str, Future]] = []

        for camera, client in self._camera_clients.items():
            if not client.wait_for_service(timeout_sec=self._wait_for_service_sec):
                failures.append(f"{camera}: service unavailable")
                continue

            req = CameraCommands.Request()
            req.command = command
            futures.append((camera, client.call_async(req)))

        deadline = time.monotonic() + self._fanout_timeout_sec
        while futures and time.monotonic() < deadline:
            if all(fut.done() for _, fut in futures):
                break
            time.sleep(0.01)

        for camera, future in futures:
            if not future.done():
                failures.append(f"{camera}: timeout")
                continue

            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001
                failures.append(f"{camera}: exception={exc}")
                continue

            if not result.success:
                failures.append(f"{camera}: {result.message}")

        response.success = len(failures) == 0
        response.message = "ok" if response.success else "; ".join(failures)

        if command == "idle" and response.success:
            self._stop_rosbag_recording()
        elif _is_timelapse_command(request.command) and response.success:
            self._start_rosbag_recording()
        elif _is_timelapse_command(request.command):
            self.get_logger().warn(
                f"timelapse fanout failed; rosbag not started: {response.message}"
            )

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HsiCameraCommandsController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._stop_rosbag_recording()
        executor.shutdown()
        node.destroy_node()
        # mission_state callback may already have called rclpy.shutdown(); avoid double shutdown.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
