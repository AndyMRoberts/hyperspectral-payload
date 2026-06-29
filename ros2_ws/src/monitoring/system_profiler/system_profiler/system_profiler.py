import csv
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

# Jetson Nano INA3221 rail 0 input voltage (mV per kernel IIO sysfs)
_IN_VOLTAGE = Path("/sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon1/in1_input")
_IN_CURRENT = Path("/sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon1/curr1_input")
_CPU_TEMP = Path("/sys/class/thermal/thermal_zone0/temp")
_GPU_TEMP = Path("/sys/class/thermal/thermal_zone1/temp")
_SOC0_TEMP = Path("/sys/class/thermal/thermal_zone5/temp")
_SOC1_TEMP = Path("/sys/class/thermal/thermal_zone6/temp")
_SOC2_TEMP = Path("/sys/class/thermal/thermal_zone7/temp")


def _read_sysfs_values(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return "na"


def _mem_available_mb() -> str:
    try:
        with open("/proc/meminfo", encoding="utf-8") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    kb = int(line.split()[1])
                    return str(round(kb / 1024))
    except (OSError, ValueError, IndexError):
        return "na"


def _disk_free_gb(mount: str) -> str:
    try:
        free = shutil.disk_usage(mount).free
        return str(round(free / (1024**3), 1))
    except OSError:
        return "na"


def _unique_csv_path(log_dir: Path, stem: str) -> Path:
    """Pick ``{stem}.csv`` or ``{stem}_NN.csv`` if the base name already exists."""
    candidate = log_dir / f"{stem}.csv"
    if not candidate.exists():
        return candidate
    n = 1
    while True:
        alt = log_dir / f"{stem}_{n:02d}.csv"
        if not alt.exists():
            return alt
        n += 1


class SystemProfiler(Node):
    def __init__(self) -> None:
        super().__init__("system_profiler")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("log_csv", True)
        self.declare_parameter("log_max_bytes", 10 * 1024 * 1024) # initially 10Mb
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("run_name_path", "")
        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            rate_hz = 1.0
            self.get_logger().warn("rate_hz must be > 0; using 1.0 Hz")

        self._log_csv = bool(self.get_parameter("log_csv").value)
        self._log_max_bytes = int(self.get_parameter("log_max_bytes").value)
        run_name_path = str(self.get_parameter("run_name_path").value).strip()
        if not run_name_path:
            raise ValueError("run_name_path parameter must be set to an absolute output directory")
        self._run_dir = Path(run_name_path)
        self._log_dir = self._run_dir / "logs"
        self.get_logger().info(
            f"run_name_path={self._run_dir} csv_log_dir={self._log_dir}"
        )

        self._csv_file = None
        self._csv_writer = None
        self._csv_path: Path | None = None
        self._csv_disabled_reason: str | None = None
        self._mission_shutdown_pending = False

        self._publisher = self.create_publisher(String, "/system_profile", 10)
        period_s = 1.0 / rate_hz
        self._timer = self.create_timer(period_s, self._on_timer)

        mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self._mission_sub = self.create_subscription(
            String, mission_state_topic, self._on_mission_state, 10
        )
    
    def _on_mission_state(self, msg: String) -> None:
        if msg.data.strip().lower() != "shutdown":
            return
        if self._mission_shutdown_pending:
            return
        self._mission_shutdown_pending = True
        self.get_logger().info("mission_state shutdown - exiting system_profiler cleanly")
        self._close_csv_log()
        try:
            self.destroy_timer(self._timer)
        except (RuntimeError, ValueError):
            pass
        rclpy.shutdown()

    def _on_timer(self) -> None:
        vin_raw = _read_sysfs_values(_IN_VOLTAGE)
        cin_raw = _read_sysfs_values(_IN_CURRENT)
        try:
            vin_f = float(vin_raw) / 1000.0
            vin_v = f"{vin_f:.3f}"
        except (TypeError, ValueError):
            vin_f = None
            vin_v = vin_raw
        try:
            cin_f = float(cin_raw) / 1000.0
            cin_a = f"{cin_f:.3f}"
        except (TypeError, ValueError):
            cin_f = None
            cin_a = cin_raw
        if vin_f is not None and cin_f is not None:
            pin_w = f"{vin_f * cin_f:.3f}"
        else:
            pin_w = "na"
        cpu_temp_raw = _read_sysfs_values(_CPU_TEMP)
        gpu_temp_raw = _read_sysfs_values(_GPU_TEMP)
        soc0_temp_raw = _read_sysfs_values(_SOC0_TEMP)
        soc1_temp_raw = _read_sysfs_values(_SOC1_TEMP)
        soc2_temp_raw = _read_sysfs_values(_SOC2_TEMP)
        try:
            cpu_temp = float(cpu_temp_raw) / 1000.0
            cpu_temp_f = f"{cpu_temp:.2f}"
        except (TypeError, ValueError):
            cpu_temp_f = None
            cpu_temp = cpu_temp_raw
        try:
            gpu_temp = float(gpu_temp_raw) / 1000.0
            gpu_temp_f = f"{gpu_temp:.2f}"
        except (TypeError, ValueError):
            gpu_temp_f = None
            gpu_temp = gpu_temp_raw
        try:
            soc0_temp = float(soc0_temp_raw) / 1000.0
            soc0_temp_f = f"{soc0_temp:.2f}"
        except (TypeError, ValueError):
            soc0_temp_f = None
            soc0_temp = soc0_temp_raw
        try:
            soc1_temp = float(soc1_temp_raw) / 1000.0
            soc1_temp_f = f"{soc1_temp:.2f}"
        except (TypeError, ValueError):
            soc1_temp_f = None
            soc1_temp = soc1_temp_raw
        try:
            soc2_temp = float(soc2_temp_raw) / 1000.0
            soc2_temp_f = f"{soc2_temp:.2f}"
        except (TypeError, ValueError):
            soc2_temp_f = None
            soc2_temp = soc2_temp_raw

        root_disk_gb = _disk_free_gb("/")
        data_disk_gb = _disk_free_gb("/mnt/data")
        mem_mb = _mem_available_mb()
        msg = (
            f"Input Voltage(V)={vin_v}\n"
            f"Input Current(A)={cin_a}\n"
            f"Input Power(W)={pin_w}\n"
            f"Main Disk Free(Gb)={root_disk_gb}\n"
            f"Data Disk Free(Gb)={data_disk_gb}\n"
            f"Memory Available (Mb)={mem_mb}\n"
            f"CPU Temp (*C)={cpu_temp_f}\n"
            f"GPU Temp (*C)={gpu_temp_f}\n"
            f"SOC0 Temp (*C)={soc0_temp_f}\n"
            f"SOC1 Temp (*C)={soc1_temp_f}\n"
            f"SOC2 Temp (*C)={soc2_temp_f}"
        )
        self._publisher.publish(String(data=msg))
        stamp = datetime.now().isoformat(timespec="milliseconds")
        self._append_csv_row(
            stamp,
            vin_v,
            cin_a,
            pin_w,
            root_disk_gb,
            data_disk_gb,
            mem_mb,
            cpu_temp_f,
            gpu_temp_f,
            soc0_temp_f,
            soc1_temp_f,
            soc2_temp_f,
        )

    def _append_csv_row(
        self,
        stamp: str,
        vin_v: str,
        cin_a: str,
        pin_w: str,
        root_disk_gb: str,
        data_disk_gb: str,
        mem_mb: str,
        cpu_temp_f: str | None,
        gpu_temp_f: str | None,
        soc0_temp_f: str | None,
        soc1_temp_f: str | None,
        soc2_temp_f: str | None,
    ) -> None:
        if not self._log_csv:
            return
        if self._csv_disabled_reason is not None:
            return

        def cell(v: str | None) -> str:
            return "" if v is None else str(v)

        row = [
            stamp,
            vin_v,
            cin_a,
            pin_w,
            root_disk_gb,
            data_disk_gb,
            mem_mb,
            cell(cpu_temp_f),
            cell(gpu_temp_f),
            cell(soc0_temp_f),
            cell(soc1_temp_f),
            cell(soc2_temp_f),
        ]

        try:
            if self._csv_file is None:
                self._open_new_csv_log()
            assert self._csv_writer is not None
            assert self._csv_file is not None
            self._csv_writer.writerow(row)
            self._csv_file.flush()
            if self._log_max_bytes > 0 and self._csv_path is not None:
                if self._csv_path.stat().st_size >= self._log_max_bytes:
                    self._close_csv_log()
        except OSError as e:
            self._csv_disabled_reason = str(e)
            self._close_csv_log()
            self.get_logger().error(f"CSV logging disabled: {e}")

    def _open_new_csv_log(self) -> None:
        self._close_csv_log()
        stem = f'system_profile_{datetime.now().strftime("%Y%m%d_%H%M")}'
        self._log_dir.mkdir(parents=True, exist_ok=True)
        path = _unique_csv_path(self._log_dir, stem)
        self._csv_file = open(path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "timestamp",
                "input_voltage_v",
                "input_current_a",
                "input_power_w",
                "main_disk_free_gb",
                "data_disk_free_gb",
                "memory_available_mb",
                "cpu_temp_c",
                "gpu_temp_c",
                "soc0_temp_c",
                "soc1_temp_c",
                "soc2_temp_c",
            ]
        )
        self._csv_path = path
        self.get_logger().info(f"Writing system profile CSV to {path}")

    def _close_csv_log(self) -> None:
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except OSError:
                pass
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None

    def destroy_node(self) -> bool:
        self._close_csv_log()
        return super().destroy_node()

    def run(self) -> None:
        """Prefer :func:`main`, which uses :class:`MultiThreadedExecutor` for mission shutdown."""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = SystemProfiler()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # Avoid double shutdown: mission_state (or SIGINT) may already have shut down
        # the default context; a second rclpy.shutdown() raises RuntimeError.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
