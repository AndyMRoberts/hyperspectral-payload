# common utilities used by many nodes
import re
from datetime import datetime
from pathlib import Path

# Matches hsi_camera_node.cpp: kDataRoot + "timelapses"
data_root = "/mnt/data/timelapses"

_STAMPED_RUN_NAME_RE = re.compile(r"^\d{8}_\d{6}_")


def make_timestamped_run_name(run_name: str) -> str:
    """Return ``YYYYMMDD_HHMMSS_<run_name>``, or *run_name* if already stamped."""
    suffix = (run_name or "default").strip() or "default"
    if _STAMPED_RUN_NAME_RE.match(suffix):
        return suffix
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{stamp}_{suffix}"


def create_run_name_path(run_name):
    """Build run output directory path: ``<root>/YYYYMMDD_HHMMSS_<run_name>``.

    Aligns with ``HsiCameraAcquisitionNode::make_run_output_dir`` / ``make_run_dir_name``
    in ``hsi_camera_node.cpp`` (local time, ``%Y%m%d_%H%M%S``).
    """
    return str(Path(data_root) / make_timestamped_run_name(run_name))
