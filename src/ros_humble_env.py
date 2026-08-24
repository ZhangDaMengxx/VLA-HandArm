"""Locate and validate the small Python 3.10 ROS Humble runtime.

The main application runs under Python 3.12. ROS Humble on Ubuntu 22.04 ships
``rclpy`` as a CPython 3.10 extension, so only ROS reader/writer/runner
processes use this interpreter. The protocol between them remains ROS topics
or JSON lines; RGB-D frames and IK do not pass through this boundary.
"""
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
WORKSPACE = REPO.parent
EXPECTED_PYTHON = (3, 10)
DEFAULT_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash"
    f" && source {shlex.quote(str(WORKSPACE / 'install/setup.bash'))}"
)


def ros_humble_python() -> str:
    """Return the ROS interpreter without inheriting the active Conda Python."""
    if value := os.environ.get("ROS_PYTHON"):
        return value
    home = Path.home()
    candidates = (
        home / "miniconda3/envs/ros-humble/bin/python3",
        home / "anaconda3/envs/ros-humble/bin/python3",
        home / "miniforge3/envs/ros-humble/bin/python3",
        Path("/opt/conda/envs/ros-humble/bin/python3"),
        REPO / ".envs/ros-humble/bin/python3",
        REPO / ".envs/ros-humble/bin/python",
        Path("/usr/bin/python3"),
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return "/usr/bin/python3"


def ros_humble_setup() -> str:
    return os.environ.get("ROS_SETUP", DEFAULT_ROS_SETUP)


def ros_log_dir() -> Path:
    path = Path(os.environ.get("ROS_LOG_DIR", REPO / ".runtime/ros_log"))
    path.mkdir(parents=True, exist_ok=True)
    return path


def _ros_inner(command: str) -> str:
    return (
        f"export ROS_LOG_DIR={shlex.quote(str(ros_log_dir()))}"
        f" && {ros_humble_setup()} && exec {command}"
    )


def check_ros_humble() -> tuple[bool, str]:
    """Check Python ABI and rclpy in the exact environment used by children."""
    python = ros_humble_python()
    probe = (
        "import rclpy,sys; "
        "assert sys.version_info[:2] == (3,10), sys.version; "
        "print(f'python={sys.version_info.major}.{sys.version_info.minor} rclpy=ok')"
    )
    inner = _ros_inner(f"{shlex.quote(python)} -c {shlex.quote(probe)}")
    try:
        result = subprocess.run(
            ["bash", "-lc", inner],
            cwd=REPO,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, str(error)
    detail = result.stdout.strip() or f"exit={result.returncode}"
    return result.returncode == 0, detail


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", action="store_true", help="print selected interpreter")
    parser.add_argument("--setup", action="store_true", help="print selected ROS setup command")
    parser.add_argument("--check", action="store_true", help="verify Python 3.10 and rclpy")
    parser.add_argument(
        "--run", nargs=argparse.REMAINDER, metavar="ARG",
        help="replace this process with a Python script inside ROS Humble",
    )
    args = parser.parse_args()
    if args.setup:
        print(ros_humble_setup())
        return 0
    if args.check:
        ok, detail = check_ros_humble()
        print(detail, file=sys.stdout if ok else sys.stderr)
        return 0 if ok else 1
    if args.run is not None:
        if not args.run:
            parser.error("--run requires a script and optional arguments")
        command = " ".join(
            shlex.quote(value) for value in (ros_humble_python(), *args.run)
        )
        inner = _ros_inner(command)
        os.execvp("bash", ["bash", "-lc", inner])
    print(ros_humble_python())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
