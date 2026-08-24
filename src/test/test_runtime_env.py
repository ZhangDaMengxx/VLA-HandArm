from __future__ import annotations

import os
import sys
import importlib.util
from pathlib import Path


SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

import ros_humble_env as runtime  # noqa: E402


def _load_v3_web_entrypoint():
    path = SRC / "lerobot_v3" / "app_web.py"
    spec = importlib.util.spec_from_file_location("lerobot_v3_app_web_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


def test_ros_python_override_is_preserved(monkeypatch):
    monkeypatch.setenv("ROS_PYTHON", "/tmp/custom-ros-python")
    assert runtime.ros_humble_python() == "/tmp/custom-ros-python"


def test_ros_setup_override_is_preserved(monkeypatch):
    monkeypatch.setenv("ROS_SETUP", "source /tmp/custom-setup.bash")
    assert runtime.ros_humble_setup() == "source /tmp/custom-setup.bash"


def test_default_ros_python_is_not_active_conda(monkeypatch):
    monkeypatch.delenv("ROS_PYTHON", raising=False)
    selected = Path(runtime.ros_humble_python())
    assert selected.name in {"python", "python3"}
    assert "lerobot-v3" not in str(selected)


def test_default_setup_names_humble_and_workspace(monkeypatch):
    monkeypatch.delenv("ROS_SETUP", raising=False)
    setup = runtime.ros_humble_setup()
    assert "/opt/ros/humble/setup.bash" in setup
    assert str(runtime.WORKSPACE / "install/setup.bash") in setup


def test_ros_log_dir_can_be_overridden(monkeypatch, tmp_path):
    target = tmp_path / "ros-log"
    monkeypatch.setenv("ROS_LOG_DIR", str(target))
    assert runtime.ros_log_dir() == target
    assert target.is_dir()


def test_v3_web_entrypoint_rejects_missing_websocket_transport(monkeypatch):
    web = _load_v3_web_entrypoint()
    real_find_spec = web.importlib.util.find_spec
    monkeypatch.setattr(
        web.importlib.util,
        "find_spec",
        lambda name: None if name == "websockets" else real_find_spec(name),
    )
    try:
        web.require_websocket_transport()
    except SystemExit as exc:
        assert "websockets" in str(exc)
    else:
        raise AssertionError("missing WebSocket transport must stop v3 Web startup")
