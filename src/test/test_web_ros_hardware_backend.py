from __future__ import annotations

import asyncio
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest


SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import app_web  # noqa: E402
from robot_backend import BackendBusyError, create_backend  # noqa: E402


class _DummyPipe:
    def write(self, _value: str) -> None:
        pass

    def flush(self) -> None:
        pass

    def __iter__(self):
        return iter(())


class _DummyProcess:
    def __init__(self, command) -> None:
        self.command = list(command)
        self.stdin = _DummyPipe()
        self.stdout = _DummyPipe()

    def poll(self):
        return None


class _DummyThread:
    def __init__(self, *args, **kwargs) -> None:
        pass

    def start(self) -> None:
        pass


def _capture_processes(monkeypatch):
    processes: list[_DummyProcess] = []

    def popen(command, **_kwargs):
        process = _DummyProcess(command)
        processes.append(process)
        return process

    monkeypatch.setattr(app_web.subprocess, "Popen", popen)
    monkeypatch.setattr(app_web.threading, "Thread", _DummyThread)
    monkeypatch.setattr(app_web, "_ros_cmd", lambda args: ["ROS", *args])
    return processes


def test_real_hand_session_uses_ros_worker(monkeypatch):
    processes = _capture_processes(monkeypatch)
    session = app_web.HandDebugSession(asyncio.new_event_loop())

    session.start(mock=False)

    assert processes[0].command[:4] == [
        "ROS", "src/ros_web_hardware.py", "--device", "hand"
    ]
    assert "src/hand_console.py" not in processes[0].command


def test_mock_hand_session_keeps_local_console(monkeypatch):
    processes = _capture_processes(monkeypatch)
    session = app_web.HandDebugSession(asyncio.new_event_loop())

    session.start(mock=True)

    assert processes[0].command[:3] == [
        "python3", "src/hand_console.py", "--mock"
    ]


def test_real_arm_session_uses_ros_worker(monkeypatch):
    processes = _capture_processes(monkeypatch)
    session = app_web.ArmDebugSession(asyncio.new_event_loop())

    session.start(mock=False, speed=15)

    assert processes[0].command[:4] == [
        "ROS", "src/ros_web_hardware.py", "--device", "arm"
    ]
    assert processes[0].command[-2:] == ["--speed", "15"]
    assert "src/arm_console.py" not in processes[0].command


def test_mock_arm_session_keeps_local_console(monkeypatch):
    processes = _capture_processes(monkeypatch)
    session = app_web.ArmDebugSession(asyncio.new_event_loop())

    session.start(mock=True, speed=15)

    assert processes[0].command[:3] == [
        "python3", "src/arm_console.py", "--mock"
    ]
    assert processes[0].command[-2:] == ["--speed", "15"]


def test_real_hand_session_can_use_direct_backend(monkeypatch):
    processes = _capture_processes(monkeypatch)
    monkeypatch.setattr(
        app_web, "CONFIGURED_HARDWARE_BACKEND", create_backend("direct"))
    session = app_web.HandDebugSession(asyncio.new_event_loop())

    session.start(mock=False)

    assert processes[0].command[:3] == [
        "python3", "src/hand_console.py", "--no-mock"
    ]
    assert session.backend.name == "direct"
    assert session.mock is False


def test_real_arm_session_can_use_direct_backend(monkeypatch):
    processes = _capture_processes(monkeypatch)
    monkeypatch.setattr(
        app_web, "CONFIGURED_HARDWARE_BACKEND", create_backend("direct"))
    session = app_web.ArmDebugSession(asyncio.new_event_loop())

    session.start(mock=False, speed=15)

    assert processes[0].command[:3] == [
        "python3", "src/arm_console.py", "--no-mock"
    ]
    assert processes[0].command[-2:] == ["--speed", "15"]
    assert session.backend.name == "direct"


def test_backend_change_requires_disconnected_session(monkeypatch):
    _capture_processes(monkeypatch)
    session = app_web.HandDebugSession(asyncio.new_event_loop())
    session.start(mock=False)

    with pytest.raises(BackendBusyError, match="请先断开"):
        session.start(mock=True)


def test_backend_status_reports_configuration_and_capabilities(monkeypatch):
    monkeypatch.setattr(app_web, "_arm", None)
    monkeypatch.setattr(app_web, "_hand", None)
    monkeypatch.setattr(
        app_web, "CONFIGURED_HARDWARE_BACKEND", create_backend("ros"))

    payload = app_web._hardware_backend_payload()

    assert payload["configured"]["name"] == "ros"
    assert payload["configured"]["capabilities"]["tracking"] is True
    assert payload["configured"]["capabilities"]["per_channel_force"] is False
    assert payload["active"] == {"arm": None, "hand": None}
    assert payload["switchable"] is True


def test_ros_worker_subscribes_driver_state_and_joint_states():
    source = (SRC / "ros_web_hardware.py").read_text(encoding="utf-8")

    assert '"/nero/driver_state"' in source
    assert '"/joint_states"' in source
    assert '"/nero/arm/set_joints"' in source
    assert '"/nero/arm/tracking_begin"' in source
    assert '"/nero/arm/set_tracking_joints"' in source
    assert '"/nero/arm/tracking_end"' in source
    assert '"/nero/hand/set_angles"' in source
    assert "ROS2 Driver 暂未提供 CPV 实时跟随接口" not in source
    assert "ROS2 Driver 暂未提供联合轨迹 Action 接口" not in source
    assert "InspireHand(" not in source
    assert "NeroArm(" not in source


def test_skill_state_uses_current_ros_arm_session(monkeypatch):
    process = _DummyProcess([])
    monkeypatch.setattr(app_web, "_live", None)
    monkeypatch.setattr(app_web, "_hand", None)
    monkeypatch.setattr(app_web, "_arm", SimpleNamespace(
        ready=True,
        error=None,
        console=process,
        latest={"enabled": True},
    ))

    assert app_web._skill_hardware_state() == (True, True)


def test_combo_prepare_waits_for_worker_ack():
    session = app_web.ArmDebugSession(asyncio.new_event_loop())
    session.console = _DummyProcess([])
    result: dict = {}

    def invoke() -> None:
        result.update(session.command_wait_combo_prepare(
            {"cmd": "combo_prepare", "token": "combo-ack"}, timeout=1.0))

    thread = threading.Thread(target=invoke)
    thread.start()
    while "combo-ack" not in session._combo_waiters:
        pass
    session._resolve_combo_ack({
        "type": "ack", "cmd": "combo_prepare", "token": "combo-ack",
        "ok": True,
    })
    thread.join(timeout=1.0)

    assert result["ok"] is True


def test_combo_prepare_propagates_worker_rejection():
    session = app_web.ArmDebugSession(asyncio.new_event_loop())
    session.console = _DummyProcess([])
    result: dict = {}

    def invoke() -> None:
        result.update(session.command_wait_combo_prepare(
            {"cmd": "combo_prepare", "token": "combo-error"}, timeout=1.0))

    thread = threading.Thread(target=invoke)
    thread.start()
    while "combo-error" not in session._combo_waiters:
        pass
    session._resolve_combo_ack({
        "type": "error", "cmd": "combo_prepare", "token": "combo-error",
        "msg": "driver rejected",
    })
    thread.join(timeout=1.0)

    assert result["ok"] is False
    assert result["msg"] == "driver rejected"
