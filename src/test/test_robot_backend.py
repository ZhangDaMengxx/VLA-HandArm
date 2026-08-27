from __future__ import annotations

import sys
from pathlib import Path

import pytest


SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_backend import (  # noqa: E402
    BackendConfigError,
    backend_for_request,
    create_backend,
)


def test_ros_backend_uses_ros_worker_without_owning_hardware():
    backend = create_backend("ros2")
    spec = backend.worker_spec("arm", hz=20, speed=35)

    assert backend.name == "ros"
    assert backend.owns_hardware is False
    assert spec.requires_ros is True
    assert spec.argv[-2:] == ("--speed", "35")


def test_direct_backend_exposes_direct_only_hand_capabilities():
    backend = create_backend("direct")
    spec = backend.worker_spec("hand", hz=30, player_hz=200)

    assert backend.owns_hardware is True
    assert backend.capabilities.per_channel_force is True
    assert backend.capabilities.clear_error is True
    assert spec.argv[:3] == ("python3", "src/hand_console.py", "--no-mock")
    assert spec.requires_ros is False


def test_direct_backend_reuses_persistent_hardware_environment(monkeypatch):
    monkeypatch.setenv("NERO_HAND_PORT", "/dev/inspire_hand")
    monkeypatch.setenv("NERO_CAN_CHANNEL", "can-test")
    monkeypatch.setenv("NERO_FIRMWARE", "v111")
    backend = create_backend("direct")

    hand = backend.worker_spec("hand", hz=30).argv
    arm = backend.worker_spec("arm", hz=20, speed=15).argv

    assert hand[-2:] == ("--port", "/dev/inspire_hand")
    assert arm[arm.index("--channel") + 1] == "can-test"
    assert arm[arm.index("--firmware") + 1] == "v111"


def test_mock_request_overrides_configured_real_backend():
    backend = backend_for_request(mock=True, configured=create_backend("direct"))

    assert backend.name == "mock"
    assert backend.mock is True


def test_invalid_backend_fails_with_supported_choices():
    with pytest.raises(BackendConfigError, match="direct, mock, ros"):
        create_backend("serial-ish")
