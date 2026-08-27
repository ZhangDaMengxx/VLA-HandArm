#!/usr/bin/env python3
"""Backend selection for Web hardware sessions.

The Web process keeps one JSON-lines protocol for arm and hand sessions.  A
backend only decides which worker implements that protocol; skills, combo
timing, video tracking, WebSockets, and HTTP handlers stay backend-neutral.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
import os
from typing import Callable, Literal, Sequence


Device = Literal["arm", "hand"]
RosCommand = Callable[[list[str]], list[str]]


class BackendConfigError(ValueError):
    """Raised when the configured backend name is invalid."""


class BackendBusyError(RuntimeError):
    """Raised when a live session is asked to change backend."""


@dataclass(frozen=True)
class BackendCapabilities:
    tracking: bool
    combo: bool
    per_channel_force: bool
    clear_error: bool

    def as_dict(self) -> dict[str, bool]:
        return asdict(self)


@dataclass(frozen=True)
class WorkerSpec:
    argv: tuple[str, ...]
    requires_ros: bool = False

    def command(self, ros_command: RosCommand) -> list[str]:
        argv = list(self.argv)
        return ros_command(argv) if self.requires_ros else argv


class RobotBackend(ABC):
    """Transport boundary used by the Web arm and hand sessions."""

    name: str
    label: str
    mock: bool
    owns_hardware: bool
    capabilities: BackendCapabilities

    @abstractmethod
    def worker_spec(
        self,
        device: Device,
        *,
        hz: float,
        speed: int | None = None,
        player_hz: float | None = None,
    ) -> WorkerSpec:
        """Return a worker that implements the shared JSON-lines protocol."""

    def public_info(self) -> dict[str, object]:
        return {
            "name": self.name,
            "label": self.label,
            "mock": self.mock,
            "owns_hardware": self.owns_hardware,
            "capabilities": self.capabilities.as_dict(),
        }


class Ros2Backend(RobotBackend):
    name = "ros"
    label = "ROS2"
    mock = False
    owns_hardware = False
    capabilities = BackendCapabilities(
        tracking=True,
        combo=True,
        per_channel_force=False,
        clear_error=False,
    )

    def worker_spec(
        self,
        device: Device,
        *,
        hz: float,
        speed: int | None = None,
        player_hz: float | None = None,
    ) -> WorkerSpec:
        argv = ["src/ros_web_hardware.py", "--device", device, "--hz", str(hz)]
        if device == "arm":
            argv += ["--speed", str(20 if speed is None else int(speed))]
        return WorkerSpec(tuple(argv), requires_ros=True)


class DirectBackend(RobotBackend):
    name = "direct"
    label = "Direct"
    mock = False
    owns_hardware = True
    capabilities = BackendCapabilities(
        tracking=True,
        combo=True,
        per_channel_force=True,
        clear_error=True,
    )

    def worker_spec(
        self,
        device: Device,
        *,
        hz: float,
        speed: int | None = None,
        player_hz: float | None = None,
    ) -> WorkerSpec:
        python = os.environ.get("WEB_HARDWARE_PYTHON", "python3")
        if device == "hand":
            argv = [python, "src/hand_console.py", "--no-mock", "--hz", str(hz)]
            hand_port = (os.environ.get("WEB_HAND_PORT")
                         or os.environ.get("NERO_HAND_PORT")
                         or os.environ.get("INSPIRE_HAND_PORT"))
            if hand_port:
                argv += ["--port", hand_port]
            if player_hz is not None:
                argv += ["--player-hz", str(player_hz)]
        else:
            argv = [python, "src/arm_console.py", "--no-mock", "--hz", str(hz),
                    "--channel", os.environ.get(
                        "WEB_CAN_CHANNEL", os.environ.get("NERO_CAN_CHANNEL", "can0")),
                    "--firmware", os.environ.get(
                        "WEB_ARM_FIRMWARE", os.environ.get("NERO_FIRMWARE", "auto")),
                    "--speed", str(20 if speed is None else int(speed))]
        return WorkerSpec(tuple(argv))


class MockBackend(RobotBackend):
    name = "mock"
    label = "Mock"
    mock = True
    owns_hardware = False
    capabilities = BackendCapabilities(
        tracking=True,
        combo=True,
        per_channel_force=True,
        clear_error=True,
    )

    def worker_spec(
        self,
        device: Device,
        *,
        hz: float,
        speed: int | None = None,
        player_hz: float | None = None,
    ) -> WorkerSpec:
        python = os.environ.get("WEB_HARDWARE_PYTHON", "python3")
        if device == "hand":
            argv = [python, "src/hand_console.py", "--mock", "--hz", str(hz)]
            if player_hz is not None:
                argv += ["--player-hz", str(player_hz)]
        else:
            argv = [python, "src/arm_console.py", "--mock", "--hz", str(hz),
                    "--speed", str(20 if speed is None else int(speed))]
        return WorkerSpec(tuple(argv))


_BACKENDS: dict[str, type[RobotBackend]] = {
    "ros": Ros2Backend,
    "direct": DirectBackend,
    "mock": MockBackend,
}
_ALIASES = {"ros2": "ros"}


def create_backend(name: str | None = None) -> RobotBackend:
    raw = name if name is not None else os.environ.get("WEB_HARDWARE_BACKEND", "ros")
    normalized = _ALIASES.get(str(raw).strip().lower(), str(raw).strip().lower())
    try:
        return _BACKENDS[normalized]()
    except KeyError as exc:
        choices = ", ".join(sorted(_BACKENDS))
        raise BackendConfigError(
            f"WEB_HARDWARE_BACKEND must be one of {choices}; got {raw!r}"
        ) from exc


def backend_for_request(
    *, mock: bool, configured: RobotBackend
) -> RobotBackend:
    """Keep the existing mock flag while abstracting the real transport."""
    if mock or configured.mock:
        return MockBackend()
    return configured


def available_backends() -> Sequence[str]:
    return tuple(sorted(_BACKENDS))
