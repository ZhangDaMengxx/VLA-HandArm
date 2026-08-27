from __future__ import annotations

import time

from nero_arm import NERO_HOME_POSE
from ros_combo_playback import RosComboController


class FakeRosBackend:
    def __init__(self) -> None:
        self._positions = list(NERO_HOME_POSE)
        self._speed = 20
        self.state = {
            "state": "READY", "enabled": True, "frozen": False,
        }
        self.accepting = True
        self.triggers: list[str] = []
        self.tracking_targets: list[list[float]] = []
        self.reject_tracking = False

    @property
    def speed(self) -> int:
        return self._speed

    def device_state(self) -> dict:
        return dict(self.state)

    def driver_accepts_commands(self) -> bool:
        return self.accepting

    def positions(self) -> list[float]:
        return list(self._positions)

    def set_int(self, name: str, value: int) -> tuple[bool, str]:
        assert name == "arm_speed"
        self._speed = value
        return True, "ok"

    def set_positions(self, values: list[float]) -> tuple[bool, str]:
        self._positions = list(values)
        return True, "ok"

    def set_tracking_positions(self, values: list[float]) -> tuple[bool, str]:
        if self.reject_tracking:
            return False, "rejected"
        self.tracking_targets.append(list(values))
        self._positions = list(values)
        return True, "ok"

    def trigger(self, name: str) -> tuple[bool, str]:
        self.triggers.append(name)
        return True, "ok"


def _prepare(controller: RosComboController, *, token: str = "combo-test") -> dict:
    return controller.handle({
        "cmd": "combo_prepare", "token": token, "name": "test pack",
        "mode": "keyframe",
        "waypoints": [{"t_ns": 0, "rad": list(NERO_HOME_POSE)}],
    })


def _ready(controller: RosComboController, events: list[dict]) -> None:
    controller.tick()
    assert events[-1]["type"] == "combo_ready"


def test_prepare_start_plays_through_tracking_services() -> None:
    backend = FakeRosBackend()
    events: list[dict] = []
    controller = RosComboController(backend, events.append)

    prepared = _prepare(controller)
    assert prepared["type"] == "ack" and prepared["ok"] is True
    assert prepared["phase"] == "approaching"

    _ready(controller, events)
    assert backend.triggers == ["arm_tracking_begin"]

    started = controller.handle({
        "cmd": "combo_start", "token": "combo-test",
        "start_at": time.monotonic() + 0.001,
    })
    assert started["type"] == "ack" and started["ok"] is True
    time.sleep(0.003)
    controller.tick()

    assert backend.tracking_targets == [list(NERO_HOME_POSE)]
    assert backend.triggers[-1] == "arm_tracking_end"
    assert events[-1]["type"] == "combo_done"
    assert events[-1]["stopped"] is False
    assert controller.status() is None


def test_tracking_rejection_reports_failure_and_ends_cpv() -> None:
    backend = FakeRosBackend()
    events: list[dict] = []
    controller = RosComboController(backend, events.append)
    assert _prepare(controller)["ok"] is True
    _ready(controller, events)
    backend.reject_tracking = True
    assert controller.handle({
        "cmd": "combo_start", "token": "combo-test",
        "start_at": time.monotonic() + 0.001,
    })["ok"] is True

    time.sleep(0.003)
    controller.tick()

    assert events[-1]["type"] == "combo_failed"
    assert "拒绝轨迹帧" in events[-1]["msg"]
    assert backend.triggers[-1] == "arm_tracking_end"
    assert controller.status() is None


def test_driver_disable_during_ready_aborts_combo() -> None:
    backend = FakeRosBackend()
    events: list[dict] = []
    controller = RosComboController(backend, events.append)
    assert _prepare(controller)["ok"] is True
    _ready(controller, events)
    backend.state["enabled"] = False

    controller.tick()

    assert events[-1]["type"] == "combo_failed"
    assert "失能" in events[-1]["msg"]
    assert backend.triggers[-1] == "arm_tracking_end"


def test_stale_token_cannot_stop_new_combo() -> None:
    backend = FakeRosBackend()
    events: list[dict] = []
    controller = RosComboController(backend, events.append)
    assert _prepare(controller, token="current")["ok"] is True

    result = controller.handle({"cmd": "combo_stop", "token": "stale"})

    assert result["type"] == "error"
    assert "token 不匹配" in result["msg"]
    assert controller.status() is not None


def test_prepare_rejects_bad_waypoint_with_same_token() -> None:
    backend = FakeRosBackend()
    controller = RosComboController(backend, lambda _event: None)

    result = controller.handle({
        "cmd": "combo_prepare", "token": "bad-pack",
        "waypoints": [{"t_ns": 0, "rad": [0.0]}],
    })

    assert result["type"] == "error"
    assert result["token"] == "bad-pack"
    assert "需要 7 个角" in result["msg"]
