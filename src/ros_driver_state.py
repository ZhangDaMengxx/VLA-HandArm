"""Pure-Python connection state used by the ROS hardware driver.

This module deliberately has no ROS imports so reconnect and watchdog behavior can
be unit-tested without sourcing a ROS environment.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DeviceState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    READY = "READY"
    FAULT = "FAULT"


@dataclass
class DeviceHealth:
    name: str
    retry_initial_s: float = 1.0
    retry_max_s: float = 30.0
    read_failure_limit: int = 3
    state: DeviceState = DeviceState.DISCONNECTED
    last_error: str | None = None
    last_success_at: float | None = None
    next_retry_at: float = 0.0
    read_failures: int = 0
    _retry_delay_s: float = 0.0

    @property
    def ready(self) -> bool:
        return self.state is DeviceState.READY

    def retry_due(self, now: float) -> bool:
        return self.state is not DeviceState.READY and now >= self.next_retry_at

    def begin_connect(self, now: float) -> bool:
        if not self.retry_due(now):
            return False
        self.state = DeviceState.CONNECTING
        return True

    def mark_ready(self, now: float) -> None:
        self.state = DeviceState.READY
        self.last_error = None
        self.last_success_at = now
        self.next_retry_at = 0.0
        self.read_failures = 0
        self._retry_delay_s = 0.0

    def mark_read_success(self, now: float) -> None:
        self.last_success_at = now
        self.read_failures = 0
        self.last_error = None

    def mark_read_failure(self, error: str) -> bool:
        self.read_failures += 1
        self.last_error = error
        return self.read_failures >= max(1, self.read_failure_limit)

    def mark_fault(self, error: str, now: float) -> None:
        delay = self._retry_delay_s or max(0.0, self.retry_initial_s)
        self.state = DeviceState.FAULT
        self.last_error = error
        self.next_retry_at = now + delay
        self.read_failures = 0
        self._retry_delay_s = min(max(delay * 2.0, self.retry_initial_s),
                                  max(self.retry_initial_s, self.retry_max_s))

    def mark_disconnected(self, reason: str | None = None) -> None:
        self.state = DeviceState.DISCONNECTED
        self.last_error = reason
        self.next_retry_at = float("inf")
        self.read_failures = 0

    def snapshot(self, now: float) -> dict:
        retry_in = None
        if self.state is DeviceState.FAULT:
            retry_in = max(0.0, self.next_retry_at - now)
        age = None if self.last_success_at is None else max(0.0, now - self.last_success_at)
        return {
            "state": self.state.value,
            "ready": self.ready,
            "last_error": self.last_error,
            "last_success_age_s": None if age is None else round(age, 3),
            "retry_in_s": None if retry_in is None else round(retry_in, 3),
            "read_failures": self.read_failures,
        }
