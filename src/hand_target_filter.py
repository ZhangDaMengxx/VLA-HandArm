"""Low-latency adaptive filtering for live hand joint targets."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class FilterResult:
    angles: tuple[float, ...]
    changed: bool
    reset: bool
    suppressed: int
    raw_delta_rad: float
    filtered_delta_rad: float


class OneEuroJointFilter:
    """Apply a One Euro filter and per-joint deadband to six joint targets."""

    def __init__(
        self,
        *,
        min_cutoff_hz: float = 1.5,
        beta: float = 2.5,
        derivative_cutoff_hz: float = 1.0,
        deadband_rad: Sequence[float] = (0.0005,) * 6,
        reset_after_ms: float = 200.0,
    ) -> None:
        if min_cutoff_hz <= 0 or derivative_cutoff_hz <= 0:
            raise ValueError("cutoff frequencies must be positive")
        if beta < 0 or reset_after_ms <= 0:
            raise ValueError("beta must be non-negative and reset interval positive")
        if len(deadband_rad) != 6 or any(value < 0 for value in deadband_rad):
            raise ValueError("deadband_rad must contain six non-negative values")

        self._min_cutoff = float(min_cutoff_hz)
        self._beta = float(beta)
        self._derivative_cutoff = float(derivative_cutoff_hz)
        self._deadband = tuple(float(value) for value in deadband_rad)
        self._reset_after = reset_after_ms / 1000.0
        self.reset()

    def reset(self) -> None:
        self._timestamp: float | None = None
        self._raw: tuple[float, ...] | None = None
        self._filtered: tuple[float, ...] | None = None
        self._derivative: tuple[float, ...] | None = None
        self._emitted: tuple[float, ...] | None = None

    def update(self, angles: Sequence[float], timestamp: float) -> FilterResult:
        values = self._normalize(angles)
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        if self._timestamp is None:
            return self._initialize(values, timestamp)

        dt = timestamp - self._timestamp
        if dt <= 0 or dt > self._reset_after:
            return self._initialize(values, timestamp)

        assert self._raw is not None
        assert self._filtered is not None
        assert self._derivative is not None
        assert self._emitted is not None

        raw_delta = max(abs(value - previous) for value, previous in zip(values, self._raw))
        derivative_alpha = self._alpha(self._derivative_cutoff, dt)
        derivatives = []
        filtered = []
        for value, raw_previous, filtered_previous, derivative_previous in zip(
            values, self._raw, self._filtered, self._derivative
        ):
            derivative = (value - raw_previous) / dt
            derivative_hat = self._lowpass(derivative, derivative_previous, derivative_alpha)
            cutoff = self._min_cutoff + self._beta * abs(derivative_hat)
            value_hat = self._lowpass(value, filtered_previous, self._alpha(cutoff, dt))
            derivatives.append(derivative_hat)
            filtered.append(value_hat)

        emitted = list(self._emitted)
        suppressed = 0
        for index, (value, previous, threshold) in enumerate(
            zip(filtered, self._emitted, self._deadband)
        ):
            if abs(value - previous) < threshold:
                suppressed += 1
            else:
                emitted[index] = value

        emitted_tuple = tuple(emitted)
        filtered_delta = max(
            abs(value - previous) for value, previous in zip(emitted_tuple, self._emitted)
        )
        self._timestamp = timestamp
        self._raw = values
        self._filtered = tuple(filtered)
        self._derivative = tuple(derivatives)
        self._emitted = emitted_tuple
        return FilterResult(
            angles=emitted_tuple,
            changed=suppressed != 6,
            reset=False,
            suppressed=suppressed,
            raw_delta_rad=raw_delta,
            filtered_delta_rad=filtered_delta,
        )

    def _initialize(self, values: tuple[float, ...], timestamp: float) -> FilterResult:
        self._timestamp = timestamp
        self._raw = values
        self._filtered = values
        self._derivative = (0.0,) * 6
        self._emitted = values
        return FilterResult(
            angles=values,
            changed=True,
            reset=True,
            suppressed=0,
            raw_delta_rad=0.0,
            filtered_delta_rad=0.0,
        )

    @staticmethod
    def _normalize(angles: Sequence[float]) -> tuple[float, ...]:
        if len(angles) != 6:
            raise ValueError("angles must contain six values")
        values = tuple(float(value) for value in angles)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("angles must be finite")
        return values

    @staticmethod
    def _alpha(cutoff_hz: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        return 1.0 / (1.0 + tau / dt)

    @staticmethod
    def _lowpass(value: float, previous: float, alpha: float) -> float:
        return alpha * value + (1.0 - alpha) * previous
