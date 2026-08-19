"""Latest-only hand target scheduling with one serial command in flight."""
from __future__ import annotations

import asyncio
import inspect
import math
import time
from dataclasses import dataclass
from typing import Awaitable, Callable


@dataclass(frozen=True)
class HandTarget:
    owner: str
    frame_id: object
    angles: tuple[float, ...]
    created_at: float
    replaced: int = 0


@dataclass(frozen=True)
class SubmitResult:
    accepted: bool
    replaced: int = 0
    reason: str | None = None


class LatestTargetMailbox:
    """Run a latest-value control loop at a bounded rate.

    ``submit`` never waits for hardware. While one target is awaiting its ACK,
    at most one newer target is retained. A different owner cannot take over
    until the current owner releases the mailbox.
    """

    def __init__(
        self,
        sender: Callable[[HandTarget], Awaitable[dict]],
        *,
        rate_hz: float = 30.0,
        max_age_ms: float = 250.0,
        ack_timeout_ms: float = 100.0,
        angle_count: int = 6,
        reporter: Callable[[dict], object] | None = None,
    ) -> None:
        self._sender = sender
        self._interval = 1.0 / max(1.0, rate_hz)
        self._max_age = max_age_ms / 1000.0
        self._ack_timeout = ack_timeout_ms / 1000.0
        self._angle_count = max(1, int(angle_count))
        self._reporter = reporter
        self._owner: str | None = None
        self._pending: HandTarget | None = None
        self._event = asyncio.Event()
        self._worker: asyncio.Task | None = None
        self._closed = False
        self._in_flight = False
        self._next_send_at = 0.0

    @property
    def owner(self) -> str | None:
        return self._owner

    @property
    def pending_count(self) -> int:
        return int(self._pending is not None)

    @property
    def in_flight_count(self) -> int:
        return int(self._in_flight)

    def submit(
        self,
        owner: str,
        frame_id: object,
        angles: list[float] | tuple[float, ...],
        *,
        created_at: float | None = None,
    ) -> SubmitResult:
        if self._closed:
            return SubmitResult(False, reason="closed")
        if self._owner not in (None, owner):
            return SubmitResult(False, reason="owner_busy")
        if len(angles) != self._angle_count:
            return SubmitResult(False, reason="invalid_angles")
        try:
            normalized = tuple(float(value) for value in angles)
        except (TypeError, ValueError):
            return SubmitResult(False, reason="invalid_angles")
        if not all(math.isfinite(value) for value in normalized):
            return SubmitResult(False, reason="invalid_angles")

        self._owner = owner
        replaced = (self._pending.replaced + 1) if self._pending else 0
        self._pending = HandTarget(
            owner=owner,
            frame_id=frame_id,
            angles=normalized,
            created_at=time.monotonic() if created_at is None else created_at,
            replaced=replaced,
        )
        self._ensure_worker()
        self._event.set()
        return SubmitResult(True, replaced=replaced)

    def release(self, owner: str) -> bool:
        if self._owner != owner:
            return False
        self._owner = None
        self._pending = None
        self._event.clear()
        return True

    def reset(self) -> None:
        self._owner = None
        self._pending = None
        self._event.clear()

    async def close(self) -> None:
        self._closed = True
        self.reset()
        task = self._worker
        self._worker = None
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    def _ensure_worker(self) -> None:
        if self._worker is None or self._worker.done():
            self._worker = asyncio.create_task(self._run())

    async def _run(self) -> None:
        while not self._closed:
            await self._event.wait()
            delay = self._next_send_at - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)

            target = self._pending
            if target is None:
                self._event.clear()
                continue
            self._pending = None
            self._event.clear()
            if target.owner != self._owner:
                continue

            started_at = time.monotonic()
            wait_ms = (started_at - target.created_at) * 1000.0
            if wait_ms > self._max_age * 1000.0:
                await self._report(target, wait_ms, "stale", None)
                continue

            self._in_flight = True
            status = "ok"
            ack: dict | None = None
            try:
                ack = await asyncio.wait_for(
                    self._sender(target), timeout=self._ack_timeout
                )
                if not ack.get("ok", False):
                    status = "error"
            except asyncio.TimeoutError:
                status = "timeout"
            except asyncio.CancelledError:
                if self._closed:
                    raise
                status = "cancelled"
            except Exception as error:  # noqa: BLE001
                status = f"error:{type(error).__name__}"
            finally:
                self._in_flight = False
                self._next_send_at = started_at + self._interval

            await self._report(target, wait_ms, status, ack)
            if self._pending is not None:
                self._event.set()

    async def _report(
        self,
        target: HandTarget,
        wait_ms: float,
        status: str,
        ack: dict | None,
    ) -> None:
        if self._reporter is None:
            return
        result = self._reporter({
            "id": target.frame_id,
            "replaced": target.replaced,
            "wait_ms": round(wait_ms, 2),
            "age_ms": round((time.monotonic() - target.created_at) * 1000.0, 2),
            "status": status,
            "ack": ack,
        })
        if inspect.isawaitable(result):
            await result
