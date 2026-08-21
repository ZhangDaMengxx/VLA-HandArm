"""Latest-only asynchronous scheduling for live arm IK."""
from __future__ import annotations

import asyncio
import concurrent.futures
import inspect
import math
import time
from dataclasses import dataclass, field
from typing import Awaitable, Callable


@dataclass(frozen=True)
class IKTarget:
    owner: str
    session_generation: int
    frame_id: object
    anchor_revision: int
    target_pose: tuple[float, ...]
    created_at: float
    epoch: int
    replaced: int = 0
    context: dict = field(default_factory=dict)


@dataclass(frozen=True)
class IKSubmitResult:
    accepted: bool
    replaced: int = 0
    reason: str | None = None


class LatestIKScheduler:
    """Solve at most one IK target while retaining only the newest pending one.

    The synchronous solver runs on one owned thread. ``submit`` never waits for
    it. ``release`` invalidates both pending work and the result of any solve
    already in flight, so a closed session or superseded anchor cannot move the
    arm later.
    """

    def __init__(
        self,
        solver: Callable[[IKTarget], dict],
        on_result: Callable[[IKTarget, dict, dict], Awaitable[dict] | dict],
        *,
        max_input_age_ms: float = 200.0,
        max_result_age_ms: float = 250.0,
        reporter: Callable[[dict], Awaitable[object] | object] | None = None,
    ) -> None:
        self._solver = solver
        self._on_result = on_result
        self._max_input_age = max_input_age_ms / 1000.0
        self._max_result_age = max_result_age_ms / 1000.0
        self._reporter = reporter
        self._owner: str | None = None
        self._pending: IKTarget | None = None
        self._event = asyncio.Event()
        self._task: asyncio.Task | None = None
        self._executor: concurrent.futures.ThreadPoolExecutor | None = None
        self._in_flight = False
        self._closed = False
        self._epoch = 0
        self._latest_result: dict | None = None

    @property
    def owner(self) -> str | None:
        return self._owner

    @property
    def pending_count(self) -> int:
        return int(self._pending is not None)

    @property
    def in_flight_count(self) -> int:
        return int(self._in_flight)

    @property
    def latest_result(self) -> dict | None:
        return dict(self._latest_result) if self._latest_result is not None else None

    def submit(
        self,
        owner: str,
        session_generation: int,
        frame_id: object,
        anchor_revision: int,
        target_pose: list[float] | tuple[float, ...],
        *,
        created_at: float | None = None,
        context: dict | None = None,
    ) -> IKSubmitResult:
        if self._closed:
            return IKSubmitResult(False, reason="closed")
        if self._owner not in (None, owner):
            return IKSubmitResult(False, reason="owner_busy")
        if len(target_pose) != 16:
            return IKSubmitResult(False, reason="invalid_pose")
        try:
            pose = tuple(float(value) for value in target_pose)
        except (TypeError, ValueError):
            return IKSubmitResult(False, reason="invalid_pose")
        if not all(math.isfinite(value) for value in pose):
            return IKSubmitResult(False, reason="invalid_pose")

        self._owner = owner
        replaced = (self._pending.replaced + 1) if self._pending else 0
        self._pending = IKTarget(
            owner=owner,
            session_generation=int(session_generation),
            frame_id=frame_id,
            anchor_revision=int(anchor_revision),
            target_pose=pose,
            created_at=time.monotonic() if created_at is None else created_at,
            epoch=self._epoch,
            replaced=replaced,
            context=dict(context or {}),
        )
        self._ensure_worker()
        self._event.set()
        return IKSubmitResult(True, replaced=replaced)

    def release(self, owner: str) -> bool:
        if self._owner not in (None, owner):
            return False
        owned = self._owner == owner
        self._owner = None
        self._invalidate()
        return owned

    def reset(self) -> None:
        self._owner = None
        self._invalidate()

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._owner = None
        self._invalidate()
        self._event.set()
        if self._task is not None:
            task = self._task
            # Python 3.10 can lose the direct Task wake-up in the narrow race
            # where release() clears the event while the executor future is
            # completing. Poll only during shutdown; the live loop never polls.
            while not task.done():
                await asyncio.sleep(0.001)
            task.result()
            self._task = None
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._executor = None

    def _invalidate(self) -> None:
        self._epoch += 1
        self._pending = None
        self._latest_result = None
        self._event.clear()

    def _ensure_worker(self) -> None:
        if self._executor is None:
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="live-ik"
            )
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run())

    async def _run(self) -> None:
        assert self._executor is not None
        loop = asyncio.get_running_loop()
        while True:
            await self._event.wait()
            if self._closed:
                return
            target = self._pending
            self._pending = None
            self._event.clear()
            # close() may set the event between the first closed check and
            # clear(). Re-check here so that wake-up cannot be lost.
            if self._closed:
                return
            if target is None or not self._is_current(target):
                continue

            input_age = time.monotonic() - target.created_at
            if input_age > self._max_input_age:
                await self._report(target, "stale_input", input_age, 0.0)
                if self._pending is not None:
                    self._event.set()
                continue

            self._in_flight = True
            solve_started = time.monotonic()
            status = "ok"
            try:
                solved = await loop.run_in_executor(
                    self._executor, self._solver, target
                )
            except Exception as error:  # noqa: BLE001
                status = f"error:{type(error).__name__}"
                solved = {"ok": False, "ik_ok": False, "error": str(error)}
            finally:
                self._in_flight = False
            solve_ms = (time.monotonic() - solve_started) * 1000.0
            result_age = time.monotonic() - target.created_at

            if self._closed:
                return
            if not self._is_current(target):
                await self._report(target, "invalidated", result_age, solve_ms)
            elif result_age > self._max_result_age:
                await self._report(target, "stale_result", result_age, solve_ms)
            else:
                metrics = {
                    "source_frame_id": target.frame_id,
                    "replaced": target.replaced,
                    "input_age_ms": round(input_age * 1000.0, 2),
                    "age_ms": round(result_age * 1000.0, 2),
                    "ik_ms": round(solve_ms, 2),
                    "status": status,
                }
                handled = self._on_result(target, solved, metrics)
                if inspect.isawaitable(handled):
                    handled = await handled
                if self._is_current(target) and isinstance(handled, dict):
                    self._latest_result = dict(handled)
                await self._report(target, status, result_age, solve_ms)

            if self._pending is not None:
                self._event.set()
            if self._closed:
                return

    def _is_current(self, target: IKTarget) -> bool:
        return target.epoch == self._epoch and target.owner == self._owner

    async def _report(
        self, target: IKTarget, status: str, age_s: float, solve_ms: float
    ) -> None:
        if self._reporter is None:
            return
        report = self._reporter({
            "id": target.frame_id,
            "replaced": target.replaced,
            "age_ms": round(age_s * 1000.0, 2),
            "ik_ms": round(solve_ms, 2),
            "status": status,
        })
        if inspect.isawaitable(report):
            await report
