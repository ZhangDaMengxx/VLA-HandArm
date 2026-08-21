#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import threading
import time
import unittest

from live_ik_scheduler import LatestIKScheduler


class LatestIKSchedulerTest(unittest.IsolatedAsyncioTestCase):
    async def asyncTearDown(self) -> None:
        scheduler = getattr(self, "scheduler", None)
        if scheduler:
            await scheduler.close()

    async def test_slow_ik_does_not_block_submit_and_latest_pending_wins(self) -> None:
        release_first = threading.Event()
        first_started = threading.Event()
        solved_frames = []
        completed_frames = []

        def solve(target):
            solved_frames.append(target.frame_id)
            if target.frame_id == 1:
                first_started.set()
                release_first.wait(1.0)
                time.sleep(0.1)
            return {"ok": True, "ik_ok": True, "q": [0.0] * 7}

        async def completed(target, solved, metrics):
            completed_frames.append(target.frame_id)
            return {"source_frame_id": target.frame_id, "ik_ok": solved["ik_ok"]}

        self.scheduler = LatestIKScheduler(solve, completed)
        started = time.perf_counter()
        self.assertTrue(self._submit(1).accepted)
        self.assertLess(time.perf_counter() - started, 0.02)
        await self._wait_until(first_started.is_set)
        self._submit(2)
        third = self._submit(3)
        self.assertEqual(third.replaced, 1)
        self.assertEqual(self.scheduler.in_flight_count, 1)
        self.assertEqual(self.scheduler.pending_count, 1)
        release_first.set()
        await self._wait_until(lambda: completed_frames == [1, 3])
        self.assertEqual(solved_frames, [1, 3])
        self.assertEqual(self.scheduler.latest_result["source_frame_id"], 3)

    async def test_release_invalidates_in_flight_result(self) -> None:
        release = threading.Event()
        started = threading.Event()
        completed_frames = []

        def solve(target):
            started.set()
            release.wait(1.0)
            return {"ok": True, "ik_ok": True, "q": [0.0] * 7}

        async def completed(target, solved, metrics):
            completed_frames.append(target.frame_id)
            return {"source_frame_id": target.frame_id}

        self.scheduler = LatestIKScheduler(solve, completed)
        self._submit(1)
        await self._wait_until(started.is_set)
        self.assertTrue(self.scheduler.release("camera"))
        release.set()
        await self._wait_until(lambda: self.scheduler.in_flight_count == 0)
        self.assertEqual(completed_frames, [])
        self.assertIsNone(self.scheduler.latest_result)

    async def test_stale_result_is_never_applied(self) -> None:
        completed_frames = []
        reports = []

        def solve(target):
            time.sleep(0.03)
            return {"ok": True, "ik_ok": True, "q": [0.0] * 7}

        async def completed(target, solved, metrics):
            completed_frames.append(target.frame_id)
            return {"source_frame_id": target.frame_id}

        self.scheduler = LatestIKScheduler(
            solve, completed, max_result_age_ms=5, reporter=reports.append
        )
        self._submit(1)
        await self._wait_until(lambda: bool(reports))
        self.assertEqual(reports[0]["status"], "stale_result")
        self.assertEqual(completed_frames, [])

    async def test_invalid_pose_does_not_claim_owner(self) -> None:
        self.scheduler = LatestIKScheduler(lambda target: {}, lambda *args: {})
        result = self.scheduler.submit("camera", 1, 1, 1, [0.0] * 15)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "invalid_pose")
        self.assertIsNone(self.scheduler.owner)

    async def test_close_waits_for_in_flight_solve_without_applying_it(self) -> None:
        release = threading.Event()
        started = threading.Event()
        completed_frames = []

        def solve(target):
            started.set()
            release.wait(1.0)
            return {"ok": True, "ik_ok": True, "q": [0.0] * 7}

        async def completed(target, solved, metrics):
            completed_frames.append(target.frame_id)
            return {"source_frame_id": target.frame_id}

        self.scheduler = LatestIKScheduler(solve, completed)
        self._submit(1)
        await self._wait_until(started.is_set)
        close_task = asyncio.create_task(self.scheduler.close())
        await asyncio.sleep(0.01)
        self.assertFalse(close_task.done())
        release.set()
        await asyncio.wait_for(close_task, 0.3)
        self.assertEqual(completed_frames, [])
        self.assertIsNone(self.scheduler.latest_result)

    async def test_release_then_close_during_in_flight_solve_is_bounded(self) -> None:
        release = threading.Event()
        started = threading.Event()
        completed_frames = []

        def solve(target):
            started.set()
            release.wait(1.0)
            return {"ok": True, "ik_ok": True, "q": [0.0] * 7}

        async def completed(target, solved, metrics):
            completed_frames.append(target.frame_id)
            return {"source_frame_id": target.frame_id}

        self.scheduler = LatestIKScheduler(solve, completed)
        self._submit(1)
        await self._wait_until(started.is_set)
        self.scheduler.release("camera")
        close_task = asyncio.create_task(self.scheduler.close())
        release.set()
        await asyncio.wait_for(close_task, 0.3)
        self.assertEqual(completed_frames, [])
        self.assertIsNone(self.scheduler.latest_result)

    def _submit(self, frame_id):
        return self.scheduler.submit(
            "camera", 7, frame_id, 3, [float(frame_id)] * 16,
            context={"authorization_revision": 2},
        )

    async def _wait_until(self, predicate, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            await asyncio.sleep(0.002)
        self.fail("condition was not reached before timeout")


if __name__ == "__main__":
    unittest.main()
