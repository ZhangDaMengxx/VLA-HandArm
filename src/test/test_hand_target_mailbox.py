#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import time
import unittest

from hand_target_mailbox import LatestTargetMailbox


class LatestTargetMailboxTest(unittest.IsolatedAsyncioTestCase):
    async def asyncTearDown(self) -> None:
        mailbox = getattr(self, "mailbox", None)
        if mailbox:
            await mailbox.close()

    async def test_latest_target_wins_and_only_one_is_in_flight(self) -> None:
        release_first = asyncio.Event()
        first_started = asyncio.Event()
        sent = []
        active = 0
        max_active = 0

        async def sender(target):
            nonlocal active, max_active
            active += 1
            max_active = max(max_active, active)
            sent.append(target.frame_id)
            if target.frame_id == 1:
                first_started.set()
                await release_first.wait()
            active -= 1
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(sender, rate_hz=1000)
        self.mailbox.submit("camera-a", 1, [0] * 6)
        await asyncio.wait_for(first_started.wait(), 0.2)
        self.mailbox.submit("camera-a", 2, [0.2] * 6)
        result = self.mailbox.submit("camera-a", 3, [0.3] * 6)
        self.assertEqual(result.replaced, 1)
        self.assertEqual(self.mailbox.pending_count, 1)
        self.assertEqual(self.mailbox.in_flight_count, 1)
        release_first.set()
        await self._wait_until(lambda: sent == [1, 3])
        self.assertEqual(max_active, 1)

    async def test_owner_must_release_before_takeover(self) -> None:
        async def sender(target):
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(sender, rate_hz=1000)
        self.assertTrue(self.mailbox.submit("camera-a", 1, [0] * 6).accepted)
        denied = self.mailbox.submit("camera-b", 2, [0] * 6)
        self.assertFalse(denied.accepted)
        self.assertEqual(denied.reason, "owner_busy")
        self.assertTrue(self.mailbox.release("camera-a"))
        self.assertTrue(self.mailbox.submit("camera-b", 3, [0] * 6).accepted)

    async def test_invalid_target_is_rejected_without_claiming_owner(self) -> None:
        async def sender(target):
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(sender)
        result = self.mailbox.submit("camera-a", 1, [0, 0, 0, 0, 0, float("nan")])
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "invalid_angles")
        self.assertIsNone(self.mailbox.owner)

    async def test_release_discards_pending_target(self) -> None:
        sent = []
        blocker = asyncio.Event()
        first_started = asyncio.Event()

        async def sender(target):
            sent.append(target.frame_id)
            first_started.set()
            await blocker.wait()
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(sender, rate_hz=1000)
        self.mailbox.submit("camera-a", 1, [0] * 6)
        await asyncio.wait_for(first_started.wait(), 0.2)
        self.mailbox.submit("camera-a", 2, [0] * 6)
        self.mailbox.release("camera-a")
        blocker.set()
        await asyncio.sleep(0.02)
        self.assertEqual(sent, [1])

    async def test_stale_target_is_dropped(self) -> None:
        sent = []
        reports = []

        async def sender(target):
            sent.append(target.frame_id)
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(
            sender, rate_hz=1000, max_age_ms=5, reporter=reports.append
        )
        self.mailbox.submit(
            "camera-a", 1, [0] * 6, created_at=time.monotonic() - 1
        )
        await self._wait_until(lambda: bool(reports))
        self.assertEqual(sent, [])
        self.assertEqual(reports[0]["status"], "stale")

    async def test_configurable_angle_count_supports_arm_targets(self) -> None:
        sent = []

        async def sender(target):
            sent.append(target.angles)
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(sender, rate_hz=1000, angle_count=7)
        rejected = self.mailbox.submit("arm", 1, [0.0] * 6)
        accepted = self.mailbox.submit("arm", 2, [0.1] * 7)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "invalid_angles")
        self.assertTrue(accepted.accepted)
        await self._wait_until(lambda: len(sent) == 1)
        self.assertEqual(len(sent[0]), 7)

    async def test_ack_timeout_does_not_block_newest_target(self) -> None:
        sent = []
        reports = []

        async def sender(target):
            sent.append(target.frame_id)
            if target.frame_id == 1:
                await asyncio.Event().wait()
            return {"ok": True}

        self.mailbox = LatestTargetMailbox(
            sender,
            rate_hz=1000,
            ack_timeout_ms=5,
            reporter=reports.append,
        )
        self.mailbox.submit("camera-a", 1, [0] * 6)
        await self._wait_until(lambda: sent == [1])
        self.mailbox.submit("camera-a", 2, [0.2] * 6)
        await self._wait_until(lambda: sent == [1, 2])
        await self._wait_until(lambda: len(reports) == 2)
        self.assertEqual([row["status"] for row in reports], ["timeout", "ok"])

    async def _wait_until(self, predicate, timeout=0.3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            await asyncio.sleep(0.002)
        self.fail("condition was not reached before timeout")


if __name__ == "__main__":
    unittest.main()
