#!/usr/bin/env python3
from __future__ import annotations

import unittest

from hand_target_filter import OneEuroJointFilter


class OneEuroJointFilterTest(unittest.TestCase):
    def test_first_target_passes_through_and_resets(self) -> None:
        filt = OneEuroJointFilter()
        result = filt.update([0.1] * 6, 1.0)
        self.assertTrue(result.changed)
        self.assertTrue(result.reset)
        self.assertEqual(result.angles, (0.1,) * 6)

    def test_deadband_suppresses_stationary_noise(self) -> None:
        filt = OneEuroJointFilter()
        filt.update([0.5] * 6, 1.0)
        result = filt.update([0.5001] * 6, 1.0 + 1.0 / 30.0)
        self.assertFalse(result.changed)
        self.assertEqual(result.suppressed, 6)
        self.assertEqual(result.angles, (0.5,) * 6)

    def test_sustained_motion_eventually_crosses_deadband(self) -> None:
        filt = OneEuroJointFilter()
        filt.update([0.0] * 6, 1.0)
        changed = False
        result = None
        for frame in range(1, 20):
            result = filt.update([frame * 0.01] * 6, 1.0 + frame / 30.0)
            changed = changed or result.changed
        self.assertTrue(changed)
        self.assertIsNotNone(result)
        self.assertGreater(result.angles[0], 0.1)

    def test_large_motion_is_smoothed_without_being_dropped(self) -> None:
        filt = OneEuroJointFilter(deadband_rad=(0.0,) * 6)
        filt.update([0.0] * 6, 1.0)
        result = filt.update([1.0] * 6, 1.0 + 1.0 / 30.0)
        self.assertTrue(result.changed)
        self.assertGreater(result.angles[0], 0.0)
        self.assertLess(result.angles[0], 1.0)

    def test_default_filter_keeps_up_with_fast_ramp(self) -> None:
        filt = OneEuroJointFilter()
        filt.update([0.0] * 6, 0.0)
        result = None
        for frame in range(1, 9):
            value = frame / 15.0
            result = filt.update([value] * 6, frame / 30.0)
        self.assertIsNotNone(result)
        self.assertLess(value - result.angles[0], 0.075)

    def test_settling_tail_does_not_release_visible_deadband_steps(self) -> None:
        filt = OneEuroJointFilter()
        filt.update([0.0] * 6, 0.0)
        filt.update([0.2] * 6, 1.0 / 30.0)
        previous = None
        tail_deltas = []
        for frame in range(2, 32):
            result = filt.update([0.2] * 6, frame / 30.0)
            if previous is not None and frame >= 10:
                tail_deltas.append(abs(result.angles[0] - previous))
            previous = result.angles[0]
        self.assertLessEqual(max(tail_deltas), 0.002)
        self.assertLess(abs(0.2 - previous), 0.002)

    def test_gap_reinitializes_without_old_pose_lag(self) -> None:
        filt = OneEuroJointFilter(reset_after_ms=200)
        filt.update([0.0] * 6, 1.0)
        result = filt.update([1.0] * 6, 1.25)
        self.assertTrue(result.reset)
        self.assertEqual(result.angles, (1.0,) * 6)

    def test_rejects_invalid_input(self) -> None:
        filt = OneEuroJointFilter()
        with self.assertRaises(ValueError):
            filt.update([0.0] * 5, 1.0)
        with self.assertRaises(ValueError):
            filt.update([0.0] * 5 + [float("nan")], 1.0)


if __name__ == "__main__":
    unittest.main()
