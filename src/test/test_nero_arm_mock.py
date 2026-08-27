from __future__ import annotations

import math

from nero_arm import (NERO_ARM_LIMITS, NERO_HOME_POSE,
                      NERO_TRACKING_READY_POSE, NeroArm)


def test_mock_starts_bent_and_away_from_joint_limits():
    arm = NeroArm(mock=True)
    assert arm.connected is False
    arm.connect()

    assert arm.connected is True
    assert arm.last_read_ok is True
    assert arm.target == NERO_TRACKING_READY_POSE
    assert arm.target[3] >= math.radians(60.0)
    margins = [
        min(value - lower, upper - value)
        for value, (lower, upper) in zip(arm.target, NERO_ARM_LIMITS)
    ]
    assert min(margins) >= math.radians(40.0)


def test_mock_angles_do_not_move_without_a_command():
    arm = NeroArm(mock=True)
    arm.connect()

    samples = [arm.read_angles() for _ in range(20)]
    assert all(sample == NERO_TRACKING_READY_POSE for sample in samples)

    target = [0.1, -0.6, 0.2, 1.2, -0.1, 0.2, 0.1]
    assert arm.move_cpv_pos(target)
    assert arm.read_angles() == target
    assert arm.read_angles() == target


def test_home_pose_is_the_straight_zero_pose():
    assert NERO_HOME_POSE == [0.0] * 7


def test_mock_disconnect_invalidates_read_health():
    arm = NeroArm(mock=True)
    arm.connect()
    arm.disconnect()

    assert arm.connected is False
    assert arm.last_read_ok is False
    assert arm.enabled is False
    assert arm.read_angles() == NERO_TRACKING_READY_POSE
    assert arm.last_read_ok is False
