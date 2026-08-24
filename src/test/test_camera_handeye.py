from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera.handeye import make_transform, solve_eye_in_hand, transform_to_dict


def _T(rotvec, xyz):
    R, _ = cv2.Rodrigues(np.asarray(rotvec, dtype=np.float64))
    return make_transform(R, xyz)


def _angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def test_eye_in_hand_recovers_camera_to_gripper_transform():
    # 固定未知量：相机 -> gripper；固定场景量：target -> base。
    T_g_c_true = _T([0.18, -0.11, 0.07], [0.045, -0.018, 0.082])
    T_b_t = _T([-0.08, 0.04, 0.12], [0.42, 0.03, 0.28])
    base_gripper = []
    camera_target = []
    for i in range(16):
        T_b_g = _T(
            [0.15 * np.sin(i * 0.7), 0.22 * np.cos(i * 0.43), -0.12 + 0.025 * i],
            [0.12 + 0.018 * i, -0.18 + 0.012 * (i % 5), 0.31 + 0.015 * np.sin(i)],
        )
        # T_b_g @ T_g_c @ T_c_t = T_b_t
        T_c_t = np.linalg.inv(T_b_g @ T_g_c_true) @ T_b_t
        base_gripper.append(T_b_g)
        camera_target.append(T_c_t)

    result = solve_eye_in_hand(base_gripper, camera_target)

    delta = np.linalg.inv(T_g_c_true) @ result.T_gripper_camera
    assert np.linalg.norm(delta[:3, 3]) < 1e-6
    assert _angle_deg(delta[:3, :3]) < 1e-5
    assert result.translation_rmse_m < 1e-8
    assert result.rotation_rmse_deg < 1e-5


def test_export_contract_is_xyzw_and_matrix_is_preserved():
    T = _T([0.0, 0.0, np.pi / 2], [0.1, 0.2, 0.3])
    value = transform_to_dict(T)
    assert np.allclose(value["matrix"], T)
    assert np.allclose(value["translation_xyz_m"], [0.1, 0.2, 0.3])
    assert np.allclose(value["quaternion_xyzw"], [0, 0, np.sqrt(0.5), np.sqrt(0.5)])


def test_translation_only_samples_are_rejected_as_degenerate():
    base = [_T([0, 0, 0], [0.01 * i, 0, 0]) for i in range(5)]
    T_g_c = _T([0.1, -0.05, 0.02], [0.04, 0, 0.08])
    T_b_t = _T([0, 0, 0], [0.4, 0, 0.25])
    camera = [np.linalg.inv(T_b_g @ T_g_c) @ T_b_t for T_b_g in base]
    with pytest.raises(ValueError, match="旋转运动不足"):
        solve_eye_in_hand(base, camera)
