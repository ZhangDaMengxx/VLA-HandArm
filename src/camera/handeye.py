"""眼在手上(eye-in-hand)标定的纯几何实现。

约定与本仓库一致：列向量、左乘、长度米；四元数导出为 ``xyzw``。
常见手眼 API 的输入命名容易混淆，本模块统一使用明确的
齐次矩阵命名：

``T_base_gripper``：所选刚性末端坐标中的点 -> 机械臂 base 坐标
``T_camera_target``：标定板坐标中的点 -> 相机坐标
``T_gripper_camera``：相机坐标中的点 -> 所选刚性末端坐标（求解结果）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


def _as_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError("变换必须是有限的 4x4 矩阵")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError("齐次变换最后一行必须为 [0,0,0,1]")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=2e-4) or np.linalg.det(R) < 0.0:
        raise ValueError("变换旋转部分不是合法的右手旋转矩阵")
    return T


def make_transform(R: np.ndarray, t: Iterable[float]) -> np.ndarray:
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(list(t), dtype=np.float64).reshape(-1)
    if R.shape != (3, 3) or t.shape != (3,):
        raise ValueError("R 必须为 3x3，t 必须为 3 维")
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return _as_transform(T)


def _rotation_error_deg(R: np.ndarray) -> float:
    c = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _rotation_log(R: np.ndarray) -> np.ndarray:
    """SO(3) 对数向量；标定运动应远离 180 度退化点。"""
    theta = np.arccos(float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)))
    if theta < 1e-10:
        return np.zeros(3, dtype=np.float64)
    if abs(np.sin(theta)) < 1e-7:
        raise ValueError("相对旋转接近 180 度，无法稳定求解；请更换一组标定姿态")
    return theta / (2.0 * np.sin(theta)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    )


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """Chordal/SVD 平均，足够用于标定一致性报告。"""
    U, _, Vt = np.linalg.svd(np.sum(rotations, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def _quat_xyzw(R: np.ndarray) -> list[float]:
    # 不依赖 scipy；这是标准旋转矩阵 -> xyzw 转换。
    tr = float(np.trace(R))
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qx, qy, qz, qw = 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
        elif i == 1:
            s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qx, qy, qz, qw = (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
        else:
            s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qx, qy, qz, qw = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q.tolist()


@dataclass
class HandEyeResult:
    T_gripper_camera: np.ndarray
    translation_rmse_m: float
    translation_max_m: float
    rotation_rmse_deg: float
    rotation_max_deg: float
    samples: int
    translation_errors_m: list[float]
    rotation_errors_deg: list[float]

    @property
    def ok(self) -> bool:
        return bool(np.isfinite(self.translation_rmse_m) and np.isfinite(self.rotation_rmse_deg))


def solve_eye_in_hand(
    T_base_gripper: Iterable[np.ndarray],
    T_camera_target: Iterable[np.ndarray],
) -> HandEyeResult:
    """求解 ``T_gripper_camera``，适用于固定标定板、相机安装在法兰上。

    至少需要 4 个姿态，实际建议 15--25 个，且要有明显的平移和旋转变化。
    """
    base = [_as_transform(T) for T in T_base_gripper]
    camera = [_as_transform(T) for T in T_camera_target]
    if len(base) != len(camera) or len(base) < 4:
        raise ValueError("手眼标定至少需要 4 组一一对应的姿态")
    # G_i X C_i = Y（固定 target），两组相消得到 A X = X B：
    # A = inv(G_j) G_i，B = C_j inv(C_i)。使用全部姿态对增强约束。
    motions: list[tuple[np.ndarray, np.ndarray]] = []
    alpha, beta = [], []
    for i in range(len(base)):
        for j in range(i + 1, len(base)):
            A = np.linalg.inv(base[j]) @ base[i]
            B = camera[j] @ np.linalg.inv(camera[i])
            a = _rotation_log(A[:3, :3])
            b = _rotation_log(B[:3, :3])
            if np.linalg.norm(a) < 1e-5 or np.linalg.norm(b) < 1e-5:
                continue
            motions.append((A, B))
            alpha.append(a)
            beta.append(b)
    if len(motions) < 3:
        raise ValueError("有效旋转运动不足；请从不同倾角观察棋盘格，不能只做平移")

    # Park-Martin/Wahba：log(R_A) = R_X log(R_B)。
    H = sum(np.outer(a, b) for a, b in zip(alpha, beta))
    U, singular, Vt = np.linalg.svd(H)
    if singular[1] < max(singular[0] * 1e-4, 1e-8):
        raise ValueError("旋转激励退化；至少需要绕两个不同方向明显转动相机")
    D = np.eye(3)
    D[2, 2] = np.linalg.det(U @ Vt)
    R_c2g = U @ D @ Vt

    # (R_A-I)t_X = R_X t_B-t_A，全部相对运动联合最小二乘。
    lhs, rhs = [], []
    for A, B in motions:
        lhs.append(A[:3, :3] - np.eye(3))
        rhs.append(R_c2g @ B[:3, 3] - A[:3, 3])
    L = np.vstack(lhs)
    if np.linalg.matrix_rank(L) < 3:
        raise ValueError("平移约束退化；请同时改变相机位置和朝向")
    t_c2g, *_ = np.linalg.lstsq(L, np.concatenate(rhs), rcond=None)
    T_g_c = make_transform(R_c2g, t_c2g)

    # 标定板在 base 中应该是固定的；用该不变量报告几何一致性。
    fixed_targets = [b @ T_g_c @ c for b, c in zip(base, camera)]
    t_mean = np.mean([T[:3, 3] for T in fixed_targets], axis=0)
    R_mean = _mean_rotation([T[:3, :3] for T in fixed_targets])
    t_err = [float(np.linalg.norm(T[:3, 3] - t_mean)) for T in fixed_targets]
    r_err = [_rotation_error_deg(R_mean.T @ T[:3, :3]) for T in fixed_targets]
    return HandEyeResult(T_g_c, float(np.sqrt(np.mean(np.square(t_err)))), max(t_err),
                         float(np.sqrt(np.mean(np.square(r_err)))), max(r_err), len(base),
                         t_err, r_err)


def transform_to_dict(T: np.ndarray) -> dict:
    T = _as_transform(T)
    return {
        "matrix": T.tolist(),
        "translation_xyz_m": T[:3, 3].tolist(),
        "quaternion_xyzw": _quat_xyzw(T[:3, :3]),
    }
