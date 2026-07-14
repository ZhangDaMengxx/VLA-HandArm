"""sim/wrist_stabilize.py — 手腕朝向稳定化(轻量版,单目)。

背景(实验坐实,2026-07-13):臂只由手腕朝向驱动,朝向相对首帧漂到 43°,其中 **91%
落在出平面(绕相机 X/Y,即手掌法向倾斜)**——正是单目深度歧义估不准的方向;面内(绕光轴 Z,
图像内滚转)只有几度,基本是真手势。滤波(SavGol/卡尔曼)治高频方差,治不了这种低频偏置。
故这里做两件针对性的事:

  1. gate_outliers  : 残差门限。帧间旋转增量超阈值(离群跳变,单帧手飞出去)则限幅到阈值,
                      避免一帧脏数据经后续平滑污染整段。
  2. attenuate_out_of_plane : 相对参考帧,把出平面(绕相机 X/Y)朝向分量按 alpha 衰减,
                      面内(绕光轴 Z)全保留。alpha=1 不衰减;alpha 越小,越贴参考帧的出平面朝向。

这是"各向异性可观测性加权"的轻量近似(固定/前缩调制的 alpha),不是完整 RTS/因子图——
后者等 Femto 深度(每像素置信通道 + 出平面歧义本身变小)进来再建。见项目记忆。

相机系约定:深度沿 +Z(estimate_wrist backproject 的 Z 为正深度),故光轴=Z 轴,
出平面=绕 X/Y。
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

OPTICAL_AXIS = np.array([0.0, 0.0, 1.0])   # 相机光轴(+Z=深度方向)


def gate_outliers(quats: np.ndarray, gate_deg: float) -> np.ndarray:
    """帧间旋转增量限幅。quats:(F,4) 已符号对齐的四元数。返回同形状。

    gate_deg<=0 时不处理。超过 gate 的单帧增量沿测地线限幅到 gate,后续帧以限幅后的姿态为基,
    这样偶发跳变不会带着后面一起飞,也不会被 SavGol 抹开成一片脏。
    """
    if gate_deg is None or gate_deg <= 0:
        return quats
    F = len(quats)
    gate = np.deg2rad(gate_deg)
    out = quats.copy()
    n_clamped = 0
    for i in range(1, F):
        r_prev = Rot.from_quat(out[i - 1])
        r_cur = Rot.from_quat(out[i])
        delta = (r_cur * r_prev.inv()).as_rotvec()   # 相机系下的增量旋转
        ang = np.linalg.norm(delta)
        if ang > gate:
            delta = delta * (gate / ang)             # 限幅到 gate
            out[i] = (Rot.from_rotvec(delta) * r_prev).as_quat()
            n_clamped += 1
    gate_outliers.last_clamped = n_clamped
    return out


def attenuate_out_of_plane(Rs: np.ndarray, alpha: float,
                           ref: int = 0, optical=OPTICAL_AXIS) -> np.ndarray:
    """相对参考帧衰减出平面朝向分量。Rs:(F,3,3) 手→相机旋转。返回 (F,3,3)。

    对每帧,取相对参考帧的旋转向量(相机系),沿光轴的分量=面内(保留),
    垂直光轴的分量=出平面(乘 alpha)。alpha=1 原样;alpha<1 压出平面。
    """
    if alpha is None or alpha >= 1.0:
        return Rs
    optical = np.asarray(optical, float)
    optical = optical / np.linalg.norm(optical)
    R_ref = Rs[ref]
    out = np.empty_like(Rs)
    for f in range(len(Rs)):
        r = Rot.from_matrix(Rs[f] @ R_ref.T).as_rotvec()   # 相对参考,相机系
        r_ip = (r @ optical) * optical                     # 面内(绕光轴)
        r_op = r - r_ip                                    # 出平面(绕 X/Y)
        r_new = r_ip + alpha * r_op
        out[f] = Rot.from_rotvec(r_new).as_matrix() @ R_ref
    return out
