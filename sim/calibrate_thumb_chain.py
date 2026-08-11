"""用实测碰撞边界反解拇指链参数(第 3 步标定)

背景:URDF 自身不自洽(mimic 顶出从动关节限位 53%),且 thumb_2 上限有三个互斥值
(0.48 / 0.6 / 0.698)。无法从外部测量定 raw↔rad,转而从已有的碰撞边界反推。

旧表三个实测堵转点:(T=300, index_min=225), (450, 52), (600, 0)。
待定参数:thumb_2 有效 span(S2),thumb_3 mimic 系数(k3)。
约束:几何接触点 ≥ 实测堵转(判据一,保守方向);T=600 必须全程不碰。

用法:
    python calibrate_thumb_chain.py
    -> 输出最优 S2 / k3,及三点的拟合误差

如果三点无解,说明接触判据需要加深度阈值(浅触不算堵)。
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize

import collision_checker as C

# 实测堵转点(raw0=张开1000,raw0=闭合0;越小越紧)
MEASURED = [(300, 225), (450, 52), (600, 0)]


def raw_to_rad_custom(name: str, raw: float, span2: float) -> float:
    """可调 span 的 raw→rad。只改 thumb_2,其他用固定值。"""
    if name == "right_thumb_2_joint":
        return (1.0 - min(max(raw / 1000.0, 0.0), 1.0)) * span2
    return C.raw_to_rad(name, raw)


def raw6_to_rad6_custom(raw6, span2: float) -> np.ndarray:
    return np.array([raw_to_rad_custom(n, r, span2)
                     for n, r in zip(C.DRIVEN_JOINTS, raw6)])


def first_contact_raw(ck: C.ThumbIndexChecker, T: int, span2: float,
                      step: int = 20) -> int | None:
    """给定 (T, span2),从张开扫到闭合,返回首次接触的 index raw。"""
    for raw in range(1000, -1, -step):
        q = raw6_to_rad6_custom([T, T, raw, 1000, 1000, 1000], span2)
        r = ck.check(q, contact_detail=False)
        if not r.feasible:
            return raw
    return None


def objective(x, ck: C.ThumbIndexChecker) -> float:
    """x = [span2, k3]。损失 = sum(违反量²) + 惩罚(不满足约束)。"""
    span2, k3 = x
    if span2 < 0.4 or span2 > 0.75:
        return 1e9
    if k3 < 0.5 or k3 > 1.6:
        return 1e9

    # 覆盖 mimic 系数(thumb_3 = thumb_2 × k3;thumb_4 依旧从 thumb_3 派生)
    old_k3 = ck.model.joints["right_thumb_3_joint"].mimic_k
    ck.model.joints["right_thumb_3_joint"].mimic_k = k3
    try:
        loss = 0.0
        for T, meas in MEASURED:
            contact = first_contact_raw(ck, T, span2)
            if contact is None:
                if meas > 0:
                    loss += 1e6  # T=600 全程不碰 ✓;T=300/450 不碰 ✗ 危险
                continue
            gap = contact - meas
            if gap < 0:
                loss += 1e6 + gap * gap  # 几何更松 ✗ 危险
            else:
                loss += gap * gap / 1000.0  # 越保守越好,但过度保守浪费行程
        return loss
    finally:
        ck.model.joints["right_thumb_3_joint"].mimic_k = old_k3


def main():
    ck = C.ThumbIndexChecker()
    print("开始优化...初值:span2=0.6, k3=1.0")
    res = minimize(objective, x0=[0.6, 1.0], args=(ck,),
                   method="Nelder-Mead",
                   options={"maxiter": 200, "xatol": 0.005, "fatol": 100})
    if not res.success:
        print(f"⚠ 未收敛:{res.message}")
    span2, k3 = res.x
    print(f"\n最优参数:span2={span2:.4f} rad, k3={k3:.4f}")
    print(f"损失 {res.fun:.1f}\n")

    # 验收
    old = ck.model.joints["right_thumb_3_joint"].mimic_k
    ck.model.joints["right_thumb_3_joint"].mimic_k = k3
    try:
        print("拟合结果:")
        for T, meas in MEASURED:
            contact = first_contact_raw(ck, T, span2)
            if contact is None:
                shown, flag = "全程不碰", ("✓ T=600正确" if T == 600 else "✗ 危险")
            else:
                gap = contact - meas
                shown = f"{contact}"
                flag = "✓ 保守" if gap >= 0 else f"✗ 危险,几何更松 {gap}"
            print(f"  T={T:4d}  实测={meas:4d}  几何={shown:>8s}  差={gap if contact else 'N/A':>5}  {flag}")
    finally:
        ck.model.joints["right_thumb_3_joint"].mimic_k = old


if __name__ == "__main__":
    main()
