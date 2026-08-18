"""第二课实物:用雅可比把"末端想这么动"当场解成"每个关节动多少"。

这演示的是现代机器人控制的心脏那条链:
    把姿势冻住 → J 变常量 → 末端误差对 Δq 线性 → 平方成碗 → 凸问题 → 秒解

问题(每一帧都在解的那个):
    我想让末端产生一个位移 e(6维:平移+转动)。
    未知数是 Δq(7个关节各动多少)。
    关系(线性化): e ≈ J · Δq       ← J 是这一帧的常量矩阵,Δq 是变量
    目标: 让 ‖J·Δq − e‖² 最小(末端尽量到位)  +  λ‖Δq‖²(关节尽量少动)
         └────────── 二次碗,凸 ──────────┘     └── 就是"关节动最少",也顺手躲奇异 ──┘

这个带正则的最小二乘,有闭式解(不用装 QP 求解器):
    Δq = Jᵀ (J Jᵀ + λI)⁻¹ e
你 nero_kin.py 里 ik() 用的"阻尼最小二乘"就是这个 —— 那个公式本身就是这个 QP 的解。
(要加硬约束如关节限位,才需要真正的 QP 求解器 osqp,那就是 WBC/MPC 干的事。)
"""
import numpy as np
import pinocchio as pin
from pathlib import Path
from nero_kin import NeroKin

np.set_printoptions(precision=4, suppress=True, linewidth=140)
URDF = Path(__file__).resolve().parent.parent / "assets" / "nero_description" / "urdf" / "nero_description.urdf"
kin = NeroKin(URDF)
model = kin.model


def jac(q):
    """这一帧、这个姿势下的 6x7 雅可比(世界对齐)。"""
    pin.forwardKinematics(model, kin.data, q)
    pin.updateFramePlacements(model, kin.data)
    pin.computeJointJacobians(model, kin.data, q)
    return pin.getFrameJacobian(model, kin.data, kin.ee, pin.LOCAL_WORLD_ALIGNED)


def qp_step(q, e, lam=1e-4):
    """解一帧:给定末端想动 e(6维),返回关节该动 Δq。就是那条闭式解。"""
    J = jac(q)                                   # 常量矩阵
    dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
    return dq


# ---- 起始姿势(和 print_jacobian 一样,方便对照)----
q = np.array([0.3, -0.6, 0.4, -1.2, 0.5, 0.8, 0.0])
p0 = kin.fk(q)[:3, 3]
print("起始末端位置 (x,y,z) m:", p0)

# ---- 目标:末端往 +x 挪 3cm、往 +z 抬 2cm,姿态不变 ----
d_pos = np.array([0.03, 0.0, 0.02])       # 想要的平移
e = np.concatenate([d_pos, np.zeros(3)])  # 6维:后3维=0 表示姿态别动
print("想要的末端位移 (平移3 + 转动3):", e)

# ---- 解一帧 QP,看每个关节该动多少 ----
dq = qp_step(q, e)
print("\n解出的 Δq (每个关节该动多少 rad):", dq)
print("  → 妙处:我们要求'姿态不变'(e 后3维=0),但 J2/J4 挪位置时会把朝向带歪,")
print("    于是 QP 调只转不挪的 J7(+J5/J6)去抵消姿态漂移。6个目标同时统筹。")

q1 = np.clip(q + dq, kin.lo, kin.hi)
p1 = kin.fk(q1)[:3, 3]
print("\n动完之后末端实际位置:", p1)
print("实际位移:", p1 - p0, " 想要:", d_pos)
print("残差(实际 vs 想要) m:", np.linalg.norm((p1 - p0) - d_pos),
      "  ← 一步有点误差,因为线性化只在原姿势附近准")

# ---- 反复解(每步重算 J)=收敛到目标,这就是 nero_kin 里的 IK 循环 ----
print("\n反复解(每帧重算 J,逼近目标)—— 这就是 IK / MPC 的迭代:")
q_it = q.copy()
for i in range(6):
    p = kin.fk(q_it)[:3, 3]
    e_i = np.concatenate([(p0 + d_pos) - p, np.zeros(3)])   # 还差多少
    q_it = np.clip(q_it + qp_step(q_it, e_i), kin.lo, kin.hi)
    err = np.linalg.norm(kin.fk(q_it)[:3, 3] - (p0 + d_pos))
    print(f"  第{i+1}步  末端离目标还差 {err*1000:7.3f} mm")
