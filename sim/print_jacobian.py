"""看懂雅可比:把 NERO 在某个姿势下的 J 矩阵打成屏幕上的真数字。

雅可比 J 是一张 6x7 的表(6 行 = 末端的 6 维速度,7 列 = 7 个关节)。
它回答一个问题:每个关节转一点点,末端(link7)会怎么动?

    末端速度  v(6x1)  =  J(6x7)  ·  关节速度 q̇(7x1)

- 上 3 行:末端线速度 (vx, vy, vz),单位 m/s per rad/s
- 下 3 行:末端角速度 (wx, wy, wz),单位 rad/s per rad/s
这里用 LOCAL_WORLD_ALIGNED:方向按世界坐标轴,原点在末端。看起来最直观。
"""
import numpy as np
import pinocchio as pin
from pathlib import Path
from nero_kin import NeroKin

np.set_printoptions(precision=4, suppress=True, linewidth=140)

URDF = Path(__file__).resolve().parent.parent / "assets" / "nero_description" / "urdf" / "nero_description.urdf"

kin = NeroKin(URDF)
model, data = kin.model, kin.ee  # 只是为了下面少打字
model = kin.model

# ---- 1. 选一个姿势(7 个关节角,单位弧度)。不是全零,免得太特殊 ----
q = np.array([0.3, -0.6, 0.4, -1.2, 0.5, 0.8, 0.0])
print("关节角 q (rad):", q)

# ---- 2. 末端在哪(前向运动学)----
T = kin.fk(q)
print("\n末端 link7 的位置 (x,y,z) m:", T[:3, 3])

# ---- 3. 算这个姿势下的雅可比 ----
pin.forwardKinematics(model, kin.data, q)
pin.updateFramePlacements(model, kin.data)
pin.computeJointJacobians(model, kin.data, q)
J = pin.getFrameJacobian(model, kin.data, kin.ee, pin.LOCAL_WORLD_ALIGNED)  # 6x7

# ---- 4. 把这张表带标签打出来 ----
rows = ["vx (m/s)", "vy (m/s)", "vz (m/s)", "wx (rad/s)", "wy (rad/s)", "wz (rad/s)"]
cols = [f"J{i+1}" for i in range(model.nv)]
print("\n雅可比 J  (行=末端速度分量, 列=关节):")
print("            " + "".join(f"{c:>9}" for c in cols))
for name, r in zip(rows, J):
    print(f"{name:>11} " + "".join(f"{x:>9.4f}" for x in r))

# ---- 5. 一列的含义:只转 J4,末端会朝哪动? ----
j = 3  # 第 4 个关节(0 基)
print(f"\n只让第 {j+1} 个关节以 1 rad/s 转,末端瞬时速度 = J 的第 {j+1} 列:")
print(f"  线速度 (vx,vy,vz) = {J[:3, j]}  |{np.linalg.norm(J[:3, j]):.4f}| m/s")
print(f"  角速度 (wx,wy,wz) = {J[3:, j]}  |{np.linalg.norm(J[3:, j]):.4f}| rad/s")

# ---- 6. 验证:J 真的等于"位置对角度的导数"吗?用有限差分对一下 ----
print("\n验证(有限差分 vs 雅可比线速度部分,应几乎相等):")
eps = 1e-6
p0 = kin.fk(q)[:3, 3]
for j in range(model.nv):
    dq = np.zeros(model.nv); dq[j] = eps
    p1 = kin.fk(q + dq)[:3, 3]
    fd = (p1 - p0) / eps            # 数值导数:末端位置随第 j 关节的变化率
    err = np.linalg.norm(fd - J[:3, j])
    print(f"  J{j+1}: 有限差分 {fd}  误差 {err:.2e}")
