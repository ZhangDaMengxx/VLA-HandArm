"""NERO 7-DoF FK/IK —— 纯 pinocchio,从 URDF 加载(默认 assets/nero)。
替代 pinocchio-kinematics-lite 的 NeroKinematics,使仓库不依赖那个第三方仓库。
IK 用阻尼最小二乘(DLS,frame 版,含 Jlog6 修正),和 pinocchio 官方 IK 例子一致。
"""
import numpy as np
import pinocchio as pin


class NeroKin:
    def __init__(self, urdf_path, ee_frame="link7"):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.ee = self.model.getFrameId(ee_frame)
        self.lo = np.asarray(self.model.lowerPositionLimit, dtype=float)
        self.hi = np.asarray(self.model.upperPositionLimit, dtype=float)

    def fk(self, q):
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return np.array(self.data.oMf[self.ee].homogeneous)   # 4x4

    def ik(self, T_target, q_init, iters=200, eps=1e-4, damp=1e-6, dt=1.0,
           q_rest=None, k_null=0.0):
        """阻尼最小二乘 frame IK。

        q_rest/k_null: 零空间正则。7-DoF 对 6-DoF 任务有 1 维冗余,不加约束时那个
        自由度会随噪声乱飘(joint5/7 被放大成大幅摆动)。给定 q_rest 后,在不扰动
        手腕位姿的零空间里把关节往 q_rest 拉(k_null>0),得到"达成目标所需的最小运动",
        出更干净的臂标签。k_null=0 时退化为原纯任务空间 IK(向后兼容)。
        """
        q = np.array(q_init, dtype=float)
        oMdes = pin.SE3(np.asarray(T_target, dtype=float))
        q_rest = None if q_rest is None else np.asarray(q_rest, dtype=float)
        I6 = np.eye(6)
        In = np.eye(self.model.nv)
        ok = False
        for _ in range(iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            iMd = self.data.oMf[self.ee].actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < eps:
                ok = True
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee)
            J = -pin.Jlog6(iMd.inverse()) @ J
            JJt = J @ J.T + damp * I6
            dq = -J.T @ np.linalg.solve(JJt, err)                 # 主任务:达成位姿
            if q_rest is not None and k_null != 0.0:
                Jpinv = J.T @ np.linalg.inv(JJt)                  # 阻尼伪逆 (nv x 6)
                null_proj = In - Jpinv @ J                        # 零空间投影
                dq += null_proj @ (k_null * (q_rest - q))         # 次任务:往 q_rest 靠
            q = pin.integrate(self.model, q, dq * dt)
            q = np.clip(q, self.lo, self.hi)
        return q, ok
