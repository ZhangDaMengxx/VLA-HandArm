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

    def ik(self, T_target, q_init, iters=200, eps=1e-4, damp=1e-6, dt=1.0):
        q = np.array(q_init, dtype=float)
        oMdes = pin.SE3(np.asarray(T_target, dtype=float))
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
            dq = -J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
            q = pin.integrate(self.model, q, dq * dt)
            q = np.clip(q, self.lo, self.hi)
        return q, ok
