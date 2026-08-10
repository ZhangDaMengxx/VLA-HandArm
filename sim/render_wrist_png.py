"""按 URDF FK 渲染腕部装配(link8+法兰+手基座及手指)到 PNG,自查贴合/朝向。"""
import sys
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from paths import REPO, ASSEMBLY_URDF, GRIPPER_URDF
sys.path.insert(0, str(Path(__file__).resolve().parent))
import pinocchio as pin
from replay_rerun import RobotModel, load_meshes
OUT = REPO / "sim/out"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    m = RobotModel(ASSEMBLY_URDF)
    meshes = load_meshes(m)
    q = np.zeros(m.model.nq)
    pin.forwardKinematics(m.model, m.data, q)
    pin.updateGeometryPlacements(m.model, m.data, m.visual, m.gdata, q)
    P = [m.gdata.oMg[i].homogeneous.copy() for i in range(m.visual.ngeoms)]

    tris_all, cols = [], []
    for i, mesh in enumerate(meshes):
        if mesh is None:
            continue
        T = P[i]; V = (T[:3, :3] @ mesh["V"].T).T + T[:3, 3]
        name = m.visual.geometryObjects[i].name.lower()
        # 只画腕部附近(Z>0.70),避免整臂太小
        if V[:, 2].max() < 0.70:
            continue
        is_fl = "flange" in name or "adapter" in name
        tris_all.append((V[mesh["F"]], "#ff9628" if is_fl else "#9aa0aa", 0.9 if is_fl else 0.5))

    allV = np.vstack([t[0].reshape(-1, 3) for t in tris_all])
    lo, hi = allV.min(0), allV.max(0); ctr = (lo+hi)/2; rad = (hi-lo).max()/2*1.05
    for name, elev, azim in [("front", 5, -90), ("side", 5, 0), ("iso", 20, 40)]:
        fig = plt.figure(figsize=(6, 7)); ax = fig.add_subplot(111, projection="3d")
        for V, c, a in tris_all:
            ax.add_collection3d(Poly3DCollection(V, facecolor=c, edgecolor="none", alpha=a))
        ax.set_xlim(ctr[0]-rad, ctr[0]+rad); ax.set_ylim(ctr[1]-rad, ctr[1]+rad); ax.set_zlim(ctr[2]-rad, ctr[2]+rad)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z"); ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"wrist assembly {name}")
        p = OUT / f"wrist_{name}.png"; fig.savefig(p, dpi=95, bbox_inches="tight"); plt.close(fig)
        print("wrote", p)


if __name__ == "__main__":
    main()
