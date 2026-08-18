"""渲染 inspire 手基座网格(含 frame 原点与坐标轴)多视角 PNG,看清安装面与朝向约定。"""
import sys
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "src/out"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    h = trimesh.load(REPO / "assets/inspire_hand/meshes/visual/right_base_link.glb",
                     force="mesh", process=False)
    V = h.vertices
    lo, hi = V.min(0), V.max(0)
    ctr = (lo + hi) / 2; rad = (hi - lo).max() / 2 * 1.05
    L = 0.03
    axes_v = [([0, 0, 0], [L, 0, 0], "r"), ([0, 0, 0], [0, L, 0], "g"), ([0, 0, 0], [0, 0, L], "b")]

    for name, elev, azim in [("XY", 90, -90), ("XZ", 0, -90), ("YZ", 0, 0), ("iso", 22, 45)]:
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(V[h.faces], facecolor="#7fb0d0",
                                             edgecolor="none", alpha=0.6))
        for o, e, c in axes_v:
            ax.plot(*zip(o, e), color=c, linewidth=3)
        ax.set_xlim(ctr[0]-rad, ctr[0]+rad); ax.set_ylim(ctr[1]-rad, ctr[1]+rad)
        ax.set_zlim(ctr[2]-rad, ctr[2]+rad)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"inspire base (R=red X, G=Y, B=Z)  {name}")
        p = OUT / f"hand_base_{name}.png"
        fig.savefig(p, dpi=95, bbox_inches="tight"); plt.close(fig)
        print("wrote", p)


if __name__ == "__main__":
    main()
