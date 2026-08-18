"""把 link8 + 适配法兰按当前装配变换渲染成多视角 PNG,肉眼核对贴合面。

link8 在其局部系;法兰按 link8_to_flange 变换(默认 +Z 抬 15mm)放置。
输出 src/out/flange_fit_*.png,三个正交视角 + 一个斜视。
"""
import sys
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

REPO = Path(__file__).resolve().parents[1]
M = REPO / "assets/nero_description/meshes"
OUT = REPO / "src/out"


def load(path, scale=1.0, T=None):
    m = trimesh.load(path, force="mesh", process=False)
    V = m.vertices * scale
    if T is not None:
        V = (T[:3, :3] @ V.T).T + T[:3, 3]
    return V, m.faces


def add(ax, V, F, color, alpha):
    tris = V[F]
    ax.add_collection3d(Poly3DCollection(tris, facecolor=color, edgecolor="none", alpha=alpha))


def main():
    flange_dz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.015
    OUT.mkdir(parents=True, exist_ok=True)
    v8, f8 = load(M / "Link8.STL")
    T = np.eye(4); T[2, 3] = flange_dz
    vf, ff = load(M / "NERO+因时RH56DF_适配法兰.stl", scale=0.001, T=T)

    allV = np.vstack([v8, vf])
    lo, hi = allV.min(0), allV.max(0)
    ctr = (lo + hi) / 2
    rad = (hi - lo).max() / 2 * 1.1

    views = [("XZ_front", 0, 0), ("YZ_side", 0, 90), ("XY_top", 90, -90), ("iso", 22, 35)]
    for name, elev, azim in views:
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111, projection="3d")
        add(ax, v8, f8, "#8a8f99", 0.55)
        add(ax, vf, ff, "#ff9628", 0.85)
        ax.set_xlim(ctr[0]-rad, ctr[0]+rad)
        ax.set_ylim(ctr[1]-rad, ctr[1]+rad)
        ax.set_zlim(ctr[2]-rad, ctr[2]+rad)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"link8(gray)+flange(orange) dz={flange_dz*1000:.0f}mm  {name}")
        p = OUT / f"flange_fit_{name}.png"
        fig.savefig(p, dpi=95, bbox_inches="tight"); plt.close(fig)
        print("wrote", p)


if __name__ == "__main__":
    main()
