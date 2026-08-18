"""几何分析:适配法兰与 link8 的贴合面,判断单位变换下是否穿插、该如何对齐。

对每个网格:找主轴方向上的平坦端面(法向±轴、且顶点聚在同一坐标=贴合面),
报告端面位置、外径、是否有台阶。据此判断法兰哪个面贴 link8、哪个面接手。
"""
import sys
from pathlib import Path
import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[1]
MESHES = REPO / "assets/nero_description/meshes"


def flat_faces_along(mesh, axis, scale=1.0):
    """沿 axis(0/1/2)找平坦端面:法向与该轴近平行的三角面,按其轴向坐标聚类。"""
    V = mesh.vertices * scale
    fn = mesh.face_normals
    fc = V[mesh.faces].mean(axis=1)
    area = mesh.area_faces * (scale ** 2)
    out = []
    for sign in (+1, -1):
        sel = fn[:, axis] * sign > 0.94         # 法向近似 ±axis
        if not sel.any():
            continue
        coords = fc[sel, axis]
        a = area[sel]
        # 聚类:按 0.5mm 分箱找主端面
        lo, hi = coords.min(), coords.max()
        edges = np.arange(lo, hi + 0.0006, 0.0005)
        idx = np.clip(np.digitize(coords, edges), 1, len(edges) - 1)
        best_bin = max(set(idx), key=lambda b: a[idx == b].sum())
        face_c = float(np.average(coords[idx == best_bin], weights=a[idx == best_bin]))
        face_area = float(a[idx == best_bin].sum())
        out.append((sign, face_c, face_area))
    return out


def report(name, path, scale):
    m = trimesh.load(path, process=False)
    V = m.vertices * scale
    print(f"\n=== {name}  (scale={scale}) ===")
    print(f" bounds(m): min={np.round(V.min(0),4)}  max={np.round(V.max(0),4)}")
    ext = V.max(0) - V.min(0)
    axis = int(np.argmin(ext))            # 最短边通常是厚度=轴向
    print(f" extents(m): {np.round(ext,4)}  推测轴向=Z轴序号{axis}(最短边)")
    for ax in range(3):
        faces = flat_faces_along(m, ax, scale)
        if faces:
            fs = "  ".join(f"[{'+' if s>0 else '-'}法向 @ {c:+.4f}m 面积{ar*1e4:.1f}cm²]" for s, c, ar in faces)
            print(f"  轴{ax} 平坦端面: {fs}")


def main():
    report("适配法兰", MESHES / "NERO+因时RH56DF_适配法兰.stl", 0.001)
    report("Link8", MESHES / "Link8.STL", 1.0)
    print("\n判读:法兰两端面 = 贴 link8 的面 + 接手的面;link8 的输出端面应与法兰贴合面对齐。")
    print("若单位变换下法兰跨过 link8 原点(端面一正一负),说明穿插,需沿轴平移法兰厚度。")


if __name__ == "__main__":
    main()
