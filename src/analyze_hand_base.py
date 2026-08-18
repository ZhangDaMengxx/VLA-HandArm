"""找 inspire 手基座的安装面(最大平坦面)及其中心,用于定 flange_to_hand 变换。"""
import trimesh, numpy as np
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
h = trimesh.load(REPO / "assets/inspire_hand/meshes/visual/right_base_link.glb",
                 force="mesh", process=False)
V = h.vertices; fn = h.face_normals; fc = V[h.faces].mean(1); ar = h.area_faces
print("bounds =", np.round(h.bounds, 4))
print("extents =", np.round(h.extents, 4))
print("\n各轴向主平坦面(候选安装面):")
for ax in range(3):
    for sign in (+1, -1):
        sel = fn[:, ax] * sign > 0.9
        if not sel.any():
            continue
        c = fc[sel, ax]; a = ar[sel]
        edges = np.arange(c.min(), c.max() + 0.001, 0.001)
        idx = np.clip(np.digitize(c, edges), 1, len(edges) - 1)
        bb = max(set(idx), key=lambda b: a[idx == b].sum())
        pos = float(np.average(c[idx == bb], weights=a[idx == bb]))
        area = float(a[idx == bb].sum())
        if area > 3e-4:
            ctr = fc[sel][np.abs(c - pos) < 0.002].mean(0)
            s = "+" if sign > 0 else "-"
            print(f"  axis{ax}{s}  face@{pos:+.4f}m  area={area*1e4:5.1f}cm2  center={np.round(ctr,4)}")
print("\n判读:面积最大且与手指伸展方向垂直的那个面=贴法兰的安装面;其 center 的另两轴=圆心偏移。")
