#!/usr/bin/env python3
"""Step 15: hand translation with exact R; adapter bores; adapter inertia."""
import numpy as np
import trimesh

from stl_probe import load_stl
from feat import sample_tris
from icp import fixed_r_icp
from cyl import cyl_axes, radii_at

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
FLG = f'{A}/nero_description/meshes/NERO+因时RH56DF_适配法兰.stl'

# ---- 1. hand base pose with rotation snapped to exact permutation ----------
R = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
_, th = load_stl(f'{A}/inspire_hand/meshes/visual/right_base_link.stl')
src = sample_tris(th * 1000.0, density=0.6, seed=7)
palm = np.load('_cache/asm_big.npz')['b1'].astype(np.float64)
t, rmse, md = fixed_r_icp(src, palm, R, [0.0, 5.95, -7.15], keep=0.6)
print(f'hand_base in adapter frame: t = {np.round(t, 4)} mm')
print(f'  fixed-R trimmed rmse={rmse:.4f} mm  mean_all={md:.4f} mm')

# ---- 2. adapter bores / bolt pattern (cylinders parallel to z) -------------
nf, tf = load_stl(FLG)
peaks, dat = cyl_axes(tf, nf, axis=2, smin=1.0, smax=14.0, npeak=10, nms=2.5)
print('\nadapter: cylinder axes parallel to z  (x, y) mm')
c, nn, w = dat
for p, v in peaks:
    h, e = radii_at(c, p, w, 0, 15, 60)
    i = int(np.argmax(h))
    print(f'   x={p[0]:+7.3f} y={p[1]:+7.3f} vote={v:8.1f}'
          f'  dominant r={0.5 * (e[i] + e[i + 1]):5.2f}')

# ---- 3. adapter mass properties by voxelisation ---------------------------
m = trimesh.load(FLG, force='mesh', process=False)
vg = m.voxelized(pitch=0.4).fill()
pts = vg.points
vol = len(pts) * 0.4 ** 3
com = pts.mean(0)
d = pts - com
Ixx = ((d[:, 1] ** 2 + d[:, 2] ** 2)).sum()
Iyy = ((d[:, 0] ** 2 + d[:, 2] ** 2)).sum()
Izz = ((d[:, 0] ** 2 + d[:, 1] ** 2)).sum()
Ixy = -(d[:, 0] * d[:, 1]).sum()
Ixz = -(d[:, 0] * d[:, 2]).sum()
Iyz = -(d[:, 1] * d[:, 2]).sum()
for rho in (2700.0, 7850.0):
    mass = rho * vol * 1e-9
    k = rho * (0.4 ** 3) * 1e-15
    print(f'\nadapter: vol={vol:.1f} mm^3  rho={rho:.0f} -> mass={mass:.4f} kg')
    print(f'  com = {np.round(com, 4)} mm')
    print(f'  ixx={Ixx * k:.9f} iyy={Iyy * k:.9f} izz={Izz * k:.9f}')
    print(f'  ixy={Ixy * k:.9f} ixz={Ixz * k:.9f} iyz={Iyz * k:.9f}')
