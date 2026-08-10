#!/usr/bin/env python3
"""Step 10: resolve the 180deg (eps) ambiguity of link7 frame about tool axis."""
import numpy as np
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import T_from

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
Z7 = 42.542                                        # joint7 axis height [mm]
T_J7 = T_from([0, -23.5, 0], [1.5708, 0, 0])       # link6 -> link7 (th7=0)
T_J6 = T_from([0, 0, 0], [1.5708, -1.5708, 0])     # link5 -> link6 (th6=0)

asm = np.load('_cache/asm_v.npy').astype(np.float64) * 1000.0
reg = asm[(asm[:, 2] > 12) & (asm[:, 2] < 235)]
rng = np.random.default_rng(0)
reg = reg[rng.choice(len(reg), min(500000, len(reg)), replace=False)]
tree = cKDTree(reg)
print('assembly ref pts', len(reg), 'bbox',
      np.round(reg.min(0), 1), np.round(reg.max(0), 1))

meshes = {}
for nm, f in (('link5', 'Link5.STL'), ('link6', 'Link6.STL'),
              ('link7', 'Link7.STL')):
    _, t = load_stl(f'{M}/{f}')
    meshes[nm] = sample_tris(t * 1000.0, density=0.4, seed=3)

for eps in (+1, -1):
    R = np.array([[0, 0, eps], [0, eps, 0], [-1.0, 0, 0]])
    T7 = np.eye(4)
    T7[:3, :3] = R
    T7[:3, 3] = [23.5 * eps, 0, Z7]
    T6 = T7 @ np.linalg.inv(T_J7)
    T5 = T6 @ np.linalg.inv(T_J6)
    print(f'\n== eps = {eps:+d} ==')
    for nm, T in (('link5', T5), ('link6', T6), ('link7', T7)):
        p = (T[:3, :3] @ meshes[nm].T).T + T[:3, 3]
        d, _ = tree.query(p, workers=-1)
        print(f'  {nm}: bbox {np.round(p.min(0), 1)} .. {np.round(p.max(0), 1)}'
              f'  nn mean={d.mean():6.2f} med={np.median(d):6.2f}'
              f' q90={np.quantile(d, 0.9):6.2f}')
