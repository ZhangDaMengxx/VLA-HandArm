#!/usr/bin/env python3
"""Two-way surface comparison: assembly flange region vs standalone flange."""
import numpy as np
import trimesh
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
LO = np.array([-19.76, -19.76, -15.01])
HI = np.array([19.76, 18.91, 10.01])

m = trimesh.load(f'{M}/nero_RH56DF.stl', force='mesh', process=False)
m.merge_vertices()
keep = []
for c in m.split(only_watertight=False):
    lo, hi = c.bounds
    if np.all(lo >= LO - 0.5) and np.all(hi <= HI + 0.5):
        keep.append(c)
asm = trimesh.util.concatenate(keep)
print(f'flange-region bodies: {len(keep)}  tris={len(asm.faces)}')
print(f'  bbox {np.round(asm.bounds[0], 3)} .. {np.round(asm.bounds[1], 3)}')

_, tf = load_stl(f'{M}/rh56df_adapter_flange.stl')
print(f'standalone flange tris={len(tf)}')

pa = sample_tris(asm.triangles, density=30.0, seed=51)
pf = sample_tris(tf, density=30.0, seed=52)
for nm, a, b in (('asm -> std', pa, pf), ('std -> asm', pf, pa)):
    d, _ = cKDTree(b).query(a, workers=-1)
    print(f'  {nm}: n={len(a)} mean={d.mean():.4f} med={np.median(d):.4f} '
          f'q90={np.quantile(d, 0.9):.4f} max={d.max():.3f} mm')
    far = a[d > 2.0]
    if len(far):
        print(f'    pts>2mm: {len(far)}  bbox {np.round(far.min(0), 2)}'
              f' .. {np.round(far.max(0), 2)}')
