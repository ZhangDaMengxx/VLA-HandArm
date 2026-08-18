#!/usr/bin/env python3
"""Test all 48 axis-aligned transforms (24 rot + 24 reflect) for the flange."""
import itertools

import numpy as np
import trimesh

from stl_probe import load_stl
from feat import sample_tris
from icp import fixed_r_icp

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
LO = np.array([-19.76, -19.76, -15.01])
HI = np.array([19.76, 18.91, 10.01])

m = trimesh.load(f'{M}/nero_RH56DF.stl', force='mesh', process=False)
m.merge_vertices()
keep = [c for c in m.split(only_watertight=False)
        if np.all(c.bounds[0] >= LO - 0.5) and np.all(c.bounds[1] <= HI + 0.5)]
tgt = sample_tris(trimesh.util.concatenate(keep).triangles, density=20.0,
                  seed=61)

_, tf = load_stl(f'{M}/rh56df_adapter_flange.stl')
src_all = sample_tris(tf, density=20.0, seed=62)
rng = np.random.default_rng(0)
src = src_all[rng.choice(len(src_all), 8000, replace=False)]
print(f'target pts={len(tgt)}  source pts={len(src)}')

mats = []
for perm in itertools.permutations(range(3)):
    for sg in itertools.product((1, -1), repeat=3):
        R = np.zeros((3, 3))
        for i, p in enumerate(perm):
            R[i, p] = sg[i]
        mats.append(R)

res = []
for R in mats:
    t0 = tgt.mean(0) - R @ src.mean(0)
    t, rmse, md = fixed_r_icp(src, tgt, R, t0, keep=0.9)
    res.append((rmse, md, np.linalg.det(R), R, t))
res.sort(key=lambda r: r[0])
print(f'{"rmse":>8} {"mean":>8} {"det":>4}   R (rows)                t (mm)')
for rmse, md, dt, R, t in res[:6]:
    rr = ' '.join(str(R[i].astype(int)) for i in range(3))
    print(f'{rmse:8.4f} {md:8.4f} {dt:+4.0f}   {rr}  {np.round(t, 3)}')
for rmse, md, dt, R, t in res:
    if np.allclose(R, np.eye(3)):
        print(f'\nidentity: rmse={rmse:.4f} mean={md:.4f} t={np.round(t, 3)}')
