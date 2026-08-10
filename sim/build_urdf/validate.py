#!/usr/bin/env python3
"""End-to-end check: adapter identity, link8 mount value, hand pose vs assembly."""
import numpy as np
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import T_from

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
M = f'{A}/nero_description/meshes'
T7 = np.eye(4)
T7[:3, :3] = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
T7[:3, 3] = [23.5, 0.0, 42.489]                        # asm <- link7

asm = np.load('_cache/asm_v.npy').astype(np.float64) * 1000.0
big = np.load('_cache/asm_big.npz')


def resid(pts, tgt, label):
    d, _ = cKDTree(tgt).query(pts, workers=-1)
    print(f'  {label}: n={len(pts)} mean={d.mean():.3f} med={np.median(d):.3f}'
          f' q90={np.quantile(d, 0.9):.3f} max={d.max():.2f} mm')


_, tf = load_stl(f'{M}/NERO+因时RH56DF_适配法兰.stl')
fp = sample_tris(tf, density=1.0, seed=11)
near = asm[(asm[:, 2] > -17) & (asm[:, 2] < 12)]
print('1) adapter mesh -> assembly cloud, assuming identity')
resid(fp, near, 'flange')

l8 = np.vstack([big[k] for k in ('b23', 'b29', 'b33')]).astype(np.float64)
print(f'\n2) link8 target b23+b29+b33: bbox {np.round(l8.min(0), 2)}'
      f' .. {np.round(l8.max(0), 2)}')
_, t8 = load_stl(f'{M}/Link8.STL')
p8 = sample_tris(t8 * 1000.0, density=0.5, seed=12)
for x in (30.5, 31.0, 32.0):
    T = T7 @ T_from([x, 0.0, -23.5], [-1.5708, 0.0, -1.5708])
    q = (T[:3, :3] @ p8.T).T + T[:3, 3]
    print(f'  mount x={x} -> bbox {np.round(q.min(0), 2)}'
          f' .. {np.round(q.max(0), 2)}')
    resid(q, l8, f'   link8 @ {x}')

Rh = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
_, th = load_stl(f'{A}/inspire_hand/meshes/visual/right_base_link.stl')
ph = (Rh @ sample_tris(th * 1000.0, density=0.5, seed=13).T).T \
    + np.array([-0.0417, 5.9617, -7.158])
print('\n3) hand base_link -> assembly palm b1')
resid(ph, big['b1'].astype(np.float64), 'hand_base')
