#!/usr/bin/env python3
"""Decisive eps test: link8 mesh is faithful (0.3mm), so use it to fix eps."""
import numpy as np
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import T_from

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
big = np.load('_cache/asm_big.npz')
l8 = np.vstack([big[k] for k in ('b23', 'b29', 'b33')]).astype(np.float64)
tree = cKDTree(l8)
_, t8 = load_stl(f'{M}/Link8.STL')
p8 = sample_tris(t8 * 1000.0, density=0.8, seed=21)


def rpy_from_R(R):
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-6:
        return (np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0]))
    return (0.0, p, np.arctan2(-R[0, 1], R[1, 1]))


def T7of(eps):
    T = np.eye(4)
    T[:3, :3] = np.array([[0., 0., eps], [0., eps, 0.], [-1., 0., 0.]])
    T[:3, 3] = [23.5 * eps, 0.0, 42.489]
    return T


print('link8 placed via measured link7 pose, mount x=31.0mm:')
for eps in (+1, -1):
    T = T7of(eps) @ T_from([31.0, 0.0, -23.5], [-1.5708, 0.0, -1.5708])
    q = (T[:3, :3] @ p8.T).T + T[:3, 3]
    d, _ = tree.query(q, workers=-1)
    print(f'  eps={eps:+d}: mean={d.mean():.3f} med={np.median(d):.3f}'
          f' q90={np.quantile(d, 0.9):.3f} max={d.max():.2f} mm')

Ta = T_from([42.489, 0.0, -23.5], [0.0, -1.5708, 0.0])     # link7 -> adapter
print(f'\nlink7 -> adapter : xyz={np.round(Ta[:3, 3] / 1000, 6)}'
      f' rpy={np.round(rpy_from_R(Ta[:3, :3]), 6)}')
for x in (31.0, 32.0):
    T8 = T_from([x, 0.0, -23.5], [-1.5708, 0.0, -1.5708])
    T8a = np.linalg.inv(T8) @ Ta
    print(f'link8(x={x}) -> adapter: xyz={np.round(T8a[:3, 3] / 1000, 6)}'
          f' rpy={np.round(rpy_from_R(T8a[:3, :3]), 6)}')
