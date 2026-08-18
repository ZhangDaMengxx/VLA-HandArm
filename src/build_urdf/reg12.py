#!/usr/bin/env python3
"""Step 12: refine joint7 axis (circle fit) and resolve eps by Link6 ICP."""
import numpy as np

from stl_probe import load_stl
from feat import sample_tris
from icp import register
from urdf_fk import T_from

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'

n, t = load_stl(f'{M}/nero_RH56DF.stl')
c3 = t.mean(1)
nn = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
m = ((np.abs(nn[:, 0]) < 0.2) & (c3[:, 2] > 25) & (c3[:, 2] < 62)
     & (np.abs(c3[:, 0]) < 42) & (np.abs(c3[:, 1]) < 30))
p = c3[m][:, 1:]
d = np.linalg.norm(p - np.array([0.266, 42.542]), axis=1)
q = p[(d > 20.5) & (d < 23.5)]
A = np.c_[q, np.ones(len(q))]
sol, *_ = np.linalg.lstsq(A, (q ** 2).sum(1), rcond=None)
cy, cz = sol[0] / 2, sol[1] / 2
r = np.sqrt(sol[2] + cy ** 2 + cz ** 2)
res = np.abs(np.linalg.norm(q - [cy, cz], axis=1) - r)
print(f'joint7 axis: y={cy:+.3f} z={cz:+.3f} r={r:.3f} n={len(q)} '
      f'res mean={res.mean():.3f} max={res.max():.3f}')

big = np.load('_cache/asm_big.npz')
tgt = big['b36'].astype(np.float64)
_, t6 = load_stl(f'{M}/Link6.STL')
src = sample_tris(t6 * 1000.0, density=1.0, seed=5)
R, tt, rmse, q70, md = register(src, tgt, n_src=6000, keep=0.7)
print(f'\nICP Link6 -> body36: rmse={rmse:.3f} q70={q70:.3f} mean={md:.3f}')
print('  R_icp =', np.array2string(np.round(R, 3), prefix='          '))
print('  t_icp =', np.round(tt, 2))
for eps in (+1, -1):
    R7 = np.array([[0, 0, eps], [0, eps, 0], [-1.0, 0, 0]])
    T7 = np.eye(4)
    T7[:3, :3] = R7
    T7[:3, 3] = [23.5 * eps, 0, cz]
    T6 = T7 @ np.linalg.inv(T_from([0, -23.5, 0], [1.5708, 0, 0]))
    a = np.degrees(np.arccos(np.clip(
        (np.trace(R.T @ T6[:3, :3]) - 1) / 2, -1, 1)))
    print(f'  eps={eps:+d}: dR(icp,pred)={a:6.2f} deg '
          f' t_pred={np.round(T6[:3, 3], 2)}')
