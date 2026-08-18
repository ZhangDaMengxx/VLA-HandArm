#!/usr/bin/env python3
"""Check flange truly sits at identity in assembly; report palm dir at q=0."""
import numpy as np
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import parse_urdf, fk

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
U = '/home/zhang123/ros2_ws/lerobotTest/assets/assembled/nero_inspire_right.urdf'

_, tf = load_stl(f'{M}/rh56df_adapter_flange.stl')
dense = sample_tris(tf, density=60.0, seed=41)      # ~0.13mm spacing
b40 = np.load('_cache/asm_big.npz')['b40'].astype(np.float64)
d, _ = cKDTree(dense).query(b40, workers=-1)
print(f'assembly flange body b40 -> standalone flange surface, {len(dense)}'
      ' sampled pts (identity):')
print(f'  n={len(b40)} mean={d.mean():.4f} med={np.median(d):.4f} '
      f'q90={np.quantile(d, 0.9):.4f} max={d.max():.3f} mm')

links, joints = parse_urdf(U)
pose = fk(links, joints, root_link='world')
R = pose['hand_base_link'][:3, :3]


def axis(v):
    i = int(np.argmax(np.abs(v)))
    return f"{'+' if v[i] > 0 else '-'}{'xyz'[i]}   {np.round(v, 4)}"


print('\nat q=0 (arm straight up), hand_base_link in world frame:')
print('  palm normal (-x_hand):', axis(-R[:, 0]))
print('  finger dir  (-y_hand):', axis(-R[:, 1]))
print('  spread axis (+z_hand):', axis(R[:, 2]))
print('\nfingertips at q=0 (m):')
for nm in ('index_tip', 'middle_tip', 'pinky_tip', 'thumb_tip'):
    print(f'  {nm:11s} {np.round(pose[nm][:3, 3], 5)}')
