#!/usr/bin/env python3
"""Final check: FK the generated URDF at q=0, compare parts with assembly STL."""
import numpy as np
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import parse_urdf, fk

U = '/home/zhang123/ros2_ws/lerobotTest/assets/assembled/nero_inspire_right.urdf'
ASM = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'nero_RH56DF.stl')

links, joints = parse_urdf(U)
pose = fk(links, joints, root_link='world')
T_a_l7 = np.eye(4)
T_a_l7[:3, :3] = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
T_a_l7[:3, 3] = [0.0235, 0.0, 0.042489]
T_w_a = pose['link7'] @ np.linalg.inv(T_a_l7)

_, at = load_stl(ASM)
av = at.reshape(-1, 3) * 1e-3
rng = np.random.default_rng(0)
av = av[rng.choice(len(av), 1500000, replace=False)]
aw = (T_w_a[:3, :3] @ av.T).T + T_w_a[:3, 3]
tree = cKDTree(aw)
print('assembly mapped into world:', np.round(aw.min(0), 4),
      np.round(aw.max(0), 4))

for name, sc in (('link8', 1.0), ('rh56df_adapter_flange', 1e-3),
                 ('hand_base_link', 1.0)):
    lk = links[name]
    mv = [m for m in lk['meshes'] if m['kind'] == 'visual'] or lk['meshes']
    f = mv[0]['file'].replace('.glb', '.stl')
    _, t = load_stl(f)
    p = sample_tris(t * sc, density=2e5, seed=31)
    T = pose[name]
    q = (T[:3, :3] @ p.T).T + T[:3, 3]
    d, _ = tree.query(q, workers=-1)
    print(f'\n{name}:')
    print(f'  world origin {np.round(T[:3, 3], 5)}')
    print(f'  zaxis {np.round(T[:3, 2], 4)}  n={len(q)}')
    print(f'  residual mean={d.mean() * 1000:.3f} med={np.median(d) * 1000:.3f}'
          f' q90={np.quantile(d, 0.9) * 1000:.3f} mm')

for nm in ('middle_tip', 'thumb_tip'):
    if nm in pose:
        print(f'{nm} at q=0: {np.round(pose[nm][:3, 3], 5)}')
