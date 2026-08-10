#!/usr/bin/env python3
"""Step 1: arm zero-pose FK + cloud bbox, vs assembly STL bbox (mm -> m)."""
import numpy as np

from stl_probe import load_stl
from urdf_fk import parse_urdf, fk, link_cloud

MESH = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description'
URDF = MESH + '/urdf/nero_with_hand_flange_description.urdf'
PKG = [MESH]

links, joints = parse_urdf(URDF)
pose = fk(links, joints)
print('== zero-pose link origins (m) ==')
for n in ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6',
          'link7', 'link8']:
    T = pose[n]
    z = T[:3, 2]
    print(f'  {n:10s} p={np.round(T[:3, 3], 4)}  zaxis={np.round(z, 3)}')

clouds = link_cloud(links, pose, PKG, kind='collision')
allv = np.vstack(list(clouds.values()))
print(f'\n== arm cloud (zero pose, {len(allv)} verts) ==')
print('  lo  ', np.round(allv.min(0), 4))
print('  hi  ', np.round(allv.max(0), 4))
print('  size', np.round(allv.max(0) - allv.min(0), 4))
for n, c in clouds.items():
    print(f'  {n:10s} lo={np.round(c.min(0), 3)} hi={np.round(c.max(0), 3)}')

_, atris = load_stl(MESH + '/meshes/nero_RH56DF.stl')
av = atris.reshape(-1, 3) * 1e-3
print(f'\n== assembly cloud ({len(av)} verts, scaled to m) ==')
print('  lo  ', np.round(av.min(0), 4))
print('  hi  ', np.round(av.max(0), 4))
print('  size', np.round(av.max(0) - av.min(0), 4))
np.save('_cache/asm_v.npy', av.astype(np.float32))
np.save('_cache/arm_v.npy', allv.astype(np.float32))
