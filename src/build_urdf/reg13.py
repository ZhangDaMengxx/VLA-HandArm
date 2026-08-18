#!/usr/bin/env python3
"""Step 13: parse detailed .dae visual meshes; retest eps with better geometry."""
import xml.etree.ElementTree as ET

import numpy as np

from icp import register
from urdf_fk import T_from

D = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/dae'


def dae_verts(path):
    """Concatenate every float_array that looks like a position array."""
    root = ET.parse(path).getroot()
    out = []
    for fa in root.iter():
        if not fa.tag.endswith('float_array'):
            continue
        c = int(fa.get('count', '0'))
        if c % 3 or c < 9:
            continue
        if 'position' not in (fa.get('id') or ''):
            continue
        v = np.fromstring(fa.text, sep=' ')
        if v.size != c:
            continue
        out.append(v.reshape(-1, 3))
    return np.vstack(out) if out else None


big = np.load('_cache/asm_big.npz')
for nm in ('link6', 'link7', 'link5'):
    v = dae_verts(f'{D}/{nm}.dae') * 1000.0
    print(f'{nm}.dae: {len(v):7d} verts  bbox {np.round(v.min(0), 2)}'
          f' .. {np.round(v.max(0), 2)}')

v6 = dae_verts(f'{D}/link6.dae') * 1000.0
tgt = big['b36'].astype(np.float64)
R, tt, rmse, q70, md = register(v6, tgt, n_src=8000, keep=0.7)
print(f'\nICP link6.dae -> body36: rmse={rmse:.3f} q70={q70:.3f} mean={md:.3f}')
print('  R_icp =', np.array2string(np.round(R, 3), prefix='          '))
print('  t_icp =', np.round(tt, 2))
for eps in (+1, -1):
    T7 = np.eye(4)
    T7[:3, :3] = np.array([[0, 0, eps], [0, eps, 0], [-1.0, 0, 0]])
    T7[:3, 3] = [23.5 * eps, 0, 42.489]
    T6 = T7 @ np.linalg.inv(T_from([0, -23.5, 0], [1.5708, 0, 0]))
    a = np.degrees(np.arccos(np.clip(
        (np.trace(R.T @ T6[:3, :3]) - 1) / 2, -1, 1)))
    print(f'  eps={eps:+d}: dR={a:6.2f} deg  t_pred={np.round(T6[:3, 3], 2)}')
