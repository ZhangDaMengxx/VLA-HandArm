#!/usr/bin/env python3
"""Step 8: planar-face survey of adapter / revo2 flange / arm wrist links (mm)."""
import numpy as np

from stl_probe import load_stl
from feat import normal_hist, planes_along

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
JOBS = [
    (f'{A}/nero_description/meshes/NERO+因时RH56DF_适配法兰.stl', 1.0, 'adapter'),
    (f'{A}/nero_old/meshes/revo2_flange.stl', 1000.0, 'revo2_flange'),
    (f'{A}/nero_description/meshes/Link6.STL', 1000.0, 'Link6'),
    (f'{A}/nero_description/meshes/Link7.STL', 1000.0, 'Link7'),
    (f'{A}/inspire_hand/meshes/visual/right_base_link.stl', 1000.0, 'hand_base'),
]

for path, sc, name in JOBS:
    n, t = load_stl(path)
    t = t * sc
    v = t.reshape(-1, 3)
    print(f'\n=== {name}: bbox {np.round(v.min(0), 3)} .. {np.round(v.max(0), 3)} mm')
    print('  dominant normals (area-weighted, mm^2):')
    for nn, w in normal_hist(n, t)[:6]:
        print(f'    {np.round(nn, 3)}  area={w:9.1f}')
    for ax, lbl in ((0, 'x'), (1, 'y'), (2, 'z')):
        pl = planes_along(n, t, axis=ax, min_area=15.0)
        if not pl:
            continue
        print(f'  planes perpendicular to {lbl}:')
        for sgn, off, ar, cnt in pl[:8]:
            s = '+' if sgn > 0 else '-'
            print(f'    n={s}{lbl} at {off:+9.3f}  area={ar:9.1f}  ntri={cnt}')
