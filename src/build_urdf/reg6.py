#!/usr/bin/env python3
"""Step 6: signature-match assembly bodies against known arm/hand/flange STLs."""
import glob
import os

import numpy as np
import trimesh

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
ASM = f'{A}/nero_description/meshes/nero_RH56DF.stl'

cands = []
for pat, scale in ((f'{A}/nero_description/meshes/*.STL', 1000.0),
                   (f'{A}/nero_old/meshes/*.stl', 1000.0),
                   (f'{A}/nero_official/meshes/*.stl', 1000.0),
                   (f'{A}/inspire_hand/meshes/visual/*.stl', 1000.0)):
    for p in sorted(glob.glob(pat)):
        if 'nero_RH56DF' in p:
            continue
        m = trimesh.load(p, force='mesh', process=False)
        v = m.vertices * scale
        m2 = trimesh.Trimesh(v, m.faces, process=False)
        cands.append((os.path.relpath(p, A), np.sort(v.max(0) - v.min(0)),
                      m2.area, abs(m2.volume)))

m = trimesh.load(ASM, force='mesh', process=False)
m.merge_vertices()
bodies = sorted(m.split(only_watertight=False), key=lambda c: -len(c.faces))[:45]
sig = [(k, np.sort(c.bounds[1] - c.bounds[0]), c.area, abs(c.volume), c.bounds)
       for k, c in enumerate(bodies)]

print(f'{"body":>5} {"area":>9} {"vol":>10} {"dims_sorted":>24} -> best candidates')
for k, dims, ar, vol, bnd in sig:
    sc = []
    for nm, cd, ca, cv in cands:
        d = np.abs(dims - cd).max()
        ra = abs(ar - ca) / max(ar, ca, 1e-9)
        rv = abs(vol - cv) / max(vol, cv, 1e-9)
        sc.append((d + 40 * ra + 40 * rv, d, ra, rv, nm))
    sc.sort()
    tag = ' | '.join(f'{nm}(d={d:.2f},a={ra:.2f},v={rv:.2f})'
                     for _, d, ra, rv, nm in sc[:2])
    print(f'{k:5d} {ar:9.1f} {vol:10.1f} {str(np.round(dims, 2)):>24} -> {tag}')
