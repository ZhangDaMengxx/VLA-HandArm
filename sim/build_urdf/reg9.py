#!/usr/bin/env python3
"""Step 9: find joint7 axis in assembly wrist region (cyl axes || x_asm)."""
import numpy as np

from stl_probe import load_stl
from cyl import cyl_axes, radii_at

ASM = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'nero_RH56DF.stl')
n, t = load_stl(ASM)
c3 = t.mean(1)
m = ((c3[:, 2] > -6) & (c3[:, 2] < 95) & (np.abs(c3[:, 0]) < 42)
     & (np.abs(c3[:, 1]) < 42))
print('wrist-region tris:', int(m.sum()))
tw, nw = t[m], n[m]

peaks, dat = cyl_axes(tw, nw, axis=0, smin=1.5, smax=32.0, npeak=12)
print('\n== cylinder axes parallel to x_asm: peak (y,z) mm, vote ==')
for p, v in peaks:
    print(f'   y={p[0]:+8.3f}  z={p[1]:+8.3f}   vote={v:9.1f}')

c, nn, w = dat
print('\n== radius spectrum around strongest peaks ==')
for p, v in peaks[:4]:
    h, e = radii_at(c, p, w, 0, 30, 60)
    top = np.argsort(h)[::-1][:6]
    s = '  '.join(f'r={0.5 * (e[i] + e[i + 1]):5.2f}:{h[i]:7.1f}'
                 for i in sorted(top))
    print(f'  peak y={p[0]:+7.2f} z={p[1]:+7.2f}: {s}')
