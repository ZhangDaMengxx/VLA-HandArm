#!/usr/bin/env python3
"""Step 3: split assembly STL into connected bodies, report per-body signature."""
import numpy as np
import trimesh

P = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/nero_RH56DF.stl'
m = trimesh.load(P, force='mesh', process=False)
print('loaded', m.faces.shape, m.vertices.shape)
m.merge_vertices()
print('merged verts ->', m.vertices.shape)
comps = m.split(only_watertight=False)
print('bodies:', len(comps))
rows = []
for i, c in enumerate(comps):
    lo, hi = c.bounds
    try:
        vol = abs(c.volume)
    except Exception:
        vol = float('nan')
    rows.append((i, len(c.faces), c.area, vol, lo, hi, c.centroid))
rows.sort(key=lambda r: -r[1])
print(f'{"id":>4} {"ntri":>8} {"area_mm2":>10} {"vol_mm3":>11}  '
      f'{"size(mm)":>24}  {"centroid(mm)":>24}')
for i, nf, a, v, lo, hi, cen in rows:
    sz = np.round(hi - lo, 2)
    print(f'{i:4d} {nf:8d} {a:10.1f} {v:11.1f}  '
          f'{str(sz):>24}  {str(np.round(cen, 2)):>24}')
np.save('_cache/comp_bounds.npy',
        np.array([np.concatenate([r[4], r[5]]) for r in rows]))
with open('_cache/comp_order.txt', 'w') as f:
    for r in rows:
        f.write(f'{r[0]} {r[1]}\n')
