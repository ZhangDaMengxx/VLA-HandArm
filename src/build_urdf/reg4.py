#!/usr/bin/env python3
"""Step 4: assembly bodies, largest first; save big-body clouds for matching."""
import numpy as np
import trimesh

P = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/nero_RH56DF.stl'
m = trimesh.load(P, force='mesh', process=False)
m.merge_vertices()
comps = m.split(only_watertight=False)
big = sorted(comps, key=lambda c: -len(c.faces))
print(f'{"rk":>3} {"ntri":>8} {"area":>10} {"vol":>11} '
      f'{"lo(mm)":>26} {"hi(mm)":>26} {"size":>22}')
tot = 0
for k, c in enumerate(big[:45]):
    lo, hi = c.bounds
    try:
        v = abs(c.volume)
    except Exception:
        v = float('nan')
    tot += len(c.faces)
    print(f'{k:3d} {len(c.faces):8d} {c.area:10.1f} {v:11.1f} '
          f'{str(np.round(lo, 2)):>26} {str(np.round(hi, 2)):>26} '
          f'{str(np.round(hi - lo, 2)):>22}')
print(f'top45 tri={tot} / total={len(m.faces)}')
rest = sum(len(c.faces) for c in big[45:])
print(f'remaining {len(big) - 45} bodies, {rest} tri')
np.savez_compressed('_cache/asm_big.npz',
                    **{f'b{k}': c.vertices.astype(np.float32)
                       for k, c in enumerate(big[:45])})
