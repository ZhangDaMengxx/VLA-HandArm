#!/usr/bin/env python3
"""Step 5: is assembly hand left or right? Register both base_link meshes."""
import numpy as np

from stl_probe import load_stl
from icp import register
from feat import sample_tris

HAND = '/home/zhang123/ros2_ws/lerobotTest/assets/inspire_hand/meshes/visual'
FLG = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'NERO+因时RH56DF_适配法兰.stl')

big = np.load('_cache/asm_big.npz')
palm = big['b1'].astype(np.float64)          # assembly palm shell, mm
print('target palm verts', palm.shape, np.round(palm.min(0), 2),
      np.round(palm.max(0), 2))

for side in ('right', 'left'):
    _, tris = load_stl(f'{HAND}/{side}_base_link.stl')
    src = sample_tris(tris * 1000.0, density=0.6, seed=1)   # mm, ~ area/1.6
    R, t, rmse, q75, md = register(src, palm, n_src=6000, keep=0.6)
    print(f'\n== {side}_base_link -> assembly palm ==')
    print(f'  src pts {len(src)}  rmse={rmse:.4f} mm  q60={q75:.4f}  mean={md:.4f}')
    print('  R =', np.array2string(np.round(R, 4), prefix='       '))
    print('  t =', np.round(t, 3), 'mm')

# sanity: the standalone flange should sit at identity inside the assembly
_, ftris = load_stl(FLG)
fsrc = sample_tris(ftris, density=1.0, seed=2)
flg_body = big['b40'].astype(np.float64)
R, t, rmse, q, md = register(fsrc, flg_body, n_src=6000, keep=0.9,
                             rots=[np.eye(3)])
print('\n== flange STL -> assembly body b40 (identity start) ==')
print(f'  rmse={rmse:.5f} mm  q90={q:.5f}  mean={md:.5f}')
print('  R =', np.array2string(np.round(R, 5), prefix='       '))
print('  t =', np.round(t, 4), 'mm')
