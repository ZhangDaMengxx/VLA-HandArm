#!/usr/bin/env python3
"""Direct slice profile: where is the O23 boss in the assembly flange body?"""
import numpy as np

from stl_probe import load_stl

M = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes'
b40 = np.load('_cache/asm_big.npz')['b40'].astype(np.float64)
_, tf = load_stl(f'{M}/rh56df_adapter_flange.stl')
std = tf.reshape(-1, 3)


def prof(v, name):
    print(f'\n{name}: n={len(v)} z=[{v[:, 2].min():.2f}, {v[:, 2].max():.2f}]')
    print('  z_slice    npts  max_r   x_range           y_range')
    for z0 in np.arange(-16, 11, 1.0):
        s = v[(v[:, 2] >= z0) & (v[:, 2] < z0 + 1.0)]
        if not len(s):
            continue
        r = np.hypot(s[:, 0], s[:, 1]).max()
        print(f'  {z0:+6.1f} {len(s):7d} {r:6.2f}  '
              f'[{s[:, 0].min():+6.2f},{s[:, 0].max():+6.2f}] '
              f'[{s[:, 1].min():+6.2f},{s[:, 1].max():+6.2f}]')


prof(b40, 'assembly flange body b40')
prof(std, 'standalone flange STL')
