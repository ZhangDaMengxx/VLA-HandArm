#!/usr/bin/env python3
"""Step 2: z-slice profile of assembly vs arm zero-pose cloud."""
import numpy as np

asm = np.load('_cache/asm_v.npy').astype(np.float64)
arm = np.load('_cache/arm_v.npy').astype(np.float64)


def profile(v, name, step=0.02):
    lo, hi = v[:, 2].min(), v[:, 2].max()
    edges = np.arange(np.floor(lo / step) * step, hi + step, step)
    idx = np.clip(np.digitize(v[:, 2], edges) - 1, 0, len(edges) - 2)
    print(f'== {name}: z {lo:.4f}..{hi:.4f} ==')
    print('   z_lo    n      x_lo    x_hi    y_lo    y_hi   cx      cy')
    for i in range(len(edges) - 1):
        s = v[idx == i]
        if len(s) == 0:
            continue
        print(f'  {edges[i]:+.3f} {len(s):7d} '
              f'{s[:, 0].min():+.4f} {s[:, 0].max():+.4f} '
              f'{s[:, 1].min():+.4f} {s[:, 1].max():+.4f} '
              f'{s[:, 0].mean():+.4f} {s[:, 1].mean():+.4f}')


profile(asm, 'assembly (m)')
print()
profile(arm, 'arm zero-pose (m)')
