#!/usr/bin/env python3
"""Step 14: locate joint7 cylindrical boss along its own axis, in both frames."""
import xml.etree.ElementTree as ET

import numpy as np

from stl_probe import load_stl

D = '/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/dae'
ASM = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'nero_RH56DF.stl')
RLO, RHI = 21.3, 23.5


def dae_verts(path):
    out = []
    for fa in ET.parse(path).getroot().iter():
        if fa.tag.endswith('float_array') and 'position' in (fa.get('id') or ''):
            v = np.fromstring(fa.text, sep=' ')
            if v.size == int(fa.get('count')):
                out.append(v.reshape(-1, 3))
    return np.vstack(out)


def prof(a, lo, hi, step=2.0, label=''):
    e = np.arange(lo, hi + step, step)
    h, _ = np.histogram(a, bins=e)
    print(f'  {label}: range [{a.min():+.2f}, {a.max():+.2f}]  n={len(a)}')
    print('   bins ' + ' '.join(f'{e[i]:+.0f}:{h[i]}' for i in range(len(h))
                                if h[i] > 0))


v6 = dae_verts(f'{D}/link6.dae') * 1000.0
r6 = np.hypot(v6[:, 0], v6[:, 2])
sel = v6[(r6 > RLO) & (r6 < RHI)]
print('link6.dae: cylinder r in [%.1f,%.1f] about local y axis' % (RLO, RHI))
prof(sel[:, 1], -30, 26, 2.0, 'local y of cyl pts')

n, t = load_stl(ASM)
c3 = t.mean(1)
nn = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
m = ((np.abs(nn[:, 0]) < 0.25) & (c3[:, 2] > 22) & (c3[:, 2] < 66)
     & (np.abs(c3[:, 0]) < 42) & (np.abs(c3[:, 1]) < 32))
c = c3[m]
rr = np.hypot(c[:, 1] - 0.200, c[:, 2] - 42.489)
sa = c[(rr > RLO) & (rr < RHI)]
print('\nassembly wrist: same cylinder about measured joint7 axis')
prof(sa[:, 0], -32, 34, 2.0, 'x_asm of cyl pts')
print('\nmapping: x_asm = -eps * y_link6  (link6 origin at x_asm=0)')
print(f'  eps=+1 predicts x_asm in [{-sel[:, 1].max():+.2f},'
      f' {-sel[:, 1].min():+.2f}]')
print(f'  eps=-1 predicts x_asm in [{sel[:, 1].min():+.2f},'
      f' {sel[:, 1].max():+.2f}]')
