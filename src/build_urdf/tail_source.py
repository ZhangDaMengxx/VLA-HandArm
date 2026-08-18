#!/usr/bin/env python3
"""定位两处长尾的具体来源。

法兰:4.5% 的远点全在 z=-5 单个平面 => 查装配体在该处到底有没有面。
手掌:13.4% 的远点在 y_asm∈[13,31] 薄层 => 查是 URDF 网格多料还是装配体多料。
"""
import numpy as np

from stl_probe import load_stl, area as tri_area

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
ASM = f'{A}/nero_description/meshes/nero_RH56DF.stl'
_, ta = load_stl(ASM)
cen = ta.mean(1)
ar = tri_area(ta)

print('=== 法兰:主板朝手那面(装配系 z≈-5)===')
for lo, hi, tag in ((-6.0, -4.0, 'z∈[-6,-4] 该面所在'),
                    (-16.0, -14.0, 'z∈[-16,-14] 对照:方板另一端'),
                    (9.0, 11.0, 'z∈[9,11] 对照:圆段末端')):
    m = ((cen[:, 2] > lo) & (cen[:, 2] < hi)
         & (np.abs(cen[:, 0]) < 20) & (np.abs(cen[:, 1]) < 20))
    print(f'  {tag:26s} 三角面 {int(m.sum()):6d}  面积 {ar[m].sum():8.1f} mm²')

print('\n=== 手掌:薄层 y_asm∈[13,31] 里两边各有多少料 ===')
_, th = load_stl(f'{A}/inspire_hand/meshes/visual/right_base_link.stl')
R = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
t = np.array([-0.0417, 5.9617, -7.158])
hv = (R @ (th.reshape(-1, 3) * 1000.0).T).T + t          # URDF 网格 -> 装配系
m = (cen[:, 2] > -150) & (cen[:, 2] < -5) & (np.abs(cen[:, 0]) < 45)
av = ta[m].reshape(-1, 3)
for lo, hi in ((13.0, 31.0), (-30.0, 13.0)):
    a = hv[(hv[:, 1] >= lo) & (hv[:, 1] < hi)]
    b = av[(av[:, 1] >= lo) & (av[:, 1] < hi)]
    print(f'  y∈[{lo:+.0f},{hi:+.0f}]: URDF 顶点 {len(a):7d}  装配体顶点 {len(b):8d}')
    if len(a) and len(b):
        print(f'      URDF   y上界 {a[:, 1].max():+7.2f}  z范围'
              f' [{a[:, 2].min():+8.2f},{a[:, 2].max():+8.2f}]')
        print(f'      装配体 y上界 {b[:, 1].max():+7.2f}  z范围'
              f' [{b[:, 2].min():+8.2f},{b[:, 2].max():+8.2f}]')
print(f'\n  URDF 网格整体 y 上界 {hv[:, 1].max():+.2f} mm')
print(f'  装配体手区整体 y 上界 {av[:, 1].max():+.2f} mm')
print(f'  => URDF 比装配体多出 {hv[:, 1].max() - av[:, 1].max():+.2f} mm')
