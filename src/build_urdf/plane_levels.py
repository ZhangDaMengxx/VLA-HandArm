#!/usr/bin/env python3
"""量法兰区域里垂直于 z 的平面到底在哪几个高度,定案 4.5% 长尾的来源。

我摆放的法兰在装配系 z=-5 有个 1197mm² 满外形面;装配体在 z∈[-6,-4] 有 1146mm²。
若装配体那面实际在 -6.7 而非 -5,则我的点会差 1.7mm(正好等于实测 q99=1.72)。
"""
import numpy as np

from stl_probe import load_stl
from feat import planes_along

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
M = f'{A}/nero_description/meshes'

_, ta = load_stl(f'{M}/nero_RH56DF.stl')
cen = ta.mean(1)
m = (np.abs(cen[:, 0]) < 20) & (np.abs(cen[:, 1]) < 20) \
    & (cen[:, 2] > -18) & (cen[:, 2] < 12)
nrm = np.cross(ta[:, 1] - ta[:, 0], ta[:, 2] - ta[:, 0])
nrm = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
print(f'装配体法兰区域三角面 {int(m.sum())}')
print('  垂直于 z 的平面(面积 >30mm²):')
for sgn, off, a, n in planes_along(nrm[m], ta[m], axis=2, min_area=30.0)[:10]:
    print(f'    n={"+" if sgn > 0 else "-"}z  z={off:+8.3f}  面积 {a:8.1f}  面数 {n}')

_, tf = load_stl(f'{M}/rh56df_adapter_flange.stl')
R = np.diag([-1.0, 1.0, -1.0])
tp = (R @ tf.reshape(-1, 3).T).T + np.array([0.0, 0.0, -5.0])
tp = tp.reshape(-1, 3, 3)
npf = np.cross(tp[:, 1] - tp[:, 0], tp[:, 2] - tp[:, 0])
npf = npf / np.maximum(np.linalg.norm(npf, axis=1, keepdims=True), 1e-12)
print('\n我摆放的法兰(Ry180 + z-5)的 z 向平面:')
for sgn, off, a, n in planes_along(npf, tp, axis=2, min_area=30.0)[:10]:
    print(f'    n={"+" if sgn > 0 else "-"}z  z={off:+8.3f}  面积 {a:8.1f}  面数 {n}')
