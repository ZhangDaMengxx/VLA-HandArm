#!/usr/bin/env python3
"""核对手指几何:dex-urdf 的手指布局 vs 装配体实测。

手指屈曲绕各自关节轴转,该轴 ∥ 手的 z 轴 → 映射到装配系的 x 轴。
故**沿 x_asm 的位置与屈曲角无关**,可直接比;另两轴会随屈曲变,不比。
"""
import numpy as np

from stl_probe import load_stl
from urdf_fk import parse_urdf, fk, T_from

A = '/home/zhang123/ros2_ws/lerobotTest/assets'
HAND = f'{A}/inspire_hand/inspire_hand_right.urdf'
VIS = f'{A}/inspire_hand/meshes/visual'

R = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = [-0.0417, 5.9617, -7.158]          # 手掌 ICP 定出,mm

# reg6 签名匹配 + x_asm 排序得到的对应关系
PAIR = {'index_proximal': 'b9', 'middle_proximal': 'b8',
        'ring_proximal': 'b7', 'pinky_proximal': 'b10',
        'index_intermediate': 'b6', 'middle_intermediate': 'b3',
        'ring_intermediate': 'b5', 'pinky_intermediate': 'b4'}
MESH = {'index_proximal': 'right_index_proximal', 'middle_proximal': 'right_index_proximal',
        'ring_proximal': 'right_index_proximal', 'pinky_proximal': 'right_index_proximal',
        'index_intermediate': 'right_index_intermediate',
        'middle_intermediate': 'right_middle_intermediate',
        'ring_intermediate': 'right_index_intermediate',
        'pinky_intermediate': 'right_pinky_intermediate'}

links, joints = parse_urdf(HAND)
pose = fk(links, joints, root_link='hand_base_link')
big = np.load('_cache/asm_big.npz')

print('沿 x_asm(与屈曲无关)对比,单位 mm')
print(f'{"link":22s} {"URDF 预测":>16s} {"装配体实测":>16s} {"中心差":>8s} {"宽度差":>8s}')
worst = 0.0
for lk, bid in PAIR.items():
    _, tri = load_stl(f'{VIS}/{MESH[lk]}.stl')
    v = tri.reshape(-1, 3) * 1000.0
    Pl = pose[lk].copy()
    Pl[:3, 3] *= 1000.0                       # pose 是米,顶点已转毫米
    Tl = T @ Pl
    p = (Tl[:3, :3] @ v.T).T + Tl[:3, 3]
    a, b = p[:, 0].min(), p[:, 0].max()
    o = big[bid][:, 0]
    c, d = float(o.min()), float(o.max())
    dc = (a + b) / 2 - (c + d) / 2
    dw = (b - a) - (d - c)
    worst = max(worst, abs(dc))
    print(f'{lk:22s} [{a:+7.2f},{b:+7.2f}] [{c:+7.2f},{d:+7.2f}] '
          f'{dc:+8.2f} {dw:+8.2f}')
print(f'\n最大中心偏差 {worst:.2f} mm')
