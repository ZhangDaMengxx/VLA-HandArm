#!/usr/bin/env python3
"""查残差长尾:是真实形状差异,还是我把目标连通体取窄了?

residual_breakdown.py 的目标是单个连通体(如手只取 body 1 手掌壳)。若 URDF 网格
里某部分在装配体中属于**别的**连通体,就会被判为"远"。这里改用**区域**取目标:
把件的采样点包围盒外扩 3mm,该区域内所有装配体三角面都算目标,与连通体划分无关。
若长尾随之消失 => 原先是目标取窄;若仍在 => 是真实形状差异。
"""
import numpy as np
import trimesh
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import parse_urdf, fk

S = '/home/zhang123/ros2_ws/lerobotTest/sim/assets/nero_inspire_right.urdf'
ASM = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'nero_RH56DF.stl')

nrm, tri_a = load_stl(ASM)
cen = tri_a.mean(1)
links, joints = parse_urdf(S)
pose = fk(links, joints, root_link='world')
Ta = np.eye(4)
Ta[:3, :3] = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
Ta[:3, 3] = [0.0235, 0.0, 0.042489]
T_a_w = np.linalg.inv(pose['link7'] @ np.linalg.inv(Ta))

for name in ('link8', 'rh56df_adapter_flange', 'hand_base_link'):
    fn = [x for x in links[name]['meshes'] if x['kind'] == 'visual'][0]['file']
    _, tri = load_stl(fn.replace('.glb', '.stl'))
    q = sample_tris(tri if 'flange' in name else tri * 1000.0,
                    density=5.0, seed=81) * 1e-3
    P = pose[name]
    qw = (P[:3, :3] @ q.T).T + P[:3, 3]
    qa = ((T_a_w[:3, :3] @ qw.T).T + T_a_w[:3, 3]) * 1000.0   # mm
    lo, hi = qa.min(0) - 3.0, qa.max(0) + 3.0
    sel = np.all((cen >= lo) & (cen <= hi), axis=1)
    ts = sample_tris(tri_a[sel], density=40.0, seed=82)
    d = cKDTree(ts).query(qa, workers=-1)[0]
    print(f'{name}: 目标三角面 {int(sel.sum())} (区域取), 采样 {len(ts)}')
    print(f'  点→面 中位 {np.median(d):.3f}  q90 {np.quantile(d, 0.9):.3f}'
          f'  q99 {np.quantile(d, 0.99):.3f}  max {d.max():.2f} mm')
    far = qa[d > 1.0]
    print(f'  >1mm 占 {len(far) / len(qa) * 100:4.1f}%', end='')
    if len(far):
        print(f', 位于 x[{far[:, 0].min():+.0f},{far[:, 0].max():+.0f}]'
              f' y[{far[:, 1].min():+.0f},{far[:, 1].max():+.0f}]'
              f' z[{far[:, 2].min():+.0f},{far[:, 2].max():+.0f}]')
    else:
        print()
