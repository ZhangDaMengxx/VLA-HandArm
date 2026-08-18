#!/usr/bin/env python3
"""拆解残差来源:度量假象 vs 真实错位。

final_check.py 报的是「件的采样点 → 装配体**顶点**」的距离,且装配体顶点还降采样
到 20%。装配体三角面本身有大小,落在面中心的点离最近顶点天生就有距离 —— 这部分是
**度量假象**,与贴合好坏无关。本脚本分离两者:

  假象     = 装配体自己的表面采样点 → 自己的顶点(纯度量下限,件都不参与)
  点→顶点  = 件 → 装配体顶点(复现 final_check 的口径)
  点→面    = 件 → 装配体表面密集采样(真实贴合)
"""
import numpy as np
import trimesh
from scipy.spatial import cKDTree

from stl_probe import load_stl
from feat import sample_tris
from urdf_fk import parse_urdf, fk

S = '/home/zhang123/ros2_ws/lerobotTest/assets/assembled/nero_inspire_right.urdf'
ASM = ('/home/zhang123/ros2_ws/lerobotTest/assets/nero_description/meshes/'
       'nero_RH56DF.stl')
BODY = {'link8': [23, 29, 33], 'rh56df_adapter_flange': [40],
        'hand_base_link': [1]}
rng = np.random.default_rng(0)

m = trimesh.load(ASM, force='mesh', process=False)
m.merge_vertices()
bodies = sorted(m.split(only_watertight=False), key=lambda c: -len(c.faces))
links, joints = parse_urdf(S)
pose = fk(links, joints, root_link='world')
Ta = np.eye(4)
Ta[:3, :3] = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
Ta[:3, 3] = [0.0235, 0.0, 0.042489]
T_a_w = np.linalg.inv(pose['link7'] @ np.linalg.inv(Ta))

print(f'{"件":24s}{"面积mm²":>9s}{"三角边":>7s}{"假象":>6s}'
      f'{"点→顶点":>9s}{"点→面":>7s}   (中位,mm)')
for name, ids in BODY.items():
    tgt = trimesh.util.concatenate([bodies[i] for i in ids])
    tv = tgt.vertices * 1e-3
    sub = tv[rng.choice(len(tv), int(len(tv) * 0.204), replace=False)]
    ts = sample_tris(tgt.triangles, density=25.0, seed=71) * 1e-3
    kv, ks = cKDTree(sub), cKDTree(ts)
    art = np.median(kv.query(ts, workers=-1)[0]) * 1000
    fn = [x for x in links[name]['meshes'] if x['kind'] == 'visual'][0]['file']
    _, tri = load_stl(fn.replace('.glb', '.stl'))
    q = sample_tris(tri if 'flange' in name else tri * 1000.0,
                    density=5.0, seed=72) * 1e-3
    P = pose[name]
    qw = (P[:3, :3] @ q.T).T + P[:3, 3]
    qa = (T_a_w[:3, :3] @ qw.T).T + T_a_w[:3, 3]
    dv = np.median(kv.query(qa, workers=-1)[0]) * 1000
    dsurf = ks.query(qa, workers=-1)[0]
    edge = np.sqrt(2 * tgt.area / len(tgt.faces))
    print(f'{name:24s}{tgt.area:9.0f}{edge:7.2f}{art:6.2f}'
          f'{dv:9.2f}{np.median(dsurf) * 1000:7.2f}')
    far = qa[dsurf > 1e-3]
    if len(far):
        b = np.array([far.min(0), far.max(0)]) * 1000
        print(f'{"":24s}  点→面 >1mm 占 {len(far) / len(qa) * 100:4.1f}%,位于 '
              f'x[{b[0, 0]:+.0f},{b[1, 0]:+.0f}] y[{b[0, 1]:+.0f},{b[1, 1]:+.0f}]'
              f' z[{b[0, 2]:+.0f},{b[1, 2]:+.0f}]')
