#!/usr/bin/env python3
"""Parse a URDF, run FK at a given joint config, return link poses + mesh clouds."""
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

from stl_probe import load_stl


def rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def T_from(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_to_R(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints, links = [], {}
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else [0, 0, 0])]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0').split() if o is not None else [0, 0, 0])]
        ax = j.find('axis')
        axis = [float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1]
        joints.append(dict(name=j.get('name'), type=j.get('type'),
                           parent=j.find('parent').get('link'),
                           child=j.find('child').get('link'),
                           xyz=xyz, rpy=rpy, axis=axis))
    for lk in root.findall('link'):
        meshes = []
        for tag in ('visual', 'collision'):
            for el in lk.findall(tag):
                m = el.find('geometry/mesh')
                if m is None:
                    continue
                o = el.find('origin')
                mxyz = [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else [0, 0, 0])]
                mrpy = [float(v) for v in (o.get('rpy', '0 0 0').split() if o is not None else [0, 0, 0])]
                meshes.append(dict(kind=tag, file=m.get('filename'), xyz=mxyz, rpy=mrpy))
        links[lk.get('name')] = dict(name=lk.get('name'), meshes=meshes)
    return links, joints


def fk(links, joints, q=None, root_link='world'):
    q = q or {}
    pose = {root_link: np.eye(4)}
    remaining = list(joints)
    for _ in range(len(joints) + 1):
        for j in list(remaining):
            if j['parent'] in pose:
                T = T_from(j['xyz'], j['rpy'])
                if j['type'] in ('revolute', 'continuous'):
                    Rj = np.eye(4)
                    Rj[:3, :3] = axis_angle_to_R(j['axis'], q.get(j['name'], 0.0))
                    T = T @ Rj
                elif j['type'] == 'prismatic':
                    Tp = np.eye(4)
                    Tp[:3, 3] = np.array(j['axis']) * q.get(j['name'], 0.0)
                    T = T @ Tp
                pose[j['child']] = pose[j['parent']] @ T
                remaining.remove(j)
    return pose


def resolve(fn, pkg_dirs):
    m = re.match(r'package://([^/]+)/(.*)', fn)
    rel = m.group(2) if m else fn
    for d in pkg_dirs:
        cand = os.path.join(d, rel)
        if os.path.exists(cand):
            return cand
    return None


def link_cloud(links, pose, pkg_dirs, kind='collision', skip=()):
    out = {}
    for name, lk in links.items():
        if name in skip or name not in pose:
            continue
        for m in lk['meshes']:
            if m['kind'] != kind:
                continue
            p = resolve(m['file'], pkg_dirs)
            if p is None or not p.lower().endswith('.stl'):
                continue
            _, tris = load_stl(p)
            v = tris.reshape(-1, 3)
            T = pose[name] @ T_from(m['xyz'], m['rpy'])
            out[name] = (T[:3, :3] @ v.T).T + T[:3, 3]
            break
    return out
