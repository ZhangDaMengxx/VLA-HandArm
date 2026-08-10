#!/usr/bin/env python3
"""Compose the exact mount strings for build_nero_inspire.py and verify chain."""
import numpy as np

from urdf_fk import T_from

np.set_printoptions(precision=9, suppress=True)


def rpy_of(R):
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-7:
        return np.array([np.arctan2(R[2, 1], R[2, 2]), p,
                         np.arctan2(R[1, 0], R[0, 0])])
    return np.array([0.0, p, np.arctan2(-R[0, 1], R[1, 1])])


def show(nm, T):
    print(f'{nm:26s} xyz="{T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f}"  '
          f'rpy="{rpy_of(T[:3, :3])[0]:.6f} {rpy_of(T[:3, :3])[1]:.6f} '
          f'{rpy_of(T[:3, :3])[2]:.6f}"')


# measured, in link7 frame (metres)
T_l7_flange = T_from([0.042489, 0.0, -0.0235], [0.0, -1.5707963, 0.0])
Rh = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
T_flange_hbase = np.eye(4)
T_flange_hbase[:3, :3] = Rh
T_flange_hbase[:3, 3] = [-0.0000417, 0.0059617, -0.007158]
# hand urdf's own base -> hand_base_link
T_base_hbase = T_from([0, 0, 0], [-1.57079, 0.0, 3.14159])
T_flange_base = T_flange_hbase @ np.linalg.inv(T_base_hbase)

for x in (0.031, 0.032):
    T_l7_l8 = T_from([x, 0.0, -0.0235], [-1.5707963, 0.0, -1.5707963])
    T_l8_flange = np.linalg.inv(T_l7_l8) @ T_l7_flange
    print(f'--- with joint8 x={x} ---')
    show('link8 -> flange', T_l8_flange)

show('flange -> hand base', T_flange_base)
show('flange -> hand_base_link', T_flange_hbase)
print('\ncheck: recompose link7 -> hand_base_link')
show('link7 -> hand_base_link', T_l7_flange @ T_flange_hbase)
T_chk = T_l7_flange @ T_flange_base @ T_base_hbase
show('  via base link', T_chk)
