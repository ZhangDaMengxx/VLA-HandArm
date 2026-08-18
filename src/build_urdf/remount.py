#!/usr/bin/env python3
"""Recompute mount values with the corrected flange placement (Ry180, z-5mm)."""
import numpy as np

from urdf_fk import T_from

np.set_printoptions(precision=6, suppress=True)


def rpy_of(R):
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-7:
        return np.array([np.arctan2(R[2, 1], R[2, 2]), p,
                         np.arctan2(R[1, 0], R[0, 0])])
    return np.array([0.0, p, np.arctan2(-R[0, 1], R[1, 1])])


def show(nm, T):
    r = rpy_of(T[:3, :3])
    print(f'{nm:28s} xyz="{T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f}"'
          f'  rpy="{r[0]:.6f} {r[1]:.6f} {r[2]:.6f}"')


# assembly frame <- link7  (joint7 axis at z=42.489mm, eps=+1), metres
T_a_l7 = np.eye(4)
T_a_l7[:3, :3] = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
T_a_l7[:3, 3] = [0.0235, 0.0, 0.042489]

# assembly frame <- flange local  (Ry180 + z -5mm), from 48-transform fit
T_a_fl = np.eye(4)
T_a_fl[:3, :3] = np.diag([-1.0, 1.0, -1.0])
T_a_fl[:3, 3] = [0.0, 0.0, -0.005]

# assembly frame <- hand_base_link (palm ICP)
T_a_hb = np.eye(4)
T_a_hb[:3, :3] = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
T_a_hb[:3, 3] = [-0.0000417, 0.0059617, -0.007158]

T_l7_fl = np.linalg.inv(T_a_l7) @ T_a_fl
show('link7 -> flange', T_l7_fl)
for x in (0.031, 0.032):
    T_l7_l8 = T_from([x, 0.0, -0.0235], [-1.5707963, 0.0, -1.5707963])
    show(f'link8(x={x}) -> flange', np.linalg.inv(T_l7_l8) @ T_l7_fl)

T_fl_hb = np.linalg.inv(T_a_fl) @ T_a_hb
show('flange -> hand_base_link', T_fl_hb)
T_b_hb = T_from([0, 0, 0], [-1.57079, 0.0, 3.14159])
show("flange -> hand root 'base'", T_fl_hb @ np.linalg.inv(T_b_hb))
print('\nflange local z of hand mount face (assembly z=-6.996):',
      round(float((np.linalg.inv(T_a_fl) @ np.array([0, 0, -0.006996, 1]))[2])
            * 1000, 3), 'mm')
