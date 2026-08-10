#!/usr/bin/env python3
"""Coarse-to-fine rigid registration: 24 axis rotations + trimmed ICP."""
import itertools

import numpy as np
from scipy.spatial import cKDTree


def axis_rotations():
    """All 24 right-handed axis-aligned rotation matrices."""
    out = []
    for perm in itertools.permutations(range(3)):
        for sx, sy, sz in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            R[0, perm[0]] = sx
            R[1, perm[1]] = sy
            R[2, perm[2]] = sz
            if abs(np.linalg.det(R) - 1) < 1e-9:
                out.append(R)
    return out


def kabsch(P, Q):
    """Rigid transform mapping P onto Q (both N x 3)."""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    return R, qc - R @ pc


def trimmed_icp(src, tgt_tree, R0, t0, iters=60, keep=0.75):
    R, t = R0.copy(), t0.copy()
    rmse = np.inf
    for _ in range(iters):
        p = (R @ src.T).T + t
        d, idx = tgt_tree.query(p, workers=-1)
        thr = np.quantile(d, keep)
        m = d <= thr
        rmse_new = float(np.sqrt((d[m] ** 2).mean()))
        Rn, tn = kabsch(src[m], tgt_tree.data[idx[m]])
        R, t = Rn, tn
        if abs(rmse - rmse_new) < 1e-7:
            rmse = rmse_new
            break
        rmse = rmse_new
    p = (R @ src.T).T + t
    d, _ = tgt_tree.query(p, workers=-1)
    return R, t, rmse, float(np.quantile(d, keep)), float(d.mean())


def fixed_r_icp(src, tgt, R, t0, iters=80, keep=0.7):
    """Trimmed ICP with the rotation held fixed; only translation is fitted."""
    tree = cKDTree(tgt)
    t = np.asarray(t0, float).copy()
    for _ in range(iters):
        p = (R @ src.T).T + t
        d, idx = tree.query(p, workers=-1)
        m = d <= np.quantile(d, keep)
        dt = (tree.data[idx[m]] - p[m]).mean(0)
        t = t + dt
        if np.linalg.norm(dt) < 1e-7:
            break
    p = (R @ src.T).T + t
    d, _ = tree.query(p, workers=-1)
    m = d <= np.quantile(d, keep)
    return t, float(np.sqrt((d[m] ** 2).mean())), float(d.mean())


def register(src, tgt, n_src=6000, seed=0, keep=0.75, rots=None):
    rng = np.random.default_rng(seed)
    s = src[rng.choice(len(src), min(n_src, len(src)), replace=False)]
    tree = cKDTree(tgt)
    best = None
    for R0 in (rots if rots is not None else axis_rotations()):
        t0 = tgt.mean(0) - R0 @ s.mean(0)
        R, t, rmse, q, md = trimmed_icp(s, tree, R0, t0, keep=keep)
        if best is None or rmse < best[2]:
            best = (R, t, rmse, q, md)
    return best
