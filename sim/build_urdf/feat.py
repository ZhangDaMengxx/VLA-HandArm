#!/usr/bin/env python3
"""Geometric feature extraction: planar face levels + rasterized hole finding."""
import numpy as np

from stl_probe import load_stl, area as tri_area


def normal_hist(normals, tris, nbin=None):
    """Report dominant face normal directions weighted by area."""
    a = tri_area(tris)
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    keys = np.round(n, 2)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    w = np.bincount(inv, weights=a, minlength=len(uniq))
    order = np.argsort(-w)
    return [(uniq[i], w[i]) for i in order[:nbin] if w[i] > 1.0]


def planes_along(normals, tris, axis=2, tol_n=0.02, tol_d=0.05, min_area=3.0):
    """Find planes whose normal is +-axis; return (sign, offset, area)."""
    a = tri_area(tris)
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    out = []
    for sgn in (+1, -1):
        m = n[:, axis] * sgn > 1 - tol_n
        if not m.any():
            continue
        off = tris[m][:, :, axis].mean(1)
        aa = a[m]
        order = np.argsort(off)
        off, aa = off[order], aa[order]
        start = 0
        for i in range(1, len(off) + 1):
            if i == len(off) or off[i] - off[start] > tol_d:
                seg = slice(start, i)
                tot = aa[seg].sum()
                if tot >= min_area:
                    out.append((sgn, float(np.average(off[seg], weights=aa[seg])),
                                float(tot), int(i - start)))
                start = i
    return sorted(out, key=lambda r: -r[2])


def sample_tris(tris, density=25.0, seed=0):
    """Uniformly sample points on triangles, ~density points per mm^2."""
    rng = np.random.default_rng(seed)
    a = tri_area(tris)
    cnt = np.maximum(1, np.ceil(a * density).astype(int))
    idx = np.repeat(np.arange(len(tris)), cnt)
    u = rng.random(len(idx))
    v = rng.random(len(idx))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = tris[idx]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])
