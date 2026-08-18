#!/usr/bin/env python3
"""Cylinder-axis detection by surface-normal voting (axes parallel to one axis)."""
import numpy as np

from stl_probe import area as tri_area


def cyl_axes(tris, normals, axis=0, smin=1.5, smax=32.0, ds=0.25,
             cell=0.25, ntol=0.25, bounds=None, npeak=15, nms=3.0):
    """Vote along surface normals; peaks = cylinder axes parallel to `axis`."""
    ii = [k for k in range(3) if k != axis]
    n = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    m = np.abs(n[:, axis]) < ntol
    if bounds is not None:
        c3 = tris.mean(1)
        for k, (lo, hi) in enumerate(bounds):
            m &= (c3[:, k] >= lo) & (c3[:, k] <= hi)
    if m.sum() == 0:
        return [], None
    w = tri_area(tris)[m]
    c = tris[m].mean(1)[:, ii]
    nn = n[m][:, ii]
    nn = nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12)
    lo = c.min(0) - smax - 1
    hi = c.max(0) + smax + 1
    shape = (np.ceil((hi - lo) / cell).astype(int) + 1)
    acc = np.zeros(shape, dtype=np.float32)
    for s in np.arange(smin, smax + 1e-9, ds):
        for sg in (+1.0, -1.0):
            p = c + (sg * s) * nn
            gi = np.floor((p - lo) / cell).astype(int)
            ok = np.all((gi >= 0) & (gi < shape), axis=1)
            np.add.at(acc, (gi[ok, 0], gi[ok, 1]), w[ok])
    peaks = []
    flat = np.argsort(acc.ravel())[::-1]
    for f in flat:
        if len(peaks) >= npeak:
            break
        gy, gz = np.unravel_index(f, acc.shape)
        p = lo + (np.array([gy, gz]) + 0.5) * cell
        if any(np.linalg.norm(p - q) < nms for q, _ in peaks):
            continue
        peaks.append((p, float(acc[gy, gz])))
    return peaks, (c, nn, w)


def radii_at(c, axis_pt, w, lo=0.0, hi=40.0, nb=80):
    d = np.linalg.norm(c - axis_pt, axis=1)
    h, e = np.histogram(d, bins=nb, range=(lo, hi), weights=w)
    return h, e
