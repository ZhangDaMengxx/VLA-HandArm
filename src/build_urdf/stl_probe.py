#!/usr/bin/env python3
"""Probe binary STL files: bbox, centroid, triangle stats. Read-only analysis."""
import struct
import sys
import numpy as np


def load_stl(path):
    with open(path, 'rb') as fh:
        fh.read(80)
        n = struct.unpack('<I', fh.read(4))[0]
        raw = np.frombuffer(fh.read(n * 50), dtype=np.uint8)
    if raw.size != n * 50:
        raise ValueError(f'{path}: short read')
    blk = raw.reshape(n, 50)
    floats = blk[:, :48].copy().view('<f4').reshape(n, 4, 3)
    normals = floats[:, 0, :]
    tris = floats[:, 1:4, :]
    return normals.astype(np.float64), tris.astype(np.float64)


def area(tris):
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    return 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)


def report(path):
    normals, tris = load_stl(path)
    v = tris.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    a = area(tris)
    print(f'--- {path}')
    print(f'  ntri   : {len(tris)}')
    print(f'  bbox lo: {np.round(lo, 4)}')
    print(f'  bbox hi: {np.round(hi, 4)}')
    print(f'  size   : {np.round(hi - lo, 4)}')
    print(f'  vmean  : {np.round(v.mean(0), 4)}')
    print(f'  area   : {a.sum():.4f}  (tri area min/med/max '
          f'{a.min():.3e}/{np.median(a):.3e}/{a.max():.3e})')
    return normals, tris


if __name__ == '__main__':
    for p in sys.argv[1:]:
        report(p)
