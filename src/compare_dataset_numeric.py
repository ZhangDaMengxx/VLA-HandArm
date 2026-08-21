"""Compare numeric Parquet content of two LeRobotDataset roots.

Path, manifest, checksum, and other metadata may differ. Numeric frame columns
must remain equal during a storage-only migration.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def _load_frames(root: Path):
    import pandas as pd

    files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
    if not files:
        raise FileNotFoundError(f"no data parquet under {root}")
    frame = pd.concat([pd.read_parquet(path) for path in files], ignore_index=True)
    order = [name for name in ("episode_index", "frame_index", "index") if name in frame.columns]
    return frame.sort_values(order).reset_index(drop=True) if order else frame.reset_index(drop=True)


def _numeric_array(series) -> np.ndarray | None:
    if len(series) == 0:
        return np.empty((0,), dtype=np.float64)
    first = series.iloc[0]
    if np.isscalar(first) and not isinstance(first, (str, bytes, bool)):
        try:
            return series.to_numpy(dtype=np.float64)
        except (TypeError, ValueError):
            return None
    try:
        values = np.stack(series.to_numpy())
    except (TypeError, ValueError):
        return None
    try:
        return values.astype(np.float64)
    except (TypeError, ValueError):
        return None


def compare_numeric(left_root: Path, right_root: Path, atol: float = 0.0) -> dict:
    left = _load_frames(left_root)
    right = _load_frames(right_root)
    if len(left) != len(right):
        raise AssertionError(f"frame count differs: {len(left)} != {len(right)}")
    left_numeric = {
        column: values
        for column in left.columns
        if (values := _numeric_array(left[column])) is not None
    }
    right_numeric = {
        column: values
        for column in right.columns
        if (values := _numeric_array(right[column])) is not None
    }
    if left_numeric.keys() != right_numeric.keys():
        left_only = sorted(left_numeric.keys() - right_numeric.keys())
        right_only = sorted(right_numeric.keys() - left_numeric.keys())
        raise AssertionError(
            f"numeric column sets differ: left_only={left_only}, right_only={right_only}"
        )
    compared = []
    worst = 0.0
    for column in sorted(left_numeric):
        left_values = left_numeric[column]
        right_values = right_numeric[column]
        if left_values.shape != right_values.shape:
            raise AssertionError(
                f"{column} shape differs: {left_values.shape} != {right_values.shape}"
            )
        delta = np.abs(left_values - right_values)
        finite_delta = delta[np.isfinite(delta)]
        column_worst = float(finite_delta.max()) if finite_delta.size else 0.0
        same_nan = np.array_equal(np.isnan(left_values), np.isnan(right_values))
        if not same_nan or not np.allclose(left_values, right_values, rtol=0.0, atol=atol, equal_nan=True):
            raise AssertionError(f"{column} differs; max_abs_delta={column_worst}")
        compared.append(column)
        worst = max(worst, column_worst)
    if not compared:
        raise AssertionError("no common numeric frame columns")
    return {
        "frames": len(left),
        "numeric_columns": compared,
        "max_abs_delta": worst,
        "atol": atol,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("left", type=Path)
    ap.add_argument("right", type=Path)
    ap.add_argument("--atol", type=float, default=0.0)
    args = ap.parse_args()
    result = compare_numeric(args.left, args.right, args.atol)
    print(
        f"numeric datasets match: frames={result['frames']} "
        f"columns={len(result['numeric_columns'])} max_abs_delta={result['max_abs_delta']:.3g}"
    )


if __name__ == "__main__":
    main()
