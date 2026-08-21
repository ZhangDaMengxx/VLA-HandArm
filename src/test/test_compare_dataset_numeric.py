from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

import compare_dataset_numeric as numeric_compare  # noqa: E402
from compare_dataset_numeric import _numeric_array  # noqa: E402


class FakeSeries:
    def __init__(self, values):
        self._values = np.asarray(values, dtype=object)
        self.iloc = self

    def __len__(self):
        return len(self._values)

    def __getitem__(self, index):
        return self._values[index]

    def to_numpy(self, dtype=None):
        return np.asarray(list(self._values), dtype=dtype)


class FakeFrame:
    def __init__(self, **columns):
        self._columns = {name: FakeSeries(values) for name, values in columns.items()}
        self.columns = list(self._columns)

    def __len__(self):
        return len(next(iter(self._columns.values())))

    def __getitem__(self, name):
        return self._columns[name]


def test_numeric_array_preserves_vector_values() -> None:
    values = FakeSeries([np.array([1.0, 2.0]), np.array([3.0, np.nan])])
    result = _numeric_array(values)
    np.testing.assert_allclose(result, [[1.0, 2.0], [3.0, np.nan]], equal_nan=True)


def test_numeric_array_skips_strings() -> None:
    assert _numeric_array(FakeSeries(["task a", "task b"])) is None


def test_compare_rejects_a_missing_numeric_column(monkeypatch, tmp_path: Path) -> None:
    left = FakeFrame(frame_index=[0, 1], value=[1.0, 2.0])
    right = FakeFrame(frame_index=[0, 1])
    monkeypatch.setattr(
        numeric_compare,
        "_load_frames",
        lambda root: left if root.name == "left" else right,
    )
    with pytest.raises(AssertionError, match="numeric column sets differ"):
        numeric_compare.compare_numeric(tmp_path / "left", tmp_path / "right")
