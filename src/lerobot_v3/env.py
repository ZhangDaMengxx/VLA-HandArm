"""Locate the named Python 3.12 + LeRobot 0.6.1 offline environment."""
from __future__ import annotations

import os
import sys
from pathlib import Path


ENV_NAME = "lerobot-v3"
PREFERRED_PYTHON = "3.12"
REPO = Path(__file__).resolve().parents[2]


def _prefix_candidates() -> list[Path]:
    home = Path.home()
    candidates: list[Path] = []
    if base := os.environ.get("CONDA_BASE"):
        candidates.append(Path(base) / "envs" / ENV_NAME)
    candidates += [
        home / "miniconda3/envs" / ENV_NAME,
        home / "anaconda3/envs" / ENV_NAME,
        home / "miniforge3/envs" / ENV_NAME,
        home / "mambaforge/envs" / ENV_NAME,
        Path("/opt/conda/envs") / ENV_NAME,
        REPO / ".envs" / ENV_NAME,
    ]
    return candidates


def lerobot_v3_prefix() -> Path | None:
    if value := os.environ.get("LEROBOT_V3_PREFIX"):
        return Path(value)
    if value := os.environ.get("LEROBOT_V3_PY"):
        prefix = Path(value).resolve().parent.parent
        if prefix.is_dir():
            return prefix
    return next((path for path in _prefix_candidates() if (path / "bin").is_dir()), None)


def lerobot_v3_python() -> str:
    if value := os.environ.get("LEROBOT_V3_PY"):
        if not Path(value).is_file():
            print(
                f"[lerobot_v3.env] warn: $LEROBOT_V3_PY does not exist: {value}",
                file=sys.stderr,
                flush=True,
            )
        return value
    prefix = lerobot_v3_prefix()
    if prefix is not None:
        for name in (f"python{PREFERRED_PYTHON}", "python3", "python"):
            executable = prefix / "bin" / name
            if executable.is_file():
                return str(executable)
    print(
        "[lerobot_v3.env] warn: named environment 'lerobot-v3' was not found; "
        "falling back to python3. Set LEROBOT_V3_PY explicitly if needed.",
        file=sys.stderr,
        flush=True,
    )
    return "python3"


def _main() -> int:
    option = (sys.argv[1:] or ["--python"])[0]
    if option == "--python":
        print(lerobot_v3_python())
        return 0
    if option == "--prefix":
        prefix = lerobot_v3_prefix()
        if prefix is None:
            return 1
        print(prefix)
        return 0
    print(f"Usage: {sys.argv[0]} [--python|--prefix]", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(_main())
