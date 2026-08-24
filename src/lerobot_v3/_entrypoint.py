"""Run a shared implementation under the validated LeRobot v3 runtime."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path


EXPECTED_PYTHON = (3, 12)
EXPECTED_LEROBOT = "0.6.1"
SRC = Path(__file__).resolve().parents[1]


def require_v3_runtime() -> None:
    if sys.version_info[:2] != EXPECTED_PYTHON:
        actual = f"{sys.version_info.major}.{sys.version_info.minor}"
        raise SystemExit(
            f"lerobot_v3 requires Python 3.12, got {actual}. "
            "Run `conda activate lerobot-v3` first."
        )
    try:
        import lerobot
    except ImportError as exc:
        raise SystemExit(
            "lerobot_v3 requires lerobot 0.6.1. "
            "Run `conda activate lerobot-v3` first."
        ) from exc
    actual = getattr(lerobot, "__version__", None)
    if actual != EXPECTED_LEROBOT:
        raise SystemExit(
            f"lerobot_v3 requires lerobot {EXPECTED_LEROBOT}, got {actual or 'unknown'}. "
            "Run `conda activate lerobot-v3` first."
        )


def run_shared(script_name: str) -> None:
    require_v3_runtime()
    script = SRC / script_name
    if not script.is_file():
        raise SystemExit(f"Missing shared implementation: {script}")
    src_text = str(SRC)
    if src_text not in sys.path:
        sys.path.insert(0, src_text)
    runpy.run_path(str(script), run_name="__main__")
