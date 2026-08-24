#!/usr/bin/env python3
"""Run the Web workbench from the validated Python 3.12 main runtime."""
import importlib.util

from _entrypoint import run_shared


def require_websocket_transport() -> None:
    """Fail loudly instead of serving a page whose realtime 3D silently stalls."""
    if importlib.util.find_spec("websockets") is None:
        raise SystemExit(
            "lerobot_v3 Web requires websockets. Run "
            "`python -m pip install -r environment/lerobot-v3-dataset.txt`."
        )


if __name__ == "__main__":
    require_websocket_transport()
    run_shared("app_web.py")
