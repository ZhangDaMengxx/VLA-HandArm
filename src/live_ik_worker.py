#!/usr/bin/env python3
"""Persistent NERO FK/IK worker for the Web real-time tracking session.

The Web process may run in a lightweight environment without Pinocchio.  This
worker is launched with ``LEROBOT_PY`` and speaks one JSON object per line.
"""
from __future__ import annotations

import json
import sys

import numpy as np

from nero_kin import NeroKin
from robot_specs import NERO_INSPIRE_RGB


def emit(value: dict) -> None:
    print(json.dumps(value, ensure_ascii=False, separators=(",", ":")), flush=True)


def main() -> None:
    spec = NERO_INSPIRE_RGB
    kin = NeroKin(spec.arm_urdf, ee_frame=spec.ee_frame)
    previous = spec.q_home.copy()
    emit({"ok": True, "type": "ready", "q_home": previous.tolist()})
    for line in sys.stdin:
        try:
            request = json.loads(line)
            command = request.get("cmd")
            if command == "close":
                return
            if command == "fk":
                q = np.asarray(request.get("q"), dtype=np.float64)
                if q.shape != (7,) or not np.all(np.isfinite(q)):
                    raise ValueError("fk requires seven finite joint angles")
                previous = q.copy()
                emit({"ok": True, "pose": kin.fk(q).reshape(-1).tolist()})
                continue
            if command == "solve":
                target = np.asarray(request.get("target_pose"), dtype=np.float64)
                if target.size != 16 or not np.all(np.isfinite(target)):
                    raise ValueError("solve requires a finite 4x4 target_pose")
                q, ok = kin.ik(
                    target.reshape(4, 4), previous, iters=80, eps=2e-4,
                    q_rest=spec.q_home, k_null=spec.k_null,
                )
                if ok:
                    previous = q
                emit({"ok": True, "ik_ok": bool(ok), "q": q.tolist()})
                continue
            raise ValueError(f"unknown command: {command}")
        except Exception as error:  # noqa: BLE001
            emit({"ok": False, "error": str(error)})


if __name__ == "__main__":
    main()
