#!/usr/bin/env python3
"""需求侧:轨迹本身要求多快的关节角速度。**纯离线,不碰硬件。**

为什么要先算这个:在问"臂能不能跟上"之前,得先知道要它跟多快。
供给侧(臂实际能到多少 deg/s、到位要多久)必须实测;需求侧不用,
Capture 的 `exports/workbench/robot_traj.npz` 里已经有全部答案了。

⚠ 时间轴不在 npz 里 —— 只有 (N,7) 角度矩阵。帧率按源视频的 30fps 取,
见 gesture_pack.py:53「30fps 下 stride=1 一秒就 30 帧」。改 --fps 可换算。
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from capture_bundle import discover_trajectory_npz


def analyze(path: Path, fps: float) -> dict:
    d = np.load(path, allow_pickle=True)
    if "arm" not in d.files:
        return {}
    arm = np.asarray(d["arm"], dtype=float)          # (N, 7) rad
    names = [str(s) for s in d["arm_joint_names"]] if "arm_joint_names" in d.files else \
            [f"joint{i+1}" for i in range(arm.shape[1])]
    dt = 1.0 / fps
    # 一阶差分 → rad/s → deg/s。N 帧给出 N-1 个速度样本。
    vel = np.diff(arm, axis=0) / dt * 180.0 / math.pi   # (N-1, 7) deg/s
    acc = np.diff(vel, axis=0) / dt                      # (N-2, 7) deg/s^2
    return {
        "path": path, "n": arm.shape[0], "sec": arm.shape[0] / fps,
        "names": names, "vel": np.abs(vel), "acc": np.abs(acc),
    }


def report(r: dict) -> None:
    print(f"\n=== {r['path'].name}  {r['n']} 帧 / {r['sec']:.1f}s ===")
    v, a = r["vel"], r["acc"]
    print(f"{'关节':<8}{'p50':>9}{'p95':>9}{'max':>9}   {'|加速度 p95':>12}{'max':>10}")
    print(f"{'':8}{'deg/s':>9}{'deg/s':>9}{'deg/s':>9}   {'deg/s²':>12}{'deg/s²':>10}")
    for i, nm in enumerate(r["names"]):
        print(f"{nm:<8}{np.percentile(v[:,i],50):>9.1f}{np.percentile(v[:,i],95):>9.1f}"
              f"{v[:,i].max():>9.1f}   {np.percentile(a[:,i],95):>12.0f}{a[:,i].max():>10.0f}")
    # 整体上界:任一关节任一时刻的最大需求,决定"最慢的那一关节能不能跟上"
    print(f"{'—全体—':<8}{np.percentile(v,50):>9.1f}{np.percentile(v,95):>9.1f}"
          f"{v.max():>9.1f}   {np.percentile(a,95):>12.0f}{a.max():>10.0f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fps", type=float, default=30.0,
                    help="源轨迹帧率(npz 里没有时间列,只能外部给)")
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle；不传则读取 datasets/captures/ 中最新一次")
    ap.add_argument("--legacy-out", action="store_true", help="显式扫描旧 src/out")
    ap.add_argument("files", nargs="*", help="显式轨迹路径；默认扫描当前 Capture")
    args = ap.parse_args()

    try:
        paths = [Path(f) for f in args.files] or discover_trajectory_npz(
            capture_root=args.capture_root,
            legacy_out=args.legacy_out,
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"没找到 npz：{exc}")
        return 1
    if not paths:
        print("没找到 npz。先跑 derive_embodiment.py --emit-traj")
        return 1

    allv = []
    for p in paths:
        r = analyze(p, args.fps)
        if not r:
            print(f"跳过 {p.name}(没有 arm 字段)")
            continue
        report(r)
        allv.append(r["vel"])

    if allv:
        v = np.concatenate(allv, axis=0)
        print(f"\n### 跨全部轨迹:p50={np.percentile(v,50):.1f}  "
              f"p95={np.percentile(v,95):.1f}  max={v.max():.1f} deg/s")
        print("### 供给侧要实测的就是这三个数对应的档位能不能跟上 —— "
              "见 ARM_DEBUG.md「供给侧实测」")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
