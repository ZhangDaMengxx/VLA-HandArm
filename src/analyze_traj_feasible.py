#!/usr/bin/env python3
"""src/analyze_traj_feasible.py — 轨迹可行域诊断(纯离线,不碰硬件)。

第 0 步在 TrajectoryBackend 加载期拦下 replay_rgb_demo 的 78 帧后,这个脚本
回答"然后呢":

  1. 越界帧落在可行域表的哪个区?域内(实测过)还是钳制区(T≤300,表在外插)?
     —— 决定"78"这个数字可信到什么程度。
  2. 越界多深(raw counts)?浅 = 平滑残差,深 = 真实手势被原样放过。
  3. 成簇还是散布?连续段 = 动捕里真有那个动作,孤立点 = 滤波毛刺。
  4. 通过的那条轨迹是真安全,还是压根没靠近边界?
     —— "通过"如果只是因为没走到危险区,它不构成"判据正确"的证据。

判据与 hand_pose.check_feasible 完全一致(同一张 FEASIBLE 表、同一个 max() 压
一维),所以这里的结论直接解释第 0 步的行为,不引入第二套标准。

⚠ 帧间穿越查不了 —— 那要第 4 步的路径检查器。这里只看每一帧的终点。

用法:
    /usr/bin/python3 src/analyze_traj_feasible.py              # 全部 robot_traj_*.npz
    /usr/bin/python3 src/analyze_traj_feasible.py --npz <path> # 只看一个
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "skills"))
from hand_pose import (FEASIBLE, HAND_JOINTS, INDEX, index_min_raw,  # noqa: E402
                       rad_to_raw)
sys.path.insert(0, str(REPO / "src"))
from capture_bundle import discover_trajectory_npz  # noqa: E402

# 与 backend.HAND_NAMES 同序 —— 靠名字对齐,不靠列位置
HAND_NAMES = list(HAND_JOINTS)
YAW, PITCH = 0, 1
T_EDGE = FEASIBLE[0][0]          # 300:表的最小实测点,以下全是钳制外插


def load_hand6(path: Path) -> np.ndarray:
    """npz → (N,6) 驱动关节弧度,按 HAND_NAMES 顺序。夹爪本体抛 ValueError。"""
    d = np.load(path, allow_pickle=True)
    if "hand" not in d.files or "hand_joint_names" not in d.files:
        raise ValueError("npz 缺 hand / hand_joint_names")
    hand_all = np.asarray(d["hand"], dtype=float)
    names = [str(x) for x in d["hand_joint_names"]]
    miss = [n for n in HAND_NAMES if n not in names]
    if miss:
        raise ValueError(f"缺驱动关节 {miss}(实有 {len(names)} 个,可能是夹爪本体)")
    return hand_all[:, [names.index(n) for n in HAND_NAMES]]


def per_frame(hand6: np.ndarray) -> dict:
    """逐帧算 T / 食指下界 / 余量。余量 <0 = 不可行,数值就是差多少 raw。"""
    n = len(hand6)
    raws = np.empty((n, 6), dtype=int)
    for i in range(n):
        raws[i] = [rad_to_raw(nm, r) for nm, r in zip(HAND_NAMES, hand6[i])]
    T = np.maximum(raws[:, YAW], raws[:, PITCH])
    lo = np.array([index_min_raw(int(a), int(b))
                   for a, b in zip(raws[:, YAW], raws[:, PITCH])])
    return {"raws": raws, "T": T, "lo": lo, "margin": raws[:, INDEX] - lo}


def runs(idx: np.ndarray) -> list[tuple[int, int]]:
    """连续帧号 → [(起, 止)] 闭区间。用来区分"成簇"和"散布"。"""
    if not len(idx):
        return []
    out, s = [], idx[0]
    for a, b in zip(idx, idx[1:]):
        if b != a + 1:
            out.append((int(s), int(a)))
            s = b
    out.append((int(s), int(idx[-1])))
    return out


def report(path: Path) -> bool:
    """打印一份诊断。返回 True 表示这条轨迹全帧终点可行。"""
    print(f"\n{'=' * 72}")
    try:
        hand6 = load_hand6(path)
    except (ValueError, KeyError) as e:
        print(f"{path.name}\n  跳过: {e}")
        return True
    m = per_frame(hand6)
    raws, T, lo, margin = m["raws"], m["T"], m["lo"], m["margin"]
    n = len(margin)
    bad = np.where(margin < 0)[0]
    print(f"{path.name}   {n} 帧")
    print(f"  终点不可行 {len(bad)} 帧 ({100.0 * len(bad) / n:.1f}%)")

    # ---- 这张表在这条轨迹上适用吗 ----
    clamped = int((T <= T_EDGE).sum())
    print(f"\n  [1] 可行域表的适用性   实测点 {FEASIBLE}")
    print(f"    T ≤ {T_EDGE}(表外插,下界恒取 {FEASIBLE[0][1]}): "
          f"{clamped}/{n} 帧 ({100.0 * clamped / n:.1f}%)")
    print(f"    T 实际范围 [{int(T.min())}, {int(T.max())}]  中位 {int(np.median(T))}")
    if len(bad):
        cb = int((T[bad] <= T_EDGE).sum())
        print(f"    不可行帧落在外插区: {cb}/{len(bad)}")
        if cb == len(bad):
            print(f"    ⚠ 全部不可行帧都在表的域外。「{len(bad)} 帧」是拿钳制常数")
            print(f"      {FEASIBLE[0][1]} 算的,不是实测判据 —— 真实帧数待 t5 标定后重算。")

    # ---- 越界多深 ----
    if len(bad):
        d = -margin[bad]
        print(f"\n  [2] 越界深度(raw counts,越大越深)")
        print(f"    min {int(d.min())}  中位 {int(np.median(d))}  "
              f"max {int(d.max())} @帧 {int(bad[np.argmax(d)])}")
        deep = int((d > 100).sum())
        print(f"    >100 counts(真实手势级,不是滤波毛刺): {deep}/{len(bad)} 帧")

    # ---- 成簇还是散布 ----
    if len(bad):
        seg = runs(bad)
        lens = [b - a + 1 for a, b in seg]
        print(f"\n  [3] 时间结构   {len(seg)} 段,长度 min {min(lens)} / "
              f"max {max(lens)} / 合计 {sum(lens)}")
        iso = sum(1 for L in lens if L == 1)
        for a, b in seg[:8]:
            w = np.arange(a, b + 1)
            print(f"    帧 {a:4d}-{b:<4d} ({b - a + 1:2d} 帧)  "
                  f"T {int(T[w].min())}-{int(T[w].max())}  "
                  f"食指 {int(raws[w, INDEX].min())}-{int(raws[w, INDEX].max())}  "
                  f"最深 {int((-margin[w]).max())}")
        if len(seg) > 8:
            print(f"    …还有 {len(seg) - 8} 段")
        print(f"    孤立单帧 {iso}/{len(seg)} 段 —— "
              f"{'多为连续段,动捕里真有这个动作' if iso < len(seg) / 2 else '多为孤立点,像滤波毛刺'}")

    # ---- 通过是否构成证据 ----
    if not len(bad):
        mn, i_mn = int(margin.min()), int(np.argmin(margin))
        print(f"\n  [4] 最小余量 {mn} raw @帧 {i_mn}"
              f"(T={int(T[i_mn])} 下界={int(lo[i_mn])} 食指={int(raws[i_mn, INDEX])})")
        if mn > 200:
            print(f"    ⚠ 余量始终 >200 raw —— 这条轨迹压根没靠近边界。")
            print(f"      「通过」只说明它没走到危险区,不构成「判据正确」的证据。")
    return len(bad) == 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", type=Path, default=None, help="只看这一个文件")
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle；不传则读取 datasets/captures/ 中最新一次")
    ap.add_argument("--legacy-out", action="store_true", help="显式扫描旧 src/out")
    args = ap.parse_args()

    if args.npz:
        paths = [args.npz]
        if not args.npz.exists():
            print(f"✗ 文件不存在: {args.npz}")
            return 1
    else:
        try:
            paths = discover_trajectory_npz(
                capture_root=args.capture_root,
                legacy_out=args.legacy_out,
            )
        except (FileNotFoundError, ValueError) as exc:
            print(f"✗ {exc}")
            return 1
        if not paths:
            print("✗ 当前 Capture 中没有 robot_traj.npz")
            return 1

    ok = [report(p) for p in paths]
    print(f"\n{'=' * 72}")
    print(f"{sum(ok)}/{len(ok)} 条轨迹全帧终点可行")
    print("终点可行 ≠ 路径可行 —— 帧间穿越要第 4 步的路径检查器才看得见。")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
