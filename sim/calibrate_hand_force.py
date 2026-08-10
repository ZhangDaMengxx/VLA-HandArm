#!/usr/bin/env python3
"""空载力控校准(FORCE_CLB=1009),并验证偏置是否真的归零。要连真手。

**前提:手必须空载且张开。** 校准是把当前读数当零点,手上有东西或手指顶着任何
东西的话,那个力会被记成"零",之后所有力控判据都偏掉,而且看不出来。

⚠ 校准结果是否掉电保存,手册没写。写 SAVE(1005) 会烧 Flash(伤寿命),
   所以这里**不写 SAVE** —— 如果掉电丢失,就每次连上都跑一次校准。
   本脚本会同时报告校准前后的值,方便判断下次上电要不要重跑。

用法:
  python3 sim/calibrate_hand_force.py            # 只看当前偏置,不校准
  python3 sim/calibrate_hand_force.py --run      # 真做校准
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (HAND_JOINTS, InspireHand,  # noqa: E402
                          InspireHandConfig, PROJECT_TO_VENDOR)

OPEN_RAW = 1000
NAMES = ["yaw", "pitch", "index", "middle", "ring", "pinky"]


def read_proj(hand: InspireHand, reg: str, nb: int = 12, fmt: str = "6h",
              tries: int = 14) -> list[int] | None:
    """连续两次读到相同值才采信 —— 单次读失败率实测约 40%,且会拿到旧帧。"""
    prev = None
    for _ in range(tries):
        v = hand.read_regs(reg, nb, fmt)
        if v is not None:
            cur = [int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)]
            if prev == cur:
                return cur
            prev = cur
        time.sleep(0.08)
    return prev


def fmt(vals: list[int] | None) -> str:
    if vals is None:
        return "(read failed)"
    return " ".join(f"{n}={v:5d}" for n, v in zip(NAMES, vals))


def sample_bias(hand: InspireHand, n: int = 20,
                settle: float = 0.0) -> tuple[list[int], int]:
    """采样 FORCE_ACT 直到**读数稳定**,返回 (稳定读数, 最大绝对值)。

    ⚠ 不能只取最后一次:校准刚做完时读数还在漂,4 秒都不够。
    实测校准后立刻采样得到最大偏置 175,再多等一会儿并重新张开后是 10 ——
    前者是瞬态,当成结果会误判"校准没生效"(我第一版就是这么误判的)。
    判据改成"连续 4 次完全相同",空载稳定时实测 20 次读数全同,这个条件不苛刻。
    """
    if settle:
        time.sleep(settle)
    stable_need, same, last = 4, 0, None
    for _ in range(n):
        v = read_proj(hand, "FORCE_ACT")
        if v is not None:
            same = same + 1 if v == last else 0
            last = v
            if same >= stable_need - 1:
                return (v, max(abs(x) for x in v))
        time.sleep(0.2)
    if last is None:
        return ([0] * 6, -1)
    print("  ! 读数一直没稳定下来,下面这个值只是最后一次")
    return (last, max(abs(x) for x in last))


def check_unloaded(hand: InspireHand) -> bool:
    """确认手是张开且空载的。校准前提不满足就别校准。

    判据不能只看角度:手张开但指尖压着桌面也会有力。所以两个都查 ——
    角度接近全开,且电流为零(有负载电机会持续吃电流)。
    """
    ang = read_proj(hand, "ANGLE_ACT")
    cur = read_proj(hand, "CURRENT")
    ok = True
    if ang is None:
        print("  ! 读不到角度,无法确认是否张开")
        ok = False
    else:
        far = [(NAMES[i], v) for i, v in enumerate(ang) if v < OPEN_RAW - 120]
        if far:
            print(f"  ! 这些通道没张开: {far}")
            ok = False
    if cur is not None and any(abs(c) > 20 for c in cur):
        print(f"  ! 电流不为零,可能有负载: {fmt(cur)}")
        ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--run", action="store_true", help="真执行校准")
    ap.add_argument("--force-anyway", action="store_true",
                    help="前提检查不通过也照做(不推荐)")
    args = ap.parse_args()

    hand = InspireHand(InspireHandConfig(port=args.port, mock=False))
    try:
        if not hand.connect():
            print("connect returned False")
            return 2
    except Exception as exc:                      # noqa: BLE001
        print(f"connect failed: {type(exc).__name__}: {exc}")
        return 2

    try:
        print("先张开手,确保空载...")
        hand.set_force(200)
        hand.write_shorts("ANGLE_SET", [OPEN_RAW] * 6)
        time.sleep(2.5)

        print("\n前提检查:")
        ready = check_unloaded(hand)
        if ready:
            print("  空载且张开,可以校准")

        before, before_max = sample_bias(hand)
        print(f"\n校准前 FORCE_ACT: {fmt(before)}")
        print(f"  最大绝对偏置 {before_max}")

        if not args.run:
            print("\n只读模式。加 --run 才真做校准。")
            return 0
        if not ready and not args.force_anyway:
            print("\n前提不满足,拒绝校准(加 --force-anyway 可强制)。")
            return 1

        print("\n写 FORCE_CLB=1...")
        ok = hand.write_byte("FORCE_CLB", 1)
        print(f"  write_byte 返回 {ok}  ← 写指令收不到回复也返回 True,不可信")
        # 校准后重新张开再采样:校准过程本身可能让手指微动,不重新回位会把
        # 那点残余当成偏置。settle 给足 —— 实测 4 秒不够。
        time.sleep(3.0)
        hand.write_shorts("ANGLE_SET", [OPEN_RAW] * 6)
        print("  重新张开并等待读数稳定...")
        after, after_max = sample_bias(hand, settle=3.0)
        print(f"\n校准后 FORCE_ACT: {fmt(after)}")
        print(f"  最大绝对偏置 {after_max}")

        print("\n" + "=" * 56)
        if after_max < 0:
            print("读不到校准后的值,无法判定")
            return 1
        if after_max <= 5:
            print(f"校准生效:偏置 {before_max} -> {after_max}")
        elif after_max < before_max:
            print(f"偏置减小但未归零:{before_max} -> {after_max}")
            print("  (手册没说 FORCE_CLB 能做到多少,这可能就是它的能力上限)")
        else:
            print(f"偏置没改善:{before_max} -> {after_max}")
            print("  FORCE_CLB 可能不是这个用法,或需要 SAVE 才生效(本脚本不写 SAVE)")
    finally:
        hand.write_shorts("ANGLE_SET", [OPEN_RAW] * 6)
        time.sleep(1.2)
        hand.set_force(hand.cfg.init_force)
        hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
