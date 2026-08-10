#!/usr/bin/env python3
"""测试哪些自接触手势能真正接触到（STATUS=3），哪些因硬件限制够不着。

拇指只能到物理行程 86%，远端关节耦合有 11° 误差。这些会让部分手势做不到接触。
必须实测才知道边界在哪——别靠想象设计手势清单。

测法：指令给到接触点**之后**，设中等力控（500g），监控 STATUS 3-5 秒：
  - STATUS=3 → 成功接触
  - STATUS=2 → 够不着，或者力控设太高、接触了但没触发阈值
  - STATUS=1 持续 → 在动但没到位，可能卡住了

每个动作结束都回张开位，避免下一个测试继承上一个的状态。

用法:
  python3 sim/test_hand_contact.py
  python3 sim/test_hand_contact.py --gesture fist  # 只测一个
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (HAND_JOINTS, HAND_LIMITS, InspireHand,  # noqa: E402
                          InspireHandConfig)

# 测试手势：名字 → (目标角度rad 6个, 描述)
# 角度**故意给到接触点之后**，让力控停住它。全按 HAND_LIMITS 上限的 90% 给。
GESTURES = {
    "fist": (
        [1.0, 0.54, 1.3, 1.3, 1.3, 1.3],  # 拇指两关节都高，四指全合
        "握拳：五指全合，拇指压在其他手指或掌心上"
    ),
    "pinch": (
        [0.8, 0.54, 1.3, 0.0, 0.0, 0.0],  # 拇指+食指合，其余张开
        "捏（自己）：拇指尖和食指尖对压"
    ),
    "ok_sign": (
        [0.9, 0.50, 1.0, 0.0, 0.0, 0.0],  # 拇指+食指形成圆环，其余伸直
        "OK手势：拇指尖碰食指第一关节内侧"
    ),
    "thumbs_up": (
        [0.0, 0.0, 1.3, 1.3, 1.3, 1.3],   # 拇指伸直，其余四指握拳
        "竖拇指：拇指伸直顶在握紧的四指上"
    ),
}

STATUS_NAMES = {
    0: "松开中", 1: "抓取中", 2: "位置到位", 3: "力控到位",
    5: "电流保护", 6: "堵转", 7: "故障", 255: "未收到指令"
}


def wait_and_monitor(hand: InspireHand, duration: float, desc: str) -> dict:
    """监控 STATUS/TEMP 若干秒，返回最终状态和过程摘要。"""
    t0, samples = time.time(), []
    status_seen = set()
    while time.time() - t0 < duration:
        telem = hand.telemetry()
        if telem.get("mock"):
            return {"final_status": [2] * 6, "temps": [30] * 6,
                    "status_seen": {2}, "max_temp": 30, "mock": True}
        st, temp = telem.get("status"), telem.get("temp")
        if st and temp:
            samples.append((time.time() - t0, st, temp))
            status_seen.update(st)
        time.sleep(0.1)
    if not samples:
        return {"error": "读不到遥测"}
    final_t, final_st, final_temp = samples[-1]
    return {
        "final_status": final_st,
        "temps": final_temp,
        "status_seen": status_seen,
        "max_temp": max(max(s[2]) for s in samples),
        "samples": len(samples),
        "duration": final_t,
    }


def test_gesture(hand: InspireHand, name: str, angles: list[float], desc: str,
                 force: int = 500) -> dict:
    """测一个手势：设力控 → 发角度 → 监控 → 回张开位。"""
    print(f"\n{'='*60}")
    print(f"测试: {name}")
    print(f"描述: {desc}")
    print(f"目标角度(rad): {[f'{a:.2f}' for a in angles]}")
    print(f"力控阈值: {force}g")

    # 先回张开位，清掉上一个动作的状态
    hand.set_force(300)  # 回位用低力，避免之前有东西卡着
    hand.set_angles([0.0] * 6)
    time.sleep(1.5)

    # 设力控并发目标角度
    hand.set_force(force)
    time.sleep(0.2)
    ok = hand.set_angles(angles)
    if not ok:
        return {"error": "set_angles 失败", "name": name}

    # 监控 3 秒
    print("  监控中...", end="", flush=True)
    result = wait_and_monitor(hand, 3.0, desc)
    result["name"] = name
    result["desc"] = desc
    result["force_set"] = force

    if result.get("mock"):
        print(" (mock 模式，跳过)")
        return result
    if result.get("error"):
        print(f" {result['error']}")
        return result

    # 判定
    st = result["final_status"]
    st_counts = {s: st.count(s) for s in set(st)}
    has_contact = 3 in result["status_seen"]
    final_all_contact = all(s == 3 for s in st)
    final_all_reached = all(s == 2 for s in st)

    result["verdict"] = "unknown"
    if final_all_contact:
        result["verdict"] = "success_contact"
        print(f" ✓ 全部力控到位 (STATUS=3)")
    elif has_contact:
        result["verdict"] = "partial_contact"
        print(f" ⚠ 部分接触: STATUS {st_counts}")
    elif final_all_reached:
        result["verdict"] = "unreachable"
        print(f" ✗ 位置到位但未接触 (STATUS=2) —— 够不着")
    else:
        result["verdict"] = "stuck"
        print(f" ✗ 未到位: STATUS {st_counts}")

    print(f"  温度: {result['temps']} (最高 {result['max_temp']}°C)")
    print(f"  STATUS 历史: {sorted(result['status_seen'])}")

    # 回张开位
    hand.set_force(300)
    hand.set_angles([0.0] * 6)
    time.sleep(1.0)
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gesture", choices=list(GESTURES.keys()),
                    help="只测一个手势")
    ap.add_argument("--force", type=int, default=500,
                    help="力控阈值 g (默认 500)")
    args = ap.parse_args()

    hand = InspireHand(InspireHandConfig(port="/dev/ttyUSB0", mock=False))
    if not hand.connect():
        print(f"连不上: {hand.last_error}")
        return 2

    try:
        # 初始遥测
        telem = hand.telemetry()
        if not telem.get("mock"):
            print(f"初始温度: {telem.get('temp')}")
            print(f"初始 STATUS: {telem.get('status')}")

        # 跑测试
        todo = {args.gesture: GESTURES[args.gesture]} if args.gesture else GESTURES
        results = []
        for name, (angles, desc) in todo.items():
            r = test_gesture(hand, name, angles, desc, args.force)
            results.append(r)
            if r.get("max_temp", 0) > 50:
                print(f"\n⚠ 温度 {r['max_temp']}°C 超过 50，暂停 10 秒降温")
                time.sleep(10)

        # 汇总
        print(f"\n{'='*60}")
        print("汇总:")
        for r in results:
            if r.get("mock") or r.get("error"):
                continue
            v = r["verdict"]
            mark = "✓" if v == "success_contact" else "⚠" if v == "partial_contact" else "✗"
            print(f"  {mark} {r['name']:12s} {v:20s} 最高温度 {r['max_temp']}°C")

        success = [r for r in results if r.get("verdict") == "success_contact"]
        print(f"\n完全接触: {len(success)}/{len(results)}")
        print("\n能注册为 hand_grasp 的手势:")
        for r in success:
            print(f"  - {r['name']}: {r['desc']}")

        if any(r.get("verdict") == "unreachable" for r in results):
            print("\n够不着的手势(硬件限制，不要注册):")
            for r in results:
                if r.get("verdict") == "unreachable":
                    print(f"  - {r['name']}: {r['desc']}")

    finally:
        hand.set_force(hand.cfg.init_force)
        hand.set_angles([0.0] * 6)
        hand.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
