#!/usr/bin/env python3
"""验证 FORCE_SET 到底吃不吃 >1000 的值(拇指 1500)。**要连真手。**

为什么必须验:手册两个寄存器矛盾,而我们按大的那个改了代码。
    FORCE_SET(1498)          运行期写的就是它 —— 手册标 0-1000,全通道
    DEFAULT_FORCE_SET(1044)  上电默认值      —— 手册标 0-1000,拇指 0-1500
inspire_hand.FORCE_MAX 现在给拇指 1500,属于**超手册量程使用**。三种可能都要能分辨:

    ① 照收        读回 1500  → 改动有效,FORCE_MAX 保持
    ② 内部夹取    读回 1000  → 改动无效,把 FORCE_MAX 的拇指改回 1000
    ③ 拒绝整帧    读回旧值    → **危险**,见下

③ 最要紧:它意味着"想给拇指加力"会让**六个通道全部没设上**,而代码看不出来 ——
write_shorts 对写指令收不到回复也返回 True(手对写偶发不回但动作照做,那里返回
False 会让上层误判掉线)。所以判据只能是**逐通道读回对照**,不能看返回值。

⚠ 跑 --write 会真的改握持力。手上不要有东西。力控不断电保存,脚本结束恢复 init_force。

用法:
  python3 sim/verify_force_limits.py                    # 只读当前值,不写
  python3 sim/verify_force_limits.py --write             # 写 1500 再读回对照
  python3 sim/verify_force_limits.py --write --port /dev/ttyUSB0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (FORCE_MAX, HAND_JOINTS, InspireHand,   # noqa: E402
                          InspireHandConfig, PROJECT_TO_VENDOR)

FAILED: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    tag = "PASS" if cond else "FAIL"
    print(f"  [{tag}] {name}" + (f" — {detail}" if detail else ""))
    if not cond:
        FAILED.append(name)
    return cond


def read_force(hand: InspireHand) -> list[int] | None:
    """读 FORCE_SET 回来,转成**项目顺序**。

    注意读的是 FORCE_SET(设定值)不是 FORCE_ACT(实测力):要验的是"设定有没有生效",
    不是"手在使多大力"。FORCE_ACT 空载有负偏置,拿它验设定值只会看到噪声。
    """
    raw = hand.read_regs("FORCE_SET", 12, "6h")
    if raw is None:
        return None
    # 厂商顺序 → 项目顺序。PROJECT_TO_VENDOR 自逆,但**显式反向索引**,不靠这个巧合。
    return [int(raw[PROJECT_TO_VENDOR[i]]) for i in range(6)]


def show(label: str, vals: list[int] | None) -> None:
    if vals is None:
        print(f"  {label}: 读不到")
        return
    print(f"  {label}:")
    for name, v in zip(HAND_JOINTS, vals):
        cap = FORCE_MAX[name]
        flag = "  ← 超手册 1000" if v > 1000 else ""
        print(f"      {name:32s} {v:5d}  (代码上限 {cap}){flag}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--write", action="store_true",
                    help="真写 1500 并读回对照(会改握持力,手上别有东西)")
    args = ap.parse_args()

    # ⚠ mock 默认是 True(InspireHandConfig.mock)。不显式关掉,整个脚本会在 mock 里
    # 空跑并"全部 PASS" —— 而 mock 的 set_force 直接 return True、read 全 None,
    # 什么都没验到。这类"测试跑在假实现上"的假绿是最难发现的。
    hand = InspireHand(InspireHandConfig(port=args.port, mock=False))
    if not hand.connect():
        print(f"连不上 {args.port}: {hand.last_error}")
        print("(usbipd 转发进 WSL 了吗?见 device-deployment-wsl-usb 那条笔记)")
        return 2
    try:
        before = read_force(hand)
        show("当前 FORCE_SET", before)
        if not args.write:
            print("\n只读模式。加 --write 才会写 1500 验证。")
            return 0

        print("\n写标量 1500(拇指应拿 1500,四指应被夹到 1000)...")
        ok = hand.set_force(1500)
        print(f"  set_force 返回 {ok}  ← 这个值不可信,写指令收不到回复也返回 True")
        time.sleep(0.3)
        after = read_force(hand)
        show("写入后 FORCE_SET", after)

        if after is None:
            check("能读回 FORCE_SET", False, "读不到,无法判定")
            return 1
        thumb = after[0], after[1]
        four = after[2:]
        # ③ 拒绝整帧:六通道全等于写之前的值
        rejected = before is not None and after == before
        check("整帧没被拒绝", not rejected,
              "六通道全是写之前的值 —— 越界导致整帧丢失,四指也没设上" if rejected
              else "至少有通道变了")
        check("四指被夹到 1000", all(v == 1000 for v in four), f"四指读回 {four}")
        if all(v == 1500 for v in thumb):
            print("  → ① 手接受 1500,FORCE_MAX 保持不变")
        elif all(v == 1000 for v in thumb):
            print("  → ② 手内部夹到 1000,**把 inspire_hand.FORCE_MAX 的拇指改回 1000**")
            FAILED.append("拇指 1500 未生效")
        else:
            print(f"  → 拇指读回 {thumb},既不是 1500 也不是 1000,需要人看")
            FAILED.append("拇指读回值意外")
    finally:
        hand.set_force(hand.cfg.init_force)
        print(f"\n已恢复力控为 init_force={hand.cfg.init_force}")
        hand.disconnect()

    print("\n" + "=" * 56)
    print("FAIL: " + "; ".join(FAILED) if FAILED else "全部 PASS")
    return 1 if FAILED else 0


if __name__ == "__main__":
    raise SystemExit(main())
