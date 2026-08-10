#!/usr/bin/env python3
"""扫 margin/threshold:换 B 层之前,先问"调参数够不够"。

C2 找到的那条危险错(「下使能机械臂」→ arm_enable, 分差 0.11)只比 margin 0.10 高一点。
所以在动模型之前必须先回答:**把 margin 抬一点是不是就够了?**
如果够,整个 B 层投资就不必做 —— 这是最省钱的那条路,必须先排除它。

代价结构:margin 抬高 → wrong 变 ask(安全网接住),但 hit 也会掉(本来对的被拖进歧义)。
这个脚本把两条曲线一起画出来,让取舍看得见,而不是我替你选一个数。

危险集刻意包含**非精确翻转**(「下使能机械臂」这种不在别名表里的),因为精确翻转会被
parse 的第一步救掉,只测它等于测不到风险。

用法:
  python3 sim/skills/check_margin_sweep.py
"""
from __future__ import annotations

import sys
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SKILLS_DIR))

from schema import get_registry, _norm            # noqa: E402
from intent import parse                          # noqa: E402
from gen_phrasings import build                   # noqa: E402

# 手工列的危险集:极性词 + 对象名的组合,一个都不在别名表里。
# 这些是真人真会说的话("把机械臂下使能"),而清单只收了裸词("下使能")。
DANGER = [
    ("下使能机械臂", "arm_disable"),
    ("去使能机械臂", "arm_disable"),
    ("断使能机械臂", "arm_disable"),
    ("把机械臂下使能", "arm_disable"),
    ("机械臂下使能", "arm_disable"),
    ("下使能一下机械臂", "arm_disable"),
    ("使能机械臂", "arm_enable"),
    ("把机械臂使能", "arm_enable"),
    ("机械臂上使能", "arm_enable"),
    ("把手握上", "hand_close"),
    ("把手张开", "hand_open"),
    # 否定式:现状和论文的字面词表**都**解不了(词表是字面匹配,看不见"别")。
    # 留在这里当已知缺口的标记,不是拿来凑失败数的。
    ("手别张开", "hand_close"),
]
NEGATION = {"手别张开"}


def tally(cases, threshold: float, margin: float) -> dict:
    out = {"hit": 0, "wrong": 0, "ask": 0}
    detail = []
    for text, want in cases:
        it = parse(text, voice_only=False, threshold=threshold, margin=margin)
        if it.ok:
            k = "hit" if it.skill_id == want else "wrong"
        else:
            k = "ask"
        out[k] += 1
        if k == "wrong":
            detail.append((text, want, it.skill_id or "", it.confidence))
    out["detail"] = detail
    return out


def main() -> int:
    reg = get_registry()
    rows = build()["rows"]
    shells = [(r["text"], r["skill_id"]) for r in rows if r["rule"] != "core"]
    danger = [(t, w) for t, w in DANGER if t not in NEGATION]

    print(f"危险集 {len(danger)} 条(极性词+对象名,均不在别名表里)")
    print(f"口语外壳集 {len(shells)} 条")
    print(f"\n{'margin':>7} | {'危险集 hit/wrong/ask':>22} | {'外壳集 hit/wrong/ask':>22}")
    print("-" * 60)
    for m in (0.10, 0.12, 0.15, 0.20, 0.25, 0.30):
        d = tally(danger, 0.58, m)
        s = tally(shells, 0.58, m)
        print(f"{m:7.2f} | {d['hit']:6d}/{d['wrong']:5d}/{d['ask']:4d}"
              f"{'':>5} | {s['hit']:6d}/{s['wrong']:5d}/{s['ask']:4d}")

    print("\nmargin=0.10 (现状) 的危险集失败明细:")
    for text, want, got, conf in tally(danger, 0.58, 0.10)["detail"]:
        print(f"    {text!r} 应为 {want}, 判成 {got} (conf {conf:.3f})")

    print("\n否定式(现状与论文字面词表都解不了):")
    for t in sorted(NEGATION):
        it = parse(t, voice_only=False)
        print(f"    {t!r} → {it.skill_id if it.ok else '(' + str(it.reason) + ')'}")

    print("\n" + "=" * 60)
    print("读法: margin 抬高把 wrong 换成 ask(安全),代价是 hit 下降。")
    print("若某个 margin 能让危险集 wrong=0 而外壳集 hit 几乎不掉,就先调参数,别换模型。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
