#!/usr/bin/env python3
"""第二梯队:现状 `_sim()` 的配对回归基线。决定 B 层到底值不值得换。

**为什么必须配对测,不能只看一个总分**:字面/模糊匹配和句向量的弱点是**互补**的。
  · 字面匹配强在反义(「下使能」比「使能」多一个字,字符串看得见),弱在没见过的说法
  · 句向量正好相反
所以换 B 层不是纯升级,是**换弱点**。只报一个总准确率会把这件事藏起来。

两组互不重叠的用例:
  组 A 反义对    现状应该强 → 换 BGE 后这组是**回归风险**,必须守住
  组 B 口语外壳  现状应该弱 → 这组是换 B 层的**收益来源**
决策规则:新方案必须在 B 上涨、同时 A 不掉。任一条不满足就不值得换。

⚠ 组 B 是模板生成的口语外壳(把/请/一下),不是真多样性。它量的是"对客套话的鲁棒性",
   这确实是真人会加的东西;但"别人用完全不同的词描述同一件事"这一类,只能等
   /api/voice/parse 的 no_match 日志。别把这里的数字当成"覆盖了真实语言"。

用法:
  python3 src/skills/check_intent_baseline.py
  python3 src/skills/check_intent_baseline.py --show 40   # 多列几条失败样本
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SKILLS_DIR))

from schema import get_registry, _norm            # noqa: E402
from intent import parse                          # noqa: E402
from gen_phrasings import build, POLARITY_PAIRS    # noqa: E402


def run_one(text: str, want: str) -> str:
    """跑线上那条 parse(),把结果归成四类。

    **四类而不是对/错两类**,因为对机器人来说这三种"不对"的代价差一个量级:
      hit     判对了
      wrong   判成了别的技能 —— 危险,会真的动
      ask     no_match/ambiguous —— 安全,交回去让人点(这是设计意图,不是失败)
      error   解析器自己抛了
    把 wrong 和 ask 混成一个"错误率",就看不见安全网有没有在工作。
    """
    try:
        it = parse(text, voice_only=False)
    except Exception:                             # noqa: BLE001
        return "error"
    if it.ok:
        return "hit" if it.skill_id == want else "wrong"
    return "ask"


def group_a_antonyms(reg) -> list[tuple[str, str]]:
    """组 A:反义对。别名原句 + 翻转句,各自都该落到自己的技能上。

    只收**翻转后精确命中另一个技能**的样本 —— 那才有明确的正确答案。
    像「上下使能」这种翻转产物不是人话,没有 ground truth,放进来只会污染分数。
    """
    idx = reg.alias_index()
    cases: list[tuple[str, str]] = []
    for a, b in POLARITY_PAIRS:
        for alias, sid in idx.items():
            if a not in alias:
                continue
            flipped = alias.replace(a, b)
            owner = idx.get(_norm(flipped))
            if owner and owner != sid:
                cases.append((alias, sid))
                cases.append((flipped, owner))
    seen, out = set(), []
    for t, s in cases:
        if (t, s) not in seen:
            seen.add((t, s))
            out.append((t, s))
    return out


def group_b_shells(rows: list[dict]) -> list[tuple[str, str]]:
    """组 B:带口语外壳的说法。**排除**核心句(seed 本身),那些是精确别名,必中。"""
    out = []
    for r in rows:
        if r["rule"] == "core" or r["rule"].startswith("syn:") and "|" not in r["rule"]:
            continue
        out.append((r["text"], r["skill_id"]))
    return out


def report(title: str, cases: list[tuple[str, str]], show: int) -> Counter:
    tally: Counter = Counter()
    bad: list[tuple[str, str, str]] = []
    for text, want in cases:
        res = run_one(text, want)
        tally[res] += 1
        if res in ("wrong", "error"):
            bad.append((text, want, res))
    n = max(1, sum(tally.values()))
    print(f"\n{title}  (n={sum(tally.values())})")
    print(f"  hit   {tally['hit']:5d}  {tally['hit'] / n:6.1%}   判对")
    print(f"  wrong {tally['wrong']:5d}  {tally['wrong'] / n:6.1%}   判成别的技能(危险)")
    print(f"  ask   {tally['ask']:5d}  {tally['ask'] / n:6.1%}   交回去让人点(安全)")
    if tally["error"]:
        print(f"  error {tally['error']:5d}  {tally['error'] / n:6.1%}")
    for text, want, res in bad[:show]:
        it = parse(text, voice_only=False)
        got = it.skill_id if it.ok else "-"
        cand = ", ".join(f"{c.skill_id}:{c.score:.2f}"
                         for c in (it.candidates or [])[:3])
        print(f"    [{res}] {text!r} 应为 {want}, 得到 {got}  | 候选 {cand}")
    if len(bad) > show:
        print(f"    ... 另有 {len(bad) - show} 条")
    return tally


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", type=int, default=10)
    args = ap.parse_args()

    reg = get_registry()
    data = build()

    a = report("组 A 反义对(现状应该强 —— 换 BGE 后这组是回归风险)",
               group_a_antonyms(reg), args.show)
    b = report("组 B 口语外壳(现状应该弱 —— 这组是换 B 层的收益来源)",
               group_b_shells(data["rows"]), args.show)

    print("\n" + "=" * 62)
    na, nb = max(1, sum(a.values())), max(1, sum(b.values()))
    print(f"基线: A 判对 {a['hit'] / na:.1%} 危险错 {a['wrong'] / na:.1%}  |  "
          f"B 判对 {b['hit'] / nb:.1%} 危险错 {b['wrong'] / nb:.1%}")
    print("换 B 层的判据: B 的 hit 明显上涨, 且 A 的 wrong 不增加。")
    print("注意 ask 不是失败 —— 它是安全网在工作。真正要压的只有 wrong。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
