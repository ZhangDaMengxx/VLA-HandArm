#!/usr/bin/env python3
"""第一梯队体检:清单本身够不够格上「方向性建模」那套头部。

**为什么这个脚本必须先跑**:论文的前提是"训练数据里反义是分开的"。如果我们自己的
别名表就把反义混在一起,那不是模型问题,是清单的规格 bug —— 拿这种数据去训头部,
只会把方向盲教得更自信。这个检验零依赖、秒级,是全套计划里最便宜的否决器。

四项:
  C1 别名撞车        同一句话映到两个技能 → 规格 bug,人来定,不静默去重
  C2 反义是否分开    把别名里的极性词换成对立词,是否仍落回原技能 → 方向盲
  C3 极性词表覆盖率  清单里有反义对、但词表没收 → 门控会判"无方向",静默退回余弦
  C4 词长分布        论文的选词器只在"方向词是少数派"时有用;2 字指令没少数派可救

C2 是核心。它不测模型,测的是**我们的数据有没有资格**用论文那套方法。

用法:
  python3 sim/skills/check_polarity_readiness.py          # 报告
  python3 sim/skills/check_polarity_readiness.py --strict # C1/C2 不过就退出码 1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
# 只把 skills/ 放进 path,**不要**放 sim/ —— sim/schema.py 和 skills/schema.py 撞名,
# 放了 sim/ 就会 import 到错的那个(intent.py 也是只放 skills/,保持一致)。
sys.path.insert(0, str(SKILLS_DIR))

from schema import get_registry, _norm            # noqa: E402
from intent import _sim, parse                    # noqa: E402
from gen_phrasings import POLARITY_PAIRS, POLARITY_WORDS  # noqa: E402

FAILED: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    tag = "PASS" if cond else "FAIL"
    print(f"  [{tag}] {name}" + (f" — {detail}" if detail else ""))
    if not cond:
        FAILED.append(name)
    return cond


def best_match(text: str, alias_index: dict[str, str]) -> list[tuple[float, str, str]]:
    """按 intent._sim 给全部别名打分,返回按分降序的 (score, skill_id, alias)。

    刻意复用 intent._sim 而不是自己写一遍 —— 这里要测的就是**线上那条路**的行为,
    自己实现一份相似度只会测到我对它的想象。
    """
    per_skill: dict[str, tuple[float, str]] = {}
    q = _norm(text)
    for alias, sid in alias_index.items():
        sc = _sim(q, _norm(alias))
        if sc > per_skill.get(sid, (0.0, ""))[0]:
            per_skill[sid] = (sc, alias)
    out = [(sc, sid, al) for sid, (sc, al) in per_skill.items() if sc > 0]
    out.sort(key=lambda x: (-x[0], x[1]))
    return out


def c1_alias_collisions(reg) -> None:
    """同一归一化字符串映到两个技能。schema 里已有校验,这里独立再验一次:
    别名表是后面所有事情的地基,地基塌了下游全是幻觉。"""
    print("\nC1 别名撞车")
    seen: dict[str, list[str]] = {}
    for s in reg:
        for al in s.aliases:
            seen.setdefault(_norm(al), []).append(s.id)
    dup = {k: v for k, v in seen.items() if len(v) > 1}
    for k, v in dup.items():
        print(f"       撞车: {k!r} → {v}")
    check("无别名撞车", not dup, f"{len(seen)} 条别名, {len(dup)} 处撞车")


def c2_antonym_separation(reg, margin: float = 0.10) -> list[dict]:
    """**核心检验**:把别名里的极性词换成对立词,看还落不落回原技能。

    落回原技能 = 方向盲。这是最危险的一类错:说「下使能」而系统执行「使能」,
    臂会突然带力发硬 —— 而用户此时很可能正伸手准备扶它。

    三种结局分开记:
      direction_blind  第一名还是原技能            → 危险,论文要治的正是这个
      thin_margin      换对了但前两名分差 < margin → 会走歧义路径,安全但要人点
      clean            换对了且分差够开             → 现状已够用
    """
    print("\nC2 反义翻转是否分开(核心)")
    idx = reg.alias_index()
    rows: list[dict] = []
    for a, b in POLARITY_PAIRS:
        for alias, sid in list(idx.items()):
            if a not in alias:
                continue
            flipped = alias.replace(a, b)
            if _norm(flipped) == _norm(alias):
                continue
            ranked = best_match(flipped, idx)
            if not ranked:
                continue
            top_sc, top_sid, top_al = ranked[0]
            gap = top_sc - ranked[1][0] if len(ranked) > 1 else top_sc
            # 翻转后的词自己精确命中了哪个技能(有则为"正确答案")
            owner = idx.get(_norm(flipped)) or idx.get(_norm(b))
            if top_sid == sid and owner != sid:
                verdict = "direction_blind"
            elif gap < margin:
                verdict = "thin_margin"
            else:
                verdict = "clean"
            rows.append(dict(src_alias=alias, src_skill=sid, flipped=flipped,
                             top_skill=top_sid, top_score=round(top_sc, 3),
                             gap=round(gap, 3), owner=owner, verdict=verdict,
                             matched_alias=top_al, pair=f"{a}/{b}"))
    return rows


def c3_confusable_pairs(reg, hi: float = 0.45) -> list[dict]:
    """跨技能的高相似别名对 = 真正的危险集。

    刻意**不**从我的词表出发。词表是我凭记忆写的,拿它核对清单只能查出我已经想到的;
    从相似度反向找,才能捞出我没想到的那些。捞出来后再标注"词表解不解释得了":
    解释不了的那些,门控会判成"无方向",于是静默退回普通余弦 —— 正是要防的失效,
    但变成看不见的。
    """
    print("\nC3 跨技能高相似别名对(自动发现危险集)")
    items = [(al, s.id) for s in reg for al in s.aliases]
    rows: list[dict] = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a_al, a_sid = items[i]
            b_al, b_sid = items[j]
            if a_sid == b_sid:
                continue
            sc = _sim(_norm(a_al), _norm(b_al))
            if sc < hi:
                continue
            explained = any(
                (p in a_al and q in b_al) or (q in a_al and p in b_al)
                for p, q in POLARITY_PAIRS)
            rows.append(dict(a=a_al, a_skill=a_sid, b=b_al, b_skill=b_sid,
                             score=round(sc, 3), explained=explained))
    rows.sort(key=lambda r: -r["score"])
    return rows


def c4_length_profile(reg) -> dict:
    """词长与"方向词占比"。决定论文的选词器对我们有没有用。

    选词器是在"方向信号只占句子一小部分"时救那个少数派 token。「急停」两个字整句
    就是极性词,没有少数派可救 —— 那种情况下有用的只是门控 + 双子空间。
    """
    print("\nC4 词长与方向词占比")
    lens, ratios, with_pol = [], [], 0
    for s in reg:
        for al in s.aliases:
            n = len(_norm(al))
            lens.append(n)
            hit = sum(len(w) for w in POLARITY_WORDS if w in al)
            if hit:
                with_pol += 1
                ratios.append(min(1.0, hit / max(1, n)))
    lens.sort()
    return dict(n=len(lens), min=lens[0], median=lens[len(lens) // 2],
                max=lens[-1], mean=round(sum(lens) / len(lens), 2),
                with_polarity=with_pol,
                mean_polarity_ratio=round(sum(ratios) / len(ratios), 3) if ratios else 0.0)


def real_parse(text: str) -> tuple[str, str]:
    """走线上那条完整路径(含精确匹配前置),返回 (结局, 命中技能)。

    必须和 best_match 分开报:parse() 先做精确别名匹配,「下使能」是 arm_disable 的
    精确别名,这一步就救回来了。模糊打分的危险只在**非精确**的翻转上暴露
    (「下使能机械臂」不在别名表里)。混着报会把风险说小。
    """
    try:
        it = parse(text, voice_only=False)
    except Exception as exc:                      # noqa: BLE001
        return ("error:" + type(exc).__name__, "")
    if it.ok:
        return ("ok", it.skill_id or "")
    return (it.reason or "unknown",
            it.candidates[0].skill_id if it.candidates else "")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--strict", action="store_true",
                    help="C1/C2 出现 direction_blind 就退出码 1")
    args = ap.parse_args()

    reg = get_registry()
    print(f"清单: {len(reg)} 个技能, {len(reg.alias_index())} 条别名")

    c1_alias_collisions(reg)

    rows = c2_antonym_separation(reg)
    blind = [r for r in rows if r["verdict"] == "direction_blind"]
    thin = [r for r in rows if r["verdict"] == "thin_margin"]
    print(f"       翻转样本 {len(rows)} 条: "
          f"clean {len(rows) - len(blind) - len(thin)}, "
          f"thin_margin {len(thin)}, direction_blind {len(blind)}")
    for r in blind + thin:
        outcome, hit = real_parse(r["flipped"])
        r["parse_outcome"], r["parse_hit"] = outcome, hit
        flag = "!!" if outcome == "ok" and hit == r["src_skill"] else "  "
        print(f"    {flag} [{r['verdict']:15s}] {r['pair']:9s} "
              f"{r['flipped']!r} → 模糊第一名 {r['top_skill']} "
              f"({r['top_score']}, gap {r['gap']}) | 线上 parse: {outcome} {hit}")
    really_wrong = [r for r in blind
                    if r.get("parse_outcome") == "ok"
                    and r.get("parse_hit") == r["src_skill"]]
    check("反义翻转后不会静默命中原技能", not really_wrong,
          f"{len(really_wrong)} 条会被线上路径确信地判错")

    conf = c3_confusable_pairs(reg)
    unexplained = [r for r in conf if not r["explained"]]
    for r in conf[:12]:
        mark = "词表可解释" if r["explained"] else "词表解释不了"
        print(f"       {r['score']:.3f}  {r['a']!r}({r['a_skill']}) vs "
              f"{r['b']!r}({r['b_skill']})  [{mark}]")
    if len(conf) > 12:
        print(f"       ... 另有 {len(conf) - 12} 对")
    print(f"       高相似跨技能对 {len(conf)} 组, 其中词表解释不了 {len(unexplained)} 组")

    prof = c4_length_profile(reg)
    print(f"       别名字数: min {prof['min']} / 中位 {prof['median']} / "
          f"max {prof['max']} / 均值 {prof['mean']}")
    print(f"       含极性词的别名 {prof['with_polarity']}/{prof['n']}, "
          f"方向词平均占比 {prof['mean_polarity_ratio']}")
    print(f"       → 选词器价值判据: 占比越接近 1.0,越没有'少数派信号'可救")

    print("\n" + "=" * 60)
    if FAILED:
        print("FAIL: " + "; ".join(FAILED))
    else:
        print("全部 PASS")
    return 1 if (FAILED and args.strict) else 0


if __name__ == "__main__":
    raise SystemExit(main())
