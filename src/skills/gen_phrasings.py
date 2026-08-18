#!/usr/bin/env python3
"""从 registry.yaml 的别名种子机械扩写说法,并自动打**极性标签**。

用途:给「词元级方向性建模 + 双语义空间」那套头部准备训练数据。论文要三种标注:
  · α̂  词元级极性标签(哪几个字是方向词)  → 本脚本按闭集词典自动标
  · ĝ   句级有无方向性                    → 同上,有极性词就是 1
  · 正负意图对(排序损失用)                 → 由 skill_id 天然给出,同 id 为正

⚠ **这是模板扩写,不是造多样性**。它只会把你自己的说法排列组合,教出来的模型
   擅长的是「你的说话习惯」。真多样性只能来自 /api/voice/parse 的 no_match 日志
   —— 那才是别人真会说、而清单没覆盖的话。两者要混着训,别只用这个。

⚠ 极性词典是**字面匹配**,抓不到组合极性(「不要张开」「稍微慢一点」)。论文用滑动
   窗口正是为了这个。现在这 11 条技能里没有否定式,够用;将来加了要手工过一遍。

用法:
  python3 src/skills/gen_phrasings.py                    # 只看统计和体检
  python3 src/skills/gen_phrasings.py --out out/phrasings.yaml
  python3 src/skills/gen_phrasings.py --check            # 体检不过就退出码 1
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
# 只放 skills/ —— src/schema.py 与 skills/schema.py 撞名,放了 src/ 会 import 到错的。
sys.path.insert(0, str(SKILLS_DIR))

from schema import get_registry, _norm  # noqa: E402

# 极性词对:成对写,校验才能查「反义有没有被分开」。顺序无意义,只用来配对。
POLARITY_PAIRS = [
    ("使能", "下使能"), ("上使能", "去使能"), ("张开", "握拳"), ("张开", "合上"),
    ("打开", "关闭"), ("开", "关"), ("上", "下"), ("增大", "减小"),
    ("加大", "减小"), ("快", "慢"), ("加速", "减速"), ("升", "降"),
    ("前", "后"), ("左", "右"), ("急停", "复位"),
]
# 拆成扁平集合供逐字打标签用
POLARITY_WORDS = sorted({w for a, b in POLARITY_PAIRS for w in (a, b)},
                        key=len, reverse=True)

# 口语外壳:前缀 × 后缀。"" 表示不加,所以原句也在结果里。
PREFIXES = ["", "把", "给我", "帮我", "现在", "麻烦", "请", "你"]
SUFFIXES = ["", "一下", "吧", "一下吧"]

# 同义替换:**闭集**,只换我确定等价的。开放改写是 LLM 那一步的事,不在这儿硬编。
SYNONYMS = {
    "回零位": ["回原点", "归零", "回初始位置", "回家"],
    "使能": ["上电", "通电", "上使能"],
    "复位": ["清错", "清故障", "重置"],
    "张开手": ["把手张开", "手张开", "松开手", "开手"],
    "握拳": ["把手握上", "手握紧", "抓紧", "攥拳"],
    "急停": ["紧急停止", "立刻停", "马上停"],
    # 「开慢点」删掉:它以「开」开头,而「开」是 hand_open 的极性词。修饰词剥离会把
    # 「慢点」当速度修饰摘走,剩下的「开」直接命中张开手 —— 一句"慢一点"变成手张开。
    # 这不是匹配器的锅,是这条同义词本身选得差(而且它更像"开车慢点",不是臂的说法)。
    "慢速": ["降速", "慢一点"],
}


def label_polarity(text: str) -> tuple[list[int], int]:
    """逐字打极性标签,返回 (α̂ 列表, ĝ)。

    α̂[j]=1 表示第 j 个字属于某个方向词。长词优先匹配,避免「下使能」被「下」抢先。
    """
    marks = [0] * len(text)
    for w in POLARITY_WORDS:                  # 已按长度降序
        start = 0
        while True:
            i = text.find(w, start)
            if i < 0:
                break
            # 已被更长的词占掉就不重复标,保持「谁先占谁算」的确定性
            if not any(marks[i:i + len(w)]):
                for j in range(i, i + len(w)):
                    marks[j] = 1
            start = i + 1
    return marks, int(any(marks))


def _syn_safe(seed: str, key: str, reg) -> bool:
    """同义替换会不会**破坏极性**。用清单判,不再堆一层词表。

    判据:若 key 本身是技能 X 的精确别名,而 seed 是技能 Y 的精确别名,且 X≠Y,
    那么在 seed 内部替换 key 就是在抹掉 X 和 Y 的区别 —— 禁止。

      · 「下使能」(arm_disable) 里换「使能」(arm_enable) → 禁止,否则得到「下上电」
      · 「使能机械臂」(arm_enable) 里换「使能」(arm_enable) → 允许,得到「上电机械臂」
      · 「复位到零」(go_home) 里换「复位」(arm_reset) → 禁止,否则得到「清错到零」

    为什么用清单而不是极性词表:「断使能」的「断」不在我的词表里,词表方案漏它;
    但清单知道「断使能」是 arm_disable。**清单是真源,词表是我的记忆**,冲突时信清单。
    """
    idx = reg.alias_index()
    key_owner = idx.get(_norm(key))
    seed_owner = idx.get(_norm(seed))
    if key_owner and seed_owner and key_owner != seed_owner:
        return False
    return True


def expand(seed: str, reg=None) -> list[tuple[str, str]]:
    """把一条种子别名扩成多条说法,返回 [(说法, 规则来源)]。

    来源字符串是为了**可追溯**:出错时能分清是模板的锅还是种子本身的锅。
    """
    if reg is None:
        reg = get_registry()
    cores = [(seed, "seed")]
    for k, alts in SYNONYMS.items():
        if k in seed and _syn_safe(seed, k, reg):
            cores += [(seed.replace(k, a), f"syn:{k}->{a}") for a in alts]
    out = []
    for core, src in cores:
        for p in PREFIXES:
            for s in SUFFIXES:
                t = f"{p}{core}{s}"
                rule = "core" if not p and not s else f"tmpl:{p or '-'}+{s or '-'}"
                out.append((t, src if rule == "core" else f"{src}|{rule}"))
    return out


def build() -> dict:
    """遍历清单生成全部说法,并做两项体检。"""
    reg = get_registry()
    rows, by_norm, collisions = [], {}, []
    for spec in reg:
        for seed in [spec.name, *spec.aliases]:
            for text, rule in expand(seed, reg):
                marks, g = label_polarity(text)
                row = {"text": text, "skill_id": spec.id, "seed": seed,
                       "rule": rule, "alpha": marks, "g": g}
                key = _norm(text)
                # 撞车**不静默去重**:同一句话映到两个技能,训下去只会教出摇摆。
                # 撞车行进 collisions 单独报给人看,**不进 rows**(不参与训练);
                # 早先写成只挂 collision_with 字段却不 append,collisions 恒空 —— 等于
                # 体检永远通过。这类"检查项自己坏掉"的 bug 比没有检查更危险。
                if key in by_norm and by_norm[key] != spec.id:
                    row["collision_with"] = by_norm[key]
                    collisions.append(row)
                else:
                    by_norm.setdefault(key, spec.id)
                    rows.append(row)
    return {"rows": rows, "collisions": collisions,
            "antonym_errors": check_antonyms(by_norm)}


def check_antonyms(by_norm: dict[str, str]) -> list[dict]:
    """体检:把说法里的极性词换成对立词,若仍映到**同一个** skill_id 就是错。

    这条直接对着论文的前提 —— 训练数据本身必须把反义分开。数据里混了反义,
    再好的头部也学不出方向性,只会把方向盲学得更自信。
    只查生成集内部的一致性;线上匹配行为由 check_polarity_readiness.py 负责。
    """
    errs = []
    for a, b in POLARITY_PAIRS:
        for text, sid in by_norm.items():
            if a not in text:
                continue
            flipped = _norm(text.replace(a, b))
            if flipped == text:
                continue
            other = by_norm.get(flipped)
            if other is not None and other == sid:
                errs.append({"text": text, "flipped": flipped,
                             "skill_id": sid, "pair": f"{a}/{b}"})
    return errs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", help="写出 jsonl(每行一条说法)")
    ap.add_argument("--check", action="store_true", help="体检不过退出码 1")
    args = ap.parse_args()

    data = build()
    rows, coll, ant = data["rows"], data["collisions"], data["antonym_errors"]
    per_skill: dict[str, int] = {}
    for r in rows:
        per_skill[r["skill_id"]] = per_skill.get(r["skill_id"], 0) + 1
    with_g = sum(r["g"] for r in rows)

    print(f"生成说法 {len(rows)} 条, 覆盖 {len(per_skill)} 个技能")
    print(f"  含方向性(ĝ=1) {with_g} 条 ({with_g / max(1, len(rows)):.1%})")
    print(f"  每技能条数: min {min(per_skill.values())} / "
          f"max {max(per_skill.values())}")
    print(f"  撞车(不参与训练) {len(coll)} 条")
    for r in coll[:8]:
        print(f"    {r['text']!r}  {r['skill_id']} vs {r['collision_with']}")
    if len(coll) > 8:
        print(f"    ... 另有 {len(coll) - 8} 条")
    print(f"  反义未分开 {len(ant)} 条")
    for r in ant[:8]:
        print(f"    [{r['pair']}] {r['text']!r} → {r['flipped']!r} "
              f"都是 {r['skill_id']}")

    if args.out:
        p = Path(args.out)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(f"  → 写出 {p} ({len(rows)} 行)")

    bad = bool(coll or ant)
    print("体检: " + ("FAIL" if bad else "PASS"))
    return 1 if (bad and args.check) else 0


if __name__ == "__main__":
    raise SystemExit(main())
