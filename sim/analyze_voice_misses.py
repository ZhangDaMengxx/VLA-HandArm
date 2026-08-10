#!/usr/bin/env python3
"""读 out/voice_parses.jsonl,报漏词率和该补哪些别名。

这份日志是**真实语言样本的唯一来源**。模板扩写(gen_phrasings.py)只会把清单里
已有的别名排列组合,教出来的模型擅长的是我们自己的说话习惯;而这里记的是别人
真会说、清单却没覆盖的话。

报三件事,对应三种不同的处理:

  ① 漏词率        no_match+ambiguous 占比 —— 判断"要不要上更强的匹配"的唯一依据
  ② 差一点就中了  最高分接近阈值 → **补一条别名就解决**,不用动模型。最省的那条路
  ③ 完全不认识    最高分很低 → 是真的新说法,才需要更强的语义匹配

区分 ② 和 ③ 很要紧:如果九成漏词都是 ②,那正确答案是补清单,不是训模型。

用法:
  python3 sim/analyze_voice_misses.py
  python3 sim/analyze_voice_misses.py --top 30
  python3 sim/analyze_voice_misses.py --since 2026-08-01
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
LOG = SIM_DIR / "out" / "voice_parses.jsonl"
# intent.parse 的默认阈值。差一点就中的判据要跟它对齐,写死在这儿会漂。
try:
    sys.path.insert(0, str(SIM_DIR / "skills"))
    import inspect

    from intent import parse as _p
    _d = {k: v.default for k, v in inspect.signature(_p).parameters.items()}
    THRESHOLD = float(_d.get("threshold", 0.58))
    MARGIN = float(_d.get("margin", 0.12))
except Exception:                                        # noqa: BLE001
    THRESHOLD, MARGIN = 0.58, 0.12


def load(since: float | None) -> list[dict]:
    if not LOG.exists():
        return []
    rows = []
    with LOG.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue          # 半行(写的时候进程被杀)跳过,别让一行坏的挡住全部
            if since and float(r.get("ts", 0)) < since:
                continue
            rows.append(r)
    return rows


def top_score(r: dict) -> float:
    c = r.get("candidates") or []
    return float(c[0]["score"]) if c else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--top", type=int, default=20)
    ap.add_argument("--since", help="只看这天之后 (YYYY-MM-DD)")
    # 自检/压测流量必须能剔掉。它们和真人打字的记录**长得一模一样**(source 都是
    # text),混进去漏词率就是假的。所以自检时要显式传 source=selftest,
    # 分析时默认排除 —— 我第一次验证这条链路就往日志里灌了 8 条 curl 数据,
    # 只能整个删掉重来。
    ap.add_argument("--exclude-source", default="selftest",
                    help="排除这些来源(逗号分隔),默认排除 selftest")
    ap.add_argument("--all-sources", action="store_true", help="不排除任何来源")
    args = ap.parse_args()

    since = None
    if args.since:
        since = time.mktime(time.strptime(args.since, "%Y-%m-%d"))

    rows = load(since)
    if not args.all_sources and args.exclude_source:
        drop = {s.strip() for s in args.exclude_source.split(",") if s.strip()}
        before = len(rows)
        rows = [r for r in rows if r.get("source") not in drop]
        if before != len(rows):
            print(f"(排除 {before - len(rows)} 条来源为 {sorted(drop)} 的自检流量)")
    if not rows:
        print(f"没有数据: {LOG}")
        print("(还没人用过语音输入,或者服务没重启过 —— 落盘是这次才加的)")
        return 0

    ok = [r for r in rows if r.get("ok")]
    nm = [r for r in rows if r.get("reason") == "no_match"]
    am = [r for r in rows if r.get("reason") == "ambiguous"]
    other = [r for r in rows if not r.get("ok")
             and r.get("reason") not in ("no_match", "ambiguous")]
    n = len(rows)
    miss = len(nm) + len(am)

    t0 = min(float(r.get("ts", 0)) for r in rows)
    t1 = max(float(r.get("ts", 0)) for r in rows)
    span_h = (t1 - t0) / 3600 if t1 > t0 else 0
    print(f"{LOG.name}: {n} 条解析"
          + (f",跨 {span_h:.1f} 小时" if span_h else ""))
    print(f"  判对        {len(ok):5d}  {len(ok)/n:6.1%}")
    print(f"  no_match    {len(nm):5d}  {len(nm)/n:6.1%}")
    print(f"  ambiguous   {len(am):5d}  {len(am)/n:6.1%}")
    if other:
        print(f"  其他        {len(other):5d}  {len(other)/n:6.1%}"
              f"  ({Counter(r.get('reason') for r in other).most_common(3)})")
    print(f"  → 漏词率 {miss/n:.1%}")
    # 按来源分开算漏词率:打字的漏是"清单没覆盖",语音的漏可能是"听错了"。
    # 两者修法完全不同(补别名 vs 改纠错/换引擎),混在一起就找不到该修哪个。
    by_src: dict[str, list[dict]] = {}
    for r in rows:
        by_src.setdefault(r.get("source", "?"), []).append(r)
    for src, rs in sorted(by_src.items(), key=lambda kv: -len(kv[1])):
        m = sum(1 for r in rs if r.get("reason") in ("no_match", "ambiguous"))
        print(f"     来源 {src}: {len(rs)} 条,漏词率 {m/len(rs):.1%}")

    # ASR 纠错的实际效果。纠错**纠反了**比不纠更危险(极性词反向),所以要能查。
    fixed = [r for r in rows if r.get("text_raw")
             and r["text_raw"] != r.get("text")]
    if fixed:
        good = sum(1 for r in fixed if r.get("ok"))
        print(f"\n【ASR 纠错】触发 {len(fixed)} 次,其中 {good} 次纠完命中了")
        for r in fixed[:args.top]:
            mark = "✓" if r.get("ok") else "✗"
            print(f"  {mark} {r['text_raw']!r} → {r.get('text')!r}"
                  f"  {r.get('skill_id') or r.get('reason')}")
        print("  ⚠ 逐条看一遍:纠错把话改成了别的意思(尤其极性词反向)比不纠更危险")

    if not miss:
        print("\n没有漏词。清单目前覆盖得住实际说法。")
        return 0

    # ② 差一点就中了:补一条别名就解决
    near = sorted((r for r in nm if top_score(r) >= THRESHOLD - 0.15),
                  key=top_score, reverse=True)
    print(f"\n【差一点就中】{len(near)} 条 —— 补别名即可,不用动模型")
    for r in near[:args.top]:
        c = (r.get("candidates") or [{}])[0]
        print(f"  {top_score(r):.3f}  {r.get('text','')!r}"
              f"  → 最接近 {c.get('skill_id','?')}(匹配串 {c.get('matched','?')!r})")

    # ③ 完全不认识:真的新说法
    far = sorted((r for r in nm if top_score(r) < THRESHOLD - 0.15),
                 key=top_score, reverse=True)
    print(f"\n【完全不认识】{len(far)} 条 —— 这些才需要更强的语义匹配")
    for r in far[:args.top]:
        print(f"  {top_score(r):.3f}  {r.get('text','')!r}")

    # ambiguous:清单本身有歧义,或者需要更大的决策间隔
    if am:
        print(f"\n【歧义,交回让人点】{len(am)} 条")
        for r in am[:args.top]:
            cs = ", ".join(f"{c['skill_id']}:{c['score']:.3f}"
                           for c in (r.get("candidates") or [])[:3])
            print(f"  {r.get('text','')!r}  候选 {cs}")

    print("\n重复说了多次仍没中的(最该先补):")
    for text, c in Counter(r.get("text", "") for r in nm + am).most_common(10):
        if c > 1:
            print(f"  ×{c}  {text!r}")

    print(f"\n判据: threshold={THRESHOLD} margin={MARGIN}(从 intent.parse 读的)")
    print("若【差一点就中】占多数 → 补清单别名,别训模型。这是最省的那条路。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
