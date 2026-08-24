#!/usr/bin/env python3
"""src/skills/intent.py — 一句话 → 技能调用信封的意图解析。

在链路里的位置(schema.py 的 by_alias 注释把模糊匹配甩给本模块):

    麦克风/文本框 → ASR → [本模块] → 调用信封 → console_exec / runner
                                      ↑ 只产出 skill_id + params,不执行

**纯 Python**,只依赖 schema.py(不碰 ROS / numpy / 硬件),所以 app_web
(V3 主环境)和执行侧都能 import,也能脱机单测。

三条设计原则:

1. **精确优先**。整句命中别名表就直接 confidence=1.0 返回,不做任何模糊处理 ——
   "慢一点" 本身就是 set_speed_slow 的别名,不该被当成 go_home 的修饰词。
2. **重名不猜**(与 gesture_pack.find_by_name 同一原则)。前两名分差在 margin
   之内就判 ambiguous,返回候选让调用方决定,绝不随机命中。
3. **模糊匹配不放行安全闸**。本模块只出 skill_id;need_confirm 仍由执行层强制。
   所以误识别最坏结果是「弹错确认框」,不是误动作。estop 是全表唯一免确认项,
   但它的方向是 fail-safe:误停不误动。

修饰词:"回零位慢一点" → 剥掉「慢一点」→ 匹配「回零位」→ go_home,再把 slow 落到
duration。**只在技能自己声明了该参数时才落**,没声明就只提示不硬塞;最终归一和
越界夹取仍由 schema.resolve_params 负责(语音限速也在那层)。

自检:
    python3 src/skills/intent.py "回零位慢一点"
    python3 src/skills/intent.py --all       # 全部别名回归一遍
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
SIM_DIR = SKILLS_DIR.parent
sys.path.insert(0, str(SKILLS_DIR))
# 复用 schema 的归一化:别名表是用它建的索引,匹配侧必须用同一个函数,
# 否则「清单里怎么存」和「说话怎么查」会出现两份解释。
from schema import RegistryError, SkillRegistry, get_registry, _norm  # noqa: E402

# ---- 修饰词 ----
# 这里只收**不是任何技能别名**的说法。"慢一点"/"慢速"/"降速" 都是 set_speed_slow 的
# 别名,整句说它们时第 1 步精确匹配就命中了,走不到修饰词这条路。
SLOW_WORDS = ("慢一点", "慢一些", "慢点儿", "慢点", "慢些", "放慢", "慢慢", "再慢")
FAST_WORDS = ("快一点", "快一些", "快点儿", "快点", "快些", "加快", "再快")
# 力度修饰词。**为什么做成修饰词而不是独立技能的别名**:力度是"怎么做"、不是
# "做什么" —— 它对任何手部动作都适用。做成修饰词的话「轻一点捏」「用力握拳」
# 都自然成立;做成独立技能就得为 每个动作 × 每个档位 各建一条,组合爆炸。
#
# ⚠ 这里只收**不是任何技能别名**的说法。"轻一点"/"用力" 本身是 hand_grip_* 的
# 别名,整句只说它们时第 1 步精确匹配就命中了(那是"只调力度不动作"的合法意图),
# 走不到这条路。只有它们**跟在动作词旁边**时才作为修饰生效。
# ⚠ 这些词**同时**是 hand_grip_* 的别名,这是有意的、不是重复:
#   · 只说「轻一点」→ 第 1 步整句精确匹配 → hand_grip_soft(只调力度、不动作)
#   · 说「轻一点捏」→ 整句匹不上 → 剥掉「轻一点」→「捏」→ hand_pinch + soft 力度
# 第 1 步只匹配**整句**,所以两条路不打架。少了任一边都缺一种说法:
# 不在别名表里 → 单说「用力」变 no_match;不在这里 → 「轻一点捏」只调力度不捏。
SOFT_WORDS = ("轻一点", "轻一些", "轻柔地", "力小一点", "力度轻",
              "轻轻", "轻点", "轻柔", "小力", "力小", "温柔")
FIRM_WORDS = ("用力一点", "力大一点", "夹紧一点", "用点力", "力度大",
              "用力", "使劲", "大力", "力大", "夹紧")
# 语气/礼貌词。放这里而不是别名表 —— 它们和技能无关,不该占别名索引。
FILLER_WORDS = ("请", "帮我", "麻烦", "现在", "给我", "一下", "用", "吧", "呢", "啊", "把")

DURATION_FACTOR = 1.5      # slow: duration ×1.5;fast: ÷1.5
SPEED_FACTOR = 2.0         # slow: speed ÷2;fast: speed ×2

# 力度档 → (手速度, 力控阈值)。**实测常数,别凭感觉改**
# (2026-08-06 src/test_force_freeze.py 速度扫描,手指互顶、触发阈值 150):
#   speed 50 → 稳态 272g · 150 → 717g · 300 → 941g
# 决定最终握持力的主要是**速度**(它决定接触瞬间的动量和弹性变形深度),
# 力控设定值影响小得多 —— 所以两者一起给,不分开调。
# ⚠ 上面是刚性接触。柔性物同速下更轻(海绵在 200/300 下食指稳态 535g)。
# 和 registry.yaml 的 hand_grip_* 三条必须一致:那三条是"只调力度不动作",
# 这里是"动作时顺带调力度",同一套档位两个入口,数值分叉就会出现
# 「轻一点」和「轻一点捏」力度不同这种说不清的行为。
GRIP_LEVELS = {
    "soft":   {"hand_speed": 50,  "hand_force": 250},
    "normal": {"hand_speed": 500, "hand_force": 300},  # 与 gestures.yaml 默认值对齐(2026-08-10 改)
    "firm":   {"hand_speed": 300, "hand_force": 500},
}

_CN_NUM = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
           "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}
_CN_CLASS = "零一二两三四五六七八九十"


def _cn_int(s: str) -> float | None:
    """中文数字 → 数。只需覆盖 0-99(duration 上限 15、speed 上限 4)。"""
    s = s.strip()
    if not s or any(c not in _CN_NUM for c in s):
        return None
    if len(s) == 1:
        return float(_CN_NUM[s])
    if "十" in s:
        head, _, tail = s.partition("十")
        tens = _CN_NUM[head] if head else 1
        ones = _CN_NUM[tail] if tail else 0
        return float(tens * 10 + ones)
    return None                       # "一二三" 这种连写不猜


def _num(raw: str) -> float | None:
    """一段数字文本 → 数。阿拉伯数字和中文数字都吃。"""
    raw = raw.strip()
    if not raw:
        return None
    if raw[0].isdigit():
        try:
            return float(raw)
        except ValueError:
            return None
    return _cn_int(raw)


def _extract(text: str) -> tuple[str, dict, list[str]]:
    """剥出显式数值与快慢修饰,返回 (剩余文本, 修饰, 提示)。

    剩余文本才拿去匹配技能。长词优先剥 —— 否则「慢一点」会被「慢点」先咬掉一半,
    剩个「一」反倒污染匹配。
    """
    mods: dict = {}
    notes: list[str] = []
    t = str(text or "")

    def take(pattern: str, key: str) -> None:
        nonlocal t

        def repl(m: re.Match) -> str:
            v = _num(m.group(1))
            if v is None:
                return m.group(0)          # 数字读不出来就原样留着,交给模糊匹配
            mods[key] = v
            return " "
        t = re.sub(pattern, repl, t)

    take(rf"([\d.]+|[{_CN_CLASS}]+)\s*秒(?:钟)?", "seconds")
    take(rf"([\d.]+|[{_CN_CLASS}]+)\s*倍", "times")

    for w in sorted(SLOW_WORDS, key=len, reverse=True):
        if w in t:
            mods["slow"] = True
            t = t.replace(w, " ")
    for w in sorted(FAST_WORDS, key=len, reverse=True):
        if w in t:
            mods["fast"] = True
            t = t.replace(w, " ")
    # 一句里同时出现快和慢:不猜哪个是真意图,两个都丢,只提示。
    has_slow, has_fast = mods.pop("slow", False), mods.pop("fast", False)
    if has_slow and has_fast:
        notes.append("同时说了快和慢,两个修饰都忽略")
    elif has_slow:
        mods["slow"] = True
    elif has_fast:
        mods["fast"] = True

    # 力度修饰。长词优先,同「轻一点」被「轻点」咬掉一半的问题。
    soft = firm = False
    for w in sorted(SOFT_WORDS, key=len, reverse=True):
        if w in t:
            soft = True
            t = t.replace(w, " ")
    for w in sorted(FIRM_WORDS, key=len, reverse=True):
        if w in t:
            firm = True
            t = t.replace(w, " ")
    # 同时说了轻和重:**不猜**。猜错的后果是握持力反向 —— 说"轻"却用大力,
    # 东西可能被夹坏。两个都丢,让它按技能默认力度走并明确提示。
    if soft and firm:
        notes.append("同时说了轻和重,力度修饰都忽略,按默认力度")
    elif soft:
        mods["grip"] = "soft"
    elif firm:
        mods["grip"] = "firm"
    for w in FILLER_WORDS:
        t = t.replace(w, " ")
    return t.strip(), mods, notes


def _bigrams(s: str) -> set[str]:
    return {s[i:i + 2] for i in range(len(s) - 1)} or {s}


def _sim(q: str, cand: str) -> float:
    """相似度 0-1。中文按字 bigram 的 Jaccard,外加包含关系加权。

    为什么不用编辑距离:中文命令很短(2-6 字),一个字的增删就把编辑距离比例拉到
    0.2-0.3,阈值没法定。bigram 交集对「多说两个字」稳健得多。
    """
    if not q or not cand:
        return 0.0
    if q == cand:
        return 1.0
    short, long = (q, cand) if len(q) <= len(cand) else (cand, q)
    if short in long:
        # 按长度比给分:1 个字命中 6 字别名不该算高分,否则说「手」就随机命中了。
        return 0.62 + 0.33 * (len(short) / len(long))
    a, b = _bigrams(q), _bigrams(cand)
    inter = len(a & b)
    return inter / len(a | b) if inter else 0.0


@dataclass
class Candidate:
    """一个候选目标 + 它为什么被选中。matched 是命中的那条别名,便于排查误判。

    ⚠ `skill_id` 在 kind=gesture_pack 时装的是**包的相对路径**。候选必须带上它:
    两个同名包(不同目录)撞车时,若候选只有名字,调用方拿名字再查一次还是 ambiguous
    —— 用户就永远点不出结果。带路径才能一次定死。
    """
    skill_id: str
    name: str
    score: float
    matched: str
    kind: str = "skill"

    def to_public(self) -> dict:
        return {"skill_id": self.skill_id, "name": self.name, "kind": self.kind,
                "score": round(self.score, 3), "matched": self.matched}


# 所有"包"类目标的 kind。判"这是个包吗"一律用这个集合,别逐处写字符串 ——
# 加第三种包时漏掉一处的后果是:它能被匹配上,但执行层把它当技能去查清单,
# 报的是"技能不存在"而不是"包放不了"。那种错很难查。
PACK_KINDS = ("gesture_pack", "combo_pack")

# 每种包**固定**动哪些设备。包不在清单里,没法像技能那样用 console_exec.targets()
# 从 action 里推 —— 但它动什么是由包的种类定死的,所以在这里列一次。
#
# ⚠ 和 PACK_KINDS 放一起是刻意的:这两份知识必须同时更新。分开放的话,加了第三种
# 包只改 PACK_KINDS 就能跑通(判断「是不是包」不报错),但设备表查不到 → 要么
# KeyError,要么静默报成空 —— 而 devices 是确认框上「会动臂」这句提示的唯一来源,
# 报错方向的风险提示比不报更糟(人白清一次场,久了就不信这个提示了)。
PACK_DEVICES = {"gesture_pack": ["hand"], "combo_pack": ["arm", "hand"]}
assert set(PACK_DEVICES) == set(PACK_KINDS), "PACK_DEVICES 要覆盖所有 PACK_KINDS"


@dataclass
class PackTarget:
    """清单之外的**动态**目标:已录制的手势技能包。

    为什么不塞进 registry.yaml:包是磁盘上随时增删的文件,清单是**静态真源**,
    它的校验和别名撞车检查都建立在"内容固定"上。把动态文件写进去,校验就失去意义。
    所以包作为独立目标池参与**同一次打分** —— 同池是关键:包名和技能名撞车时才判得出
    ambiguous,否则某一池会悄悄赢,而"哪个赢"取决于遍历顺序。
    """
    path: str                       # 相对路径,唯一标识(重名包靠它区分)
    name: str
    frames: int = 0
    duration_ms: int = 0
    need_confirm: bool = True       # 包会真的动手,默认要确认
    # 哪种包:gesture_pack(纯手,data/gestures/)| combo_pack(臂+手,data/combos/)
    #
    # ⚠ **必须有这个字段**,不能只靠 path。两种包有各自的沙箱根,
    # `data/gestures/<同名>.json` 和 `data/combos/<同名>.json` 可以是**两个合法的不同文件**,
    # 而 path 都是相对各自根的同一文件名。执行层只拿到 path 的话不知道该用哪个根
    # 去 load —— 猜错就是放错动作,而 combo 包**会动臂**(伤害量级和手不同)。
    kind: str = "gesture_pack"

    @classmethod
    def from_list_item(cls, it: dict, kind: str = "gesture_pack") -> "PackTarget":
        """从 gesture_pack / combo_pack 的 list_packs() 一项来。坏包(带 error)不该进池。"""
        return cls(path=str(it.get("path") or ""), name=str(it.get("name") or ""),
                   frames=int(it.get("frames") or 0),
                   duration_ms=int(it.get("duration_ms") or 0), kind=kind)


@dataclass
class _Target:
    """打分池里的统一条目。技能和技能包在这一层长得一样,匹配逻辑才只有一份。"""
    kind: str                       # skill | gesture_pack
    key: str                        # skill_id 或 pack 的 rel path
    name: str
    keys: list[str]                 # 参与匹配的所有字符串
    need_confirm: bool
    spec: object | None = None      # 技能才有


def _pool(reg: SkillRegistry, packs) -> list[_Target]:
    """把技能和技能包统一成一个打分池。"""
    out = [_Target("skill", s.id, s.name, [s.id, s.name, *s.aliases],
                   s.safety.need_confirm, s) for s in reg]
    for p in (packs or []):
        if not p.path or not p.name:
            continue
        # 包只用**它自己的名字**匹配:名字是录制时人起的,没有别名表。
        # kind 从 PackTarget 带过来(gesture_pack / combo_pack)—— 不写死,
        # 否则 combo 包会被当手势包去 data/gestures/ 里找,必然 404。
        out.append(_Target(p.kind, p.path, p.name, [p.name],
                           p.need_confirm, None))
    return out


@dataclass
class Intent:
    """解析结果。skill_id 为 None 时看 reason 知道为什么没命中。

    reason 取值:
      ok                 命中且唯一
      ambiguous          前两名分差在 margin 内 —— 重名不猜,candidates 非空
      no_match           最高分没过 threshold
      not_voice_enabled  命中了,但清单里该技能 voice_enabled=false
    """
    text: str
    skill_id: str | None = None
    name: str | None = None
    confidence: float = 0.0
    params: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    candidates: list[Candidate] = field(default_factory=list)
    reason: str = "no_match"
    need_confirm: bool = True      # 前端据此弹确认框;执行层仍会独立再查一遍
    kind: str = "skill"            # skill | gesture_pack
    pack_path: str | None = None   # kind=gesture_pack 时的相对路径(重名靠它区分)

    @property
    def ok(self) -> bool:
        return self.skill_id is not None or self.pack_path is not None

    def envelope(self, source: str = "voice", confirmed: bool = False,
                 request_id: str | None = None) -> dict:
        """生成执行层要的调用信封。原话一起带上 —— 执行层会把
        (transcript, skill_id) 成对落盘,那就是以后 VLA 要的标注原料。"""
        if not self.ok:
            raise ValueError(f"意图未命中({self.reason}),不能生成信封")
        env = {"skill_id": self.skill_id, "params": dict(self.params),
               "source": source, "confirmed": bool(confirmed),
               "transcript": self.text, "confidence": round(self.confidence, 3),
               "kind": self.kind}
        if self.pack_path:
            env["pack_path"] = self.pack_path
        if request_id:
            env["request_id"] = request_id
        return env

    def to_public(self) -> dict:
        return {"text": self.text, "skill_id": self.skill_id, "name": self.name,
                "confidence": round(self.confidence, 3), "params": self.params,
                "notes": self.notes, "reason": self.reason, "ok": self.ok,
                "need_confirm": self.need_confirm, "kind": self.kind,
                "pack_path": self.pack_path,
                "candidates": [c.to_public() for c in self.candidates]}


def _apply_mods(spec, mods: dict) -> tuple[dict, list[str]]:
    """把修饰词落到技能**自己声明过**的参数上。

    没声明就只提示、不硬塞 —— 硬塞了也会被 resolve_params 当未声明键丢掉,
    不如在这里就说清楚为什么「快一点」没生效。
    """
    params: dict = {}
    notes: list[str] = []
    has_dur = "duration" in spec.params
    has_spd = "speed" in spec.params

    if "seconds" in mods:
        if has_dur:
            params["duration"] = mods["seconds"]
        else:
            notes.append(f"技能「{spec.name}」没有 duration 参数,"
                         f"忽略「{mods['seconds']:g} 秒」")
    if "times" in mods:
        if has_spd:
            params["speed"] = mods["times"]
        else:
            notes.append(f"技能「{spec.name}」没有 speed 参数,"
                         f"忽略「{mods['times']:g} 倍」")

    if mods.get("slow") or mods.get("fast"):
        slow = bool(mods.get("slow"))
        word = "慢" if slow else "快"
        # duration 优先于 speed:点位技能只有 duration,轨迹技能两者都可能有,
        # 而「慢一点」对轨迹的自然含义是降速而非拉长接近段。
        # 缩放需要一个基准值,所以要求该参数声明了 default;没 default 就没法算。
        def _scalable(k: str) -> bool:
            return k in spec.params and k not in params \
                and spec.params[k].default is not None

        target = ("duration" if _scalable("duration")
                  else "speed" if _scalable("speed") else None)
        if target == "duration":
            base = spec.params["duration"].default
            v = float(base) * (DURATION_FACTOR if slow else 1 / DURATION_FACTOR)
            params["duration"] = round(v, 3)
            notes.append(f"「{word}」→ duration {params['duration']:g}s")
        elif target == "speed":
            base = spec.params["speed"].default
            v = float(base) * (1 / SPEED_FACTOR if slow else SPEED_FACTOR)
            params["speed"] = round(v, 3)
            notes.append(f"「{word}」→ speed {params['speed']:g}×")
        elif has_dur or has_spd:
            notes.append(f"已给了显式数值,忽略「{word}」")
        else:
            notes.append(f"技能「{spec.name}」没有可调快慢的参数,忽略「{word}」")

    if "grip" in mods:
        lvl = mods["grip"]
        vals = GRIP_LEVELS.get(lvl, {})
        # 只落到技能**自己声明过**的参数上 —— 和快慢修饰同一条规则。
        # 没声明就提示,别硬塞:硬塞会被 resolve_params 丢掉,现象是"说了没反应"
        # 而且看不出原因。手部动作技能要支持力度就得声明 hand_speed/hand_force。
        applied = {k: v for k, v in vals.items() if k in spec.params}
        if applied:
            params.update(applied)
            word = {"soft": "轻", "firm": "用力"}.get(lvl, lvl)
            notes.append(f"「{word}」→ 手速度 {applied.get('hand_speed', '-')}"
                         f" / 力控 {applied.get('hand_force', '-')}")
        else:
            notes.append(f"技能「{spec.name}」不支持调力度(没声明 "
                         f"hand_speed/hand_force),忽略力度修饰")
    return params, notes


def _finish(result: Intent, tg: _Target, conf: float, mods: dict,
            notes: list[str], voice_only: bool) -> Intent:
    """填好命中结果。voice_enabled 检查放这里 —— 三条命中路径都要过它。"""
    result.candidates = [Candidate(tg.key, tg.name, conf, tg.name, tg.kind)]
    result.confidence = conf
    result.need_confirm = tg.need_confirm
    result.kind = tg.kind

    if tg.kind in PACK_KINDS:
        # 包没有 voice_enabled 开关:它不在静态清单里,不存在"清单是否许可"这件事。
        # 门槛落在别处 —— 要接入设备、要二次确认,由执行层强制。
        result.pack_path, result.name = tg.key, tg.name
        result.notes = notes + ([] if not mods else
                                ["技能包没有可调参数,快慢/秒数修饰已忽略"])
        result.reason = "ok"
        return result

    spec = tg.spec
    if voice_only and not spec.safety.voice_enabled:
        result.reason = "not_voice_enabled"
        result.notes = notes + [f"技能「{spec.name}」在清单里不允许语音触发"]
        return result
    p, pn = _apply_mods(spec, mods)
    result.skill_id, result.name = spec.id, spec.name
    result.params, result.notes = p, notes + pn
    result.reason = "ok"
    return result


def parse(text: str, reg: SkillRegistry | None = None, voice_only: bool = True,
          threshold: float = 0.58, margin: float = 0.12,
          packs: list[PackTarget] | None = None) -> Intent:
    """一句话 → Intent。三步:整句精确 → 剥修饰后精确 → 模糊。

    voice_only=True(语音路径默认)时,voice_enabled=false 的技能即使命中也拒,
    且明说是清单不许 —— 报「听不懂」会让人以为是识别问题,白排查半天。

    packs 传入已录制的手势技能包(`PackTarget`),与技能**同池打分**。不传就是纯技能,
    行为与加这个参数之前完全一致。

    margin=0.12 而不是 0.10:极性词 + 对象名的组合会踩到 _sim 的包含关系加权。
    「下使能机械臂」对「使能机械臂」得 0.895(多出来的「下」被当成"多说一个字"),
    对「下使能」只得 0.785 —— 分差恰好 0.11,压着 0.10 过线,于是**确信地判反**:
    说下使能而臂被使能,人此时很可能正伸手去扶。抬到 0.12 让它落进 ambiguous。
    代价实测为零:check_margin_sweep.py 在 3616 条口语说法上 hit 数不变(3010→3010),
    危险集 wrong 4→0。这只买到"不判反",买不到"判得对" —— 后者要靠句向量那一层。
    """
    reg = reg or get_registry()
    raw = str(text or "").strip()
    result = Intent(text=raw)
    if not raw:
        result.notes = ["空输入"]
        return result
    pool = _pool(reg, packs)

    def exact(s: str) -> list[_Target]:
        """整池精确命中。返回**全部**命中项 —— 走池而不是 reg.by_alias 那张 dict,
        因为 dict 一个键只留一个赢家,包名和技能名撞车会被悄悄吞掉。"""
        k = _norm(s)
        return [t for t in pool if any(_norm(x) == k for x in t.keys)]

    # 第 1 步:整句精确。不剥修饰词 —— 「慢一点」本身就是一条技能的别名。
    hits = exact(raw)
    if len(hits) == 1:
        return _finish(result, hits[0], 1.0, {}, [], voice_only)
    if len(hits) > 1:
        return _ambiguous(result, hits, 1.0, [])

    # 第 2 步:剥掉修饰词与显式数值,剩下的主干再精确查一次。
    stem, mods, notes = _extract(raw)
    if not stem:
        result.notes = notes + ["只说了修饰词,没说动作"]
        return result
    hits = exact(stem)
    if len(hits) == 1:
        return _finish(result, hits[0], 0.95, mods, notes, voice_only)
    if len(hits) > 1:
        return _ambiguous(result, hits, 0.95, notes)

    # 第 3 步:模糊匹配。每个目标取它所有匹配串里的最高分。
    q = _norm(stem)
    scored: list[tuple[float, str, _Target]] = []
    for t in pool:
        best, best_key = 0.0, ""
        for key in t.keys:
            sc = _sim(q, _norm(key))
            if sc > best:
                best, best_key = sc, key
        if best > 0:
            scored.append((best, best_key, t))
    scored.sort(key=lambda x: (-x[0], x[2].key))
    result.candidates = [Candidate(t.key, t.name, sc, mk, t.kind)
                         for sc, mk, t in scored[:5]]
    result.notes = notes
    if not scored or scored[0][0] < threshold:
        result.reason = "no_match"
        return result
    # 重名不猜:前两名咬得太近就交回去让人选,绝不随机命中。
    if len(scored) > 1 and (scored[0][0] - scored[1][0]) < margin:
        result.reason = "ambiguous"
        result.confidence = scored[0][0]
        return result
    return _finish(result, scored[0][2], scored[0][0], mods, notes, voice_only)


def _ambiguous(result: Intent, hits: list[_Target], conf: float,
               notes: list[str]) -> Intent:
    """精确命中了多个目标(同名技能包,或包名撞技能名)。不猜,交回候选。"""
    result.candidates = [Candidate(t.key, t.name, conf, t.name, t.kind)
                         for t in hits]
    result.confidence = conf
    result.reason = "ambiguous"
    result.notes = notes + [f"「{result.text}」同时命中 {len(hits)} 个目标"]
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("text", nargs="*", help="要解析的话")
    ap.add_argument("--all", action="store_true", help="清单里每条别名回归一遍")
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    ap.add_argument("--any", action="store_true", help="不限语音白名单")
    ap.add_argument("--packs", action="store_true",
                    help="把磁盘上已录制的手势技能包也放进池子一起匹配")
    args = ap.parse_args()
    try:
        reg = get_registry()
    except RegistryError as e:
        print(f"✗ 清单有问题: {e}")
        return 1

    packs = None
    if args.packs:
        sys.path.insert(0, str(SIM_DIR))
        import gesture_pack as gp                     # 只在要用时 import
        items = [it for it in gp.list_packs() if not it.get("error")]
        packs = [PackTarget.from_list_item(it) for it in items]
        print(f"(池子里加了 {len(packs)} 个技能包)")

    if args.all:
        bad = total = 0
        for s in reg:
            for key in [s.id, s.name, *s.aliases]:
                total += 1
                it = parse(key, reg, voice_only=not args.any, packs=packs)
                if it.skill_id != s.id:
                    print(f"✗ {key!r} → {it.skill_id}({it.reason}),应为 {s.id}")
                    bad += 1
        print(f"别名回归: {total - bad}/{total} 命中"
              + ("  全部通过" if not bad else ""))
        return 1 if bad else 0

    if not args.text:
        ap.error("给一句话,或用 --all")
    for t in args.text:
        it = parse(t, reg, voice_only=not args.any, packs=packs)
        if args.json:
            print(json.dumps(it.to_public(), ensure_ascii=False))
            continue
        tag = ("[臂+手包] " if it.kind == "combo_pack" else
               "[包] " if it.kind == "gesture_pack" else "")
        print(f"{'✓' if it.ok else '✗'} {t!r} → {tag}"
              f"{it.pack_path or it.skill_id or '—'}  "
              f"conf={it.confidence:.2f}  {it.reason}"
              + (f"  确认={'要' if it.need_confirm else '免'}" if it.ok else ""))
        if it.params:
            print(f"    params: {json.dumps(it.params, ensure_ascii=False)}")
        for n in it.notes:
            print(f"    · {n}")
        if it.reason in ("ambiguous", "no_match"):
            for c in it.candidates:
                kt = {"combo_pack": "臂+手包", "gesture_pack": "包"}.get(c.kind, "技能")
                print(f"    候选[{kt}] {c.skill_id:24} {c.score:.3f} ← {c.matched}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
