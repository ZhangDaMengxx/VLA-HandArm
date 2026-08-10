#!/usr/bin/env python3
"""sim/skills/schema.py — 技能清单的加载 + 校验 + 参数归一。

纯 Python,只依赖 PyYAML(system python3 5.4.1 / gradio_venv 6.0.3 都有)。
**不 import rclpy / numpy / rerun**,所以 app_web.py(gradio venv)和
执行器(ROS2 system python3)可以共用这一个模块,清单不会出现两份解释。

职责边界:
  - 本模块只管「清单长得对不对」和「参数落不落在 range 内」。
  - 不碰 ROS、不发消息、不读 npz 内容。执行是 runner/backend 的事。

自检:
    python3 sim/skills/schema.py            # 打印技能表
    python3 sim/skills/schema.py --json     # 输出 JSON(给前端/调试)
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# 本模块被两种方式 import:`skills.schema`(包)和裸 `schema`(调用方把 skills/
# 塞进 sys.path)。两种都要能拿到 hand_pose,所以两种都试。
try:
    from . import hand_pose as hp                       # 包内相对
except ImportError:                                     # pragma: no cover
    import hand_pose as hp                              # type: ignore  # 裸模块

# 仓库根:本文件在 <repo>/sim/skills/schema.py
REPO = Path(__file__).resolve().parents[2]
REGISTRY_PATH = Path(__file__).resolve().parent / "registry.yaml"
# 手势表:纯造型手势的真源,加载期合成成 primitive 技能并进同一份清单。
GESTURES_PATH = Path(__file__).resolve().parent / "gestures.yaml"
GESTURE_ID_PREFIX = "hand_"
# 手势表 defaults 允许的键(每个手势也能自己覆盖同名键)
GESTURE_DEFAULT_KEYS = ("duration", "hand_speed", "hand_force", "requires",
                        "voice_enabled", "need_confirm")
# 单个手势条目允许的键
GESTURE_ENTRY_KEYS = ("name", "aliases", "desc", "pose") + GESTURE_DEFAULT_KEYS

ALLOWED_KINDS = ("primitive", "trajectory", "composite")
ALLOWED_REQUIRES = ("live_session", "arm_enabled")
ALLOWED_PARAM_TYPES = ("float", "int", "bool", "str")

# 每种 kind 必须有的专属字段
KIND_REQUIRED_FIELDS = {
    "primitive": ("action",),
    "trajectory": ("source",),
    "composite": ("steps",),
}
# 顶层允许的键:公共 + 全部 kind 专属(逐条校验时再查 kind 匹配)
COMMON_FIELDS = ("id", "name", "aliases", "kind", "desc", "params",
                 "requires", "safety")
KIND_FIELDS = ("action", "pose", "source", "fps", "steps", "mode",
               "params_passthrough")
# composite 的展开方式。
#   sequence 逐条按序发,每条等自己的 duration —— 原来唯一的行为,仍是默认。
#   overlay  合成**一条**指令同时发 —— 臂和手是两个 console 进程,写各自的 stdin,
#            臂的 move_j 阻塞不了手,所以这是真并发,不是假原子性。
ALLOWED_COMPOSITE_MODES = ("sequence", "overlay")
# composite 的哪些参数**透传给每一个子步骤**。默认空 = 老行为(子步骤参数在清单里
# 写死,不受外部影响),所以现有 composite 一个字都不用改。
#
# 为什么需要它:把 hand_close 拆成"先折拇指、再收四指"两步之后,力度修饰
# (「用力握拳」→ hand_speed/hand_force)本来会**静默失效** —— 修饰词只落到技能
# 自己声明过的参数上(intent._apply_mods),而 composite 的子步骤
# `resolve_params(st.get("params"))` 不收外部值(backend.py 那行注释)。
# 于是"拆成原子技能"这个纯内部重构会退化一个对用户可见的功能。
#
# 只允许透传**力度类**参数,不允许 duration:
#   力度是"整个动作用多大劲",各阶段本该一致 —— 透传语义清楚。
#   duration 是每一步各自的节拍,透传等于把所有阶段拉成同一时长,那是错的
#   (折拇指 0.8s、收四指 0.8s,不是一条 1.6s 的指令)。
ALLOWED_PASSTHROUGH_PARAMS = ("hand_speed", "hand_force")
# action 里允许的键。pose 在加载期被展开成 hand,所以它只出现在清单里,
# 展开后的 action 不再有它(backend 那边的 known 集合也就不用加)。
ALLOWED_ACTION_KEYS = ("arm", "hand", "pose", "duration", "action", "value",
                       "estop", "hand_speed", "hand_force")


class RegistryError(Exception):
    """清单本身写错了(拼写、缺字段、id 重名、别名撞车)。一次抛出全部问题。"""


@dataclass
class ParamSpec:
    """一个可调参数的声明。range 越界时夹取,不报错——语音说"快十倍"也只到上限。"""
    name: str
    type: str = "float"
    default: Any = None
    range: tuple | None = None          # 仅 float/int 有意义

    def coerce(self, value: Any) -> tuple[Any, bool]:
        """把外部传入值转成声明类型并夹进 range。返回 (值, 是否被夹取/回退)。"""
        if value is None:
            return self.default, False
        try:
            if self.type == "float":
                v: Any = float(value)
            elif self.type == "int":
                v = int(value)
            elif self.type == "bool":
                v = value if isinstance(value, bool) else str(value).lower() in ("1", "true", "yes", "y", "on")
            else:
                v = str(value)
        except (TypeError, ValueError):
            return self.default, True           # 转不动就回退默认值,并标记
        if self.range and self.type in ("float", "int"):
            lo, hi = self.range
            cv = max(lo, min(hi, v))
            if cv != v:
                return (type(v)(cv), True)
            return v, False
        return v, False


@dataclass
class SafetySpec:
    """安全约束。默认全部保守:不许语音、要确认。清单里没写就是最严。"""
    voice_enabled: bool = False
    need_confirm: bool = True
    max_speed: float | None = None      # 语音路径下压 params.speed 的硬上限


@dataclass
class SkillSpec:
    """一条技能。上层只认 id,执行细节由 kind + 专属字段决定。"""
    id: str
    name: str
    kind: str
    desc: str = ""
    aliases: list[str] = field(default_factory=list)
    params: dict[str, ParamSpec] = field(default_factory=dict)
    requires: list[str] = field(default_factory=list)
    safety: SafetySpec = field(default_factory=SafetySpec)
    # kind 专属
    action: dict | None = None                    # primitive
    source: str | None = None                     # trajectory(相对仓库根)
    fps: float = 30.0                             # trajectory
    steps: list[dict] = field(default_factory=list)   # composite
    mode: str = "sequence"                            # composite: sequence | overlay
    # composite: 透传给每个子步骤的参数名。空 = 子步骤参数在清单里写死(老行为)。
    passthrough: tuple[str, ...] = ()
    # 手势规格原文(清单写了 pose 才有)。action.hand 已由它在加载期展开而来,
    # 留着只为显示/调试 —— 执行侧一律读 action.hand,不要在执行路径上再展开一次。
    pose: dict | None = None

    # ---- 便捷派生 ----
    @property
    def source_path(self) -> Path | None:
        return (REPO / self.source) if self.source else None

    def source_exists(self) -> bool:
        p = self.source_path
        return bool(p and p.exists())

    def resolve_params(self, given: dict | None = None,
                       via_voice: bool = False) -> tuple[dict, list[str]]:
        """把外部参数归一成完整参数字典。返回 (参数, 提示列表)。
        未声明的键直接丢弃并提示——防止上层塞进意外字段被后端当真。"""
        given = dict(given or {})
        out, notes = {}, []
        for key in given:
            if key not in self.params:
                notes.append(f"忽略未声明参数 {key}")
        for key, spec in self.params.items():
            val, clamped = spec.coerce(given.get(key))
            if clamped:
                notes.append(f"参数 {key} 已夹取/回退为 {val}")
            out[key] = val
        # 语音路径的速度硬上限:压过用户给的任何值
        if via_voice and self.safety.max_speed is not None and "speed" in out:
            if out["speed"] is not None and out["speed"] > self.safety.max_speed:
                notes.append(f"语音路径限速 {self.safety.max_speed}(原 {out['speed']})")
                out["speed"] = self.safety.max_speed
        return out, notes

    def enable_effect(self, reg: "SkillRegistry | None" = None) -> bool | None:
        """本技能执行后使能状态变成什么:True 使能 / False 失能 / None 不影响。

        从 action 推导,不硬编码 id —— 清单里改了动作,跟踪逻辑自动跟上。
        composite 取最后一个有影响的子步骤(后发生的覆盖先发生的)。
        """
        if self.kind == "primitive":
            a = self.action or {}
            if a.get("estop") or a.get("action") == "disable":
                return False
            if a.get("action") == "enable":
                return True
            return None
        if self.kind == "composite" and reg is not None:
            eff = None
            for st in self.steps:
                child = reg.get(st.get("skill")) if isinstance(st, dict) else None
                if child is not None:
                    e = child.enable_effect(reg)
                    if e is not None:
                        eff = e
            return eff
        return None

    def to_public(self) -> dict:
        """给前端的安全视图:不暴露 action 原始关节值,避免前端绕过 runner 直发。"""
        return {
            "id": self.id, "name": self.name, "kind": self.kind, "desc": self.desc,
            "aliases": list(self.aliases),
            "params": {k: {"type": v.type, "default": v.default,
                           "range": list(v.range) if v.range else None}
                       for k, v in self.params.items()},
            "requires": list(self.requires),
            # pose 可以给前端 —— 它是语义标签("拇指对掌"),不是能绕过 runner 直发的
            # 关节值。action.hand 仍然不暴露。
            "pose": dict(self.pose) if isinstance(self.pose, dict) else self.pose,
            "voice_enabled": self.safety.voice_enabled,
            "need_confirm": self.safety.need_confirm,
            "ready": self.source_exists() if self.kind == "trajectory" else True,
        }


def _parse_params(raw: Any, sid: str, errs: list[str]) -> dict[str, ParamSpec]:
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        errs.append(f"[{sid}] params 必须是映射")
        return {}
    out = {}
    for key, d in raw.items():
        if not isinstance(d, dict):
            errs.append(f"[{sid}] params.{key} 必须是映射")
            continue
        unknown = set(d) - {"type", "default", "range"}
        if unknown:
            errs.append(f"[{sid}] params.{key} 未知字段 {sorted(unknown)}")
        ptype = d.get("type", "float")
        if ptype not in ALLOWED_PARAM_TYPES:
            errs.append(f"[{sid}] params.{key}.type={ptype} 不在 {ALLOWED_PARAM_TYPES}")
            continue
        rng = d.get("range")
        if rng is not None:
            if not (isinstance(rng, (list, tuple)) and len(rng) == 2):
                errs.append(f"[{sid}] params.{key}.range 必须是 [lo, hi]")
                rng = None
            elif rng[0] > rng[1]:
                errs.append(f"[{sid}] params.{key}.range 下界大于上界")
                rng = None
            else:
                rng = (rng[0], rng[1])
        out[key] = ParamSpec(name=key, type=ptype, default=d.get("default"), range=rng)
    return out


def _parse_safety(raw: Any, sid: str, errs: list[str]) -> SafetySpec:
    if raw is None:
        return SafetySpec()
    if not isinstance(raw, dict):
        errs.append(f"[{sid}] safety 必须是映射")
        return SafetySpec()
    unknown = set(raw) - {"voice_enabled", "need_confirm", "max_speed"}
    if unknown:
        errs.append(f"[{sid}] safety 未知字段 {sorted(unknown)}")
    ms = raw.get("max_speed")
    return SafetySpec(
        voice_enabled=bool(raw.get("voice_enabled", False)),
        need_confirm=bool(raw.get("need_confirm", True)),
        max_speed=float(ms) if ms is not None else None,
    )


def _expand_pose(raw: dict, sid: str, errs: list[str]) -> tuple[dict | None, dict | None]:
    """把 pose(五指语义)展开成 action.hand(6 弧度),并查可行域。

    入参是整条 raw(顶层),因为 pose 和 action 是平级的,不是 pose 在 action 里。
    展开发生在**加载期**,所以 backend/console_exec/runner/前端全都只看见 hand,
    一个字都不用改。pose 原文留在 SkillSpec.pose 里,给前端显示和调试用。

    可行域不通过 = **加载失败**,不是警告。理由:清单里写个互顶的姿态,真机上的
    表现是两个关节卡住、电机持续堵转(ERROR Bit0),而过温位是**不可清除**的。
    宁可加载时炸,不要真机上闷着烧。现有两条(hand_pinch/hand_close)都实测过、
    都通过,所以这不是给现状放宽的门。

    返回 (action_dict, pose_原文)。pose 不存在 / 展开失败 → (None, None)。
    """
    pose_src = raw.get("pose")
    action_src = raw.get("action") or {}
    if not isinstance(action_src, dict):
        action_src = {}
    # 先看直接写 hand 的老写法,无论有没有 pose 都要查
    unknown = set(action_src) - set(ALLOWED_ACTION_KEYS)
    if unknown:
        errs.append(f"[{sid}] action 未知键 {sorted(unknown)}"
                    f"(可选 {list(ALLOWED_ACTION_KEYS)})")
    if "hand" in action_src:
        h = action_src["hand"]
        if isinstance(h, (list, tuple)) and len(h) == len(hp.HAND_JOINTS):
            why = hp.check_feasible([float(x) for x in h])
            if why:
                errs.append(f"[{sid}] action.hand 不可行:{why}")
    if pose_src is None:
        return action_src, None
    if "hand" in action_src:
        errs.append(f"[{sid}] 不能同时给 pose 和 action.hand —— 二者都指定手的角度,"
                    f"留一个。想看 pose 展开成什么,跑 hand_pose.py")
        return action_src, None
    try:
        rad6, why = hp.compile_pose(pose_src)
    except hp.PoseError as e:
        errs.append(f"[{sid}] pose 写错了:{e}")
        return action_src, None
    if why:
        errs.append(f"[{sid}] pose 不可行:{why}")
    out = dict(action_src)
    out["hand"] = [round(r, 4) for r in rad6]
    return out, dict(pose_src) if isinstance(pose_src, dict) else pose_src


def _parse_one(raw: dict, errs: list[str]) -> SkillSpec | None:
    sid = str(raw.get("id", "")).strip()
    if not sid:
        errs.append("有技能缺 id")
        return None
    unknown = set(raw) - set(COMMON_FIELDS) - set(KIND_FIELDS)
    if unknown:
        errs.append(f"[{sid}] 未知顶层字段 {sorted(unknown)}")
    kind = raw.get("kind")
    if kind not in ALLOWED_KINDS:
        errs.append(f"[{sid}] kind={kind} 不在 {ALLOWED_KINDS}")
        return None
    for f in KIND_REQUIRED_FIELDS[kind]:
        if raw.get(f) is None:
            errs.append(f"[{sid}] kind={kind} 缺必需字段 {f}")
    # kind 专属字段串味检查:primitive 不该有 source/steps,等等
    for other_kind, fields_ in KIND_REQUIRED_FIELDS.items():
        if other_kind == kind:
            continue
        for f in fields_:
            if raw.get(f) is not None:
                errs.append(f"[{sid}] kind={kind} 不该带 {other_kind} 的字段 {f}")
    reqs = raw.get("requires") or []
    if not isinstance(reqs, list):
        errs.append(f"[{sid}] requires 必须是列表")
        reqs = []
    for r in reqs:
        if r not in ALLOWED_REQUIRES:
            errs.append(f"[{sid}] requires 含未知项 {r}(可选 {ALLOWED_REQUIRES})")
    aliases = raw.get("aliases") or []
    if not isinstance(aliases, list):
        errs.append(f"[{sid}] aliases 必须是列表")
        aliases = []
    pose_src = None
    if kind == "primitive":
        action, pose_src = _expand_pose(raw, sid, errs)
    else:
        action = raw.get("action")
    if kind == "composite":
        steps = raw.get("steps") or []
        if not isinstance(steps, list) or not steps:
            errs.append(f"[{sid}] composite 的 steps 必须是非空列表")
            steps = []
        for i, st in enumerate(steps):
            if not isinstance(st, dict) or "skill" not in st:
                errs.append(f"[{sid}] steps[{i}] 必须是含 skill 键的映射")
    mode = str(raw.get("mode", "sequence"))
    if mode not in ALLOWED_COMPOSITE_MODES:
        errs.append(f"[{sid}] mode={mode!r} 不在 {ALLOWED_COMPOSITE_MODES}")
        mode = "sequence"
    if "mode" in raw and kind != "composite":
        errs.append(f"[{sid}] 只有 composite 能写 mode(当前 kind={kind})")
    # params_passthrough:只 composite 能写,且只收力度类参数,且必须在 params 里声明过
    # —— 声明过才有类型/范围校验,也才会被 intent 的修饰词看见。
    pt_raw = raw.get("params_passthrough") or []
    if "params_passthrough" in raw and kind != "composite":
        errs.append(f"[{sid}] 只有 composite 能写 params_passthrough(当前 kind={kind})")
    if not isinstance(pt_raw, (list, tuple)):
        errs.append(f"[{sid}] params_passthrough 必须是列表")
        pt_raw = []
    declared = _parse_params(raw.get("params"), sid, [])      # 只为查声明,错误已在下面报
    passthrough: list[str] = []
    for k in (str(x) for x in pt_raw):
        if k not in ALLOWED_PASSTHROUGH_PARAMS:
            errs.append(f"[{sid}] params_passthrough 不支持 {k!r} —— "
                        f"只能透传 {list(ALLOWED_PASSTHROUGH_PARAMS)}。"
                        f"duration 是每步各自的节拍,透传会把所有阶段拉成同一时长。")
            continue
        if k not in declared:
            errs.append(f"[{sid}] params_passthrough 里的 {k!r} 没在 params 里声明 —— "
                        f"没声明的参数修饰词看不见(intent 只落到声明过的参数上),"
                        f"透传等于空转。")
            continue
        passthrough.append(k)
    return SkillSpec(
        id=sid, name=str(raw.get("name", sid)), kind=kind,
        desc=str(raw.get("desc", "")), aliases=[str(a) for a in aliases],
        params=_parse_params(raw.get("params"), sid, errs),
        requires=[str(r) for r in reqs],
        safety=_parse_safety(raw.get("safety"), sid, errs),
        action=action, source=raw.get("source"),
        fps=float(raw.get("fps", 30.0)), steps=raw.get("steps") or [],
        pose=pose_src, mode=mode, passthrough=tuple(passthrough),
    )


class SkillRegistry:
    """加载好的技能表。别名索引在此建立,意图解析直接查这里。"""

    def __init__(self, skills: list[SkillSpec], version: int = 1,
                 warnings: list[str] | None = None) -> None:
        self.version = version
        self._skills = {s.id: s for s in skills}
        self.warnings = warnings or []
        # 别名 → id。别名统一去空白后小写,匹配时同样处理。
        self._alias: dict[str, str] = {}
        for s in skills:
            for key in [s.id, s.name, *s.aliases]:
                self._alias[_norm(key)] = s.id

    def __len__(self) -> int:
        return len(self._skills)

    def __contains__(self, sid: str) -> bool:
        return sid in self._skills

    def __iter__(self):
        return iter(self._skills.values())

    def get(self, sid: str) -> SkillSpec | None:
        return self._skills.get(sid)

    def ids(self) -> list[str]:
        return list(self._skills)

    def by_alias(self, text: str) -> SkillSpec | None:
        """精确别名匹配(归一化后)。模糊匹配是 intent.py 的事,这里只做确定性查表。"""
        return self._skills.get(self._alias.get(_norm(text), ""))

    def alias_index(self) -> dict[str, str]:
        return dict(self._alias)

    def voice_skills(self) -> list[SkillSpec]:
        return [s for s in self._skills.values() if s.safety.voice_enabled]

    def to_public(self) -> list[dict]:
        return [s.to_public() for s in self._skills.values()]


def _norm(s: str) -> str:
    """别名归一化:去首尾空白、去内部空格、小写。中文不受影响,英文大小写不敏感。"""
    return "".join(str(s).split()).lower()


def _cross_check(skills: list[SkillSpec], errs: list[str], warns: list[str]) -> None:
    """跨条目校验:id 重名、别名撞车、composite 引用与环路、轨迹文件缺失。"""
    seen: dict[str, int] = {}
    for s in skills:
        seen[s.id] = seen.get(s.id, 0) + 1
    for sid, n in seen.items():
        if n > 1:
            errs.append(f"id 重复 {n} 次: {sid}")

    # 别名撞车:同一句话映射到两个技能,语音会随机命中,必须当错误
    owner: dict[str, str] = {}
    for s in skills:
        for key in [s.id, s.name, *s.aliases]:
            k = _norm(key)
            if k in owner and owner[k] != s.id:
                errs.append(f"别名/名称冲突 {key!r}: {owner[k]} 与 {s.id}")
            else:
                owner[k] = s.id

    ids = set(seen)
    for s in skills:
        if s.kind != "composite":
            continue
        for i, st in enumerate(s.steps):
            if not isinstance(st, dict):
                continue
            ref = st.get("skill")
            if ref not in ids:
                errs.append(f"[{s.id}] steps[{i}] 引用不存在的技能 {ref!r}")

    # composite 环路检测(含自引用)
    graph = {s.id: [st.get("skill") for st in s.steps if isinstance(st, dict)]
             for s in skills if s.kind == "composite"}
    state: dict[str, int] = {}                # 0=未访问 1=在栈上 2=已完成

    def walk(node: str, path: list[str]) -> None:
        if state.get(node) == 1:
            errs.append("composite 存在环路: " + " → ".join(path + [node]))
            return
        if state.get(node) == 2:
            return
        state[node] = 1
        for nxt in graph.get(node, []):
            if nxt in graph:
                walk(nxt, path + [node])
        state[node] = 2

    for sid in graph:
        walk(sid, [])

    # 轨迹文件缺失只警告:清单可以先登记、文件后补(或在别的机器上)
    for s in skills:
        if s.kind == "trajectory" and not s.source_exists():
            warns.append(f"[{s.id}] 轨迹文件不存在: {s.source}")


def _synth_gestures(errs: list[str], gpath: Path | None = None) -> list[dict]:
    """读 gestures.yaml,把每行手势**合成一条 primitive 的 raw dict**。

    返回的是"看起来就像写在 registry.yaml 里"的 dict,交给 _parse_one 走完全部
    校验(含 pose 展开和可行域)。所以合成技能和手写技能**同一条校验路径** ——
    不存在"手势表绕过了某项检查"。

    表不存在 = 返回空列表,不报错。手势表是可选的:删了它系统照常跑,只是没手势。

    gpath **跟着清单走**,不是固定的 GESTURES_PATH。这条很关键:测试(和将来任何
    加载临时清单的场合)传的是 /tmp 下的清单,如果这里照读真手势表,那 9 个手势
    就会混进临时清单 —— 测试里"1 条技能的极简清单"变成 10 条,断言全崩。
    默认才用 GESTURES_PATH,即同目录的 gestures.yaml。
    """
    gp = gpath if gpath is not None else GESTURES_PATH
    if not gp.exists():
        return []
    try:
        with open(gp, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        errs.append(f"手势表 {gp.name} 不是合法 YAML: {e}")
        return []
    if not isinstance(doc, dict):
        errs.append(f"手势表顶层必须是映射(含 version / defaults / gestures)")
        return []
    unknown_top = set(doc) - {"version", "defaults", "gestures"}
    if unknown_top:
        errs.append(f"手势表顶层未知字段 {sorted(unknown_top)}")
    dflt = doc.get("defaults") or {}
    if not isinstance(dflt, dict):
        errs.append("手势表 defaults 必须是映射")
        dflt = {}
    unknown_d = set(dflt) - set(GESTURE_DEFAULT_KEYS)
    if unknown_d:
        errs.append(f"手势表 defaults 未知字段 {sorted(unknown_d)}"
                    f"(可选 {list(GESTURE_DEFAULT_KEYS)})")
    gs = doc.get("gestures") or {}
    if not isinstance(gs, dict) or not gs:
        errs.append("手势表 gestures 必须是非空映射")
        return []

    out = []
    for key, g in gs.items():
        sid = f"{GESTURE_ID_PREFIX}{key}"
        if not isinstance(g, dict):
            errs.append(f"[{sid}] 手势条目必须是映射")
            continue
        unknown_g = set(g) - set(GESTURE_ENTRY_KEYS)
        if unknown_g:
            errs.append(f"[{sid}] 手势未知字段 {sorted(unknown_g)}"
                        f"(可选 {list(GESTURE_ENTRY_KEYS)})")
        if "pose" not in g:
            errs.append(f"[{sid}] 手势缺 pose")
            continue
        # 逐键取值:手势自己写了就用它的,否则用 defaults
        def pick(k, fallback=None):
            return g[k] if k in g else dflt.get(k, fallback)
        out.append({
            "id": sid,
            "name": g.get("name", key),
            "aliases": g.get("aliases") or [],
            "kind": "primitive",
            "desc": g.get("desc") or f"{g.get('name', key)}(手势表合成,纯造型不抓取)",
            "pose": g["pose"],
            "action": {
                "duration": pick("duration", 1.5),
                "hand_speed": pick("hand_speed", 150),
                "hand_force": pick("hand_force", 250),
            },
            # 力度修饰(「轻一点比个1」)要落得进来,就得声明这两个参数 ——
            # _apply_mods 只认技能自己声明过的参数。
            "params": {
                "hand_speed": {"type": "int", "default": pick("hand_speed", 150),
                               "range": [20, 1000]},
                "hand_force": {"type": "int", "default": pick("hand_force", 250),
                               "range": [50, 1000]},
            },
            "requires": list(pick("requires", ["live_session"])),
            "safety": {"voice_enabled": bool(pick("voice_enabled", True)),
                       "need_confirm": bool(pick("need_confirm", True))},
        })
    return out


def load_registry(path: str | Path | None = None) -> SkillRegistry:
    """读 registry.yaml + gestures.yaml → 校验 → SkillRegistry。

    两个来源合并成一份清单。手势表的条目在**合成后**走同一条校验路径,
    所以 id 撞车、别名撞车都由 _cross_check 一并查出来。
    """
    p = Path(path) if path else REGISTRY_PATH
    if not p.exists():
        raise RegistryError(f"技能清单不存在: {p}")
    with open(p, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        raise RegistryError("清单顶层必须是映射(含 version / skills)")
    unknown_top = set(doc) - {"version", "skills"}
    if unknown_top:
        raise RegistryError(f"清单顶层未知字段 {sorted(unknown_top)}")
    raw_list = doc.get("skills")
    if not isinstance(raw_list, list) or not raw_list:
        raise RegistryError("skills 必须是非空列表")

    errs: list[str] = []
    warns: list[str] = []
    # 手势表合成的条目排在手写清单**后面**:_cross_check 报别名撞车时,
    # 先出现的那条算"原主",提示里指的就是手写清单那条 —— 更符合直觉。
    synth = _synth_gestures(errs, p.parent / GESTURES_PATH.name)
    all_raw = list(raw_list) + synth
    skills = [s for s in (_parse_one(r, errs) for r in all_raw
                          if isinstance(r, dict)) if s is not None]
    if len(skills) != len(all_raw):
        errs.append(f"有 {len(all_raw) - len(skills)} 条技能解析失败(见上)")
    _cross_check(skills, errs, warns)
    if errs:
        raise RegistryError(f"技能清单 {p} 有 {len(errs)} 处问题:\n  - "
                            + "\n  - ".join(errs))
    return SkillRegistry(skills, version=int(doc.get("version", 1)), warnings=warns)


# 模块级缓存:app_web 每次请求不重读磁盘;调 load_registry(force=True) 式刷新见 reload()
_cache: SkillRegistry | None = None


def get_registry(reload: bool = False) -> SkillRegistry:
    """取缓存的清单。reload=True 时重读磁盘(调 registry 字段时不用重启 Web)。"""
    global _cache
    if _cache is None or reload:
        _cache = load_registry()
    return _cache


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", action="store_true", help="输出前端视图 JSON")
    ap.add_argument("--registry", default=None, help="指定清单路径(默认同目录)")
    args = ap.parse_args()

    try:
        reg = load_registry(args.registry)
    except RegistryError as e:
        print(f"✗ {e}")
        return 1

    if args.json:
        print(json.dumps({"version": reg.version, "skills": reg.to_public(),
                          "warnings": reg.warnings},
                         ensure_ascii=False, indent=2))
        return 0

    print(f"✓ 清单校验通过 · version={reg.version} · {len(reg)} 条技能")
    print(f"{'id':22} {'kind':11} {'语音':4} {'确认':4} {'就绪':4} 名称")
    print("-" * 78)
    for s in reg:
        ready = "-" if s.kind != "trajectory" else ("是" if s.source_exists() else "缺")
        print(f"{s.id:22} {s.kind:11} {'是' if s.safety.voice_enabled else '否':4} "
              f"{'是' if s.safety.need_confirm else '否':4} {ready:4} {s.name}")
    print("-" * 78)
    print(f"语音可命中: {len(reg.voice_skills())} 条 · 别名索引: {len(reg.alias_index())} 项")
    for w in reg.warnings:
        print(f"⚠ {w}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
