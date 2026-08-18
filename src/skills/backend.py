#!/usr/bin/env python3
"""src/skills/backend.py — 技能 → writer 指令序列的展开。

设计边界(照 nero_vla_bridge/retarget_backend.py 的 TrajectorySource 写法):
  上层只认 skill_id;本模块把它展开成一串**给 JointWriter.send() 的 dict**。
  本模块不 import rclpy、不发消息、不 sleep —— 纯展开,可离线单测。
  真正的发布与节流由 runner.py 负责。

指令契约(与 src/ros_joint_writer.py 的 send() 完全一致,不新增方言):
  {"arm": [7], "hand": [6], "duration": s}   关节目标
  {"action": "enable|disable|reset|set_speed", "value": ...}   SDK 级
  {"estop": true}                            急停

需要 numpy(读 trajectory 的 npz),所以只在 ROS2 system python3 侧 import。
app_web 只 import schema,不 import 本模块。

自检:
    python3 src/skills/backend.py            # 干跑全部技能,打印指令条数
    python3 src/skills/backend.py --id go_home --dump
"""
from __future__ import annotations

import argparse
import json
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Iterator

sys.path.insert(0, str(Path(__file__).resolve().parent))
from schema import RegistryError, SkillRegistry, SkillSpec, get_registry  # noqa: E402

# writer 的 6 个驱动手关节顺序(与 ros_joint_writer.HAND_NAMES 一致)。
# 这里重复声明是为了让 backend 可以脱离 ROS 环境单测;runner 会断言两者一致。
HAND_NAMES = ["right_thumb_1_joint", "right_thumb_2_joint",
              "right_index_1_joint", "right_middle_1_joint",
              "right_ring_1_joint", "right_little_1_joint"]
ARM_DOF, HAND_DOF = 7, 6


class SkillError(Exception):
    """技能没法展开(轨迹文件缺失、action 维度不对、参数非法)。"""


class Step:
    """一条待下发指令 + 它的节拍。

    cmd    给 JointWriter.send() 的 dict
    hold   发完后等多久(秒)才发下一条。primitive 用 duration,轨迹用 1/(fps*speed)。
    label  进度显示用
    """

    __slots__ = ("cmd", "hold", "label")

    def __init__(self, cmd: dict, hold: float = 0.0, label: str = "") -> None:
        self.cmd = cmd
        self.hold = float(hold)
        self.label = label

    def __repr__(self) -> str:
        return f"Step({json.dumps(self.cmd, ensure_ascii=False)}, hold={self.hold:.3f})"


class SkillBackend(ABC):
    """技能展开器。每种 kind 一个实现;runner 只依赖这个抽象。"""

    def __init__(self, spec: SkillSpec, reg: SkillRegistry) -> None:
        self.spec = spec
        self.reg = reg

    @abstractmethod
    def total(self, params: dict) -> int:
        """预估总步数,给进度条用。轨迹是帧数,primitive 是 1。"""

    @abstractmethod
    def steps(self, params: dict) -> Iterator[Step]:
        """按序产出指令。生成器 —— 轨迹几百帧不必一次全建。"""

    def duration_hint(self, params: dict) -> float:
        """预估总时长(秒),给前端显示和超时保护。"""
        return 0.0


class PrimitiveBackend(SkillBackend):
    """primitive:action 直接就是 writer 指令,只做维度校验和 duration 覆盖。"""

    def __init__(self, spec: SkillSpec, reg: SkillRegistry) -> None:
        super().__init__(spec, reg)
        a = spec.action
        if not isinstance(a, dict) or not a:
            raise SkillError(f"[{spec.id}] action 必须是非空映射")
        # 维度校验放在构造期:清单写错在加载技能时就炸,不等到真机执行才发现
        if "arm" in a and len(a["arm"]) != ARM_DOF:
            raise SkillError(f"[{spec.id}] action.arm 需要 {ARM_DOF} 个值,给了 {len(a['arm'])}")
        if "hand" in a and len(a["hand"]) != HAND_DOF:
            raise SkillError(f"[{spec.id}] action.hand 需要 {HAND_DOF} 个值(驱动关节),"
                             f"给了 {len(a['hand'])}")
        # hand_speed / hand_force:手的速度和力控阈值(0-1000,逐通道或标量)。
        # 它们是**状态**不是动作 —— 设了就一直有效,所以一条技能可以只设它们、
        # 不带任何角度(抓握前先定力度就是这个用法)。
        # ⚠ 只在 console 直连路有效;ROS 那条路的 JointTrajectory 没有力控字段,
        #   ros_joint_writer 会在回显里报 unsupported 而不是静默忽略。
        known = {"arm", "hand", "duration", "action", "value", "estop",
                 "hand_speed", "hand_force"}
        unknown = set(a) - known
        if unknown:
            raise SkillError(f"[{spec.id}] action 含 writer 不认识的键 {sorted(unknown)}")
        for k in ("hand_speed", "hand_force"):
            v = a.get(k)
            if isinstance(v, (list, tuple)) and len(v) != HAND_DOF:
                raise SkillError(f"[{spec.id}] action.{k} 给列表时需要 {HAND_DOF} 个值,"
                                 f"给了 {len(v)}")

    def _cmd(self, params: dict) -> dict:
        cmd = dict(self.spec.action or {})
        # params.duration 覆盖 action 里的默认值(语音"慢一点"→ 拉长 duration)
        if "duration" in params and params["duration"] is not None:
            if "arm" in cmd or "hand" in cmd:
                cmd["duration"] = float(params["duration"])
        # params.value 覆盖 set_speed 的档位
        if "value" in params and params["value"] is not None and "action" in cmd:
            cmd["value"] = params["value"]
        # params.hand_speed / hand_force 覆盖力度(语音"轻一点捏"→ 降速降力控)。
        # **无条件覆盖,不要求 action 里原本有这个键** —— 一条只发角度的技能
        # (如 hand_close)加上力度参数后,力控字段是新增的而不是被改的。
        # translate() 会保证它排在 angles 之前发,时序不用这里管。
        for k in ("hand_speed", "hand_force"):
            if params.get(k) is not None:
                cmd[k] = params[k]
        return cmd

    def total(self, params: dict) -> int:
        return 1

    def duration_hint(self, params: dict) -> float:
        return float(self._cmd(params).get("duration", 0.0))

    def steps(self, params: dict) -> Iterator[Step]:
        cmd = self._cmd(params)
        # hold = duration:点位控制发完要等它走到,否则下一条会打断上一条
        yield Step(cmd, hold=float(cmd.get("duration", 0.0)), label=self.spec.name)


class TrajectoryBackend(SkillBackend):
    """trajectory:读 robot_traj_*.npz 逐帧展开。

    npz 结构(derive_embodiment --emit-traj 产出):
        arm (N,7) / hand (N,12) / arm_joint_names / hand_joint_names
    按名字从 12 列里挑 writer 要的 6 个驱动关节 —— 与 traj_player.py 同一套逻辑,
    顺序无关,靠名字对齐。剩下 6 个 mimic 关节由 URDF <mimic> 自动跟随。

    延迟加载:构造时只校验文件存在,真正 np.load 推到第一次要数据时,
    这样列技能清单不会把几百兆 npz 全读进内存。
    """

    def __init__(self, spec: SkillSpec, reg: SkillRegistry) -> None:
        super().__init__(spec, reg)
        p = spec.source_path
        if p is None:
            raise SkillError(f"[{spec.id}] 缺 source")
        if not p.exists():
            raise SkillError(f"[{spec.id}] 轨迹文件不存在: {p}")
        self._path = p
        self._arm = None
        self._hand = None            # 已按 HAND_NAMES 挑列并重排后的 (N,6)

    def _load(self) -> None:
        if self._arm is not None:
            return
        import numpy as np           # 只在真要数据时 import,保持展开层可离线测

        d = np.load(self._path, allow_pickle=True)
        missing = {"arm", "hand", "hand_joint_names"} - set(d.files)
        if missing:
            raise SkillError(f"[{self.spec.id}] npz 缺字段 {sorted(missing)}")
        arm = np.asarray(d["arm"], dtype=float)
        hand_all = np.asarray(d["hand"], dtype=float)
        if arm.ndim != 2 or arm.shape[1] != ARM_DOF:
            raise SkillError(f"[{self.spec.id}] arm 形状应为 (N,{ARM_DOF}),实为 {arm.shape}")
        names = [str(x) for x in d["hand_joint_names"]]
        miss = [n for n in HAND_NAMES if n not in names]
        if miss:
            # 夹爪本体的 npz 是 hand=(N,1),会走到这里 —— 明确报错而不是悄悄取前 6 列
            raise SkillError(
                f"[{self.spec.id}] npz 的 hand_joint_names 缺驱动关节 {miss}"
                f"(实有 {len(names)} 个: {names})。"
                "夹爪本体请等 writer 补齐夹爪通道后再登记为技能。")
        cols = [names.index(n) for n in HAND_NAMES]
        n = min(len(arm), len(hand_all))
        self._arm = arm[:n]
        self._hand = hand_all[:n][:, cols]
        # 第 0 步:加载期全帧预检 —— 拇指-食指可行域
        from hand_pose import check_feasible
        bad_frames = []
        for i in range(n):
            why = check_feasible(self._hand[i])
            if why is not None:
                bad_frames.append(i)
        if bad_frames:
            raise SkillError(
                f"[{self.spec.id}] 有 {len(bad_frames)}/{n} 帧不可行(拇指-食指碰撞),"
                f"帧号: {bad_frames[:10]}{'...' if len(bad_frames) > 10 else ''}。"
                f"需重跑 derive_embodiment 补可行域约束,或换用安全轨迹。")

    def _dt(self, params: dict) -> float:
        speed = float(params.get("speed") or 1.0)
        fps = max(1e-3, self.spec.fps)
        return 1.0 / (fps * max(0.05, speed))

    def _approach(self, params: dict) -> float:
        """接近段时长。>0 时先用这个时长走到首帧,再开始正常节拍回放。"""
        v = params.get("approach")
        return max(0.0, float(v)) if v is not None else 0.0

    def total(self, params: dict) -> int:
        self._load()
        return len(self._arm) + (1 if self._approach(params) > 0 else 0)

    def duration_hint(self, params: dict) -> float:
        self._load()
        return len(self._arm) * self._dt(params) + self._approach(params)

    def steps(self, params: dict) -> Iterator[Step]:
        self._load()
        dt = self._dt(params)
        n = len(self._arm)
        # 接近段:轨迹首帧是任意位姿,直接按 1/fps 下发等于要求臂在几十毫秒内弹过去。
        # 先用一条长 duration 的点位指令走到首帧并等它到位,再开始逐帧回放。
        ap = self._approach(params)
        if ap > 0 and n:
            yield Step(
                {"arm": [float(x) for x in self._arm[0]],
                 "hand": [float(x) for x in self._hand[0]],
                 "duration": ap},
                hold=ap,
                label=f"{self.spec.name} 接近首帧",
            )
        for i in range(n):
            yield Step(
                {"arm": [float(x) for x in self._arm[i]],
                 "hand": [float(x) for x in self._hand[i]],
                 "duration": dt},
                hold=dt,
                label=f"{self.spec.name} {i + 1}/{n}",
            )


# overlay 里不许出现的键 —— 它们是**模式切换**,不是姿态,叠不起来。
# estop 尤其不能叠:它必须单独、立刻发,排在一条合成指令里等于延迟急停。
OVERLAY_FORBIDDEN = ("estop", "action", "value")

# overlay 里每个键只能有**一个**来源。两条子技能都给 hand 的话,取谁的都是猜,
# 所以在加载期报错让人改 —— 要么拆成 sequence,要么改其中一条。
OVERLAY_EXCLUSIVE = ("arm", "hand", "hand_speed", "hand_force")


def _merge_overlay(cmds: list[dict], sid: str) -> dict:
    """把若干条 writer 指令合成一条。冲突就抛 SkillError。

    duration 取**最大值**:两个设备并行跑,要等慢的那个走完。
    """
    out: dict = {}
    owner: dict[str, int] = {}          # 键 → 哪个子步骤给的,报错时指名道姓
    for i, c in enumerate(cmds):
        for k, v in c.items():
            if k == "duration":
                out["duration"] = max(float(out.get("duration", 0.0)), float(v))
                continue
            if k in OVERLAY_EXCLUSIVE and k in out:
                raise SkillError(
                    f"[{sid}] overlay 冲突:steps[{owner[k]}] 和 steps[{i}] 都给了 "
                    f"{k!r} —— 一个 overlay 里每个通道只能有一个来源。"
                    f"改成 mode: sequence,或去掉其中一条的 {k!r}。")
            out[k] = v
            owner[k] = i
    return out


class CompositeBackend(SkillBackend):
    """composite:展开子技能。环路已在 schema 加载期排除,这里可放心递归。

    两种 mode:
      sequence(默认) 逐条按序发,每条等自己的 duration
      overlay        合成一条指令同时发 —— 臂和手各写自己 console 的 stdin,
                     臂的 move_j 阻塞不了手,所以是真并发

    overlay **只收 primitive 子技能**。trajectory 有几百帧、composite 步数不定,
    "每个子技能恰好产出一步"这个前提在加载期验不了。手势叠在轨迹上(回放臂轨迹
    时手保持某个姿态)是另一个特性,节拍语义得单独想清楚,不在这里半做。
    """

    def __init__(self, spec: SkillSpec, reg: SkillRegistry) -> None:
        super().__init__(spec, reg)
        self.mode = getattr(spec, "mode", "sequence")
        self.passthrough = tuple(getattr(spec, "passthrough", ()))
        self._children: list[tuple[SkillBackend, dict]] = []
        # 子技能的 spec 和清单里写的 params,steps() 要用它们重算透传后的参数。
        # 构造期仍然按"清单写死"解一遍(下面的 cp)—— total/duration_hint 和
        # overlay 的干跑合并都用它,不依赖运行期参数。
        self._child_specs: list[tuple[SkillSpec, dict]] = []
        for i, st in enumerate(spec.steps):
            ref = st.get("skill")
            child = reg.get(ref)
            if child is None:
                raise SkillError(f"[{spec.id}] steps[{i}] 引用不存在的技能 {ref!r}")
            if self.mode == "overlay":
                if child.kind != "primitive":
                    raise SkillError(
                        f"[{spec.id}] overlay 的 steps[{i}]({ref})是 {child.kind},"
                        f"只收 primitive —— 轨迹/嵌套组合的步数要到运行期才知道,"
                        f"没法保证「每个子技能恰好一步」。")
                bad = [k for k in OVERLAY_FORBIDDEN if k in (child.action or {})]
                if bad:
                    raise SkillError(
                        f"[{spec.id}] overlay 的 steps[{i}]({ref})带 {bad} —— "
                        f"那是模式切换不是姿态,叠不起来。用 mode: sequence。")
            # 子步骤的参数在清单里写死,**除了**父技能 params_passthrough 列出的那几个
            # —— 组合技能的语义要稳定,但"整个动作用多大劲"该跟着外部走。
            # 没写 params_passthrough 时行为和以前完全一样。
            cp, _ = child.resolve_params(st.get("params"))
            self._children.append((make_backend(child, reg), cp))
            self._child_specs.append((child, dict(st.get("params") or {})))
        if self.mode == "overlay":
            # **构造期就试合一次**,让通道冲突在这里炸。
            #
            # 为什么不能只查 spec.action 的键:PrimitiveBackend._cmd 会从 params
            # 补 hand_speed/hand_force —— 清单 action 里没写、参数里有,照样会冲突。
            # 只有真展开一遍才看得见最终键集。primitive 恒产出一步,所以这一遍很便宜。
            self._dry_merge()

    def _dry_merge(self) -> None:
        """构造期干跑一次合并,只为触发冲突检查。结果丢掉,steps() 会重算。"""
        cmds = []
        for i, (b, p) in enumerate(self._children):
            got = list(b.steps(p))
            if len(got) != 1:
                raise SkillError(f"[{self.spec.id}] overlay 的 steps[{i}] 产出 "
                                 f"{len(got)} 步,要求恰好 1 步")
            cmds.append(got[0].cmd)
        _merge_overlay(cmds, self.spec.id)

    def total(self, params: dict) -> int:
        if self.mode == "overlay":
            return 1                     # 合成一条,就是一步
        return sum(b.total(p) for b, p in self._children)

    def duration_hint(self, params: dict) -> float:
        if self.mode == "overlay":
            return max((b.duration_hint(p) for b, p in self._children), default=0.0)
        return sum(b.duration_hint(p) for b, p in self._children)

    def _child_params(self, params: dict) -> list[dict]:
        """每个子步骤这一次要用的参数。没有 passthrough 就是构造期解好的那份。

        透传的值取**父技能已解析过的** params —— 父的 resolve_params 已经套过默认值
        和范围夹取,所以子步骤拿到的一定是合法值,不用再校验一遍。
        """
        if not self.passthrough:
            return [p for _, p in self._children]
        extra = {k: params[k] for k in self.passthrough
                 if params.get(k) is not None}
        if not extra:
            return [p for _, p in self._children]
        out: list[dict] = []
        for (child, st_params), (_, cp) in zip(self._child_specs, self._children):
            # 子技能自己声明过这个参数才透传给它。没声明的话 resolve_params 会丢掉,
            # 而 PrimitiveBackend._cmd 是**无条件**覆盖 hand_speed/hand_force 的,
            # 所以这里必须按子技能的声明过滤,否则会绕过它的 range 校验。
            give = {**st_params, **{k: v for k, v in extra.items()
                                    if k in child.params}}
            if not give:
                out.append(cp)
                continue
            rp, _ = child.resolve_params(give)
            out.append(rp)
        return out

    def steps(self, params: dict) -> Iterator[Step]:
        cparams = self._child_params(params)
        if self.mode != "overlay":
            for (b, _), p in zip(self._children, cparams):
                yield from b.steps(p)
            return
        cmds, hold = [], 0.0
        for i, ((b, _), p) in enumerate(zip(self._children, cparams)):
            got = list(b.steps(p))
            # primitive 恒产出一步,构造期已挡住其它 kind;这条兜底防将来改动
            if len(got) != 1:
                raise SkillError(f"[{self.spec.id}] overlay 的 steps[{i}] 产出 "
                                 f"{len(got)} 步,要求恰好 1 步")
            cmds.append(got[0].cmd)
            hold = max(hold, got[0].hold)
        yield Step(_merge_overlay(cmds, self.spec.id), hold, self.spec.name)


_BACKENDS = {
    "primitive": PrimitiveBackend,
    "trajectory": TrajectoryBackend,
    "composite": CompositeBackend,
}


def make_backend(spec: SkillSpec, reg: SkillRegistry) -> SkillBackend:
    """工厂:按 kind 造展开器。kind 合法性已由 schema 保证。"""
    cls = _BACKENDS.get(spec.kind)
    if cls is None:
        raise SkillError(f"[{spec.id}] 没有 kind={spec.kind} 的后端")
    return cls(spec, reg)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", help="只展开这一个技能")
    ap.add_argument("--dump", action="store_true", help="打印每条指令")
    ap.add_argument("--speed", type=float, default=None, help="覆盖 speed 参数")
    args = ap.parse_args()

    try:
        reg = get_registry()
    except RegistryError as e:
        print(f"✗ 清单有问题: {e}")
        return 1

    specs = [reg.get(args.id)] if args.id else list(reg)
    if args.id and specs[0] is None:
        print(f"✗ 没有技能 {args.id};可选: {reg.ids()}")
        return 1

    bad = 0
    for spec in specs:
        given = {"speed": args.speed} if args.speed is not None else {}
        params, notes = spec.resolve_params(given)
        try:
            be = make_backend(spec, reg)
            n = be.total(params)
            secs = be.duration_hint(params)
            first = next(iter(be.steps(params)), None)
        except SkillError as e:
            print(f"✗ {spec.id:22} {e}")
            bad += 1
            continue
        print(f"✓ {spec.id:22} {spec.kind:11} {n:4d} 步  ~{secs:6.1f}s  首条: "
              f"{json.dumps(first.cmd, ensure_ascii=False)[:60] if first else '无'}")
        for note in notes:
            print(f"    · {note}")
        if args.dump:
            for i, st in enumerate(be.steps(params)):
                print(f"    [{i:4d}] hold={st.hold:.3f} "
                      f"{json.dumps(st.cmd, ensure_ascii=False)}")
    print(f"\n{len(specs) - bad}/{len(specs)} 个技能可展开")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
