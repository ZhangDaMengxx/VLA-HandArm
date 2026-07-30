#!/usr/bin/env python3
"""sim/skills/backend.py — 技能 → writer 指令序列的展开。

设计边界(照 nero_vla_bridge/retarget_backend.py 的 TrajectorySource 写法):
  上层只认 skill_id;本模块把它展开成一串**给 JointWriter.send() 的 dict**。
  本模块不 import rclpy、不发消息、不 sleep —— 纯展开,可离线单测。
  真正的发布与节流由 runner.py 负责。

指令契约(与 sim/ros_joint_writer.py 的 send() 完全一致,不新增方言):
  {"arm": [7], "hand": [6], "duration": s}   关节目标
  {"action": "enable|disable|reset|set_speed", "value": ...}   SDK 级
  {"estop": true}                            急停

需要 numpy(读 trajectory 的 npz),所以只在 ROS2 system python3 侧 import。
app_web 只 import schema,不 import 本模块。

自检:
    python3 sim/skills/backend.py            # 干跑全部技能,打印指令条数
    python3 sim/skills/backend.py --id go_home --dump
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
HAND_NAMES = ["thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
              "index_proximal_joint", "middle_proximal_joint",
              "ring_proximal_joint", "pinky_proximal_joint"]
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
        known = {"arm", "hand", "duration", "action", "value", "estop"}
        unknown = set(a) - known
        if unknown:
            raise SkillError(f"[{spec.id}] action 含 writer 不认识的键 {sorted(unknown)}")

    def _cmd(self, params: dict) -> dict:
        cmd = dict(self.spec.action or {})
        # params.duration 覆盖 action 里的默认值(语音"慢一点"→ 拉长 duration)
        if "duration" in params and params["duration"] is not None:
            if "arm" in cmd or "hand" in cmd:
                cmd["duration"] = float(params["duration"])
        # params.value 覆盖 set_speed 的档位
        if "value" in params and params["value"] is not None and "action" in cmd:
            cmd["value"] = params["value"]
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


class CompositeBackend(SkillBackend):
    """composite:按序展开子技能。环路已在 schema 加载期排除,这里可放心递归。"""

    def __init__(self, spec: SkillSpec, reg: SkillRegistry) -> None:
        super().__init__(spec, reg)
        self._children: list[tuple[SkillBackend, dict]] = []
        for i, st in enumerate(spec.steps):
            ref = st.get("skill")
            child = reg.get(ref)
            if child is None:
                raise SkillError(f"[{spec.id}] steps[{i}] 引用不存在的技能 {ref!r}")
            # 子步骤的参数在清单里写死,不接受外部覆盖 —— 组合技能的语义要稳定
            cp, _ = child.resolve_params(st.get("params"))
            self._children.append((make_backend(child, reg), cp))

    def total(self, params: dict) -> int:
        return sum(b.total(p) for b, p in self._children)

    def duration_hint(self, params: dict) -> float:
        return sum(b.duration_hint(p) for b, p in self._children)

    def steps(self, params: dict) -> Iterator[Step]:
        for b, p in self._children:
            yield from b.steps(p)


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
