#!/usr/bin/env python3
"""sim/combo_pack.py — 臂+手联合录制包:关键帧或流式,存 JSON,按名回放。

和 gesture_pack 的关系:那个只有手(6 DoF),这个是臂+手(7+6=13 DoF)。
臂手分开录的仍走 `arm_pack_*.json` + `gesture_pack`,combo_pack 是给
**从当前位姿出发、在一条时间轴上同步录制**的场景用的。

格式(schema="combo_pack/1"):
  {
    "schema": "combo_pack/1",
    "name": "抓杯子",
    "arm": "nero",
    "hand": "inspire_rh56dfx_right",
    "mode": "keyframe",               # "keyframe" 或 "stream"(显式,不靠帧间距猜)
    "recorded_from": "real",          # "real" 或 "mock"(mock 录的包在真机上警告)
    "joint_order_arm": [...7 个...],  # 写死 ARM_JOINTS,给人看
    "joint_order_hand": [...6 个...], # 写死 HAND_JOINTS
    "created_at": "2026-08-07T12:00:00",
    "note": "...",
    "frames": [
      {"label":"起点", "arm_rad":[7], "hand_rad":[6], "hand_raw":[6],
       "t_ns":0, "hold_ms":1000, "speed":500, "force":500,
       "ee_pose":[x,y,z, qw,qx,qy,qz]}
    ]
  }

⚠ 为什么**不**给 `return_home_first`:手可以安全归零,臂不行(回零路径未知)。
两者混在一起讲不清,干脆去掉这个字段 —— keyframe 录制**自然从当前位姿出发**,
第 0 帧就是接入那一刻,approach 幅度为零,不需要回位。

⚠ `ee_pose` 是**可选**的(FK 算出来,存下来给以后任务空间用)。读包时会重算一遍,
对不上就警告(说明包被手改了,但继续用 arm_rad,不被那个陈旧的 ee_pose 带偏)。
pinocchio 不可用时跳过,不算错误。

⚠ `mode` 显式而不靠帧间距猜:`combo_player` 里那个启发式是**无奈**的(它要能读
两种来源的包),这里有明确录制意图,就别把它埋在数据里。

⚠ `recorded_from="mock"` 的包在真机上回放会**警告** —— mock 的 read_angles
故意摆 ±6.9°,跑在真机上那是噪声,但 mock 下测不出来。不拦,只警告:mock 下
测试整个录制→回放链是有意义的,但上真机前应该重录。

⚠ 路径沙箱:根目录默认 `data/combos/`,可用 `COMBO_RECORD_DIR` 覆盖。和
gesture_pack 同一套防御(逐段白名单 + resolve 后验证归属 + 软链接跟踪)。
"""
from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from paths import DATA

SCHEMA = "combo_pack/1"
ACCEPT_SCHEMAS = ("combo_pack/1",)
ARM_MODEL = "nero"
HAND_MODEL = "inspire_rh56dfx_right"

HOLD_MS_MIN, HOLD_MS_MAX = 0, 60_000
MAX_FRAMES = 2400
MAX_NAME_LEN = 64
MAX_FILE_BYTES = 4 << 20          # 4MB:13 DoF 比手的 6 DoF 大一倍,容量也翻倍

# 接入 ARM_JOINTS / HAND_JOINTS 时不触发硬件初始化(和 gesture_pack 同约定):
# 这两个是常量列表,不依赖设备对象。
try:
    from nero_arm import ARM_JOINTS, NERO_ARM_LIMITS
except ImportError:
    ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
    NERO_ARM_LIMITS = [(-2.7, 2.7)] * 7        # 退化值,单测里用(无 pyAgxArm 时)

try:
    from inspire_hand import HAND_JOINTS, HAND_LIMITS
    import gesture_pack as gp                # 复用手的 rad ↔ raw 换算
except ImportError:
    HAND_JOINTS = [f"hand_{i}" for i in range(6)]
    HAND_LIMITS = [(-1.0, 1.0)] * 6
    gp = None

# FK 可选 —— pinocchio 不可用时跳过 ee_pose,不算错误。
try:
    from nero_kin import NeroKin
    from paths import NERO_URDF
    _kin = NeroKin(NERO_URDF, "link7")
except Exception:                         # noqa: BLE001
    _kin = None


class ComboError(ValueError):
    """格式/路径校验失败。web 层捕获它转 400。"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _need6(vals, what: str) -> list[float]:
    """6 个手关节角。**不补零、不截断、不过滤** —— 少给一个通道通常是调用方算错了。

    ⚠ 第一版这里写的是 `[float(x) for x in vals if not isinstance(x, bool)]`,
    有两个 bug:
      1. bool **被静默过滤掉**而不是拒绝 —— 传 `[0.1, True, 0.2, 0.3, 0.4, 0.5]`
         会变成 5 个值,然后报"需要 6 个收到 5",指向的是错的原因。
      2. `float("abc")` 抛的是**裸 ValueError 不是 ComboError** —— web 层按
         ComboError 转 400,裸的会冒成 500。校验失败报 500 是错的语义。
    """
    if not isinstance(vals, (list, tuple)) or len(vals) != 6:
        n = len(vals) if isinstance(vals, (list, tuple)) else "非数组"
        raise ComboError(f"{what} 需要 6 个值,收到 {n}")
    out = []
    for i, v in enumerate(vals):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ComboError(f"{what}[{i}] 非数值: {v!r}")
        out.append(float(v))
    return out


def _need7(vals, what: str) -> list[float]:
    """7 个臂关节角。超限拒绝,但给 1e-4 容差(prep_arm_traj round 到 5 位)。"""
    if not isinstance(vals, (list, tuple)) or len(vals) != 7:
        raise ComboError(f"{what} 需要 7 个值,收到 {len(vals) if isinstance(vals, (list, tuple)) else '非数组'}")
    out = []
    for i, v in enumerate(vals):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ComboError(f"{what}[{i}] 非数值: {v!r}")
        lo, hi = NERO_ARM_LIMITS[i]
        # ⚠ 超限**拒绝**,不夹取 —— 夹了就把"包里的数越界"这个事实抹掉了。
        # 但给微小容差:prep_arm_traj round(rad, 5),joint6 上限 radians(55)=0.95993…
        # round 成 0.96000 就"超限"了。1e-4 rad ≈ 0.0057° 够小。
        if v < lo - 1e-4 or v > hi + 1e-4:
            raise ComboError(f"{what}[{i}]({ARM_JOINTS[i]}) = {v:.5f} 超限位 [{lo:.5f}, {hi:.5f}]")
        out.append(float(v))
    return out


def _compute_ee_pose(arm_rad: list[float]) -> list[float] | None:
    """FK → [x, y, z, qw, qx, qy, qz]。kin 不可用时返回 None。"""
    if _kin is None:
        return None
    try:
        import numpy as np
        from scipy.spatial.transform import Rotation
        T = _kin.fk(arm_rad)                       # 4x4
        xyz = T[:3, 3].tolist()
        R = Rotation.from_matrix(T[:3, :3])
        q = R.as_quat()                             # [x, y, z, w] (scipy)
        return [*xyz, float(q[3]), float(q[0]), float(q[1]), float(q[2])]  # [w, x, y, z]
    except Exception:                               # noqa: BLE001
        return None


@dataclass
class ComboFrame:
    """臂+手一帧。`t_ns` 是权威时刻(纳秒),`hold_ms` 是便利字段(和 GestureFrame 同约定)。

    ⚠ `ee_pose` 可选,读包时**重算一遍**,对不上就警告。存它是为了以后任务空间用,
    但不能被手改的陈旧值带偏 —— `arm_rad` 才是权威。
    """
    arm_rad: list[float]                           # 7 rad
    hand_rad: list[float]                          # 6 rad,项目序
    hand_raw: list[int]                            # 6 raw,供应商序(ANGLE_SET 真写的)
    t_ns: int                                      # 相对包起点的绝对时刻(纳秒)
    hold_ms: int = 600
    arm_speed_percent: int = 20                    # 臂速度百分比(1-100),录制时记录
    speed: int = 500                                # 手速度(0-1000)
    force: int = 500                                # 手力控(0-1000)
    label: str = ""
    ee_pose: list[float] | None = None             # [x, y, z, qw, qx, qy, qz],可选

    @classmethod
    def build(cls, arm_rad: list[float], hand_rad: list[float] | None = None,
              hand_raw: list[int] | None = None, *, t_ns: int, hold_ms: int = 600,
              arm_speed_percent: int = 20, speed: int = 500, force: int = 500, label: str = "",
              ee_pose: list[float] | None = None, recompute_ee: bool = True) -> ComboFrame:
        """构造一帧。`hand_rad` / `hand_raw` 任给一个,另一边自动补齐(和 GestureFrame 同逻辑)。

        `recompute_ee=True` 时重算末端位姿,**不用包里的陈旧值** —— 读旧包时用。
        构造新帧时传 `recompute_ee=False` + `ee_pose=None`,由 to_dict 时再算
        (避免每帧 build 时都跑 FK)。
        """
        arm = _need7(arm_rad, "arm_rad")
        if gp is None:
            raise ComboError("gesture_pack 不可用,无法处理 hand_rad/hand_raw")
        if hand_raw is not None:
            # 以 raw 为准,折回 rad(和 GestureFrame.build 同逻辑)。
            # 先过 _need6 校验类型 —— 直接 int(v) 对字符串会抛裸 ValueError(→500)。
            hand_r = [int(_clamp(int(v), gp.RAW_MIN, gp.RAW_MAX))
                      for v in _need6(hand_raw, "hand_raw")]
            hand = gp.raw_proj_to_rad(gp.vendor_to_proj(hand_r))
            hand_r = gp.proj_to_vendor(gp.rad_to_raw_proj(hand))  # 折回
        elif hand_rad is not None:
            hand = _need6(hand_rad, "hand_rad")
            hand_r = gp.proj_to_vendor(gp.rad_to_raw_proj(hand))
            hand = gp.raw_proj_to_rad(gp.vendor_to_proj(hand_r))  # 折回
        else:
            raise ComboError("hand_rad 和 hand_raw 至少要有一个")
        ee = _compute_ee_pose(arm) if recompute_ee else ee_pose
        return cls(arm_rad=[round(x, 6) for x in arm],
                   hand_rad=[round(x, 6) for x in hand], hand_raw=hand_r,
                   t_ns=max(0, int(t_ns)),
                   hold_ms=int(_clamp(int(hold_ms), HOLD_MS_MIN, HOLD_MS_MAX)),
                   arm_speed_percent=int(_clamp(int(arm_speed_percent), 1, 100)),
                   speed=int(_clamp(int(speed), 0, 1000)),
                   force=int(_clamp(int(force), 0, 1000)),
                   label=str(label)[:MAX_NAME_LEN], ee_pose=ee)

    def to_dict(self) -> dict:
        d = {"label": self.label, "arm_rad": self.arm_rad, "hand_rad": self.hand_rad,
             "hand_raw": self.hand_raw, "t_ns": self.t_ns, "hold_ms": self.hold_ms,
             "arm_speed_percent": self.arm_speed_percent,
             "speed": self.speed, "force": self.force}
        if self.ee_pose is not None:
            d["ee_pose"] = self.ee_pose
        return d

    @classmethod
    def from_dict(cls, d: dict) -> ComboFrame:
        if not isinstance(d, dict):
            raise ComboError(f"帧必须是对象,收到 {type(d).__name__}")
        arm = d.get("arm_rad")
        hand_rad = d.get("hand_rad")
        hand_raw = d.get("hand_raw")
        t_ns = d.get("t_ns")
        if t_ns is None:
            raise ComboError("帧缺 t_ns(combo_pack 要求每帧都有绝对时刻)")
        # ⚠ recompute_ee=True:不相信包里的陈旧值,重算一遍。对不上就警告
        # (见 ComboPack.from_dict 的 ee_mismatch 计数)。
        return cls.build(arm_rad=arm, hand_rad=hand_rad, hand_raw=hand_raw,
                         t_ns=t_ns, hold_ms=d.get("hold_ms", 600),
                         arm_speed_percent=d.get("arm_speed_percent", 20),
                         speed=d.get("speed", 500), force=d.get("force", 500),
                         label=d.get("label", ""), recompute_ee=True)


@dataclass
class ComboPack:
    """臂+手联合录制包。`mode` 显式:keyframe / stream。"""
    name: str
    frames: list[ComboFrame] = field(default_factory=list)
    mode: str = "keyframe"            # "keyframe" 或 "stream"
    recorded_from: str = "real"       # "real" 或 "mock"
    note: str = ""
    arm: str = ARM_MODEL
    hand: str = HAND_MODEL
    created_at: str = ""
    ee_mismatch: int = 0              # 读包时 ee_pose 对不上的帧数(不存进 to_dict)

    @property
    def duration_ms(self) -> int:
        return sum(f.hold_ms for f in self.frames)

    def to_dict(self) -> dict:
        # ee_pose 在 to_dict 时统一补齐(而不是每帧 build 时算),减少重复 FK。
        for f in self.frames:
            if f.ee_pose is None:
                f.ee_pose = _compute_ee_pose(f.arm_rad)
        return {
            "schema": SCHEMA, "name": self.name, "mode": self.mode,
            "recorded_from": self.recorded_from,
            "arm": self.arm, "hand": self.hand,
            "joint_order_arm": list(ARM_JOINTS),
            "joint_order_hand": list(HAND_JOINTS),
            "created_at": self.created_at or datetime.now().isoformat(timespec="seconds"),
            "note": self.note,
            "frames": [f.to_dict() for f in self.frames],
        }

    @classmethod
    def from_dict(cls, d: dict) -> ComboPack:
        if not isinstance(d, dict):
            raise ComboError("包必须是 JSON 对象")
        sch = d.get("schema")
        if sch not in ACCEPT_SCHEMAS:
            raise ComboError(f"schema 不认识: {sch!r},需要 {ACCEPT_SCHEMAS}")
        name = str(d.get("name", "")).strip()
        if not name or len(name) > MAX_NAME_LEN:
            raise ComboError(f"name 为空或过长(>{MAX_NAME_LEN})")
        mode = str(d.get("mode", "keyframe"))
        if mode not in ("keyframe", "stream"):
            raise ComboError(f"mode 必须是 keyframe 或 stream,收到 {mode!r}")
        rec = str(d.get("recorded_from", "real"))
        if rec not in ("real", "mock"):
            raise ComboError(f"recorded_from 必须是 real 或 mock,收到 {rec!r}")
        # joint_order 校验,不做重排(同 GesturePack 约定)
        jo_arm = d.get("joint_order_arm")
        if jo_arm is not None and list(jo_arm) != list(ARM_JOINTS):
            raise ComboError(f"joint_order_arm 和本臂不一致,不做重排。期望 {list(ARM_JOINTS)}")
        jo_hand = d.get("joint_order_hand")
        if jo_hand is not None and list(jo_hand) != list(HAND_JOINTS):
            raise ComboError(f"joint_order_hand 和本手不一致,不做重排。期望 {list(HAND_JOINTS)}")
        raw_frames = d.get("frames")
        if not isinstance(raw_frames, list) or not raw_frames:
            raise ComboError("frames 为空 —— 包至少要有一帧")
        if len(raw_frames) > MAX_FRAMES:
            raise ComboError(f"帧数 {len(raw_frames)} 超上限 {MAX_FRAMES}")
        frames, ee_mismatch = [], 0
        for i, fd in enumerate(raw_frames):
            try:
                f = ComboFrame.from_dict(fd)
                # 校验 ee_pose:重算的和包里存的对不上就警告(手改了,但继续用 arm_rad)
                old_ee = fd.get("ee_pose")
                if old_ee is not None and f.ee_pose is not None:
                    import numpy as np
                    if not np.allclose(old_ee, f.ee_pose, atol=1e-3):
                        ee_mismatch += 1
                frames.append(f)
            except ComboError as e:
                raise ComboError(f"第 {i + 1} 帧: {e}") from e
        pack = cls(name=name, frames=frames, mode=mode, recorded_from=rec,
                   note=str(d.get("note", ""))[:500],
                   arm=str(d.get("arm", ARM_MODEL)),
                   hand=str(d.get("hand", HAND_MODEL)),
                   created_at=str(d.get("created_at", "")),
                   ee_mismatch=ee_mismatch)
        return pack


# ---------------------------------------------------------------------------
# 路径沙箱 + 文件读写
# ---------------------------------------------------------------------------
def combo_root() -> Path:
    """录制包根目录。COMBO_RECORD_DIR 可覆盖。"""
    env = os.environ.get("COMBO_RECORD_DIR")
    root = Path(env).expanduser() if env else DATA / "combos"
    return root.resolve()


def resolve_pack_path(rel: str, *, must_exist: bool = False) -> Path:
    """联合录制包路径。薄封装,沙箱逻辑复用 gesture_pack.resolve_in_root()。"""
    import gesture_pack as gp_mod
    return gp_mod.resolve_in_root(combo_root(), rel, must_exist=must_exist,
                                   err=ComboError, what="联合录制包")


def rel_of(path: Path) -> str:
    """绝对路径 → 相对根目录的展示用路径。"""
    try:
        return path.resolve().relative_to(combo_root()).as_posix()
    except ValueError:
        return path.name


def load_pack(rel: str) -> ComboPack:
    p = resolve_pack_path(rel, must_exist=True)
    if p.stat().st_size > MAX_FILE_BYTES:
        raise ComboError(f"文件过大({p.stat().st_size} 字节),不像录制包")
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise ComboError(f"JSON 解析失败: {e}") from e
    return ComboPack.from_dict(d)


def save_pack(rel: str, pack: ComboPack, *, overwrite: bool = True) -> Path:
    """原子写(先临时文件再 os.replace,和 gesture_pack 同约定)。"""
    p = resolve_pack_path(rel)
    if p.exists() and not overwrite:
        raise ComboError(f"已存在: {rel}(要覆盖请显式传 overwrite)")
    p.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(pack.to_dict(), ensure_ascii=False, indent=2)
    fd, tmp = tempfile.mkstemp(dir=str(p.parent), prefix=".tmp_", suffix=".json")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
    return p


def delete_pack(rel: str) -> Path:
    p = resolve_pack_path(rel, must_exist=True)
    p.unlink()
    return p


def list_packs() -> list[dict]:
    """列出所有包(名字 + 路径 + 帧数 + 时长)。读错的跳过不报错。"""
    root = combo_root()
    if not root.is_dir():
        return []
    packs = []
    for p in sorted(root.rglob("*.json")):
        if p.name.startswith("."):
            continue
        try:
            pack = load_pack(rel_of(p))
            packs.append({"name": pack.name, "path": rel_of(p),
                          "frames": len(pack.frames), "duration_ms": pack.duration_ms,
                          "mode": pack.mode, "recorded_from": pack.recorded_from})
        except Exception:                          # noqa: BLE001
            pass
    return packs
