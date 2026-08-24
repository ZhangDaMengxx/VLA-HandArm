#!/usr/bin/env python3
"""src/gesture_pack.py — 手势技能包:录制的关节角关键帧序列,存 JSON,可按名回放。

和 action_sequences.py 的关系:那个解析**厂商**的 DefaultAction.txt(只读、格式固定、
索引号还重复);这个是**我们自己**录的,格式自定、可读可手改、按名字定位。回放时统一
转成 ActionSequence 交给 hand_console.ActionPlayer —— 时序/暂停/继续/停止只有一份实现。

文件格式(schema="hand_gesture_pack/1"):
  {
    "schema": "hand_gesture_pack/1",
    "name": "OK手势",                       # 按名回放用这个,不是文件名
    "hand": "inspire_rh56dfx_right",
    "joint_order": [...6 个 URDF 关节名...],  # rad 字段的顺序,写死给人看
    "created_at": "2026-08-03T16:20:00",
    "note": "食指拇指捏合",
    "return_home_first": true,              # 回放前先回零位(可关)
    "frames": [
      {"label":"预备","rad":[6],"raw_vendor":[6],"speed":500,"force":500,"hold_ms":600}
    ]
  }

⚠ rad 和 raw_vendor **两份都存**,不是冗余:
  · rad 是项目关节序(HAND_JOINTS),人能读、能直接喂 3D 预览、能手改。
  · raw_vendor 是厂商通道序(m=0 小指 … m=5 拇指旋转),是真正写 ANGLE_SET 的值。
  回放优先用 raw_vendor,做到位级一致。只存 rad 的话每次回放都要重算 rad→raw
  (夹取 + 取整),来回几趟会漂。手写的文件没有 raw_vendor 就从 rad 推。

⚠ 路径是**沙箱**的:根目录默认 data/gestures/,可用 HAND_GESTURE_DIR 覆盖。
  拒绝绝对路径、拒绝 .. 逃逸、resolve() 后必须仍在根内、强制 .json 后缀。
  理由:7860 端口没有认证,开放任意路径写入等于给出远程任意文件写入原语。
"""
from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from action_sequences import ActionSequence, ActionStep
from inspire_hand import (HAND_JOINTS, HAND_LIMITS, PROJECT_TO_VENDOR,
                          RAW_MAP, RAW_MAX, RAW_MIN)
from paths import DATA

# /2:每帧加 t_ns(整数纳秒的绝对时刻),它是**权威时刻**,hold_ms 降为便利字段。
# 理由见 GestureFrame 的注释:hold_ms 是 int,30fps 下每帧少 0.333ms 且单向不抵消,
# 2400 帧攒 800ms,和臂侧对不上。
SCHEMA = "hand_gesture_pack/2"
# 读的时候两版都收。/1 的文件没有 t_ns,读进来由 ensure_t_ns() 从 hold_ms 累加补出来
# —— 那样补出来的时刻**仍带原来的漂移**(旧文件本身就是那么录的,补不回精度),
# 但至少让播放器只有一条"按绝对时刻走"的代码路径。
ACCEPT_SCHEMAS = ("hand_gesture_pack/1", "hand_gesture_pack/2")
HAND_MODEL = "inspire_rh56dfx_right"
NCH = 6                                  # 通道数,手是 6 个驱动关节
PLAYBACK_KEYFRAME = "keyframe_strict"
PLAYBACK_TIMELINE = "timeline_latest"
PLAYBACK_MODES = (PLAYBACK_KEYFRAME, PLAYBACK_TIMELINE)

HOLD_MS_MIN, HOLD_MS_MAX = 0, 60_000     # 单帧驻留上限 60s,防手滑填 6000000 卡死播放
# 单个包的帧数上限。原来 512 是按"手拖滑块能录几帧"定的(那种用法几十帧就够),
# 但视频解出来的连贯轨迹是另一个量级:30fps 下 stride=1 一秒就 30 帧,20s 素材 600 帧。
# 放到 2400(约 80s @30fps)。文件大小无所谓 —— 每帧约 200B,2400 帧也就 480KB。
MAX_FRAMES = 2400
MAX_NAME_LEN = 64


class GestureError(ValueError):
    """格式/路径校验失败。web 层捕获它转 400,不要让它冒成 500。"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------
# rad ↔ raw 换算。**故意不复用 InspireHand 的方法** —— 那是实例方法,拿它得先构造
# 一个带串口配置的对象。录制/校验是纯数据变换,不该依赖硬件对象存在。
# 算法和 InspireHand.rad_to_raw/raw_to_rad 一致(同一份 RAW_MAP),改一处要同步另一处。
# --------------------------------------------------------------------------
def rad_to_raw_proj(rad6: list[float]) -> list[int]:
    """项目序 rad → 项目序 raw(0-1000)。先按 URDF 限位夹取。"""
    out = []
    for name, r in zip(HAND_JOINTS, rad6):
        span, invert = RAW_MAP[name]
        r = _clamp(float(r), *HAND_LIMITS[name])
        r = _clamp(r, 0.0, span)
        frac = r / span if span > 0 else 0.0
        if invert:
            frac = 1.0 - frac
        out.append(int(round(RAW_MIN + frac * (RAW_MAX - RAW_MIN))))
    return out


def raw_proj_to_rad(raw6: list[int]) -> list[float]:
    """项目序 raw → 项目序 rad。"""
    out = []
    for name, v in zip(HAND_JOINTS, raw6):
        span, invert = RAW_MAP[name]
        frac = _clamp((float(v) - RAW_MIN) / (RAW_MAX - RAW_MIN), 0.0, 1.0)
        if invert:
            frac = 1.0 - frac
        out.append(_clamp(frac * span, *HAND_LIMITS[name]))
    return out


# 两个方向**都显式写出**。PROJECT_TO_VENDOR = [5,4,3,2,1,0] 恰好是自逆置换(整体倒序),
# 所以现在两个函数算出来一样 —— 但那是巧合。哪天映射改成非自逆的,依赖"反正一样"的
# 代码会静默把通道写错(手指乱动,还很难看出来)。所以按定义分开写。
def proj_to_vendor(vals: list) -> list:
    """项目序 → 厂商通道序。out[厂商通道] = 项目值。"""
    out = [None] * NCH
    for i, v in enumerate(vals):
        out[PROJECT_TO_VENDOR[i]] = v
    return out


def vendor_to_proj(vals: list) -> list:
    """厂商通道序 → 项目序。proj_to_vendor 的逆。"""
    out = [None] * NCH
    for i in range(NCH):
        out[i] = vals[PROJECT_TO_VENDOR[i]]
    return out


@dataclass
class GestureFrame:
    """一个关键帧:6 关节角 + 速度 + 力控 + 驻留时间。

    hold_ms 是**下发之后**的驻留时间(等手走到位),不是下发前的等待 —— 和
    ActionStep.delay_ms 语义完全对齐,这样转 ActionSequence 时不用改时序。

    ⚠ **t_ns 才是权威时刻,hold_ms 只是便利字段**(2026-08-03 起)。
    hold_ms 是 int,而 30fps 的真周期是 33.3333…ms,存成 33 每帧少 0.333ms 且
    **单向不抵消**:600 帧(20s)攒 200ms,2400 帧攒 800ms。臂侧按绝对时刻走,
    手侧要是按累加 hold_ms 走,末尾两边就错开 —— 手已经合上而臂还没到位。
    t_ns 用整数纳秒(对齐 ROS 的 builtin_interfaces/Duration),播放器按它**定位**
    而不是累加,残差不放大。
    hold_ms 保留是为了:①读旧文件;②人看得懂"这一帧停多久"。
    """
    rad: list[float]
    raw_vendor: list[int]
    hold_ms: int = 600
    speed: int = 500
    force: int = 500
    label: str = ""
    t_ns: int | None = None      # 相对包起点的绝对时刻(纳秒)。None = 旧文件,读时补

    @classmethod
    def build(cls, rad: list[float] | None = None,
              raw_vendor: list[int] | None = None, *, hold_ms: int = 600,
              speed: int = 500, force: int = 500, label: str = "",
              t_ns: int | None = None) -> GestureFrame:
        """从 rad 或 raw_vendor 任一侧建帧,另一侧自动补齐并夹取。

        两个都给时以 **raw_vendor 为准**:它是真正上线的值,rad 只是它的可读投影。
        录制时前端两个都传,以 raw 为准能保证「存进去的」和「回放出去的」逐位相同。
        """
        if raw_vendor is not None:
            rv = [int(_clamp(int(v), RAW_MIN, RAW_MAX)) for v in _need6(raw_vendor, "raw_vendor")]
            rad_out = raw_proj_to_rad(vendor_to_proj(rv))
            # ⚠ 再从 rad 折回 raw,让两个字段**表示同一个姿态**。
            # 不折的话手写/导入的越界 raw 会让两边打架:rad 字段会按当前资产限位
            # 夹取，而 raw 字段仍可能保留另一个姿态，导致 3D 预览与回放不一致。
            # 会不一致是因为 ActionPlayer._send_angles() 在真机上直接 write_shorts
            # ANGLE_SET,**绕过** InspireHand.set_angles 的 URDF 夹取。
            # 折回之后预览和回放表示同一姿态。
            rv = proj_to_vendor(rad_to_raw_proj(rad_out))
        elif rad is not None:
            rad_out = [float(x) for x in _need6(rad, "rad")]
            rv = proj_to_vendor(rad_to_raw_proj(rad_out))
            rad_out = raw_proj_to_rad(vendor_to_proj(rv))   # 折回,消掉取整误差
        else:
            raise GestureError("帧里 rad 和 raw_vendor 至少要有一个")
        return cls(rad=[round(x, 6) for x in rad_out], raw_vendor=rv,
                   hold_ms=int(_clamp(int(hold_ms), HOLD_MS_MIN, HOLD_MS_MAX)),
                   speed=int(_clamp(int(speed), 0, 1000)),
                   force=int(_clamp(int(force), 0, 1000)),
                   label=str(label)[:MAX_NAME_LEN],
                   t_ns=None if t_ns is None else max(0, int(t_ns)))

    def to_dict(self) -> dict:
        d = {"label": self.label, "rad": self.rad, "raw_vendor": self.raw_vendor,
             "speed": self.speed, "force": self.force, "hold_ms": self.hold_ms}
        if self.t_ns is not None:
            d["t_ns"] = self.t_ns
        return d

    @classmethod
    def from_dict(cls, d: dict) -> GestureFrame:
        if not isinstance(d, dict):
            raise GestureError(f"帧必须是对象,收到 {type(d).__name__}")
        return cls.build(rad=d.get("rad"), raw_vendor=d.get("raw_vendor"),
                         hold_ms=d.get("hold_ms", 600), speed=d.get("speed", 500),
                         force=d.get("force", 500), label=d.get("label", ""),
                         t_ns=d.get("t_ns"))


def _need6(vals, what: str) -> list:
    """长度必须是 6。**不补零、不截断** —— 少给一个通道通常是调用方算错了,
    静默补 0 会让那根手指突然张到底,是会撞东西的错误。"""
    if not isinstance(vals, (list, tuple)):
        raise GestureError(f"{what} 必须是数组,收到 {type(vals).__name__}")
    if len(vals) != NCH:
        raise GestureError(f"{what} 需要 {NCH} 个值,收到 {len(vals)}")
    for v in vals:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise GestureError(f"{what} 含非数值: {v!r}")
    return list(vals)


@dataclass
class GesturePack:
    """一个手势技能包。name 是**回放定位用的键**,和文件名各自独立。"""
    name: str
    frames: list[GestureFrame] = field(default_factory=list)
    note: str = ""
    hand: str = HAND_MODEL
    created_at: str = ""
    return_home_first: bool = True     # 回放前先回零位。见 to_action_sequence()
    playback_mode: str = PLAYBACK_KEYFRAME
    # 读文件时**补**了多少帧的 t_ns。>0 = 这个包的时间轴是从整数 hold_ms 累加出来的,
    # 带原有漂移(600 帧量级 ~200ms),不是从视频源时刻算的。
    # 不存进 to_dict —— 它描述的是"这次是怎么读进来的",不是包的内容。
    #
    # ⚠ 为什么要显式记而不是拿 drift_ms() 反推:补出来的包 drift 恒为 0,
    # 但**整数毫秒正好整除的包**(比如 10fps,每帧 100ms)drift 也是 0,而那种包
    # 时间轴是准的。拿 drift==0 当"这是旧包"会误报在后者身上。
    t_ns_filled: int = 0

    @property
    def duration_ms(self) -> int:
        if (self.playback_mode == PLAYBACK_TIMELINE and self.frames
                and self.frames[-1].t_ns is not None):
            return int(round(self.frames[-1].t_ns / 1e6)) + self.frames[-1].hold_ms
        return sum(f.hold_ms for f in self.frames)

    def ensure_t_ns(self) -> int:
        """给缺 t_ns 的帧补上(从 hold_ms 累加)。返回补了几帧。

        ⚠ 补出来的时刻**带着原有的漂移** —— 旧文件本来就是按整数 hold_ms 录的,
        信息已经丢了,补不回精度。这里补只是为了让播放器只有一条"按绝对时刻走"
        的路径,不用同时维护累加那条。
        要真正无漂移得重新从源(视频/录制)生成包。

        第一帧的 t_ns 一定是 0:hold_ms 是"下发**之后**"的驻留,所以第 k 帧的
        时刻 = 前 k 帧 hold_ms 之和,第 0 帧就是 0。
        """
        n = 0
        acc = 0
        for f in self.frames:
            if f.t_ns is None:
                f.t_ns = acc
                n += 1
            acc = f.t_ns + int(round(f.hold_ms * 1e6))
        return n

    def drift_ms(self) -> float:
        """按累加 hold_ms 走 vs 按 t_ns 走,末帧差多少毫秒。

        用来判断一个包是不是"旧的、带漂移的":新生成的包这个数接近 0,
        /1 版补出来的恒等于 0(因为就是按 hold_ms 累加补的),
        而**从源重新生成**的包会露出真实差值。
        """
        if not self.frames or self.frames[-1].t_ns is None:
            return 0.0
        acc = sum(f.hold_ms for f in self.frames[:-1])
        return self.frames[-1].t_ns / 1e6 - acc

    def to_dict(self) -> dict:
        return {
            "schema": SCHEMA, "name": self.name, "hand": self.hand,
            "joint_order": list(HAND_JOINTS),
            "created_at": self.created_at or datetime.now().isoformat(timespec="seconds"),
            "note": self.note, "return_home_first": self.return_home_first,
            "playback_mode": self.playback_mode,
            "frames": [f.to_dict() for f in self.frames],
        }

    @classmethod
    def from_dict(cls, d: dict) -> GesturePack:
        if not isinstance(d, dict):
            raise GestureError("技能包必须是 JSON 对象")
        sch = d.get("schema")
        # 只认自己的 schema。放宽成"有 frames 就当我们的"会把别的 json 误吞,
        # 报错报在半路(某一帧不合法),比在入口拒掉难查得多。
        if sch not in ACCEPT_SCHEMAS:
            raise GestureError(f"schema 不认识: {sch!r},需要 {ACCEPT_SCHEMAS}")
        name = _clean_name(d.get("name", ""))
        raw_frames = d.get("frames")
        if not isinstance(raw_frames, list) or not raw_frames:
            raise GestureError("frames 为空 —— 技能包至少要有一帧")
        if len(raw_frames) > MAX_FRAMES:
            raise GestureError(f"帧数 {len(raw_frames)} 超上限 {MAX_FRAMES}")
        # joint_order 只做**校验**,不做重排:对不上就拒。按它重排听起来更宽容,
        # 但那要求关节名集合完全一致才安全,不一致时静默重排会把角度装到错的手指上。
        jo = d.get("joint_order")
        if jo is not None and list(jo) != list(HAND_JOINTS):
            raise GestureError(f"joint_order 和本手不一致,不做自动重排。期望 {list(HAND_JOINTS)}")
        frames = []
        for i, fd in enumerate(raw_frames):
            try:
                frames.append(GestureFrame.from_dict(fd))
            except GestureError as e:
                raise GestureError(f"第 {i + 1} 帧: {e}") from e
        mode = d.get("playback_mode")
        if mode is None:
            # 2026-08-20 以前的视频导入包没有 mode,但每帧都有生成器固定写入的
            # t=<秒>s 标签。只对这种明确来源做兼容识别;普通旧包仍按严格关键帧播。
            video_labels = len(frames) > 1 and all(
                re.fullmatch(r"t=\d+(?:\.\d+)?s", f.label or "") for f in frames
            )
            mode = PLAYBACK_TIMELINE if video_labels else PLAYBACK_KEYFRAME
        mode = str(mode)
        if mode not in PLAYBACK_MODES:
            raise GestureError(f"playback_mode 不认识: {mode!r},需要 {PLAYBACK_MODES}")
        pack = cls(name=name, frames=frames, note=str(d.get("note", ""))[:500],
                   hand=str(d.get("hand", HAND_MODEL)),
                   created_at=str(d.get("created_at", "")),
                   return_home_first=bool(d.get("return_home_first", True)),
                   playback_mode=mode)
        # /1 的旧文件没有 t_ns,在**入口**补齐,这样下游(to_action_sequence、
        # ActionPlayer)只需要处理"每帧都有绝对时刻"这一种情况。
        # 补齐后 from_dict 的返回值和 /2 的文件在结构上不可区分,所以把**补了几帧**
        # 记在 t_ns_filled 上 —— 下游(联合回放)要能分清"时刻来自视频源"和
        # "时刻是累加出来的、带漂移"。不记的话只能拿 drift_ms()==0 反推,而那会
        # 误报在整数毫秒正好整除的包上(见 t_ns_filled 的注释)。
        pack.t_ns_filled = pack.ensure_t_ns()
        if pack.playback_mode == PLAYBACK_TIMELINE:
            for i in range(1, len(pack.frames)):
                if pack.frames[i].t_ns <= pack.frames[i - 1].t_ns:
                    raise GestureError(
                        f"timeline_latest 的 t_ns 必须严格递增:第 {i}、{i + 1} 帧"
                    )
        return pack


def _clean_name(name) -> str:
    """名字用来按名回放,不能为空,也不允许换行/控制字符(会把列表显示搞乱)。"""
    s = str(name or "").strip()
    if not s:
        raise GestureError("技能包必须有名字(按名回放要用)")
    if len(s) > MAX_NAME_LEN:
        raise GestureError(f"名字过长({len(s)} > {MAX_NAME_LEN})")
    if re.search(r"[\x00-\x1f\x7f]", s):
        raise GestureError("名字含控制字符")
    return s


# --------------------------------------------------------------------------
# 回放:转成 ActionSequence,交给 hand_console.ActionPlayer
# --------------------------------------------------------------------------
# 回零帧的驻留时间。手从任意姿态张开到底大约几百 ms(速度 500 时),给 500ms
# 留余量 —— 不等够就走第一帧,会从半途的姿态斜插过去,路径和录的时候不一样。
HOME_HOLD_MS = 500

# ⚠ hand_console 主循环的频率,和 app_web 起 console 时的 --hz **必须一致**。
# ActionPlayer.tick() 是在那个循环里被调用的,所以 tick 周期就是回放的时间分辨率:
# 比 PLAYER_TICK_MS 短的 hold_ms **落不到实处**,会被向上取整到一个 tick。
# 定在这里而不是散在 app_web 里,是因为"回放能多快"是技能包的语义,不是 web 层的事。
#
# 为什么是 30 而不是 20:源视频通常 30fps(帧间 33ms)。tick 50ms 时,想忠实回放
# 30fps 就得把 33ms 拉到 50ms —— 整段慢 1.5×,看起来就是"延迟"。30Hz 后 33ms 能落地。
CONSOLE_HZ = 30                     # 遥测发布率(给浏览器看的)
# 播放器 tick 率,和遥测率**分开**(hand_console 的 --player-hz)。
# 为什么要远大于 30:一步的驻留只能是整数个 tick,而 hold_ms 和 tick 同量级时是
# 最坏情况 —— 循环落在截止时刻前一点点就得再等一整个 tick,33ms 的驻留时而 33、
# 时而 66,整段拖慢三成多。实测 tick=33ms 回放 33ms/帧素材:180 帧跑成 8.10s
# (源 5.97s,慢 1.36×)。当前 200Hz 给 60fps 时间轴约 5ms 的最大调度量化。
PLAYER_HZ = 200
PLAYER_TICK_MS = int(round(1000 / PLAYER_HZ))       # 5ms,60fps 目标最大调度量化约 5ms


def to_action_sequence(pack: GesturePack, *, slot: int = -1,
                      return_home: bool | None = None) -> ActionSequence:
    """技能包 → ActionSequence。时序/暂停/停止全部复用 ActionPlayer。

    ⚠ ActionStep.angles 必须是**项目序** —— ActionPlayer._send_angles() 会调
    _vendor() 做项目→厂商的置换后才写 ANGLE_SET。我们存的 raw_vendor 是厂商序,
    所以这里要先 vendor_to_proj() 转回去,不能直接塞。

    return_home:覆盖包里的 return_home_first。None = 用包里的值。
    先回零位的意义:回放起点固定,第一帧的运动路径可预期。不回零时,如果当前姿态
    离第一帧很远,第一步就是一次大幅快速运动。
    """
    home_first = pack.return_home_first if return_home is None else bool(return_home)
    steps: list[ActionStep] = []
    if home_first:
        # 回零 = 六关节全取 URDF 下限 = 完全张开的平手。速度用第一帧的,避免
        # "回零很快、正式动作很慢"的突变观感。
        spd = pack.frames[0].speed if pack.frames else 500
        home_raw_proj = rad_to_raw_proj([HAND_LIMITS[n][0] for n in HAND_JOINTS])
        steps.append(ActionStep(angles=home_raw_proj, speeds=[spd] * NCH,
                                forces=[pack.frames[0].force if pack.frames else 500] * NCH,
                                delay_ms=HOME_HOLD_MS))
    # ⚠ t_ns 要**加上前面 home 那一步的时长**再传下去。
    # 包里的 t_ns 是"相对包起点"的,而 ActionPlayer 的时间轴从**序列**起点算 ——
    # return_home_first 时序列前面多了一步,不平移的话回零位那段会被当成
    # "第 0 帧应该在 t=0",于是第一帧的截止时刻已经过期,播放器会连着补两步。
    off_ns = int(round(HOME_HOLD_MS * 1e6)) if steps else 0
    for f in pack.frames:
        steps.append(ActionStep(angles=vendor_to_proj(f.raw_vendor),
                                speeds=[f.speed] * NCH, forces=[f.force] * NCH,
                                delay_ms=f.hold_ms,
                                t_ns=None if f.t_ns is None else f.t_ns + off_ns))
    return ActionSequence(index=-1, name=pack.name, steps=steps, slot=slot,
                          playback_mode=pack.playback_mode)


# --------------------------------------------------------------------------
# 路径沙箱
# --------------------------------------------------------------------------
# 单段目录/文件名允许的字符。中日韩汉字放行(用户就是中文命名的),但挡掉
# 路径分隔符、控制字符、Windows 保留字符,以及以点开头的隐藏名。
_SEG_RE = re.compile(r"^[^\x00-\x1f\x7f/\\:*?\"<>|]+$")
MAX_PATH_SEGS = 8              # 目录深度上限
MAX_SEG_LEN = 80


def gesture_root() -> Path:
    """技能包根目录。HAND_GESTURE_DIR 可覆盖(比如指到挂载的共享盘)。"""
    env = os.environ.get("HAND_GESTURE_DIR")
    root = Path(env).expanduser() if env else DATA / "gestures"
    return root.resolve()


def resolve_in_root(root: Path, rel: str, *, must_exist: bool = False,
                    err=None, what: str = "技能包") -> Path:
    """把前端给的相对路径解析成 `root` 内的绝对路径。任何逃逸都 raise。

    ⚠ 这是**唯一**的路径入口,web 层所有读写都必须经过它。7860 端口没有认证,
    这里漏一个 .. 就等于给出远程任意文件读写。

    ⚠ **root 参数化是给 combo_pack 复用的**(`data/combos/` 另一个根)。
    这段是安全关键代码,抄一份到那边等于以后修 bug 要记得修两处 —— 而漏掉的
    那一处不会报错,只会**静默地不安全**。所以这里参数化,那边薄封装。
    `err` 是要抛的异常类(各模块有自己的 XxxError,web 层按类型转 400)。

    四道检查,缺一不可:
      1. 逐段白名单 —— 挡掉 .. 和 / 开头的绝对路径,也挡掉 Windows 保留字符
      2. 强制 .json 后缀 —— 免得写出 .py/.sh 被别的东西捡去执行
      3. resolve() 后必须仍在根内 —— 兜住前三步没想到的组合
      4. 第 3 步用 resolve() 而不是 os.path.normpath:normpath 是**纯字符串**运算,
         不看文件系统,根目录里放一个指向 /etc 的软链接就能绕过去。resolve()
         会真的跟软链接走,所以软链接逃逸也一起挡了。
    """
    err = err or GestureError
    s = str(rel or "").strip().replace("\\", "/")
    if not s:
        raise err("路径为空")
    if s.startswith("/"):
        raise err("不接受绝对路径,只能填根目录下的相对路径")
    segs = [p for p in s.split("/") if p not in ("", ".")]
    if not segs:
        raise err(f"路径无效: {rel!r}")
    if len(segs) > MAX_PATH_SEGS:
        raise err(f"目录层级过深({len(segs)} > {MAX_PATH_SEGS})")
    for seg in segs:
        if seg == "..":
            raise err("路径不能包含 ..")
        if seg.startswith("."):
            raise err(f"不接受以点开头的名字: {seg!r}")
        if len(seg) > MAX_SEG_LEN or not _SEG_RE.match(seg):
            raise err(f"名字含非法字符或过长: {seg!r}")
    if not segs[-1].lower().endswith(".json"):
        raise err("文件名必须以 .json 结尾")
    target = (root / "/".join(segs)).resolve()
    # 第 3 道:resolve 之后重新验证归属。用 is_relative_to(Python 3.9+)。
    if target != root and not target.is_relative_to(root):
        raise err(f"路径逃出根目录: {rel!r}")
    if must_exist and not target.is_file():
        raise err(f"{what}不存在: {rel}")
    return target


def resolve_pack_path(rel: str, *, must_exist: bool = False) -> Path:
    """手势包路径。薄封装,沙箱逻辑在 resolve_in_root()。"""
    return resolve_in_root(gesture_root(), rel, must_exist=must_exist,
                           err=GestureError, what="技能包")


def rel_of(path: Path) -> str:
    """绝对路径 → 相对根目录的展示用路径。"""
    try:
        return path.resolve().relative_to(gesture_root()).as_posix()
    except ValueError:
        return path.name


# --------------------------------------------------------------------------
# 文件读写
# --------------------------------------------------------------------------
MAX_FILE_BYTES = 2 << 20          # 2MB:512 帧 × 每帧约 200B 也就 100KB,留足余量


def load_pack(rel: str) -> GesturePack:
    p = resolve_pack_path(rel, must_exist=True)
    if p.stat().st_size > MAX_FILE_BYTES:
        raise GestureError(f"文件过大({p.stat().st_size} 字节),不像技能包")
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise GestureError(f"JSON 解析失败: {e}") from e
    return GesturePack.from_dict(d)


def save_pack(rel: str, pack: GesturePack, *, overwrite: bool = True) -> Path:
    """写技能包。**原子写**:先写同目录临时文件再 os.replace()。

    直接 open(w) 写的话,写一半崩了会留下半个 JSON —— 那个文件之后每次列表都解析
    失败,而且看起来像"包坏了"而不是"上次没写完"。同目录临时文件是为了保证
    os.replace 在同一文件系统内(跨设备 replace 会 raise)。
    """
    p = resolve_pack_path(rel)
    if p.exists() and not overwrite:
        raise GestureError(f"已存在: {rel}(要覆盖请显式传 overwrite)")
    p.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(pack.to_dict(), ensure_ascii=False, indent=2)
    fd, tmp = tempfile.mkstemp(dir=str(p.parent), prefix=".tmp_", suffix=".json")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())          # 断电也不留半个文件
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
    """列出根目录下所有技能包(递归)。坏文件不抛异常,带 error 字段列出来 ——
    一个坏文件让整个列表 500 的话,页面上什么都看不到,反而不好查。"""
    root = gesture_root()
    if not root.is_dir():
        return []
    out = []
    for p in sorted(root.rglob("*.json")):
        if p.name.startswith("."):        # 跳过写入中的临时文件
            continue
        rel = rel_of(p)
        try:
            pack = load_pack(rel)
        except (GestureError, OSError) as e:
            out.append({"path": rel, "name": p.stem, "error": str(e), "frames": 0})
            continue
        out.append({"path": rel, "name": pack.name, "frames": len(pack.frames),
                    "duration_ms": pack.duration_ms, "note": pack.note,
                    "created_at": pack.created_at,
                    "return_home_first": pack.return_home_first,
                    "playback_mode": pack.playback_mode})
    return out


def find_by_name(name: str) -> list[dict]:
    """按名字找技能包。返回**所有**命中项 —— 重名不猜,由调用方决定怎么办。

    先精确匹配;没有再不分大小写匹配。语音/VLA 那条路以后接这里。
    """
    key = str(name or "").strip()
    if not key:
        return []
    items = [it for it in list_packs() if not it.get("error")]
    exact = [it for it in items if it["name"] == key]
    if exact:
        return exact
    low = key.lower()
    return [it for it in items if it["name"].lower() == low]
