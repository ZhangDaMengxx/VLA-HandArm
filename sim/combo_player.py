#!/usr/bin/env python3
"""sim/combo_player.py — 臂 + 手的**联合回放器**:一条时间轴,两个设备。

到这一步之前,`prep_arm_traj.py` 生成的 `arm_pack_*.json` 躺在 out/ 里,
**没有任何东西能把它发下去**。手侧有 ActionPlayer,臂侧是零。这个文件补上臂侧,
并且让两边共用同一个时钟 —— 那是整件事的重点:抓取动作里手合上的时刻必须和
臂到位的时刻对齐,差 200ms 就抓空。

## 一条时间轴

两个包各自的 `t_ns` 都是"相对自己起点的绝对时刻"(整数纳秒)。回放时取同一个
`t0 = time.monotonic()`,两边各自算 `now - t0` 该落在哪一帧。**不累加**周期 ——
30fps 真周期 33.3333…ms,累加整数毫秒 600 帧漂 200ms(实测 199.7ms)。

## 落后就跳帧,两边一起跳

臂用 CPV(逐关节位置伺服),手用 ANGLE_SET —— **两边都是"位置目标"**,不是
增量指令。所以落后时跳到最新的那一帧是**无损**的:中间那些帧只是连续运动的采样点,
伺服本来也要连续走过去。这和 ActionPlayer 里"宁可拖长也不丢姿态"的取舍相反,
因为那边播的是**人手编的关键帧**(有意的驻留),这边播的是**视频采样流**。
包里的 `mode` 字段区分这两种:`stream` 跳帧,`waypoints` 不跳(见 _advance)。

跳帧必须**两边同步**:只跳一边就是把两边的时间轴撕开,而对齐是这个文件存在的理由。
跳了多少帧记在 `skipped_arm` / `skipped_hand` 里报出来 —— 静默丢帧看着像"播完了"。

## ⚠ 安全

- 默认 `--mock`,两边都不碰硬件。真机要 `--no-mock-arm` / `--no-mock-hand`,
  **并且**要 `--yes`。臂是 7 轴工业臂,整条轨迹连续运动,伤害量级和单点不同。
- 起点**不在零位**(实测首帧 joint1 有 -111° 的),所以回放前有独立的 approach 段:
  低速 move_j 到首帧并**等到位**,再开始流式发。不这么做等于让臂从任意姿态猛甩过去。
- 回放前查 `ctrl_mode` 必须是 CAN_CTRL。松灵客户端把臂留在 ETHERNET 时,
  CPV 帧发出去**不报错也不动** —— 那种"跑完了但臂没动"最难查。
- 不能和 `arm_console.py --no-mock` / `nero_arm_bridge.py --no-mock` 同时跑
  (抢 can0),也不能和 `hand_console.py` 同时跑(抢 /dev/ttyUSB0)。

## 用法

    python3 sim/combo_player.py out/arm_pack_robot_traj_nero_inspire_rgbd.json
    python3 sim/combo_player.py <臂包> --hand <手包相对路径> --dry-run
    python3 sim/combo_player.py <臂包> --no-mock-arm --yes     # 真臂!
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

from nero_arm import ARM_JOINTS, NERO_ARM_LIMITS, NeroArm      # noqa: E402

# 臂包 schema。/1 没有 t_ns(只有取整过的 dt_ms),读得进来但**时间轴不可信** ——
# 所以 load_arm_pack 会显式警告,不静默接受。
ARM_SCHEMA = "arm_traj_pack/2"
ARM_ACCEPT = ("arm_traj_pack/1", "arm_traj_pack/2")

# 回放器 tick 率。手侧 ActionPlayer 用 100Hz(gesture_pack.PLAYER_HZ),理由是
# tick 和驻留同量级时最坏情况会多等一整个 tick。这边同理,但**臂侧还有一层**:
# 30fps 流的周期 33.3ms,tick 10ms 意味着发送时刻的量化误差最大 10ms
# —— 占周期 30%。所以取 200Hz(5ms),量化误差降到 15%,且仍远低于
# 实测的 CPV 发送能力(200fps × 7 帧 = 1400 帧/秒都没丢帧)。
TICK_HZ = 200.0

# approach 段的到位判据。逐关节都进这个范围才算到位。
# 0.5° 是**位置环的稳态误差量级**,不是"精确到位"—— 再小会等不到(伺服有静差),
# 再大则首帧就有可见偏差。
APPROACH_TOL_RAD = 0.5 * 3.141592653589793 / 180.0
APPROACH_TIMEOUT_S = 20.0       # 等不到就报错退出,**不硬着头皮开始流式发**


class ComboError(ValueError):
    """包不合法 / 前置条件不满足。和 GestureError 同角色。"""


@dataclass
class ArmWaypoint:
    t_ns: int
    rad: list[float]


@dataclass
class ArmTrajPack:
    name: str
    mode: str                       # "stream" | "waypoints"
    waypoints: list[ArmWaypoint]
    approach_rad: list[float]
    fps_src: float | None = None
    duration_s: float | None = None
    warnings: list[str] = field(default_factory=list)

    @property
    def dur_ns(self) -> int:
        return self.waypoints[-1].t_ns if self.waypoints else 0


def load_arm_pack(path: str | Path) -> ArmTrajPack:
    """读臂包并校验。不合法直接抛 —— **回放前失败**比发了一半才发现好。"""
    p = Path(path)
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except Exception as e:                          # noqa: BLE001
        raise ComboError(f"读不了臂包 {p}: {e}") from e
    if not isinstance(d, dict):
        raise ComboError("臂包顶层不是 object")
    sch = d.get("schema")
    if sch not in ARM_ACCEPT:
        raise ComboError(f"臂包 schema 不认识: {sch!r}(接受 {ARM_ACCEPT})")
    jo = d.get("joints")
    # ⚠ 关节顺序不一致时**不自动重排** —— 和 gesture_pack.from_dict 同约定。
    # 重排看着体贴,但猜错了就是 7 个关节全走错位置,而且没有任何报错。
    if jo is not None and list(jo) != list(ARM_JOINTS):
        raise ComboError(f"joints 和本臂不一致,不做自动重排。期望 {list(ARM_JOINTS)}")
    pts = d.get("waypoints")
    if not isinstance(pts, list) or not pts:
        raise ComboError("waypoints 为空")
    return _parse_waypoints(d, pts, sch)


def _parse_waypoints(d: dict, pts: list, sch: str) -> ArmTrajPack:
    warns: list[str] = []
    wps: list[ArmWaypoint] = []
    acc_ns = 0                       # /1 没有 t_ns 时只能拿 dt_ms 累加
    for k, w in enumerate(pts):
        rad = w.get("rad")
        if not isinstance(rad, list) or len(rad) != 7:
            raise ComboError(f"waypoint[{k}].rad 不是 7 个数")
        try:
            rad = [float(v) for v in rad]
        except (TypeError, ValueError) as e:
            raise ComboError(f"waypoint[{k}].rad 有非数值: {e}") from e
        # ⚠ 限位在这里**只查不夹**。夹了就把"包本身越界"这个事实抹掉了 ——
        # 臂会安静地走一条和包不一样的轨迹。move_cpv_pos 里还有一层夹取兜底,
        # 那层是防手滑,这层是防"包不对"。
        # 加 1e-4 容差:prep_arm_traj 把 rad round 到 5 位小数,joint6 上限
        # radians(55) = 0.95993…,round 成 0.96000 就超了。容差挡住取整残差,
        # 同时 0.0001 rad = 0.0057° 不影响"抓真超限"的判断。
        for i, v in enumerate(rad):
            lo, hi = NERO_ARM_LIMITS[i]
            if v < lo - 1e-4 or v > hi + 1e-4:
                raise ComboError(
                    f"waypoint[{k}] {ARM_JOINTS[i]} = {v:.4f} rad 超限位 "
                    f"[{lo:.4f}, {hi:.4f}] —— 包不对,不回放")
        t_ns = w.get("t_ns")
        if t_ns is None:
            t_ns = acc_ns
            dt = w.get("dt_ms")
            acc_ns += 0 if dt is None else int(round(float(dt) * 1e6))
        wps.append(ArmWaypoint(t_ns=int(t_ns), rad=rad))
    if sch == "arm_traj_pack/1":
        warns.append("臂包是 /1(无 t_ns):时刻从取整的 dt_ms 累加,**带漂移**。"
                     "重跑 prep_arm_traj.py --emit 生成 /2")
    # 时刻必须单调递增 —— 不递增的话"落后就跳到最新帧"会跳错方向
    for a, b in zip(wps, wps[1:]):
        if b.t_ns <= a.t_ns:
            raise ComboError(f"t_ns 不是严格递增: {a.t_ns} → {b.t_ns}")
    ap = d.get("approach", {}).get("rad") or wps[0].rad
    mode = d.get("mode") or "waypoints"
    if mode not in ("stream", "waypoints"):
        raise ComboError(f"mode 不认识: {mode!r}")
    return ArmTrajPack(name=str(d.get("name") or "未命名"), mode=mode,
                       waypoints=wps, approach_rad=[float(v) for v in ap],
                       fps_src=d.get("fps_src"), duration_s=d.get("duration_s"),
                       warnings=warns)


def load_combo_pack(path: str | Path) -> tuple[ArmTrajPack, list, dict]:
    """读 `combo_pack/1`(臂+手一起录的),摊成 ComboPlayer 已经吃的那两样。

    回 `(arm_pack, hand_cues, meta)`。**不改 ComboPlayer 的内部** —— 它按
    "臂包 + 手 cue 列表"工作已经测过 42 项,这里只做格式适配。

    和 `load_arm_pack` + 手势包两个文件的路径相比,combo_pack 的差别在:
      · 两侧**天生同一条时间轴**(录的时候就是一起抓的),不用对齐 t0
      · `mode` 是**显式**的,不用拿帧间距猜(见下面 _hand_is_stream 的无奈)
      · 第 0 帧就是录制那一刻的位姿 → **approach 幅度约为零**

    最后一条是这个格式真正的意义:那六个从 npz 转的臂包起点离零位 111–158°,
    approach 要走一个很大的、路径不受控的 move_j。自己录的包没有这个问题。
    """
    import combo_pack as cbp
    if Path(path).is_absolute():
        # 绝对路径 = **CLI 用法**,由人在命令行显式给出,沙箱管不到也不该管。
        # ⚠ web 层**永远不要**走这一支:那边的 path 来自网络,必须过
        # cbp.load_pack 的沙箱(7860 没有认证)。web 端点传的都是相对路径。
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        pack = cbp.ComboPack.from_dict(d)
    else:
        pack = cbp.load_pack(str(path))            # 相对路径:过沙箱

    wps = [ArmWaypoint(t_ns=f.t_ns, rad=list(f.arm_rad)) for f in pack.frames]
    for a, b in zip(wps, wps[1:]):
        if b.t_ns <= a.t_ns:
            raise ComboError(f"t_ns 不是严格递增: {a.t_ns} → {b.t_ns}")
    # combo 的 mode 是 keyframe/stream,臂包的是 waypoints/stream —— 映射一下。
    arm_mode = "stream" if pack.mode == "stream" else "waypoints"
    arm_pack = ArmTrajPack(name=pack.name, mode=arm_mode, waypoints=wps,
                           approach_rad=list(pack.frames[0].arm_rad),
                           duration_s=pack.frames[-1].t_ns / 1e9,
                           warnings=[])
    cues = [HandCue(t_ns=f.t_ns, raw_vendor=list(f.hand_raw),
                    speed=f.speed, force=f.force) for f in pack.frames]
    meta = {"mode": pack.mode, "recorded_from": pack.recorded_from,
            "ee_mismatch": pack.ee_mismatch, "name": pack.name}
    return arm_pack, cues, meta


# 手包"是视频流还是人手编的关键帧"的分界。见 ComboPlayer.__init__ 的注释。
HAND_STREAM_MAX_GAP_MS = 100.0


def _hand_is_stream(cues: list) -> bool:
    """手包看着像视频流(可跳帧)还是关键帧序列(不可跳)。

    判据:帧间距**中位数** < 100ms。用中位数而不是均值 —— 关键帧序列里
    偶尔有一两个很长的驻留,均值会被拉偏;而流式素材间距几乎全等,两者中位数
    差一个量级,判起来很稳。
    单帧包没有间距可算,当成关键帧(保守:不跳)。
    """
    if len(cues) < 2:
        return False
    gaps = [(b.t_ns - a.t_ns) / 1e6 for a, b in zip(cues, cues[1:])]
    return statistics.median(gaps) < HAND_STREAM_MAX_GAP_MS


@dataclass
class HandCue:
    """手侧的一帧,已经摊平成"绝对时刻 + 厂商序 raw"。

    为什么不直接用 GesturePack.frames:那边 t_ns 可能是 None(旧文件),
    而且 speed/force 要按"变了才发"来省串口往返。摊平一次,tick 里就只做比较。
    """
    t_ns: int
    raw_vendor: list[int]
    speed: int
    force: int


class ComboPlayer:
    """一条时间轴驱动臂 + 手。tick() 在主循环里被反复调用,不自己起线程。

    不起线程的理由和 ActionPlayer 一致:调用方(console / CLI)本来就有主循环,
    多一个线程就多一份"谁在写硬件"的不确定性,而两个设备的写入都不是线程安全的。
    """

    def __init__(self, arm_pack: ArmTrajPack, arm: NeroArm | None = None,
                 hand_cues: list[HandCue] | None = None, hand=None,
                 *, skip_arm: bool | None = None,
                 skip_hand: bool | None = None) -> None:
        self.pack = arm_pack
        self.arm = arm
        self.hand = hand
        self.cues = hand_cues or []
        # ⚠ 跳帧策略必须**两侧各判**,不能拿臂包的 mode 一个值管两边。
        # 第一版就是那样,后果:臂包是 stream(视频流,跳帧无损),而手包可能是
        # **人手编的关键帧**(实测 test1.json 42 帧铺 20.3s,平均 483ms 一帧,
        # 那些驻留是有意的)—— 用臂的 stream 去许可跳手的关键帧,就是丢动作。
        # 臂:包里的 mode 是 prep 生成时算的(均匀 + 全帧 = stream),直接用。
        self.skip_arm = (arm_pack.mode == "stream") if skip_arm is None else skip_arm
        # 手:gesture_pack 没有 mode 字段,拿**帧间距中位数**判。
        # 阈值 100ms:视频流是 33ms(30fps),厂商格式的关键帧默认驻留 600ms,
        # 两者差一个量级,100ms 落在中间且离两边都远。
        self.skip_hand = (_hand_is_stream(self.cues) if skip_hand is None
                          else skip_hand)
        self.i_arm = 0
        self.i_hand = 0
        self.paused = False
        self.stopped = False
        self.done = False
        self._t0 = 0.0
        self._paused_at = 0.0
        self._paused_total = 0.0
        self._last_sf: tuple[int, int] | None = None   # 上次发的 (speed, force)
        # 统计:发了多少、跳了多少、每次下发相对**应发时刻**晚了多少
        self.sent_arm = 0
        self.sent_hand = 0
        self.skipped_arm = 0
        self.skipped_hand = 0
        self.fail_arm = 0
        self.fail_hand = 0
        self.late_ms: list[float] = []

    # ---------------------------------------------------------------- 控制
    def start(self, *, start_at: float | None = None) -> None:
        """开始回放。

        start_at: 跨进程对齐用 —— 指定一个共同的 CLOCK_MONOTONIC 时刻作为 t0。
                  None = 立刻开始(time.monotonic())。
        """
        self.i_arm = self.i_hand = 0
        self.paused = self.stopped = self.done = False
        self._t0 = start_at if start_at is not None else time.monotonic()
        self._paused_total = 0.0
        self._last_sf = None

    def pause(self) -> None:
        if self.paused or self.done:
            return
        self.paused = True
        self._paused_at = time.monotonic()

    def resume(self) -> None:
        if not self.paused:
            return
        self.paused = False
        # 暂停时长累进 _paused_total —— 绝对时轴下不累的话,恢复瞬间
        # (now - t0) 已经跑过好几帧,两边会一起"补课"跳一大段。
        # 和 ActionPlayer.resume 同一个道理。
        self._paused_total += max(0.0, time.monotonic() - self._paused_at)

    def stop(self) -> None:
        """中断回放。**同时置 done** —— 语义上 done = 「不会再发帧了」,stop 之后
        这就是真的。

        ⚠ 只置 stopped 不置 done 会让所有「等它结束」的循环永远等下去:
        CLI 的 `while not pl.done` 死循环,arm_console 主循环则永远不清 _player
        —— 于是 cpv_end() 没人调(auto_set_motion_mode 一直关着),而且下一个
        combo_play 被「已经在回放」永久拒掉。实测踩到:stop 之后再 play 一律被拒。
        stopped 保留是为了区分「正常放完」和「被打断」,report 里会用到。
        """
        self.stopped = True
        self.done = True

    @property
    def elapsed_ns(self) -> int:
        """回放已经走过的时间(纳秒),扣掉暂停。

        ⚠ **进行中**的那段暂停也要扣。`_paused_total` 只在 resume() 里累加,
        光减它的话暂停期间这个数还在涨 —— 实测暂停 0.5s,报的 elapsed 从
        351ms 涨到 851ms(而 tick 一帧都没发)。前端进度条于是在暂停时继续走。
        注意 tick() 里**不能**用这个值:那边看的是真实时钟推进,和这里的
        「对外报告」不是一件事。
        """
        el = time.monotonic() - self._t0 - self._paused_total
        if self.paused and self._paused_at:
            el -= time.monotonic() - self._paused_at
        return int(el * 1e9)

    @property
    def total_ns(self) -> int:
        a = self.pack.dur_ns
        h = self.cues[-1].t_ns if self.cues else 0
        return max(a, h)

    def progress(self) -> float:
        """0.0-1.0。⚠ **下界也要夹**:start_at 设在未来时 elapsed_ns 是负的
        (实测起跑前报 -40ms),不夹的话前端进度条显示负值。tick() 不受影响 ——
        _advance 拿 t_ns > now_ns 判断,负的 now_ns 只是"还没到第一帧"。
        """
        tot = self.total_ns
        return 1.0 if tot <= 0 else max(0.0, min(1.0, self.elapsed_ns / tot))

    # ---------------------------------------------------------------- 时序
    @staticmethod
    def _advance(items, i: int, now_ns: int, allow_skip: bool) -> int:
        """返回**本轮该发到哪一帧**的下标,没有到期的帧就返回 -1。

        allow_skip=True:一路跳到"最后一个已到期的帧"。中间那些帧不发。
          流式素材下这是无损的 —— 它们只是连续运动的采样点,而 CPV / ANGLE_SET
          都是位置目标,伺服本来就要连续走过去。
        allow_skip=False:一次只推进一帧。关键帧素材下每个姿态都是有意的,
          丢掉就是丢动作。代价是整段会被拖长(时间轴不再对齐,所以只用于单播手包)。
        """
        if i >= len(items) or items[i].t_ns > now_ns:
            return -1
        if not allow_skip:
            return i
        j = i
        while j + 1 < len(items) and items[j + 1].t_ns <= now_ns:
            j += 1
        return j

    def tick(self) -> dict | None:
        """每 tick 调一次。返回本轮发生了什么,没动作时返回 None。"""
        if self.stopped or self.paused or self.done:
            return None
        now_ns = self.elapsed_ns
        ev: dict = {}

        ja = self._advance(self.pack.waypoints, self.i_arm, now_ns, self.skip_arm)
        if ja >= 0:
            self.skipped_arm += ja - self.i_arm
            wp = self.pack.waypoints[ja]
            ok = True if self.arm is None else self.arm.move_cpv_pos(wp.rad)
            self.sent_arm += 1
            self.fail_arm += 0 if ok else 1
            # 晚了多少 = 实际发的时刻 - 这一帧**应该**发的时刻。
            # 拿这个而不是 tick 间隔:间隔均匀也可能整体滞后一个常数。
            self.late_ms.append((now_ns - wp.t_ns) / 1e6)
            self.i_arm = ja + 1
            ev.update(arm_i=ja, arm_ok=ok)

        jh = self._advance(self.cues, self.i_hand, now_ns, self.skip_hand)
        if jh >= 0:
            self.skipped_hand += jh - self.i_hand
            ok = self._send_hand(self.cues[jh])
            self.sent_hand += 1
            self.fail_hand += 0 if ok else 1
            self.i_hand = jh + 1
            ev.update(hand_i=jh, hand_ok=ok)

        if self.i_arm >= len(self.pack.waypoints) and self.i_hand >= len(self.cues):
            # ⚠ 末帧**发完**不等于走到位:CPV 是位置目标,发完伺服还在走。
            # 这里只宣布"发完了",到位与否由调用方看遥测判断。名字不叫 arrived。
            self.done = True
            ev["done"] = True
        return ev or None

    def _send_hand(self, cue: HandCue) -> bool:
        """下发一帧手姿。speed/force **变了才发**。

        为什么要省:每次 write_shorts 是一次串口往返(实测手回复 ~3ms,
        txn_timeout 60ms)。30fps 下一帧的预算是 33ms,而"角度+速度+力控"
        三次往返就把预算吃掉大半。视频重定向出来的包整段 speed/force 是常数,
        所以省掉的是**每帧两次**往返。
        """
        if self.hand is None:
            return True
        # ⚠ mock 由**调用方**处理,不能直接调 write_shorts:
        # write_shorts 第一行是 `if self._sp is None: return False`,而 mock 没有串口,
        # 于是 mock 下每一帧都"失败"。实测踩到:42 帧全报 fail 而回放看着是好的。
        # ActionPlayer._send_angles 也是这么分流的 —— 那边把 raw 折回 rad 写进
        # _target_rad,让 3D 和滑块在 mock 下也动起来。这里照做。
        if getattr(self.hand.cfg, "mock", False):
            return self._send_hand_mock(cue)
        ok = True
        sf = (cue.speed, cue.force)
        if sf != self._last_sf:
            ok = self.hand.write_shorts("SPEED_SET", [cue.speed] * 6) and ok
            ok = self.hand.write_shorts("FORCE_SET", [cue.force] * 6) and ok
            self._last_sf = sf
        return self.hand.write_shorts("ANGLE_SET", list(cue.raw_vendor)) and ok

    def _send_hand_mock(self, cue: HandCue) -> bool:
        """mock:把厂商序 raw 折回项目序 rad 写进手的目标,让 mock 状态跟着回放走。"""
        from inspire_hand import HAND_JOINTS, PROJECT_TO_VENDOR

        rad = list(self.hand._target_rad)
        for i, n in enumerate(HAND_JOINTS):
            v = cue.raw_vendor[PROJECT_TO_VENDOR[i]]
            if v >= 0:
                rad[i] = self.hand.raw_to_rad(n, v)
        return self.hand.set_angles(rad)

    # ------------------------------------------------------------ approach
    def preflight(self) -> list[str]:
        """回放前的硬前提检查。返回**阻断性**问题列表,空 = 可以放。

        分开成一个方法而不是塞进 start():调用方要能先看一眼再决定,
        而且 CLI 的 --dry-run 只跑这个不跑运动。
        """
        bad: list[str] = []
        if self.arm is not None and not self.arm.mock:
            if not self.arm.enabled:
                bad.append("臂未使能 —— CPV 帧发出去不会动。先 enable")
            cm = self.arm.read_ctrl_mode()
            # ⚠ 这一条是"跑完了但臂没动"的唯一防线,见 read_ctrl_mode 的注释。
            if cm != "CAN_CTRL":
                bad.append(f"臂的 ctrl_mode = {cm},不是 CAN_CTRL —— "
                           f"CPV 帧会被静默忽略。去松灵客户端切成 CAN 指令模式")
            if not self.arm.velocity_is_real():
                # 不阻断:velocity 只影响遥测可读性,不影响能不能发 CPV。
                pass
        if self.hand is not None and not getattr(self.hand.cfg, "mock", True):
            if not self.hand.connected:
                bad.append("手没连上")
        return bad

    def approach(self, *, speed_pct: int = 10,
                 timeout: float = APPROACH_TIMEOUT_S) -> tuple[bool, str]:
        """把臂从**当前姿态**低速挪到首帧,等到位。返回 (成功, 说明)。

        为什么必须单独一步、而且不能当成包里的第一个路点顺手发掉:
        包的起点**不在零位**(实测 robot_traj_nero_gripper_rgbd 首帧 joint1 = -111°)。
        直接开始流式发的话,第一帧就是一次从任意姿态到 -111° 的猛甩,
        路径由伺服自己决定、不可预测。

        用 move_j 而不是 CPV:这一步要的是**规划过的、慢的**运动,正好是 move_j 的强项。
        CPV 在这里反而危险 —— 它是位置环,一个远目标下去就是全速冲。
        """
        if self.arm is None:
            return True, "无臂,跳过 approach"
        tgt = self.pack.approach_rad
        if self.arm.cpv_active:
            # move_j 要靠 auto 切模式,CPV 期间 auto 是关的。先退出。
            self.arm.cpv_end()
        old = self.arm.speed_percent
        self.arm.set_speed_percent(speed_pct)
        try:
            if not self.arm.move_j(tgt):
                return False, f"approach 的 move_j 被拒(急停中?): {self.arm.last_error}"
            if self.arm.mock:
                # ⚠ mock 下**不能**跑到位判据。mock 的 read_angles 故意在目标位附近
                # 摆 ±0.12rad(=6.9°)当"活着"的可见信号,永远进不了 0.5° 的窗
                # —— 等满 20s 然后报"超时,还差 5.25°"。那个数是摆动幅度,不是误差。
                # 说明:**到位判据只在真机上被执行到**。这是已知的覆盖缺口,
                # 不是"mock 下也验过了"。
                return True, "mock:跳过到位等待(mock 的摆动幅度 6.9° > 判据 0.5°)"
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                cur = self.arm.read_angles()
                err = max(abs(c - t) for c, t in zip(cur, tgt))
                if err <= APPROACH_TOL_RAD:
                    return True, f"到位,最大偏差 {err * 57.2958:.3f}°"
                time.sleep(0.05)
            cur = self.arm.read_angles()
            err = max(abs(c - t) for c, t in zip(cur, tgt))
            return False, (f"approach 超时 {timeout:.0f}s,最大偏差还有 "
                           f"{err * 57.2958:.2f}° —— **不开始回放**")
        finally:
            # 速度恢复:approach 的低速是这一步专用的,留着会让后面的动作也慢。
            # ⚠ old 可能是 None(speed_percent 在没写过时返回 None,见那个属性的注释),
            # 那种情况下我们不知道原值,就**不乱写**一个猜的值回去。
            if old is not None:
                self.arm.set_speed_percent(old)


def hand_cues_from_pack(rel: str) -> tuple[list[HandCue], list[str]]:
    """手包 → HandCue 列表。走 gesture_pack.load_pack,**沙箱和校验全部复用**。

    ⚠ 不自己拼路径 —— load_pack 里那套沙箱(拒绝绝对路径、拒绝 .. 逃逸、
    强制 .json、resolve 后必须还在 root 内)是因为 7860 端口没有鉴权才加的。
    这里绕过去就等于在另一个入口上重新开一个任意路径读的口子。

    ⚠ **不加 home 那一步**(to_action_sequence 的 return_home_first)。
    联合回放里手的起点由臂的 approach 决定时机,手单独先回零位会和臂的
    approach 撞在一起 —— 两个设备同时动而且没人协调那个时刻。
    要回零位应该由调用方在 approach 之前显式做。
    """
    from gesture_pack import load_pack               # 延迟导入:CLI 不带手包时不需要

    pack = load_pack(rel)
    warns: list[str] = []
    if not pack.frames:
        raise ComboError(f"手包 {rel} 没有帧")
    # from_dict 里已经 ensure_t_ns() 过,所以到这里 t_ns 一定有值。
    # 但**旧包补出来的 t_ns 带着原有漂移**(整数 hold_ms 累加的)。
    # ensure_t_ns() 返回补了几帧,pack.t_ns_filled 记着它。
    if pack.t_ns_filled:
        warns.append(f"手包 {rel} 的 t_ns 是补出来的(旧 /1 文件,补了 "
                     f"{pack.t_ns_filled} 帧):时刻由整数 hold_ms 累加,"
                     f"**带漂移**(600 帧量级 ~200ms)。要对齐得从视频重新生成")
    cues = [HandCue(t_ns=int(f.t_ns), raw_vendor=list(f.raw_vendor),
                    speed=int(f.speed), force=int(f.force)) for f in pack.frames]
    for a, b in zip(cues, cues[1:]):
        if b.t_ns < a.t_ns:
            raise ComboError(f"手包 t_ns 不单调: {a.t_ns} → {b.t_ns}")
    return cues, warns


def report(pl: ComboPlayer) -> str:
    """回放结束时的总结:发了多少、跳了多少、延迟分布。"""
    lines: list[str] = []
    lines.append(f"臂:发 {pl.sent_arm}/{len(pl.pack.waypoints)} 帧"
                 f"(跳 {pl.skipped_arm}、失败 {pl.fail_arm})")
    if pl.cues:
        lines.append(f"手:发 {pl.sent_hand}/{len(pl.cues)} 帧"
                     f"(跳 {pl.skipped_hand}、失败 {pl.fail_hand})")
    if pl.late_ms:
        s = sorted(pl.late_ms)
        p95 = s[min(len(s) - 1, int(round(0.95 * (len(s) - 1))))]
        lines.append(f"延迟(实际发送 vs 应发时刻): 中位 {statistics.median(s):.2f}ms  "
                     f"p95 {p95:.2f}ms  max {max(s):.2f}ms")
        # 判据:和**帧周期**比,不和 tick 比。
        # ⚠ tick 量化本身就保证延迟落在 [0, 1个tick),所以拿"半个 tick"当判据
        # 是把**正常的量化**判成故障 —— 第一版就是那样,mock 下 max 3.50ms
        # (tick 5ms)被报成"调度跟不上",而那趟一帧没跳、356/356 全发。
        # 真正该报的是延迟接近一个帧周期:那时候下一帧已经到期,开始跳帧了。
        tick_ms = 1000.0 / TICK_HZ
        per_ms = (pl.pack.dur_ns / 1e6 / max(1, len(pl.pack.waypoints) - 1))
        mx = max(s)
        if mx > per_ms:
            lines.append(f"  → ⚠ max {mx:.1f}ms 超帧周期 {per_ms:.1f}ms —— "
                         f"跟不上节拍,已经在跳帧")
        elif mx > tick_ms * 1.5:
            lines.append(f"  → max {mx:.1f}ms 超 tick({tick_ms:.1f}ms)的 1.5 倍,"
                         f"但仍在帧周期 {per_ms:.1f}ms 内 —— 没跳帧,偏紧")
        else:
            lines.append(f"  → OK —— 延迟在 tick 量化范围内"
                         f"(tick {tick_ms:.1f}ms,帧周期 {per_ms:.1f}ms)")
    if pl.skipped_arm or pl.skipped_hand:
        lines.append(f"  ⚠ 跳了帧:臂 {pl.skipped_arm}、手 {pl.skipped_hand} ——"
                     f"流式素材下无损(位置目标),但说明节拍没跟上")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arm_pack", nargs="?",
                    help="臂包路径(arm_pack_*.json)。用 --combo 时不填")
    ap.add_argument("--hand", metavar="REL",
                    help="手包相对路径(沙箱内)。不带则只回放臂")
    ap.add_argument("--combo", metavar="PATH",
                    help="臂+手**联合录制包**(combo_pack/1)。和 arm_pack/--hand "
                         "互斥 —— 联合包里两侧本来就在一条时间轴上,不需要分别给")
    ap.add_argument("--mock-arm", dest="mock_arm", action="store_true", default=True)
    ap.add_argument("--no-mock-arm", dest="mock_arm", action="store_false",
                    help="用真臂。默认 mock")
    ap.add_argument("--mock-hand", dest="mock_hand", action="store_true", default=True)
    ap.add_argument("--no-mock-hand", dest="mock_hand", action="store_false",
                    help="用真手。默认 mock")
    ap.add_argument("--yes", action="store_true",
                    help="跳过确认。脚本化用,**人手跑时别加** —— 那是最后一道人工检查")
    ap.add_argument("--dry-run", action="store_true",
                    help="只跑 preflight,不真的回放。看问题用")
    ap.add_argument("--speed", type=int, default=20,
                    help="臂的速度百分比(approach / 接入时用)")
    ap.add_argument("--firmware", default="auto",
                    help="臂固件版本。auto / default / v111 / v112 / v120")
    a = ap.parse_args()

    if a.combo and (a.arm_pack or a.hand):
        print("--combo 和 arm_pack/--hand 互斥:联合包里两侧已经在一条时间轴上,"
              "再给一个臂包/手包无法确定该用哪个。", file=sys.stderr)
        return 2
    if not a.combo and not a.arm_pack:
        print("要给 arm_pack,或者用 --combo 给联合录制包。", file=sys.stderr)
        return 2

    # combo 侧的显式 mode。None = 走 arm_pack 那条路(mode 由启发式判)。
    combo_mode: str | None = None
    cues, hwarns = [], []
    if a.combo:
        print("=== 联合回放器:combo_pack(臂+手同轴) ===")
        try:
            pack, cues, meta = load_combo_pack(a.combo)
        except ComboError as e:
            print(f"联合包读不了: {e}", file=sys.stderr)
            return 2
        except Exception as e:                          # noqa: BLE001
            print(f"联合包读不了: {type(e).__name__}: {e}", file=sys.stderr)
            return 2
        combo_mode = meta["mode"]
        print(f"联合包:{meta['name']}  {len(pack.waypoints)} 帧  "
              f"时长 {pack.duration_s:.1f}s  mode={combo_mode}  "
              f"录自={meta['recorded_from']}")
        if meta["recorded_from"] == "mock" and not a.mock_arm:
            # ⚠ 不拦,只警告。mock 录的包用来测录制→回放链是有意义的,
            # 但 mock 的 read_angles 故意摆 ±6.9°,那些"位姿"是噪声不是动作。
            print("  ⚠ 这个包是 **mock 录的**,现在要发到**真臂**。mock 的关节角"
                  "带 ±6.9° 的人造摆动,不是真实位姿 —— 上真机前应该重录。")
        if meta["ee_mismatch"]:
            print(f"  ⚠ {meta['ee_mismatch']} 帧的 ee_pose 和 arm_rad 算出来的对不上"
                  f"(包被手改过?)。回放**用 arm_rad**,不受影响。")
    else:
        print(f"=== 联合回放器:臂 + {'手' if a.hand else '无手'} ===")
        try:
            pack = load_arm_pack(a.arm_pack)
        except ComboError as e:
            print(f"臂包读不了: {e}", file=sys.stderr)
            return 2
        print(f"臂包:{pack.name}  {len(pack.waypoints)} 点  "
              f"时长 {pack.duration_s or pack.dur_ns/1e9:.1f}s  mode={pack.mode}")
        for w in pack.warnings:
            print(f"  ⚠ {w}")

        if a.hand:
            try:
                cues, hwarns = hand_cues_from_pack(a.hand)
            except ComboError as e:
                print(f"手包读不了: {e}", file=sys.stderr)
                return 2
            print(f"手包:{a.hand}  {len(cues)} 帧")
            for w in hwarns:
                print(f"  ⚠ {w}")
    if cues and a.combo:
        print(f"手侧:{len(cues)} 帧(同一条时间轴,来自联合包)")
    return _run(a, pack, cues, combo_mode)


def _run(a: argparse.Namespace, pack: ArmTrajPack, cues: list[HandCue],
         combo_mode: str | None = None) -> int:
    """接入硬件、preflight、approach、回放。

    `combo_mode`:联合包里**显式**的 mode。非 None 时两侧跳帧策略都按它定 ——
    ⚠ 这一点是 combo_pack 存在的理由之一:分开的两个包只能拿帧间距**猜**
    手侧是流还是关键帧(`_hand_is_stream`,阈值 100ms),而联合包录制时就知道
    意图,没必要再猜。猜错的后果是丢动作(把关键帧当流跳掉)。
    """
    arm, hand = None, None
    try:
        if not a.mock_arm:
            from nero_arm import NeroArm
            arm = NeroArm(mock=False, firmware=a.firmware)
            print(f"接入臂 {a.firmware} ...", end="", flush=True)
            if not arm.connect():
                print(f" 失败: {arm.last_error}", file=sys.stderr)
                return 1
            fw = arm.firmware_detected or a.firmware
            print(f" OK,固件 {fw}")
            arm.set_speed_percent(a.speed)
            if not arm.read_enabled(wait=0.5):
                print("⚠ 未使能 —— 读到的状态是未使能,可能是刚接入 LowSpd 帧还没到。"
                      "如果真的未使能,preflight 会挡住", file=sys.stderr)
        else:
            from nero_arm import NeroArm
            arm = NeroArm(mock=True)
            arm.connect()
            print("臂:mock")

        if cues and not a.mock_hand:
            from inspire_hand import InspireHand, InspireHandConfig
            hand = InspireHand(InspireHandConfig(mock=False))
            print("接入手 ...", end="", flush=True)
            if not hand.connect():
                print(" 失败", file=sys.stderr)
                return 1
            print(" OK")
        elif cues:
            from inspire_hand import InspireHand, InspireHandConfig
            hand = InspireHand(InspireHandConfig(mock=True))
            hand.connect()
            print("手:mock")

        # 联合包有显式 mode 就**两侧都按它**,别再走帧间距启发式(见 _run 的注释)。
        skip = None if combo_mode is None else (combo_mode == "stream")
        pl = ComboPlayer(pack, arm, cues, hand, skip_arm=skip, skip_hand=skip)
        bad = pl.preflight()
        if bad:
            print("\n=== Preflight 失败 ===", file=sys.stderr)
            for b in bad:
                print(f"  · {b}", file=sys.stderr)
            return 1
        print("Preflight OK")
        if a.dry_run:
            print("--dry-run:到此为止,不真的回放")
            return 0
        return _play(a, pl, arm)
    finally:
        if arm:
            arm.disconnect()
        if hand:
            hand.disconnect()


def _play(a: argparse.Namespace, pl: ComboPlayer, arm: NeroArm | None) -> int:
    """approach + 主循环。"""
    if not a.mock_arm and not a.yes:
        print("\n真臂!approach 会低速挪到首帧,然后**连续流式发完整条轨迹**。"
              "确认净空、有人看着、臂已使能,然后加 --yes 重跑", file=sys.stderr)
        return 2
    # approach
    print(f"\napproach:低速({a.speed}%)挪到首帧 ...", end="", flush=True)
    ok, msg = pl.approach(speed_pct=a.speed)
    print(f" {msg}")
    if not ok:
        return 1
    # CPV 进模式
    if arm and not arm.cpv_begin():
        print(f"进入 CPV 失败: {arm.last_error}", file=sys.stderr)
        return 1
    try:
        print(f"\n开始回放:预计 {pl.pack.duration_s or pl.total_ns/1e9:.1f}s ...")
        pl.start()
        next_tick = time.monotonic()
        dt = 1.0 / TICK_HZ
        last_prog = -1.0
        while not pl.done:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            pl.tick()
            next_tick += dt
            if next_tick < now:
                next_tick = now + dt
            # 进度每 5% 打一次,别刷屏
            prog = pl.progress()
            if int(prog * 20) > int(last_prog * 20):
                print(f"  {prog * 100:.0f}%", end="", flush=True)
                last_prog = prog
        print(f"\n\n{report(pl)}")
        return 0
    finally:
        if arm:
            arm.cpv_end()


if __name__ == "__main__":
    raise SystemExit(main())
