#!/usr/bin/env python3
"""sim/hand_console.py — 灵巧手调试台后端:独占 RS485 串口,JSON 走 stdin/stdout。

给 app_web.py 的「灵巧手调试」页用。和 nero_arm_bridge 的区别是**不依赖 ROS**:
调试手不需要臂、不需要 rclpy、不需要 colcon build 过的工作区,只要 pyserial。
所以它能在臂 CAN 还没通的时候单独把手调起来。

  stdin  ← {"cmd":"angles","rad":[6]}      设目标角(rad,项目顺序)
          {"cmd":"raw","raw":[6]}          直接下 raw 0-1000(厂商通道序,标定用)
          {"cmd":"speed","value":500}      速度 0-1000
          {"cmd":"force","value":500}      力控阈值 0-1000
          {"cmd":"home"}                   回安全张开位
          {"cmd":"gesture_play","pack":{...}}  回放手势技能包(内联数据,非路径)
          {"cmd":"clear_error"}
          {"cmd":"quit"}                   复位到安全位后断开
  stdout → {"type":"state","rad":[6],"raw":[6],"t":..,...}   周期遥测
          {"type":"ready"|"closed"|"error"|"ack", ...}

⚠ 串口是**独占**的:本进程跑着的时候 nero_arm_bridge --no-hand-mock 会打不开
  /dev/ttyUSB0。调试页和实时 Live 不要同时接手。

用法:
  python3 sim/hand_console.py --mock          # 无硬件空跑
  python3 sim/hand_console.py --no-mock       # 真手
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stdin_lines import StdinLines                                        # noqa: E402
from inspire_hand import (HAND_JOINTS, HAND_LIMITS, PROJECT_TO_VENDOR,   # noqa: E402
                          InspireHand, InspireHandConfig)
from action_sequences import load_default_actions, ActionSequence          # noqa: E402
from gesture_pack import (GestureError, GesturePack,                       # noqa: E402
                          to_action_sequence)

# 安全张开位 = 六个关节全取 URDF 下限 = 完全打开的平手。
# thumb_yaw 曾经取中位(方向未标定时的保守值)。现在方向已由 URDF 几何定死:
# yaw=0 是拇指躺在掌面里(raw 1000,完全打开),yaw=1.308 是垂直掌面的对掌位(raw 0)。
# 所以 0 就是张开,不用再取中位。
HOME_RAD = [HAND_LIMITS[n][0] for n in HAND_JOINTS]


class ActionPlayer:
    """动作序列播放器:逐步下发,支持暂停/继续/停止。

    时序:文件里每步的延时是**下发动作后的驻留时间**(等手走到位),不是下发前的等待。
    所以第一步立即执行,之后每步等上一步的 delay_ms 到了再走 —— 反过来写会让
    第一步白等,且最后一步的驻留时间被丢掉。

    动作序列里的 6 个通道值是**厂商顺序**(m=0 小拇指 … m=5 拇指旋转),不是项目顺序,
    所以这里下发前要用 PROJECT_TO_VENDOR 反查;-1 表示该通道本步不动作。
    """

    def __init__(self, seq: ActionSequence, hand: InspireHand) -> None:
        self.seq = seq
        self.hand = hand
        self.step_idx = 0            # 下一个要执行的步骤
        self.paused = False
        self.stopped = False
        self.done = False
        self._wait_until = 0.0       # 到这个时刻才能执行下一步
        self._pause_left = 0.0       # 暂停时剩余的驻留时间
        self._paused_at = 0.0        # 进入暂停的时刻
        # 序列起点的 monotonic 时刻。有 t_ns 时用它把绝对时刻换算成本地时刻。
        self._t0 = 0.0
        self._paused_total = 0.0     # 累计暂停了多久 —— 绝对时轴要减掉它
        # 这个序列有没有绝对时刻。厂商的 DefaultAction.txt 没有,我们的技能包有。
        self._abs = any(s.t_ns is not None for s in seq.steps)

    def start(self, *, start_at: float | None = None) -> None:
        """开始回放。

        start_at: 跨进程对齐用 —— 指定一个共同的 CLOCK_MONOTONIC 时刻作为 t0。
                  None = 立即开始(time.monotonic())。
        """
        self.step_idx = 0
        self.paused = self.stopped = self.done = False
        self._wait_until = 0.0       # 0 = 立即执行第一步
        self._t0 = start_at if start_at is not None else time.monotonic()
        self._paused_total = 0.0

    def pause(self) -> None:
        if self.paused or self.done:
            return
        self.paused = True
        self._paused_at = time.monotonic()      # resume() 要拿它算暂停了多久
        self._pause_left = max(0.0, self._wait_until - time.monotonic())

    def resume(self) -> None:
        if not self.paused:
            return
        self.paused = False
        # ⚠ 绝对时轴下必须把**暂停时长**累进 _paused_total,不能只恢复剩余驻留。
        # 不累的话恢复瞬间 (now - t0) 已经跑过了好几个 t_ns,播放器会连着补好几步
        # —— 手根本走不到位,中间的姿态直接被跳过。
        # 这和 tick() 里"落后太多就重新对齐"是同一个道理:宁可整段拖长,不能丢姿态。
        if self._abs:
            self._paused_total += max(0.0, time.monotonic() - self._paused_at)
        self._wait_until = time.monotonic() + self._pause_left   # 接着走剩下的驻留

    def stop(self) -> None:
        self.stopped = True

    def tick(self) -> dict | None:
        """每帧调用。到点就下发下一步;返回进度事件或 None。"""
        if self.stopped or self.paused or self.done:
            return None
        # ⚠ 两个检查的**顺序要紧**:必须先看驻留有没有走完,再看步骤有没有走完。
        # 反过来写(先判 step_idx >= len)会在最后一步刚下发、手还在往目标走的时候
        # 立刻宣布 done —— 最后一帧的驻留时间被整段丢掉。实测:末帧驻留 500ms,
        # action_done 在下发后 49ms 就来了。
        # 影响不只是进度显示早跳一下:谁要是拿 action_done 当"手已到位"的信号去接
        # 下一个动作(比如串放两个技能包),末姿态就会被下一个动作截断。
        if time.monotonic() < self._wait_until:
            return None                                  # 还在上一步的驻留里
        if self.step_idx >= len(self.seq.steps):
            self.done = True
            return None

        step = self.seq.steps[self.step_idx]
        # 速度/力控:本步给了就先设(抓握动作靠边合边降速,跳过会顶死或抓不稳)
        if any(v is not None for v in step.speeds):
            self.hand.write_shorts("SPEED_SET", self._vendor(step.speeds))
        if any(v is not None for v in step.forces):
            self.hand.write_shorts("FORCE_SET", self._vendor(step.forces))
        ok = self._send_angles(step.angles)

        self.step_idx += 1
        # ⚠ 下一步的截止时刻从**上一个截止时刻**累加,不是从 now 累加。
        # 从 now 累加会把 tick 量化误差**逐步攒起来**:tick 10ms、驻留 33ms 时,
        # 实际触发在 40ms(ceil(33/10)*10),而下一步又从这个 40ms 起算 —— 每步都
        # 多 7ms,整段慢 1.21×。实测 180 帧的 5.97s 素材跑成 7.28s 就是这么来的。
        # 用绝对时间轴的话触发点是 40/70/100/140…,平均正好 33.3ms,误差不累积。
        now = time.monotonic()
        # ⚠ 有绝对时刻就**按它定位**,不要累加 delay_ms。
        # delay_ms 是整数毫秒,而 30fps 的真周期是 33.3333…ms —— 每步少 0.333ms 且
        # 单向不抵消,600 帧(20s)攒 200ms、2400 帧攒 800ms。实测过:600 帧的包
        # 按累加走末尾比按 t_ns 走早 199.7ms。臂侧是按绝对时刻走的,手侧一累加
        # 两边就错开,抓取动作里手已经合上而臂还没到位。
        # 用 t_ns 后每一步的截止时刻都是**独立算出来**的,量化残差不累积。
        nxt = self.seq.steps[self.step_idx] if self.step_idx < len(self.seq.steps) else None
        if self._abs and nxt is not None and nxt.t_ns is not None:
            self._wait_until = self._t0 + self._paused_total + nxt.t_ns / 1e9
        elif self._abs and nxt is None and step.t_ns is not None:
            # 最后一步:没有"下一个 t_ns"可用,末帧的驻留只能用 delay_ms 补。
            # 这一步的驻留必须留住 —— 丢掉的话 action_done 会在手还没走到位时就来。
            self._wait_until = (self._t0 + self._paused_total
                                + step.t_ns / 1e9 + step.delay_ms / 1000.0)
        else:
            base = self._wait_until if self._wait_until else now
            self._wait_until = base + step.delay_ms / 1000.0
        # 落后太多就重新对齐(比如串口卡了一下)。不这么做的话绝对时间轴会要求
        # "补课",连续几个 tick 每次都发一步 —— 手根本走不到位,姿态直接被跳过。
        # 宁可整段稍微拖长,也不能丢姿态。
        if self._wait_until < now:
            self._wait_until = now + step.delay_ms / 1000.0
        return {"type": "action_step", "slot": self.seq.slot, "index": self.seq.index,
                "step": self.step_idx, "total": len(self.seq.steps),
                "delay_ms": step.delay_ms, "ok": ok}

    @staticmethod
    def _vendor(vals: list[int | None]) -> list[int]:
        """项目顺序 → 厂商通道顺序,None → -1(该通道不动作)。"""
        out = [-1] * 6
        for i, v in enumerate(vals):
            if v is not None:
                out[PROJECT_TO_VENDOR[i]] = int(v)
        return out

    def _send_angles(self, angles: list[int | None]) -> bool:
        vendor = self._vendor(angles)
        if not self.hand.cfg.mock:
            return self.hand.write_shorts("ANGLE_SET", vendor)
        # mock:把 raw 折回 rad 存进目标,3D 和滑块才跟着动(否则 mock 下画面不动)。
        # -1 的通道保持上一步的值。
        rad = list(self.hand._target_rad)
        for i, n in enumerate(HAND_JOINTS):
            v = vendor[PROJECT_TO_VENDOR[i]]
            if v >= 0:
                rad[i] = self.hand.raw_to_rad(n, v)
        return self.hand.set_angles(rad)


def emit(obj: dict) -> None:
    print(json.dumps(obj, ensure_ascii=False), flush=True)


_player: ActionPlayer | None = None


def handle(hand: InspireHand, cmd: dict, sequences: list[ActionSequence]) -> dict:
    """处理一条指令,返回 ack。限位夹取在 InspireHand 里做,这里不放行越界值。"""
    global _player
    c = cmd.get("cmd")
    if c == "angles":
        rad = cmd.get("rad") or []
        if len(rad) != 6:
            return {"type": "error", "msg": f"angles 需要 6 个值,收到 {len(rad)}"}
        ok = hand.set_angles([float(x) for x in rad])
        return {"type": "ack", "cmd": c, "ok": ok}
    if c == "raw":
        # 标定用:绕过 rad 换算直接下 raw(厂商通道序)。方向/端点存疑时用它对照实物。
        raw = cmd.get("raw") or []
        if len(raw) != 6:
            return {"type": "error", "msg": f"raw 需要 6 个值,收到 {len(raw)}"}
        raw = [max(-1, min(1000, int(v))) for v in raw]      # -1 = 该通道不动作
        ok = hand.write_shorts("ANGLE_SET", raw) if not hand.cfg.mock else True
        return {"type": "ack", "cmd": c, "ok": ok, "raw": raw}
    if c == "speed":
        return {"type": "ack", "cmd": c, "ok": hand.set_speed(int(cmd.get("value", 500)))}
    if c == "force":
        # value 可以是标量(全通道同值)或 6 个值(逐通道,项目顺序)。
        # 逐通道是真需求:捏鸡蛋要拇指+食指轻、其余不动。底层 write_shorts 一直是
        # 6 通道的,只是这一层以前把它拍平成标量。
        # **逐通道夹取交给 inspire_hand.set_force** —— FORCE_MAX 那张表只有一份,
        # 在这里再抄一遍必然和它漂移。
        v = cmd.get("value", 500)
        try:
            fval = [int(x) for x in v] if isinstance(v, (list, tuple)) else int(v)
        except (TypeError, ValueError):
            return {"type": "error", "msg": f"force 的 value 不是数字或数字列表: {v!r}"}
        if isinstance(fval, list) and len(fval) != 6:
            return {"type": "error",
                    "msg": f"force 给列表时需要 6 个值(项目顺序),收到 {len(fval)}"}
        return {"type": "ack", "cmd": c, "ok": hand.set_force(fval), "value": fval}
    if c == "home":
        return {"type": "ack", "cmd": c, "ok": hand.set_angles(list(HOME_RAD))}
    if c == "clear_error":
        return {"type": "ack", "cmd": c, "ok": hand.clear_error()}
    # --- 动作序列控制。定位用 slot(唯一),不用 index(文件里重复) ---
    if c == "action_start":
        slot = cmd.get("slot")
        seq = next((s for s in sequences if s.slot == slot), None)
        if seq is None:
            return {"type": "error", "msg": f"未找到动作 slot={slot}"}
        _player = ActionPlayer(seq, hand)
        _player.start()
        return {"type": "ack", "cmd": c, "slot": slot, "index": seq.index,
                "name": seq.name, "steps": len(seq.steps)}
    if c == "action_pause":
        if _player is None:
            return {"type": "ack", "cmd": c, "ok": False, "msg": "没有在播的动作"}
        _player.pause()
        return {"type": "ack", "cmd": c, "ok": True}
    if c == "action_resume":
        if _player is None:
            return {"type": "ack", "cmd": c, "ok": False, "msg": "没有在播的动作"}
        _player.resume()
        return {"type": "ack", "cmd": c, "ok": True}
    if c == "action_stop":
        if _player is not None:
            _player.stop()
            _player = None
        # 关闭 = 回初始状态。速度也恢复到初始值 —— 序列里可能把速度降到 100,
        # 不恢复的话后面手动拖滑块会慢得像卡住。
        hand.set_speed(hand.cfg.init_speed)
        hand.set_force(hand.cfg.init_force)
        ok = hand.set_angles(list(HOME_RAD))
        return {"type": "ack", "cmd": c, "ok": ok}
    # --- 手势技能包回放。pack 是**内联**的完整数据,不是路径 ---
    # 路径解析/沙箱校验全在 web 层做完(gesture_pack.resolve_pack_path)。console
    # 不碰文件系统 —— 校验逻辑散成两份的话,哪天只改了一处就是个洞。
    if c == "gesture_play":
        try:
            pack = GesturePack.from_dict(cmd.get("pack") or {})
            seq = to_action_sequence(pack, slot=-1, return_home=cmd.get("return_home"))
        except (GestureError, ValueError, TypeError) as e:
            return {"type": "error", "msg": f"技能包不合法: {e}"}
        _player = ActionPlayer(seq, hand)
        start_at = cmd.get("start_at")
        _player.start(start_at=start_at if start_at else None)
        return {"type": "ack", "cmd": c, "name": pack.name, "steps": len(seq.steps),
                "frames": len(pack.frames), "gesture": True,
                "duration_ms": pack.duration_ms}
    if c == "list_actions":
        return {"type": "actions", "sequences": [
            {"slot": s.slot, "index": s.index, "name": s.name, "steps": len(s.steps)}
            for s in sequences]}
    return {"type": "error", "msg": f"未知指令: {c}"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", dest="mock", action="store_true", default=True,
                    help="无硬件空跑(默认)")
    ap.add_argument("--no-mock", dest="mock", action="store_false",
                    help="真手 RS485")
    ap.add_argument("--port", default=os.environ.get("INSPIRE_HAND_PORT", "/dev/ttyUSB0"))
    ap.add_argument("--hand-id", type=int, default=1)
    ap.add_argument("--hz", type=float, default=20.0, help="遥测发布率")
    # ⚠ 播放器的 tick 率和遥测率**分开**。
    # ActionPlayer.tick() 的调用周期就是回放的时间分辨率:一步的驻留只能是整数个
    # tick。而且 hold_ms 和 tick 同量级时是最坏情况 —— 循环落在截止时刻前一点点
    # 就要再等一整个 tick,于是 33ms 的驻留时而 33ms、时而 66ms,整段拖慢三成多。
    # 实测:tick=33ms 回放 33ms/帧的素材,180 帧跑成 8.10s(源 5.97s,慢 1.36×)。
    # 所以 tick 要**远小于**最短驻留。100Hz(10ms)对 33ms 驻留是 ~10% 量化误差,
    # 而遥测仍按 --hz 发,不会把 100 帧/秒 JSON 灌给浏览器。
    ap.add_argument("--player-hz", type=float, default=100.0,
                    help="动作播放器 tick 率(回放时间分辨率),独立于遥测率")
    ap.add_argument("--speed", type=int, default=500, help="上电初始化速度 0-1000")
    ap.add_argument("--force", type=int, default=500, help="上电初始化力控 0-1000")
    ap.add_argument("--full-telemetry-every", type=float, default=1.0,
                    help="全量遥测(温度/电流/故障)间隔秒;角度仍按 --hz 读")
    args = ap.parse_args()

    global _player
    sequences = load_default_actions()              # 加载动作序列
    hand = InspireHand(InspireHandConfig(
        port=args.port, hand_id=args.hand_id, mock=args.mock,
        init_speed=args.speed, init_force=args.force))
    try:
        hand.connect()
    except Exception as e:                                   # noqa: BLE001
        emit({"type": "error", "fatal": True, "msg": str(e)})
        return
    emit({"type": "ready", "mock": args.mock, "port": args.port,
          "joints": HAND_JOINTS, "vendor_order": PROJECT_TO_VENDOR,
          "limits": [list(HAND_LIMITS[n]) for n in HAND_JOINTS],
          "actions": [{"slot": s.slot, "index": s.index, "name": s.name,
                       "steps": len(s.steps)} for s in sequences]})

    dt = 1.0 / max(1.0, args.hz)                       # 遥测周期
    # 播放器 tick 周期。**不播动作时不需要跑这么快** —— 空转 100Hz 纯烧 CPU,
    # 所以下面的等待时间取"遥测截止"和"播放器截止"里更近的那个,而播放器截止
    # 只在 _player 存在时才参与。
    pdt = 1.0 / max(1.0, args.player_hz)
    t0 = time.monotonic()
    last_full = 0.0
    next_tick = time.monotonic()                       # 下一次发遥测
    next_ptick = time.monotonic()                      # 下一次 tick 播放器
    stdin_lines = StdinLines()
    last_angle_at: float | None = None
    last_angle_id = None
    last_angle_serial_ms: float | None = None
    last_angle_settled_ms: float | None = None
    last_mimic_angle_at: float | None = None
    next_force_tick = time.monotonic()

    def process_line(line: str) -> None:
        nonlocal last_angle_at, last_angle_id, last_angle_serial_ms
        nonlocal last_angle_settled_ms, last_mimic_angle_at
        try:
            cmd = json.loads(line)
        except json.JSONDecodeError:
            return
        if cmd.get("cmd") == "quit":
            raise KeyboardInterrupt
        received_ns = time.perf_counter_ns()
        event = handle(hand, cmd, sequences)
        completed_ns = time.perf_counter_ns()
        if cmd.get("cmd") == "angles":
            meta = cmd.get("_perf") or {}
            try:
                enqueued_ns = int(meta.get("enqueued_ns") or received_ns)
            except (TypeError, ValueError):
                enqueued_ns = received_ns
            serial_ms = (completed_ns - received_ns) / 1e6
            event["perf"] = {
                "id": meta.get("id", cmd.get("perf_id")),
                "ack_token": meta.get("ack_token"),
                "source": meta.get("source"),
                "stdin_queue_ms": round((received_ns - enqueued_ns) / 1e6, 2),
                "serial_ms": round(serial_ms, 2),
                "enqueue_to_serial_ms": round((completed_ns - enqueued_ns) / 1e6, 2),
            }
            last_angle_at = time.monotonic()
            if meta.get("source") == "mimic":
                last_mimic_angle_at = last_angle_at
            last_angle_id = event["perf"]["id"]
            last_angle_serial_ms = serial_ms
            last_angle_settled_ms = None
        emit(event)

    try:
        while True:
            # 读 stdin。阻塞等到下一个截止时刻为止 —— 命令一到就立刻醒,
            # 不用干等满一个周期(旧写法 timeout=0 + 循环末尾无条件 sleep(dt),
            # 命令最坏要压 50ms 才被看到)。
            #
            # ⚠ 必须用 StdinLines(os.read),**不能**用 select + sys.stdin.readline:
            #   那两个看的不是同一层,readline 会把 fd 上已到的字节全抽进用户态缓冲
            #   却只返回一行,剩下的行卡在缓冲里而 fd 已空 —— 下一轮 select 说"没数据"。
            #   一个手势技能发 3 行(speed/force/angles),于是只处理掉 speed,
            #   force+angles 要等下一条命令到了才被读出来 = **手慢一个命令**。
            #   详见 stdin_lines.py 的模块说明。
            deadline = next_tick if _player is None else min(next_tick, next_ptick)
            timeout = max(0.0, deadline - time.monotonic())
            for line in stdin_lines.poll(timeout):
                process_line(line)
            if stdin_lines.eof:                              # stdin 关闭 = 上层退出
                raise KeyboardInterrupt

            # 动作播放器 tick。done 由 tick() 判定 —— 最后一步的驻留时间要走完才算完,
            # 用 step_idx >= len(steps) 直接判会提前一个驻留期宣布结束。
            # 按 pdt 走自己的节奏(比遥测快得多),否则回放分辨率被遥测率锁死。
            now = time.monotonic()
            if _player is not None and now >= next_ptick:
                # 同样按绝对时间轴累加,不是 now + pdt —— 后者每轮都把本轮的处理
                # 耗时算进周期,tick 率会系统性偏低。落后超过一个周期才重新对齐。
                next_ptick += pdt
                if next_ptick < now:
                    next_ptick = now + pdt
                ev = _player.tick()
                if ev:
                    emit(ev)
                if _player.done:
                    # name 也带上:技能包的 slot 恒为 -1(它不在 DefaultAction.txt 里),
                    # 前端光看 slot 分不出是哪个包播完了。
                    emit({"type": "action_done", "slot": _player.seq.slot,
                          "index": _player.seq.index, "name": _player.seq.name})
                    _player = None

            # 遥测没到点就回去继续等命令/下一次 player tick。
            # 少了这个判断的话,播放器每 10ms tick 一次会顺带把遥测也发 100 次/秒。
            if time.monotonic() < next_tick:
                continue

            # ANGLE_SET 优先于遥测。播放器 tick 后到达的命令在开始任何读寄存器
            # 之前再排空一次，避免被一轮 FORCE_ACT/全量遥测压住。
            for line in stdin_lines.poll(0):
                process_line(line)
            if stdin_lines.eof:
                raise KeyboardInterrupt

            now = time.monotonic()
            # raw 和 rad 一起发:调试页要看原始 ANGLE_ACT(和 demo_485 的 read6('angleAct')
            # 打印的是同一组数),rad 是换算后的。一次串口往返同时得到两者,不额外读。
            raw_vendor = (None if hand.cfg.mock
                          else hand.read_regs("ANGLE_ACT", 12, "6h"))
            if raw_vendor is not None:
                rad = [hand.raw_to_rad(n, raw_vendor[PROJECT_TO_VENDOR[i]])
                       for i, n in enumerate(HAND_JOINTS)]
            else:
                rad = hand.read_angles()
                raw_vendor = [hand.rad_to_raw(n, r)
                              for n, r in zip(HAND_JOINTS, rad)]
                # mock:上面按项目序算的,转成厂商序保持和真机同构
                vend = [0] * 6
                for i, m in enumerate(PROJECT_TO_VENDOR):
                    vend[m] = raw_vendor[i]
                raw_vendor = vend
            row = {"type": "state", "t": round(now - t0, 3),
                   "names": HAND_JOINTS, "rad": [round(v, 4) for v in rad],
                   "raw_vendor": list(raw_vendor),
                   "raw": [raw_vendor[PROJECT_TO_VENDOR[i]] for i in range(6)],
                   "target": [round(v, 4) for v in hand._target_rad]}
            if last_angle_at is not None:
                errors = [abs(target - actual)
                          for target, actual in zip(hand._target_rad, rad)]
                target_age_ms = (time.monotonic() - last_angle_at) * 1000
                if last_angle_settled_ms is None and max(errors) <= 0.05:
                    last_angle_settled_ms = target_age_ms
                row["tracking_perf"] = {
                    "id": last_angle_id,
                    "target_age_ms": round(target_age_ms, 1),
                    "mean_err_rad": round(sum(errors) / len(errors), 4),
                    "max_err_rad": round(max(errors), 4),
                    "last_serial_ms": round(last_angle_serial_ms or 0.0, 2),
                    "settled_ms": (round(last_angle_settled_ms, 1)
                                   if last_angle_settled_ms is not None else None),
                }
            # 力单独按**每帧**读(只 1 个寄存器,~3ms),不跟着全量遥测的 1s 节流走。
            #
            # 为什么必须这样:抓握时力会冲高再落回,1Hz 采样在一次 1-2 秒的抓握里
            # 只取到 1-2 个点,峰值基本靠运气 —— 真实峰值会被系统性低估。
            # 全量遥测要读 8 个寄存器所以贵、必须节流;温度/电流/状态本来变化慢,
            # 1Hz 够用。只有力需要快。
            # 项目顺序,和 telemetry() 一致(见 inspire_hand.telemetry 的注释)。
            mimic_active = (
                last_mimic_angle_at is not None
                and now - last_mimic_angle_at < 0.5
            )
            if not hand.cfg.mock and (not mimic_active or now >= next_force_tick):
                fv = hand.read_regs("FORCE_ACT", 12, "6h")
                row["force_act"] = ([int(fv[PROJECT_TO_VENDOR[i]]) for i in range(6)]
                                    if fv is not None else None)
                next_force_tick = now + (0.1 if mimic_active else dt)
            # 连续视觉控制期间全量遥测让路。温度/电流/状态变化慢，目标空闲
            # 500ms 后会立即补读，不会永久丢失安全数据。
            target_idle = last_angle_at is None or now - last_angle_at >= 0.5
            if target_idle and now - last_full >= args.full_telemetry_every:
                last_full = now
                row["tel"] = hand.telemetry()
                if hand.last_error:
                    row["last_error"] = hand.last_error
            emit(row)
            # 固定周期:扣掉这一轮串口/遥测花的时间,而不是无条件再睡 dt。
            # 旧写法实际周期 = dt + 本轮耗时,全量遥测那一拍会明显拖长。
            next_tick += dt
            if next_tick < now:                          # 落后太多(如刚跑完全量遥测)
                next_tick = now + dt                     # 就重新对齐,别追赶式空转
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前复位到安全张开位,再断开 —— 不留在握紧姿态上
        try:
            hand.set_angles(list(HOME_RAD))
            time.sleep(0.8)                                  # 给手走到位的时间
        except Exception:                                    # noqa: BLE001
            pass
        hand.disconnect()
        emit({"type": "closed"})


if __name__ == "__main__":
    main()
