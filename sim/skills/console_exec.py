#!/usr/bin/env python3
"""sim/skills/console_exec.py — 技能 → console 指令的适配层(合体页那条路)。

为什么需要这一层。技能执行原本只有一条路:runner.py → JointWriter → ROS →
nero_arm_bridge。但真机验证过的是**另一条**路:arm_console.py 独占 can0、
hand_console.py 独占 /dev/ttyUSB0,都不走 ROS。两条路不能同时在 —— 同一条通道
两个写者,后果不是报错而是**互相覆盖**(详见 COMBO_DEBUG.md)。所以语音要能推动
那台已经验过的真机,就得把技能展开的指令翻成 console 方言。

    语音/文本 → intent.py → 信封 → [本模块] → Arm/HandDebugSession.command()
                                     ↑ 只翻译 + 守闸,不碰串口/CAN 本身

**复用而不复制**:指令展开仍走 backend.py(同一套 Step 序列),确认闸仍走
runner.Gate(同一条规则)。本模块只加两件 ROS 那条路不需要的事:方言翻译、
以及**落后检测**(COMBO_DEBUG 明确要求:共享时间轴只保证命令一起发出,
不保证硬件一起到位,超阈值要报出来而不是默默吸收)。

方言对照(左:writer / backend 产出;右:console 协议):

    {"arm": [7], "duration": d}      → arm  {"cmd":"angles","rad":[7]}
    {"hand": [6], "duration": d}     → hand {"cmd":"angles","rad":[6]}
    {"hand_force": v | [6]}          → hand {"cmd":"force","value":v}   ← 排在 angles 前
    {"hand_speed": v | [6]}          → hand {"cmd":"speed","value":v}   ← 排在 angles 前
    {"action":"enable"|"disable"|"reset"} → arm {"cmd": 同名}
    {"action":"set_speed","value":v} → arm  {"cmd":"speed","value":v}
    {"estop": true}                  → arm  {"cmd":"estop"}

③ **力控只在这条路有效。** ROS 那条路(ros_joint_writer)发的是 JointTrajectory,
   消息里没有力控字段,仿真也没有对应控制器。那边会在回显里报 unsupported,
   而不是静默忽略 —— 否则同一个「轻捏」技能在两条路上行为不同,调用方看不出来。

两处**诚实标注**:

① `duration` 在 console 协议里没有对应字段。臂按 `speed_percent` 走,手近乎瞬时。
   所以 duration 只当**本地节拍**(发完等多久再发下一条),不是下给硬件的时长。
   这就是必须做落后检测的原因:我们「以为」5 秒到位,臂可能还在路上。

② **手没有急停通道**(hand_console 里 estop 出现 0 次)。最接近的 action_stop
   会把手**移动**到张开位 —— 那是运动,不是停止,不能当急停用。所以本模块的
   estop 只做两件事:停下自己的下发循环 + 给臂发 estop;手保持当前位置。
   这一点会在事件里明确报出来,不假装手也停了。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Callable, Iterator

sys.path.insert(0, str(Path(__file__).resolve().parent))
from backend import ARM_DOF, HAND_DOF, SkillError, make_backend  # noqa: E402
# Gate 与调用日志都从 runner 借:确认规则和日志格式/路径都必须只有一份实现,
# 否则两条执行路会长出两套解释,而日志正是以后 VLA 的 (instruction, trajectory) 原料。
from runner import Gate, _log_invocation  # noqa: E402
from schema import RegistryError, SkillRegistry, get_registry  # noqa: E402

# 手指令专用日志:每条 hand 指令发之前记一行,含时刻/内容/translate 结果。
# 用于诊断"随机出错" —— 对照日志和真手状态能看出规律。
_HAND_CMD_LOG = Path(__file__).resolve().parent.parent / "out" / "hand_commands.jsonl"


def _log_hand_cmd(rec: dict) -> None:
    """追加一行到 hand_commands.jsonl。格式和 skill_invocations 一致,但只记 hand。"""
    try:
        _HAND_CMD_LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(_HAND_CMD_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except Exception:  # noqa: BLE001
        pass

# 落后阈值:单关节偏差超过它(rad)就报。0.05 rad ≈ 2.9°,比视觉抖动尖峰小、
# 比「压根没走到」大。analyze_traj_demand.py 量到真实需求 p50 只有 4.7 deg/s。
LAG_TOL_RAD = 0.05


def translate(cmd: dict) -> list[tuple[str, dict]]:
    """一条 writer 指令 → [(设备, console 指令)]。设备 ∈ {"arm", "hand"}。

    一条 writer 指令可能同时含 arm 和 hand,那就产出两条 —— **分别发,不合成一条**。
    后端本来就是两个会话两条通道,硬合成只会多一层假原子性(COMBO_DEBUG 的结论)。
    """
    out: list[tuple[str, dict]] = []
    if "estop" in cmd:
        if not cmd.get("estop"):
            raise SkillError("estop=false 没有意义,不下发")
        return [("arm", {"cmd": "estop"})]     # 手没有急停通道,见模块 docstring ②
    if "action" in cmd:
        act = cmd.get("action")
        if act in ("enable", "disable", "reset"):
            return [("arm", {"cmd": act})]
        if act == "set_speed":
            # writer 的 set_speed 是**臂**的百分比(1-100)。手的速度是 0-1000,
            # 另一个量纲;清单里 set_speed_slow 的语义是臂,别顺手也发给手。
            return [("arm", {"cmd": "speed", "value": int(cmd.get("value", 20))})]
        raise SkillError(f"console 路不认识 action={act!r}")
    if "arm" in cmd:
        rad = [float(x) for x in cmd["arm"]]
        if len(rad) != ARM_DOF:
            raise SkillError(f"arm 需要 {ARM_DOF} 个值,给了 {len(rad)}")
        out.append(("arm", {"cmd": "angles", "rad": rad}))
    # 力控/速度必须排在 angles **之前**下发。
    #
    # 顺序是硬要求,不是风格:力控是**状态**,设了就一直有效。先发角度再设力控,
    # 那一次运动用的是上一条指令留下的阈值 —— 一个「轻捏」技能会用上一次「用力握」
    # 的阈值走完全程,等力控设上时东西已经被夹过了。hand_console.ActionPlayer
    # 也是这个顺序(先 SPEED_SET/FORCE_SET 再 ANGLE_SET),两条路保持一致。
    for key, ccmd in (("hand_speed", "speed"), ("hand_force", "force")):
        if cmd.get(key) is None:
            continue
        v = cmd[key]
        if isinstance(v, (list, tuple)):
            if len(v) != HAND_DOF:
                raise SkillError(
                    f"{key} 给列表时需要 {HAND_DOF} 个值(项目顺序),给了 {len(v)}")
            v = [int(x) for x in v]
        else:
            v = int(v)
        out.append(("hand", {"cmd": ccmd, "value": v}))
    if "hand" in cmd:
        rad = [float(x) for x in cmd["hand"]]
        if len(rad) != HAND_DOF:
            raise SkillError(f"hand 需要 {HAND_DOF} 个值,给了 {len(rad)}")
        out.append(("hand", {"cmd": "angles", "rad": rad}))
    if not out:
        raise SkillError(f"指令没有可执行内容: {json.dumps(cmd, ensure_ascii=False)}")
    return out


def targets(spec, reg: SkillRegistry) -> set[str]:
    """这条技能会用到哪些设备。不展开轨迹 —— 几百帧只为知道用哪条通道不值得。"""
    if spec.kind == "trajectory":
        return {"arm", "hand"}                 # npz 两者都有,backend 会校验
    if spec.kind == "composite":
        out: set[str] = set()
        for st in spec.steps:
            child = reg.get(st.get("skill"))
            if child is not None:
                out |= targets(child, reg)
        return out
    a = spec.action or {}
    # hand_force / hand_speed 也算"用手" —— 只设力控不发角度是合法指令(抓握包的
    # 第一步就是它)。漏掉的话闸会以为这条技能不碰手,于是不检查手的前置条件。
    hand_keys = ("hand", "hand_force", "hand_speed")
    return ({"arm"} if ("arm" in a or "action" in a or "estop" in a) else set()) \
        | ({"hand"} if any(k in a for k in hand_keys) else set())


class ConsoleExecutor:
    """在 app_web 进程里执行技能,指令写进两个 console 的 stdin。

    **依赖注入而不是 import app_web**:send_arm/send_hand 就是
    Arm/HandDebugSession.command,arm_state/hand_state 返回最近一帧遥测
    (session.latest)。于是本模块能用假 console 单测 —— 不起 web、不碰硬件。
    sleep/clock 也可注入,测落后检测时不必真等几秒。
    """

    def __init__(self,
                 send_arm: Callable[[dict], dict],
                 send_hand: Callable[[dict], dict],
                 arm_state: Callable[[], dict | None] | None = None,
                 hand_state: Callable[[], dict | None] | None = None,
                 reg: SkillRegistry | None = None,
                 sleep: Callable[[float], None] | None = None,
                 clock: Callable[[], float] | None = None) -> None:
        self.send = {"arm": send_arm, "hand": send_hand}
        self.state = {"arm": arm_state or (lambda: None),
                      "hand": hand_state or (lambda: None)}
        self.reg = reg or get_registry()
        self._sleep = sleep or time.sleep
        self._clock = clock or time.monotonic
        self._stop = False

    def stop(self) -> None:
        """请求停止。下发循环在每步之间和等待期间查它。

        ⚠ 这只是**停止继续下发**,不等于急停 —— 已经发出去的那条指令臂还在执行。
        真急停要另外直接给臂发 estop,不能排在本循环后面等它自己发现。
        """
        self._stop = True

    def _sleep_watching(self, secs: float) -> None:
        """等待期间保持对 stop 的响应,切成小片睡 —— 否则 5 秒的 go_home 中途叫停无效。"""
        deadline = self._clock() + secs
        while True:
            left = deadline - self._clock()
            if left <= 0 or self._stop:
                return
            self._sleep(min(0.05, left))

    # ---- 安全闸 ----
    def _gate(self, spec, env: dict, need: set[str]) -> str | None:
        """返回拒绝原因;None 放行。

        确认那条**委托给 runner.Gate**,保证两条执行路是同一条规则、不出现两份解释。
        给它传 live=None / assume_enabled=True 是刻意的:那两个探针是 ROS 侧语义
        (/joint_states 有没有发布者、bridge 不发布使能态),在 console 路上不成立;
        它们由紧接着的 console 检查替代,而且 console 这边能查到**真值**。
        """
        reason = Gate.check(spec, {"confirmed": bool(env.get("confirmed")),
                                   "assume_enabled": True, "live": None,
                                   "source": env.get("source", "unknown")})
        if reason:
            return reason
        # console 在不在:物理前提,与清单声明无关 —— 通道没开谁也发不出去
        for dev in sorted(need):
            if self.state[dev]() is None:
                return (f"{'机械臂' if dev == 'arm' else '灵巧手'} console 没在跑 —— "
                        f"先在页面上点『接入』")
        # 使能/急停只在**清单自己声明了 arm_enabled** 时预检。
        # 这条很关键:prepare_arm 和 arm_reset 都没声明它,因为它们正是解除
        # 未使能/急停的手段;按 requires 判就不会把「治病的药」也拦下来。
        if "arm_enabled" in spec.requires:
            st = self.state["arm"]() or {}
            if st.get("frozen"):
                return "急停生效中,运动被拒 —— 先执行『复位机械臂』解除"
            if not st.get("enabled"):
                return "臂未使能,运动被拒 —— 先执行『使能机械臂』(遥测显示当前未使能)"
        return None

    def _arm_lag(self, cmd: dict) -> float | None:
        """臂到位没有:遥测 rad 与本条指令目标的最大单关节偏差(rad)。

        只对含 arm 角度的指令有意义;拿不到遥测就返回 None —— 宁可不报,不报假数据。
        """
        if "arm" not in cmd:
            return None
        st = self.state["arm"]() or {}
        rad = st.get("rad")
        if not rad or len(rad) != ARM_DOF:
            return None
        try:
            return max(abs(float(a) - float(b)) for a, b in zip(rad, cmd["arm"]))
        except (TypeError, ValueError):
            return None

    # ---- 主入口 ----
    def invoke(self, env: dict) -> Iterator[dict]:
        """执行一个信封,逐个 yield 事件。

        事件格式与 runner.py 一致(start/progress/done/stopped/error),前端复用
        同一套解析;另加两种本路特有的:warn(急停只管臂)和 lag(臂落后)。
        """
        self._stop = False
        t0 = time.time()
        rid = env.get("request_id") or f"req-{int(t0 * 1000)}"
        sid, src = env.get("skill_id"), env.get("source", "unknown")
        rec = {"request_id": rid, "skill_id": sid, "source": src, "path": "console",
               "params_in": env.get("params") or {},
               "transcript": env.get("transcript"),
               "confidence": env.get("confidence"), "ts": t0}

        spec = self.reg.get(sid) if sid else None
        if spec is None:
            _log_invocation({**rec, "result": "unknown_skill"})
            yield {"type": "error", "request_id": rid,
                   "msg": f"未知技能 {sid!r};可选: {self.reg.ids()}"}
            return
        # 语音路径只许命中白名单 —— 与 runner 同一条规则,两条路都得有
        if src == "voice" and not spec.safety.voice_enabled:
            _log_invocation({**rec, "result": "voice_denied"})
            yield {"type": "error", "request_id": rid,
                   "msg": f"技能 {sid} 不允许语音触发(voice_enabled=false)"}
            return

        params, notes = spec.resolve_params(env.get("params"),
                                            via_voice=(src == "voice"))
        rec["params"], rec["notes"] = params, notes
        need = targets(spec, self.reg)
        reason = self._gate(spec, env, need)
        if reason:
            _log_invocation({**rec, "result": "gate_rejected", "reason": reason})
            yield {"type": "error", "request_id": rid, "skill_id": sid,
                   "need_confirm": spec.safety.need_confirm, "msg": reason}
            return
        try:
            be = make_backend(spec, self.reg)
            total, secs = be.total(params), be.duration_hint(params)
        except SkillError as e:
            _log_invocation({**rec, "result": "expand_failed", "reason": str(e)})
            yield {"type": "error", "request_id": rid, "msg": str(e)}
            return

        yield {"type": "start", "request_id": rid, "skill_id": sid,
               "name": spec.name, "kind": spec.kind, "total": total,
               "est_seconds": round(secs, 2), "devices": sorted(need),
               "path": "console", "notes": notes}
        yield from self._run(be, params, rid, total, t0, rec)

    def _stopped(self, rid, i, total, sent, rec, t0) -> dict:
        _log_invocation({**rec, "result": "stopped", "sent": sent,
                         "elapsed": round(time.time() - t0, 3)})
        return {"type": "stopped", "request_id": rid, "step": i, "total": total,
                "reason": "stop_requested"}

    def _hand_snapshot(self) -> dict | None:
        """读一次手遥测。字段名照 inspire_hand.telemetry(),全部**项目顺序**
        (拇指yaw, 拇指pitch, 食, 中, 无名, 小)。读不到返回 None,不抛。"""
        try:
            st = self.state["hand"]() or {}
        except Exception:  # noqa: BLE001
            return None
        if not st:
            return None
        # ⚠ 字段名照 **hand_console 的 state 帧**,不是 inspire_hand.telemetry()。
        #   两者不一样,我第一版取 angle_act 全拿到 None:
        #     state 帧(29Hz,每帧都有)  → raw(项目序) / rad / target / force_act
        #     telemetry()(1Hz 节流)    → angle_act / temp / status / error
        #   force_act 两边都有,所以"力有值、角度是 None"才那么迷惑。
        out = {k: st.get(k) for k in
               ("raw", "rad", "target", "force_act", "temp", "status", "error", "mock")}
        return out

    def _settled_snapshot(self, tries: int = 12, tol: int = 8) -> dict | None:
        """等手停稳再读。连续两次 raw 差都 ≤tol 才采信,最多等 tries×0.25s。

        为什么不能 hold 一到就读:实测 hand_close 的 hold 是 1.5s,但日志里 4 秒后
        力还在往上冲(995/1204/1426g),说明动作远没在 hold 内结束。hold 一到就读
        取到的是中途值,`verdict` 会把"还在走"误报成 off_target。

        返回最后一次快照,并加 settled 字段标明是否真等稳了 —— 没等稳也要记,
        但要让人看得出这条数据"手当时还在动"。
        """
        prev = None
        for i in range(tries):
            snap = self._hand_snapshot()
            cur = (snap or {}).get("raw")
            if prev is not None and cur is not None and len(cur) == len(prev):
                if max(abs(a - b) for a, b in zip(cur, prev)) <= tol:
                    if snap is not None:
                        snap["settled"] = True
                        snap["settle_waits"] = i
                    return snap
            prev = cur
            self._sleep_watching(0.25)
            if self._stop:
                break
        snap = self._hand_snapshot()
        if snap is not None:
            snap["settled"] = False
            snap["settle_waits"] = tries
        return snap

    @staticmethod
    def _want_raw(cmd: dict) -> list[int] | None:
        """把指令里的 hand 弧度换成期望 raw,好和遥测的 angle_act 直接比。

        换算走 hand_pose(它抄了驱动的 RAW_MAP,且有 --verify 保证不漂)——
        不 import inspire_hand,那会把 sim/ 塞进 sys.path 触发 schema 遮蔽。
        """
        rad = cmd.get("hand")
        if not isinstance(rad, (list, tuple)) or len(rad) != HAND_DOF:
            return None
        try:
            import hand_pose as hp
            return [hp.rad_to_raw(n, float(r))
                    for n, r in zip(hp.HAND_JOINTS, rad)]
        except Exception:  # noqa: BLE001
            return None

    @staticmethod
    def _diag(rec: dict) -> str:
        """自动判一句结论,省得人肉比六个数字。

        判据只用 angle_act,因为"手停在哪"是这次要查的唯一问题。
        阈值 50:实测静止后读数抖动在 ±15,50 有 3 倍余量。
        """
        want = rec.get("want_raw")
        after = (rec.get("after") or {}).get("raw")
        before = (rec.get("before") or {}).get("raw")
        if want is None or not after:
            return "no_telemetry"           # 读不到遥测,没法判
        if len(after) != len(want):
            return "telemetry_len_mismatch"
        off = [abs(a - w) for a, w in zip(after, want)]
        if max(off) <= 50:
            return "ok"                     # 六个通道都到位
        # 没等稳就别下结论 —— 手还在走,这时候的偏差不代表最终位置
        if not (rec.get("after") or {}).get("settled", True):
            bad = [i for i, d in enumerate(off) if d > 50]
            return f"still_moving(等满也没停稳,通道 {bad} 差 {[off[i] for i in bad]})"
        moved = ([abs(a - b) > 50 for a, b in zip(after, before)]
                 if before and len(before) == len(after) else None)
        bad = [i for i, d in enumerate(off) if d > 50]
        # 力冲高 = 手指互顶(堵转)。这是"到不了"而不是"没收到",两者修法完全不同,
        # 所以先判它。阈值 600:力控设定是 250-300,冲到 600 以上只能是硬碰硬。
        fa = (rec.get("after") or {}).get("force_act")
        if isinstance(fa, list) and len(fa) == len(off):
            jam = [i for i, f in enumerate(fa) if isinstance(f, (int, float)) and f > 600]
            if jam:
                return (f"jammed(通道 {jam} 力 {[fa[i] for i in jam]}g 远超设定,"
                        f"互顶堵转;偏差通道 {bad})")
        if moved is not None and not any(moved):
            return f"no_motion(通道 {bad} 差 {[off[i] for i in bad]},且和发之前一样)"
        # 反向:实际值 ≈ 1000 - 期望值,说明某一层 invert 反了
        if all(abs(after[i] - (1000 - want[i])) <= 50 for i in bad):
            return f"inverted(通道 {bad} 实际≈1000-期望,某层 invert 反了)"
        return f"off_target(通道 {bad} 差 {[off[i] for i in bad]})"

    def _run(self, be, params, rid, total, t0, rec) -> Iterator[dict]:
        """逐步翻译并下发。每步之间查 stop,发完按 hold 等待,等完量落后。"""
        sent, worst, reports = 0, 0.0, 0
        try:
            for i, step in enumerate(be.steps(params)):
                if self._stop:
                    yield self._stopped(rid, i, total, sent, rec, t0)
                    return
                # 记手指令:只记 hand 通道的,臂的不记(臂有遥测落后量,问题少)
                trans = list(translate(step.cmd))
                has_hand = any(dev == "hand" for dev, _ in trans)
                hand_log_rec = None
                if has_hand:
                    hand_log_rec = {
                        "ts": time.time(),
                        "request_id": rid,
                        "skill_id": rec["skill_id"],
                        "source": rec["source"],
                        "transcript": rec.get("transcript"),
                        "step": i,
                        "cmd": step.cmd,
                        "hold": step.hold,
                        "translated": trans,
                        # 发之前的状态,和 after 对比才知道"到底动了没有"
                        "before": self._hand_snapshot(),
                    }
                for dev, ccmd in trans:
                    res = self.send[dev](ccmd) or {}
                    if not res.get("ok"):
                        msg = f"{dev} console 写入失败: {res.get('msg')}"
                        _log_invocation({**rec, "result": "send_failed",
                                         "reason": msg})
                        yield {"type": "error", "request_id": rid, "msg": msg}
                        return
                    if ccmd.get("cmd") == "estop":
                        yield {"type": "warn", "request_id": rid,
                               "msg": "急停只作用于臂:手没有急停通道,会保持当前"
                                      "位置。臂无抱闸会缓慢下落,注意下方净空。"}
                sent += 1
                if i % 5 == 0 or i == total - 1:
                    yield {"type": "progress", "request_id": rid, "step": i + 1,
                           "total": total, "label": step.label,
                           "pct": round(100.0 * (i + 1) / max(1, total), 1)}
                if step.hold > 0:
                    self._sleep_watching(step.hold)
                # hold 之后再读一次:这才是"手最终停在哪"。和 before 一比就知道
                # 是没收到(两次一样)、还是收到了但方向反(动了但反着动)。
                if hand_log_rec is not None:
                    hand_log_rec["after"] = self._settled_snapshot()
                    hand_log_rec["want_raw"] = self._want_raw(step.cmd)
                    hand_log_rec["verdict"] = self._diag(hand_log_rec)
                    _log_hand_cmd(hand_log_rec)
                lag = self._arm_lag(step.cmd)
                if lag is not None:
                    worst = max(worst, lag)
                    if lag > LAG_TOL_RAD and reports < 5:
                        reports += 1
                        yield {"type": "lag", "request_id": rid, "step": i + 1,
                               "lag_rad": round(lag, 4), "tol_rad": LAG_TOL_RAD,
                               "msg": f"臂落后 {lag:.3f} rad({lag * 57.3:.1f}°):"
                                      "hold 不足或速度档太低,不是已到位"}
                if self._stop:
                    yield self._stopped(rid, i, total, sent, rec, t0)
                    return
        except SkillError as e:
            _log_invocation({**rec, "result": "run_failed", "reason": str(e)})
            yield {"type": "error", "request_id": rid, "msg": str(e)}
            return
        el, wl = round(time.time() - t0, 3), round(worst, 4)
        _log_invocation({**rec, "result": "done", "sent": sent,
                         "elapsed": el, "worst_lag_rad": wl})
        yield {"type": "done", "request_id": rid, "skill_id": rec["skill_id"],
               "sent": sent, "total": total, "elapsed": el,
               "worst_lag_rad": wl, "lag_exceeded": worst > LAG_TOL_RAD}


def _fake_clock() -> tuple[Callable[[], float], Callable[[float], None]]:
    """假时钟:睡觉就是把表往前拨。测试与干跑都用它 —— 几百帧轨迹不必真等十几秒,
    而且时间可复现(真 time.sleep 会让断言依赖机器负载)。"""
    now = [0.0]

    def clock() -> float:
        return now[0]

    def sleep(secs: float) -> None:
        now[0] += secs
    return clock, sleep


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skill", required=True, help="技能 id")
    ap.add_argument("--params", default="{}", help="参数 JSON")
    ap.add_argument("--confirmed", action="store_true", help="带上确认")
    ap.add_argument("--source", default="cli", help="来源;voice 会走白名单+限速")
    ap.add_argument("--max-cmds", type=int, default=12, help="最多打印多少条指令")
    ap.add_argument("--not-enabled", action="store_true", help="假装臂未使能,验安全闸")
    ap.add_argument("--no-console", action="store_true", help="假装 console 没在跑")
    args = ap.parse_args()

    try:
        reg = get_registry()
    except RegistryError as e:
        print(f"✗ 清单有问题: {e}")
        return 1

    cmds: list[tuple[str, dict]] = []

    def fake(dev: str) -> Callable[[dict], dict]:
        def send(cmd: dict) -> dict:
            cmds.append((dev, cmd))
            return {"ok": True}
        return send

    clock, sleep = _fake_clock()
    arm_st = {"rad": [0.0] * ARM_DOF, "enabled": not args.not_enabled,
              "frozen": False}
    none_if = (lambda v: None if args.no_console else v)
    ex = ConsoleExecutor(fake("arm"), fake("hand"),
                         arm_state=lambda: none_if(arm_st),
                         hand_state=lambda: none_if({"rad": [0.0] * HAND_DOF}),
                         reg=reg, sleep=sleep, clock=clock)
    env = {"skill_id": args.skill, "params": json.loads(args.params),
           "source": args.source, "confirmed": args.confirmed}
    rc = 1
    for ev in ex.invoke(env):
        if ev.get("type") == "progress":
            continue
        print(json.dumps(ev, ensure_ascii=False))
        if ev.get("type") == "done":
            rc = 0
    print(f"\n翻译出 {len(cmds)} 条 console 指令(模拟耗时 {clock():.2f}s):")
    for dev, c in cmds[:args.max_cmds]:
        print(f"  {dev:5} {json.dumps(c, ensure_ascii=False)}")
    if len(cmds) > args.max_cmds:
        print(f"  … 另有 {len(cmds) - args.max_cmds} 条")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
