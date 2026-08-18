#!/usr/bin/env python3
"""src/skills/runner.py — 技能执行器(跑在 ROS2 system python3)。

在链路里的位置:
    Web 按钮 / 语音 / 未来 VLA  →  调用信封  →  [本模块]  →  JointWriter.send()
                                                              ↑ 限位夹取唯一入口

照 traj_player.py 的做法 import JointWriter,**复用它的限位夹取与发布**,不另造
发布逻辑。真机/仿真同一条路。订阅 /nero/estop:收到即停,当场退出。

安全闸(全在本层强制,上层绕不过去):
  1. need_confirm 的技能,信封里没有 confirmed=true 就拒发。
  2. requires:live_session → 真查 /joint_states 有没有发布者。
  3. requires:arm_enabled  → **bridge 不发布使能状态,ROS 侧无法验证**。
     故要求调用方显式给 assume_enabled=true 表态,否则拒发。不假装检查过。
  4. 执行中每步都先 spin 一次看急停。

调用信封(JSON):
  {"skill_id": "go_home", "params": {...}, "source": "web|voice|vla",
   "request_id": "...", "confirmed": true, "assume_enabled": true,
   "transcript": "回零位", "confidence": 0.93}
transcript/confidence 只用于落日志 —— 语音原话 + 技能 id 的配对就是后面
VLA 要的 (instruction, trajectory) 标注,顺手存下来。

进度打 stdout(JSON 行),格式与 traj_player.py 一致,前端可复用同一套解析:
  {"type":"start"|"progress"|"done"|"stopped"|"error", ...}

两种用法:
  单发:python3 src/skills/runner.py --once '{"skill_id":"go_home","confirmed":true}'
  流式:python3 src/skills/runner.py      # 逐行读 stdin 信封
  干跑:加 --dry-run,只打印指令不发 ROS(无需 bridge 也能验证展开与安全闸)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parent
SIM_DIR = SKILLS_DIR.parent
sys.path.insert(0, str(SKILLS_DIR))
sys.path.insert(0, str(SIM_DIR))

from backend import HAND_NAMES as BE_HAND_NAMES, SkillError, make_backend  # noqa: E402
from schema import RegistryError, get_registry  # noqa: E402

# 调用日志:每次调用一行 JSON。语音原话 + skill_id 的配对是 VLA 标注的原料。
LOG_PATH = SIM_DIR / "out" / "skill_invocations.jsonl"

# 解析日志:每次意图解析一行 JSON,**成功和失败都记**。
#
# 为什么成功也记:只记失败的话,能知道"漏了 50 条",但不知道是 50/60 还是 50/5000。
# 漏词率算不出来,而那正是判断"要不要上更强的匹配"的唯一依据。分母必须有。
#
# 为什么这是最有价值的一份数据:no_match/ambiguous 的原话是**真人真会说、而清单
# 没覆盖**的说法。模板扩写造不出这种东西 —— 它只会把已有别名排列组合,教出来的
# 模型擅长的是我们自己的说话习惯。
#
# ⚠ 里面是原始语音/文本内容,只留在本地 src/out/,别往外传。
PARSE_LOG_PATH = SIM_DIR / "out" / "voice_parses.jsonl"


def _emit(ev: dict) -> None:
    print(json.dumps(ev, ensure_ascii=False), flush=True)


def _append_jsonl(path: Path, rec: dict) -> None:
    """追加一行 JSON。写失败**静默吞掉** —— 日志不能成为控制链路的故障点。"""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except Exception:                                    # noqa: BLE001
        pass


def _log_invocation(rec: dict) -> None:
    _append_jsonl(LOG_PATH, rec)


def log_parse(it, *, scope: str = "all", source: str = "unknown",
              extra: dict | None = None) -> None:
    """落盘一次意图解析。it 是 intent.Intent。

    **只在真人入口调用(app_web 的 /api/voice/parse)**,不要放进 intent.parse():
    那个函数被测试和 `intent.py --all` 调用,一次跑几千条,会把日志灌满合成数据,
    漏词率就算不出真值了。

    候选分数要留:它区分"完全不认识"和"差一点点就中了"。后者只要给清单补一条
    别名就解决,不需要动模型 —— 这是最省的那条路,得能从数据里看出来。
    """
    cands = [{"skill_id": c.skill_id, "score": round(c.score, 4),
              "matched": c.matched}
             for c in (getattr(it, "candidates", None) or [])[:5]]
    rec = {
        "ts": time.time(),
        "text": getattr(it, "text", ""),
        "reason": getattr(it, "reason", None),
        "ok": bool(getattr(it, "ok", False)),
        "skill_id": getattr(it, "skill_id", None),
        "confidence": round(float(getattr(it, "confidence", 0.0) or 0.0), 4),
        "kind": getattr(it, "kind", None),
        "scope": scope,
        "source": source,
        "candidates": cands,
        "notes": list(getattr(it, "notes", None) or []),
    }
    if extra:
        rec.update(extra)
    _append_jsonl(PARSE_LOG_PATH, rec)


class Gate:
    """安全闸判定。与 ROS 无关的部分放这里,便于单测。"""

    @staticmethod
    def check(spec, env: dict) -> str | None:
        """返回拒绝原因;None 表示放行。

        env: {"confirmed": bool, "assume_enabled": bool, "live": bool|None,
              "source": str}
        live=None 表示没查(干跑模式)。
        """
        if spec.safety.need_confirm and not env.get("confirmed"):
            return (f"技能 {spec.id} 需要确认:信封里缺 confirmed=true"
                    f"(name={spec.name})")
        if "live_session" in spec.requires:
            live = env.get("live")
            if live is False:
                return ("前置 live_session 不满足:/joint_states 没有发布者,"
                        "请先启动 nero_arm_bridge(Web 上的『实时 Live』)")
        if "arm_enabled" in spec.requires and not env.get("assume_enabled"):
            return ("前置 arm_enabled 无法从 ROS 验证(bridge 不发布使能状态)。"
                    "调用方须显式给 assume_enabled=true 表态,或先执行 arm_enable。")
        return None


class SkillRunner:
    """执行技能。dry_run=True 时完全不 import rclpy,可在无 ROS 环境验证安全闸。"""

    def __init__(self, dry_run: bool = False) -> None:
        self.dry_run = dry_run
        self.reg = get_registry()
        self.node = None
        self._stop = {"v": False}
        self._rclpy = None
        if not dry_run:
            self._init_ros()

    def _init_ros(self) -> None:
        import rclpy
        from std_msgs.msg import Bool

        from ros_joint_writer import HAND_NAMES, JointWriter

        # 两处手关节顺序必须一致,否则展开的 hand 向量会错位到别的手指上
        if list(HAND_NAMES) != list(BE_HAND_NAMES):
            raise SkillError(
                "手关节顺序不一致!\n"
                f"  ros_joint_writer.HAND_NAMES = {list(HAND_NAMES)}\n"
                f"  backend.HAND_NAMES         = {list(BE_HAND_NAMES)}")
        self._rclpy = rclpy
        rclpy.init()
        self.node = JointWriter()
        self.node.create_subscription(
            Bool, "/nero/estop",
            lambda m: self._stop.__setitem__("v", self._stop["v"] or bool(m.data)), 10)
        rclpy.spin_once(self.node, timeout_sec=0.5)      # 让发布者/订阅先建立

    # ---- 环境探测 ----
    def live_session(self) -> bool | None:
        """/joint_states 有没有发布者。干跑模式返回 None(表示没查)。"""
        if self.dry_run or self.node is None:
            return None
        try:
            return self.node.count_publishers("/joint_states") > 0
        except Exception:                                # noqa: BLE001
            return None

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

    # ---- 主入口 ----
    def invoke(self, env: dict) -> dict:
        """执行一个调用信封。全程 _emit 事件,返回最终结果 dict。"""
        t0 = time.time()
        rid = env.get("request_id") or f"req-{int(t0 * 1000)}"
        sid = env.get("skill_id")
        src = env.get("source", "unknown")
        rec = {"request_id": rid, "skill_id": sid, "source": src,
               "params_in": env.get("params") or {},
               "transcript": env.get("transcript"),
               "confidence": env.get("confidence"),
               "ts": t0}

        spec = self.reg.get(sid) if sid else None
        if spec is None:
            res = {"type": "error", "request_id": rid,
                   "msg": f"未知技能 {sid!r};可选: {self.reg.ids()}"}
            _emit(res)
            _log_invocation({**rec, "result": "unknown_skill"})
            return res

        # 语音路径只许命中白名单技能 —— 清单里 voice_enabled=false 的一律拒绝
        if src == "voice" and not spec.safety.voice_enabled:
            res = {"type": "error", "request_id": rid,
                   "msg": f"技能 {sid} 不允许语音触发(voice_enabled=false)"}
            _emit(res)
            _log_invocation({**rec, "result": "voice_denied"})
            return res

        params, notes = spec.resolve_params(env.get("params"),
                                            via_voice=(src == "voice"))
        rec["params"] = params
        rec["notes"] = notes

        reason = Gate.check(spec, {
            "confirmed": bool(env.get("confirmed")),
            "assume_enabled": bool(env.get("assume_enabled")),
            "live": self.live_session(),
            "source": src,
        })
        if reason:
            res = {"type": "error", "request_id": rid, "skill_id": sid,
                   "need_confirm": spec.safety.need_confirm, "msg": reason}
            _emit(res)
            _log_invocation({**rec, "result": "gate_rejected", "reason": reason})
            return res

        try:
            be = make_backend(spec, self.reg)
            total = be.total(params)
            secs = be.duration_hint(params)
        except SkillError as e:
            res = {"type": "error", "request_id": rid, "msg": str(e)}
            _emit(res)
            _log_invocation({**rec, "result": "expand_failed", "reason": str(e)})
            return res

        _emit({"type": "start", "request_id": rid, "skill_id": sid,
               "name": spec.name, "kind": spec.kind, "total": total,
               "est_seconds": round(secs, 2), "dry_run": self.dry_run,
               "notes": notes})
        return self._run_steps(be, params, spec, rec, rid, total, t0)

    def _run_steps(self, be, params, spec, rec, rid, total, t0) -> dict:
        """逐步下发。每步先看急停,发完按 hold 等待。"""
        self._stop["v"] = False
        sent = 0
        try:
            for i, step in enumerate(be.steps(params)):
                if not self.dry_run:
                    self._rclpy.spin_once(self.node, timeout_sec=0.0)
                if self._stop["v"]:
                    res = {"type": "stopped", "request_id": rid, "step": i,
                           "total": total, "reason": "estop"}
                    _emit(res)
                    _log_invocation({**rec, "result": "estop", "sent": sent,
                                     "elapsed": round(time.time() - t0, 3)})
                    return res
                if self.dry_run:
                    _emit({"type": "cmd", "request_id": rid, "step": i,
                           "hold": round(step.hold, 4), "cmd": step.cmd})
                else:
                    self.node.send(step.cmd)
                sent += 1
                # 进度节流:轨迹几百帧不必每帧都报
                if i % 5 == 0 or i == total - 1:
                    _emit({"type": "progress", "request_id": rid, "step": i + 1,
                           "total": total, "label": step.label,
                           "pct": round(100.0 * (i + 1) / max(1, total), 1)})
                if step.hold > 0:
                    self._sleep_watching_estop(step.hold)
                    if self._stop["v"]:
                        res = {"type": "stopped", "request_id": rid, "step": i,
                               "total": total, "reason": "estop"}
                        _emit(res)
                        _log_invocation({**rec, "result": "estop", "sent": sent,
                                         "elapsed": round(time.time() - t0, 3)})
                        return res
        except KeyboardInterrupt:
            res = {"type": "stopped", "request_id": rid, "reason": "interrupt"}
            _emit(res)
            _log_invocation({**rec, "result": "interrupt", "sent": sent})
            return res
        except SkillError as e:
            res = {"type": "error", "request_id": rid, "msg": str(e)}
            _emit(res)
            _log_invocation({**rec, "result": "run_failed", "reason": str(e)})
            return res

        el = round(time.time() - t0, 3)
        res = {"type": "done", "request_id": rid, "skill_id": spec.id,
               "sent": sent, "total": total, "elapsed": el}
        _emit(res)
        _log_invocation({**rec, "result": "done", "sent": sent, "elapsed": el})
        return res

    def _sleep_watching_estop(self, secs: float) -> None:
        """等待期间继续 spin —— 否则长 duration 的点位指令期间收不到急停。"""
        if self.dry_run:
            return                                   # 干跑不真等,否则测一次要几十秒
        deadline = time.time() + secs
        while time.time() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.0)
            if self._stop["v"]:
                return
            time.sleep(min(0.02, max(0.0, deadline - time.time())))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", metavar="JSON", help="执行一个信封后退出")
    ap.add_argument("--skill", help="快捷方式:直接给 skill_id(等价 --once)")
    ap.add_argument("--params", default="{}", help="配合 --skill 的参数 JSON")
    ap.add_argument("--confirmed", action="store_true", help="配合 --skill,带上确认")
    ap.add_argument("--assume-enabled", action="store_true",
                    help="配合 --skill,表态『臂已使能』(ROS 侧无法验证)")
    ap.add_argument("--source", default="cli", help="调用来源标记")
    ap.add_argument("--dry-run", action="store_true",
                    help="只展开打印,不发 ROS(无需 bridge / rclpy)")
    args = ap.parse_args()

    try:
        runner = SkillRunner(dry_run=args.dry_run)
    except (RegistryError, SkillError) as e:
        _emit({"type": "error", "msg": str(e)})
        return 1

    try:
        if args.skill:
            env = {"skill_id": args.skill, "params": json.loads(args.params),
                   "source": args.source, "confirmed": args.confirmed,
                   "assume_enabled": args.assume_enabled}
            res = runner.invoke(env)
            return 0 if res.get("type") == "done" else 1
        if args.once:
            res = runner.invoke(json.loads(args.once))
            return 0 if res.get("type") == "done" else 1
        # 流式:逐行读 stdin 信封
        _emit({"type": "ready", "skills": len(runner.reg),
               "dry_run": args.dry_run})
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                env = json.loads(line)
            except json.JSONDecodeError:
                _emit({"type": "error", "msg": "信封不是合法 JSON"})
                continue
            runner.invoke(env)
        return 0
    finally:
        runner.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
