"""测 console 执行路:技能 → console 方言的翻译、安全闸、落后检测。

    python3 src/skills/test_console_exec.py

用假 console(不起 web、不碰 CAN/串口)+ 假时钟(不真等几十秒)。
重点是三条最容易写错的性质:
  1. 臂和手要**分开发**,且 estop 绝不能发给手 —— 手没有急停通道,
     最接近的 action_stop 会把手**移动**到张开位,那是运动不是停止。
  2. 使能/急停预检不能把「治病的药」拦下来 —— prepare_arm / arm_reset 正是
     解除未使能与急停的手段,按 requires 判才不会自锁。
  3. 落后要报出来。共享时间轴只保证命令一起发出,不保证硬件一起到位。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skills"))
import console_exec as CE  # noqa: E402
from backend import SkillError  # noqa: E402
from schema import get_registry  # noqa: E402

PASS, FAIL = [], []
REG = get_registry()


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'✓' if cond else '✗'} {name}" + (f" · {detail}" if detail else ""))


class Fake:
    """假 console 对:记录收到的指令,可注入遥测与写失败。"""

    def __init__(self, arm_rad=None, enabled=True, frozen=False,
                 arm_up=True, hand_up=True, fail=None) -> None:
        self.sent: list[tuple[str, dict]] = []
        self.arm_rad, self.enabled, self.frozen = arm_rad, enabled, frozen
        self.arm_up, self.hand_up, self.fail = arm_up, hand_up, fail

    def send(self, dev: str):
        def _s(cmd: dict) -> dict:
            self.sent.append((dev, cmd))
            return ({"ok": False, "msg": "假的写入失败"} if self.fail == dev
                    else {"ok": True})
        return _s

    def arm_state(self):
        if not self.arm_up:
            return None
        return {"rad": self.arm_rad if self.arm_rad is not None else [0.0] * 7,
                "enabled": self.enabled, "frozen": self.frozen}

    def hand_state(self):
        return {"rad": [0.0] * 6} if self.hand_up else None

    def exec(self, reg=None) -> CE.ConsoleExecutor:
        clock, sleep = CE._fake_clock()
        return CE.ConsoleExecutor(self.send("arm"), self.send("hand"),
                                  arm_state=self.arm_state,
                                  hand_state=self.hand_state,
                                  reg=reg or REG, sleep=sleep, clock=clock)

    def devs(self) -> list[str]:
        return [d for d, _ in self.sent]

    def cmds(self, dev: str | None = None) -> list[dict]:
        return [c for d, c in self.sent if dev is None or d == dev]


def run(ex, **env) -> list[dict]:
    env.setdefault("source", "cli")
    return list(ex.invoke(env))


def types(evs) -> list[str]:
    return [e["type"] for e in evs]


# ---------------------------------------------------------------- 翻译
print("\n[1] 方言翻译:writer 指令 → console 指令")
check("arm+hand 同条 → 拆成两条分别发",
      CE.translate({"arm": [0.0] * 7, "hand": [0.0] * 6, "duration": 1.0})
      == [("arm", {"cmd": "angles", "rad": [0.0] * 7}),
          ("hand", {"cmd": "angles", "rad": [0.0] * 6})])
check("只有 arm → 只发臂",
      [d for d, _ in CE.translate({"arm": [0.0] * 7})] == ["arm"])
check("只有 hand → 只发手",
      [d for d, _ in CE.translate({"hand": [0.0] * 6})] == ["hand"])
check("enable/disable/reset 原名转给臂",
      all(CE.translate({"action": a}) == [("arm", {"cmd": a})]
          for a in ("enable", "disable", "reset")))
check("set_speed → 臂的 speed(百分比),不发给手",
      CE.translate({"action": "set_speed", "value": 20})
      == [("arm", {"cmd": "speed", "value": 20})])
check("estop → 只发臂(手没有急停通道)",
      CE.translate({"estop": True}) == [("arm", {"cmd": "estop"})])
check("duration 不进 console 指令(它只是本地节拍)",
      "duration" not in CE.translate({"arm": [0.0] * 7, "duration": 5.0})[0][1])


def rejects_translate(name: str, cmd: dict) -> None:
    try:
        CE.translate(cmd)
        check(name, False, "居然放过了")
    except SkillError:
        check(name, True)


rejects_translate("未知 action 被拒", {"action": "fly"})
rejects_translate("arm 维度不对被拒", {"arm": [0.0] * 6})
rejects_translate("hand 维度不对被拒", {"hand": [0.0] * 7})
rejects_translate("空指令被拒", {})
rejects_translate("estop=false 被拒", {"estop": False})

# ---------------------------------------------------------------- 设备推导
print("\n[2] 设备推导:不展开轨迹也知道用哪条通道")
check("go_home → 只臂", CE.targets(REG.get("go_home"), REG) == {"arm"})
check("hand_open → 只手", CE.targets(REG.get("hand_open"), REG) == {"hand"})
check("轨迹 → 臂+手", CE.targets(REG.get("replay_rgbd_demo"), REG) == {"arm", "hand"})
check("composite → 子技能并集",
      CE.targets(REG.get("prepare_arm"), REG) == {"arm", "hand"})
check("estop → 只臂", CE.targets(REG.get("estop"), REG) == {"arm"})

# ---------------------------------------------------------------- 安全闸
print("\n[3] 安全闸")
evs = run(Fake().exec(), skill_id="go_home")
check("缺 confirmed 被拒(规则来自 runner.Gate)",
      types(evs) == ["error"] and "需要确认" in evs[0]["msg"], str(evs))
f = Fake()
check("被拒时一条指令都没发", not f.sent
      and types(run(f.exec(), skill_id="go_home")) == ["error"])
evs = run(Fake(arm_up=False).exec(), skill_id="go_home", confirmed=True)
check("臂 console 没跑 → 拒,且点名是臂",
      types(evs) == ["error"] and "机械臂 console 没在跑" in evs[0]["msg"], str(evs))
evs = run(Fake(hand_up=False).exec(), skill_id="hand_open", confirmed=True)
check("手 console 没跑 → 拒,且点名是手",
      types(evs) == ["error"] and "灵巧手 console 没在跑" in evs[0]["msg"], str(evs))
evs = run(Fake(enabled=False).exec(), skill_id="go_home", confirmed=True)
check("臂未使能 → 运动被拒", types(evs) == ["error"]
      and "未使能" in evs[0]["msg"], str(evs))
evs = run(Fake(frozen=True).exec(), skill_id="go_home", confirmed=True)
check("急停生效中 → 运动被拒", types(evs) == ["error"]
      and "急停生效中" in evs[0]["msg"], str(evs))

# ---------------------------------------------------------------- 不自锁
print("\n[4] 预检不能拦下『治病的药』(按 requires 判,不硬编码 id)")
f = Fake(enabled=False, frozen=True)
evs = run(f.exec(), skill_id="arm_reset", confirmed=True)
check("未使能+急停中仍可执行 arm_reset", types(evs)[-1] == "done", str(types(evs)))
check("arm_reset 真的发了 reset", f.cmds("arm") == [{"cmd": "reset"}], str(f.sent))
f = Fake(enabled=False, frozen=True)
evs = run(f.exec(), skill_id="prepare_arm", confirmed=True)
check("未使能+急停中仍可执行 prepare_arm(它自己 reset+enable)",
      types(evs)[-1] == "done", str(types(evs)))
check("prepare_arm 顺序 reset→enable→speed→手张开→回零",
      f.sent == [("arm", {"cmd": "reset"}), ("arm", {"cmd": "enable"}),
                 ("arm", {"cmd": "speed", "value": 20}),
                 ("hand", {"cmd": "angles", "rad": [0.0] * 6}),
                 ("arm", {"cmd": "angles", "rad": [0.0] * 7})], str(f.sent))
check("未使能也能使能(arm_enable 不声明 arm_enabled)",
      types(run(Fake(enabled=False).exec(), skill_id="arm_enable",
                confirmed=True))[-1] == "done")
check("未使能也能急停",
      types(run(Fake(enabled=False).exec(), skill_id="estop"))[-1] == "done")

# ---------------------------------------------------------------- 急停
print("\n[5] 急停:只管臂,且必须明说手没停")
f = Fake()
evs = run(f.exec(), skill_id="estop")
check("estop 免确认(全表唯一)", types(evs)[-1] == "done", str(types(evs)))
check("只给臂发了 estop", f.sent == [("arm", {"cmd": "estop"})], str(f.sent))
check("手一条都没收到", not f.cmds("hand"))
warns = [e for e in evs if e["type"] == "warn"]
check("发 warn 说明手不受急停影响",
      len(warns) == 1 and "手没有急停通道" in warns[0]["msg"], str(warns))
check("warn 也提醒臂无抱闸会下落", any("下落" in w["msg"] for w in warns))

# ---------------------------------------------------------------- 语音路径
print("\n[6] 语音路径:白名单 + 限速,与 runner 同一条规则")
evs = run(Fake().exec(), skill_id="estop", source="voice")
check("语音可触发白名单技能", types(evs)[-1] == "done")
f = Fake()
evs = run(f.exec(), skill_id="replay_rgbd_demo", source="voice",
          confirmed=True, params={"speed": 4.0}, transcript="回放深度示教快一点")
start = evs[0]
check("语音限速提示出现在 start 事件里",
      any("限速" in n for n in start.get("notes", [])), str(start.get("notes")))
check("轨迹每帧发两条(臂+手)", len(f.sent) == 2 * (start["total"]), str(len(f.sent)))

# ---------------------------------------------------------------- 落后检测
print("\n[7] 落后检测:硬件没跟上要报出来,不默默吸收")
f = Fake(arm_rad=[0.0] * 7)                 # 遥测正好等于目标 → 不该报
evs = run(f.exec(), skill_id="go_home", confirmed=True)
done = evs[-1]
check("到位时不报落后", not [e for e in evs if e["type"] == "lag"]
      and done["lag_exceeded"] is False, str(done))
check("到位时 worst_lag_rad=0", done["worst_lag_rad"] == 0.0, str(done))
f = Fake(arm_rad=[0.5] * 7)                 # 遥测差 0.5 rad → 必须报
evs = run(f.exec(), skill_id="go_home", confirmed=True)
lags = [e for e in evs if e["type"] == "lag"]
check("落后超阈值必须报", len(lags) == 1, str(types(evs)))
check("落后量算对(0.5 rad)", lags and abs(lags[0]["lag_rad"] - 0.5) < 1e-6,
      str(lags))
check("done 里带最差落后与超限标志",
      evs[-1]["worst_lag_rad"] == 0.5 and evs[-1]["lag_exceeded"] is True,
      str(evs[-1]))
check("落后了仍然把指令发完(报出来,不中断)", len(f.cmds("arm")) == 1)
f = Fake(arm_rad=[0.5] * 7)
evs = run(f.exec(), skill_id="replay_rgbd_demo", confirmed=True)
check("长轨迹的落后报告有上限(不刷屏)",
      len([e for e in evs if e["type"] == "lag"]) == 5, str(len(evs)))
check("手的指令不参与落后判定(手近乎瞬时)",
      types(run(Fake(arm_rad=[0.9] * 7).exec(), skill_id="hand_open",
                confirmed=True)).count("lag") == 0)

# ---------------------------------------------------------------- 叫停
print("\n[8] 叫停:停止继续下发")
f = Fake()
ex = f.exec()
out, stopped_after = [], 0
for ev in ex.invoke({"skill_id": "replay_rgbd_demo", "confirmed": True,
                     "source": "cli"}):
    out.append(ev)
    if ev["type"] == "progress" and ev["step"] >= 6 and not stopped_after:
        ex.stop()
        stopped_after = len(f.sent)
check("stop() 后收到 stopped 事件", out[-1]["type"] == "stopped", str(out[-1]))
check("stop() 后不再继续发", len(f.sent) <= stopped_after + 2,
      f"停时 {stopped_after} 条,最终 {len(f.sent)} 条")
check("stopped 事件带停在第几步", "step" in out[-1] and out[-1]["step"] > 0)

# ---------------------------------------------------------------- 写失败
print("\n[9] console 写入失败要当场停,不假装成功")
f = Fake(fail="arm")
evs = run(f.exec(), skill_id="prepare_arm", confirmed=True)
check("写失败 → error", types(evs)[-1] == "error", str(types(evs)))
check("失败后不继续发后续步骤", len(f.sent) == 1, str(f.sent))
check("错误信息点名是哪条通道", "arm console 写入失败" in evs[-1]["msg"],
      evs[-1]["msg"])
f = Fake(fail="hand")
evs = run(f.exec(), skill_id="hand_open", confirmed=True)
check("手侧写失败同样报", types(evs)[-1] == "error"
      and "hand console" in evs[-1]["msg"], str(evs[-1]))

# ---------------------------------------------------------------- 调用日志
print("\n[10] 调用日志:原话与 skill_id 成对落盘(VLA 标注原料)")
LOG = CE._log_invocation.__globals__["LOG_PATH"]
before = LOG.stat().st_size if LOG.exists() else 0
run(Fake().exec(), skill_id="estop", source="voice", transcript="快停下",
    confidence=0.88)
check("日志文件被写入", LOG.exists() and LOG.stat().st_size > before,
      str(LOG))
tail = LOG.read_text(encoding="utf-8").strip().splitlines()[-1]
check("记录里有原话", "快停下" in tail, tail[:120])
check("记录里有 skill_id 配对", '"skill_id": "estop"' in tail, tail[:120])
check("记录标了走的是 console 路", '"path": "console"' in tail, tail[:120])
check("被拒的调用也留痕",
      "gate_rejected" in (run(Fake().exec(), skill_id="go_home") and
                          LOG.read_text(encoding="utf-8").strip()
                          .splitlines()[-1]))

# ---------------------------------------------------------------- 未知技能
print("\n[11] 杂项")
evs = run(Fake().exec(), skill_id="no_such_skill")
check("未知技能被拒且列出可选", types(evs) == ["error"]
      and "未知技能" in evs[0]["msg"], str(evs))
check("未知技能不发指令", not Fake().sent)
evs = run(Fake().exec(), skill_id="hand_open", confirmed=True)
check("start 事件报出用到哪些设备", evs[0]["devices"] == ["hand"], str(evs[0]))
check("start 事件标明走 console 路", evs[0]["path"] == "console")

# ---------------------------------------------------------------- 汇总
print("\n" + "=" * 60)
print(f"通过 {len(PASS)} · 失败 {len(FAIL)}")
if FAIL:
    for n in FAIL:
        print(f"  ✗ {n}")
    raise SystemExit(1)
print("全部通过")
