#!/usr/bin/env python3
"""nero_arm 的固件自动探测 + connect() 里 probe/开推送的顺序。**不碰真硬件。**

为什么要假 SDK 而不是静态读源码:这两条都是**时序**行为 ——
「先探还是先开推送」和「探不到时退回什么」用 grep 断言不了。假一个 pyAgxArm
注入 sys.modules,让 connect() 真的跑一遍,记录调用顺序。
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))

_FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name}  {detail}")
        _FAILS.append(name)


class FakeMsg:
    def __init__(self, msg):
        self.msg = msg


class FakeArm:
    """假臂。

    `push_needed=True` 模拟固件 ≤1.11:不调 set_normal_mode 就永远读不到关节角。
    `push_needed=None`(默认)= **按版本自动判**:≥1.12 上电自动推(SDK 原文
    「Default is follower; CAN feedback push is on at power-up」),≤1.11 需要手动开。

    ⚠ 这个默认值不能一律给 True。实际踩过:我把 `_enable_can_push` 改成
    ≥v112 跳过那个空操作后,假臂因为还等着 set_normal_mode 才推,connect() 直接超时
    —— 测试挂了但**代码是对的**,是假臂没跟上真硬件的行为。
    """

    def __init__(self, sv: str = "1.11", push_needed: bool | None = None):
        if push_needed is None:
            push_needed = not (sv and sv >= "1.12")
        self.sv, self.push_needed = sv, push_needed
        self.calls: list[str] = []
        self.pushing = not push_needed
        self.enabled = False

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def get_joint_angles(self):
        self.calls.append("get_joint_angles")
        return FakeMsg([0.1] * 7) if self.pushing else None

    def get_firmware(self):
        self.calls.append("get_firmware")
        return {"software_version": self.sv} if self.sv else None

    def set_normal_mode(self):
        self.calls.append("set_normal_mode")
        self.pushing = True                 # 开推送后才读得到

    def get_joint_enable_status(self, _=255):
        self.calls.append("get_joint_enable_status")
        return self.enabled

    def get_driver_states(self, joint_index=1):
        # read_enabled(wait=) 先等这个出值(它 None/非 None 能区分"有无数据"),
        # 缺了会让 read_enabled 抛异常并写 last_error —— 那是假臂的缺口,不是代码 bug。
        self.calls.append("get_driver_states")
        return FakeMsg(object()) if self.pushing else None


def install_fake_sdk(arms: list[FakeArm]):
    """注入假 pyAgxArm。arms 按创建顺序弹出 —— auto 模式会创建两个(探测+正式)。"""
    made: list[FakeArm] = []

    class Factory:
        @staticmethod
        def create_arm(cfg):
            a = arms.pop(0)
            a.cfg = cfg
            made.append(a)
            return a

    class FW:
        DEFAULT, V111, V112, V120 = "DEFAULT", "V111", "V112", "V120"

    m = types.ModuleType("pyAgxArm")
    m.create_agx_arm_config = lambda **kw: kw          # cfg 就是 kwargs 本身,好断言
    m.AgxArmFactory = Factory
    m.ArmModel = types.SimpleNamespace(NERO="NERO")
    m.NeroFW = FW
    sys.modules["pyAgxArm"] = m
    return made


def fresh_arm(**kw):
    import nero_arm
    nero_arm._prepare_pyagx_imports = lambda: None    # 别去找真 SDK 路径
    return nero_arm.NeroArm(mock=False, **kw)


def try_connect(a, label: str) -> bool:
    """connect() 失败要记成 FAIL,不能让整个测试崩掉。

    退化时 connect() 抛的是 RuntimeError(「通了但读不到关节角」)。裸崩的话
    后面几组断言压根不跑,看不出退化范围 —— 反向验证时实测过一次。
    """
    try:
        a.connect()
        return True
    except Exception as e:                             # noqa: BLE001
        check(f"{label}: connect() 未抛异常", False, f"{type(e).__name__}: {e}")
        return False


def main() -> int:
    import nero_arm
    N = nero_arm.NeroArm

    print("\n=== 固件门限 ===")
    # 1.21 是这台臂 2026-08-03 升级后的实际版本,实测 detect 脚本报 v120
    for sv, want in [("1.11", "v111"), ("1.12", "v112"), ("1.20", "v120"),
                     ("1.21", "v120"), ("1.10", "default"), ("1.0", "default")]:
        check(f"{sv} → {want}", N.pick_driver(sv) == want, N.pick_driver(sv))
    # 这条是**故意保留的**字符串比较缺陷,和 SDK 一致优先于正确
    check("1.9 → v120(字典序坑,与 SDK 一致)", N.pick_driver("1.9") == "v120")

    print("\n=== auto 探到 1.21 就用 v120(升级后的实际情况) ===")
    probe, real = FakeArm("1.21"), FakeArm("1.21")
    install_fake_sdk([probe, real])
    a = fresh_arm(firmware="auto")
    a.DETECT_SEC = 0.5
    a.CONNECT_PROBE_SEC = 1.0
    try_connect(a, "auto")
    check("探到 v120", a.firmware_detected == "v120", repr(a.firmware_detected))
    check("正式连接用 V120 driver", real.cfg.get("firmeware_version") == "V120",
          repr(real.cfg.get("firmeware_version")))
    check("探测用 DEFAULT driver", probe.cfg.get("firmeware_version") == "DEFAULT")
    check("探测臂被断开(不占总线)", "disconnect" in probe.calls)
    # ≥1.12 上电自动推 + set_normal_mode 是空操作 → 我们**不该**去调它
    check("v120 上不调 set_normal_mode(那是空操作)",
          "set_normal_mode" not in real.calls, str(real.calls))
    check("v120 上标记了 can_push_auto", a.can_push_auto is True)
    check("跳过不算失败(last_error 仍为 None)", a.last_error is None, repr(a.last_error))

    print("\n=== v111 上仍然要手动开推送 ===")
    p5, r5 = FakeArm("1.11"), FakeArm("1.11")
    install_fake_sdk([p5, r5])
    e5 = fresh_arm(firmware="auto")
    e5.DETECT_SEC = 0.5; e5.CONNECT_PROBE_SEC = 1.0; e5.PROBE_PREPUSH_SEC = 0.2
    try_connect(e5, "v111")
    check("探到 v111", e5.firmware_detected == "v111", repr(e5.firmware_detected))
    check("v111 上确实调了 set_normal_mode", "set_normal_mode" in r5.calls)
    check("v111 上 can_push_auto 为 False", e5.can_push_auto is False)
    check("v111 的 velocity 不可信", e5.velocity_is_real() is False)

    print("\n=== 冷启动(没人推送)也能连上 —— 顺序必须是探→开推送→再探 ===")
    real2 = FakeArm("1.11", push_needed=True)
    install_fake_sdk([real2])
    b = fresh_arm(firmware="v111")
    b.PROBE_PREPUSH_SEC = 0.2
    b.CONNECT_PROBE_SEC = 1.0
    ok = try_connect(b, "冷启动")
    c = real2.calls
    check("连上了(旧顺序这里会抛超时)", ok is True)
    # 不用裸 .index() —— 退化时 set_normal_mode 压根不在 calls 里,会抛 ValueError
    # 把测试整个搞崩(反向验证时踩到过)。缺项本身就是要报的 FAIL,不是异常。
    i_read = c.index("get_joint_angles") if "get_joint_angles" in c else -1
    i_push = c.index("set_normal_mode") if "set_normal_mode" in c else -1
    check("开推送在首次读之后", i_read >= 0 and i_push > i_read, f"calls={c}")
    check("开推送之后还有读",
          i_push >= 0 and "get_joint_angles" in c[i_push + 1:], f"calls={c}")
    check("connect_pose 拿到了", b.connect_pose == [0.1] * 7, repr(b.connect_pose))

    print("\n=== 已有推送时不重复开(白嫖) ===")
    real3 = FakeArm("1.11", push_needed=False)
    install_fake_sdk([real3])
    d = fresh_arm(firmware="v111")
    d.PROBE_PREPUSH_SEC = 0.2
    try_connect(d, "白嫖")
    check("仍然补发一次 set_normal_mode(状态归我们)",
          real3.calls.count("set_normal_mode") == 1, str(real3.calls.count("set_normal_mode")))

    print("\n=== 探不到固件 → 退回 default,不猜 ===")
    probe4, real4 = FakeArm(""), FakeArm("")
    install_fake_sdk([probe4, real4])
    e = fresh_arm(firmware="auto")
    e.DETECT_SEC = 0.3
    e.CONNECT_PROBE_SEC = 0.6
    try_connect(e, "退回 default")
    check("退回 default", e.firmware_detected == "default", repr(e.firmware_detected))
    check("记了 last_error", bool(e.last_error), repr(e.last_error))

    print("\n=== 速度只写:没设过就报 None,不报编的 100 ===")
    g = nero_arm.NeroArm(mock=True)
    check("未设过 → None", g.speed_percent is None, repr(g.speed_percent))
    g.set_speed_percent(20)
    check("设过 → 20", g.speed_percent == 20, repr(g.speed_percent))

    print("\n=== 外部使能时挂起的速度也要补上 ===")
    # 这是真机上实际踩到的:臂被松灵客户端使能,不经过我们的 handle(),
    # 原来 pending_speed 就永远补不上,--speed 20 形同虚设。
    h = nero_arm.NeroArm(mock=True)
    h.connect()
    pend = [20]
    h._enabled = False
    # 模拟遥测循环那段:未使能时不补
    if pend[0] is not None and h.enabled:
        h.set_speed_percent(pend[0]); pend[0] = None
    check("未使能时不补", pend[0] == 20 and h.speed_percent is None)
    h._enabled = True                       # ← 外部使能
    if pend[0] is not None and h.enabled:
        h.set_speed_percent(pend[0]); pend[0] = None
    check("外部使能后补上了", pend[0] is None and h.speed_percent == 20,
          f"pending={pend[0]} speed={h.speed_percent}")
    src = (SIM / "arm_console.py").read_text(encoding="utf-8")
    loop = src.split("row = {\"type\": \"state\"")[0]
    check("遥测循环里确实有这段补发(不只是测试里模拟)",
          "pending_speed[0] is not None and arm.enabled" in loop)

    print("\n=== mock 报 mock,不报 auto ===")
    f = nero_arm.NeroArm(mock=True, firmware="auto")
    f.connect()
    check("firmware_detected == 'mock'", f.firmware_detected == "mock", repr(f.firmware_detected))

    n = len(_FAILS)
    print(f"\n{'全部通过' if n == 0 else str(n) + ' 项失败: ' + ', '.join(_FAILS)}")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
