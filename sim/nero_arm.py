#!/usr/bin/env python3
"""sim/nero_arm.py — NERO 7 轴臂封装(pyAgxArm / socketcan)。

从 nero_arm_bridge.py 抽出来的,原因:bridge 模块级 `import rclpy`,而
arm_console.py 要能在**没有 ROS** 的环境里跑(和 hand_console.py 同理 —— 臂调试
不该被 ROS 环境问题挡住)。这里的角色和 inspire_hand.py 完全对应:

  inspire_hand.InspireHand  ← RS485 ─ 手     hand_console.py / bridge 都 import 它
  nero_arm.NeroArm          ← CAN   ─ 臂     arm_console.py  / bridge 都 import 它

接口刻意和 InspireHand 对齐(read_angles / connect / disconnect),这样
app_web 的会话管理和前端协议能两边共用一套。

mock=True 时不碰 CAN:在目标位附近做小幅正弦摆动,让"活着"和"收到指令"在
3D 和数值里都看得见。真机需要 can0 + pyAgxArm。

⚠ 安全:臂是 7 自由度工业臂,伤害量级和手不同。默认 mock=True;真机运动要上层
显式开启。move_j 前一律按 NERO_ARM_LIMITS 夹取。
"""
from __future__ import annotations

import math
import sys
import time
import types
from pathlib import Path
from typing import Final, Literal

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

# lerobot_env 是 ROS 环境特定的工具,非 ROS 使用场景(如 MCP)不需要
try:
    from lerobot_env import lerobot_site                          # noqa: E402
    LEROBOT_SITE = lerobot_site()
except ImportError:
    LEROBOT_SITE = None

PYAGX_ROOT = SIM.parent / "pyAgxArm-master" / "pyAgxArm-master"   # 本地 SDK 源

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]

# 关节限位(rad),来自本地 SDK 文档。注意 joint4/joint6 **不对称**,
# 前端滑块不能按 ±same 生成。和 ros_joint_writer.NERO_ARM_LIMITS 是同一套数。
NERO_ARM_LIMITS = [
    (math.radians(-155.0), math.radians(155.0)),   # joint1
    (math.radians(-100.0), math.radians(100.0)),   # joint2
    (math.radians(-158.0), math.radians(158.0)),   # joint3
    (math.radians(-58.0), math.radians(123.0)),    # joint4  不对称
    (math.radians(-158.0), math.radians(158.0)),   # joint5
    (math.radians(-42.0), math.radians(55.0)),     # joint6  不对称
    (math.radians(-90.0), math.radians(90.0)),     # joint7
]

# 关节**额定**最大速度(deg/s),**手册**给的(2026-08-03)。
# ⚠ 注意 joint4:手册写 225,但**臂里实际设的是 180**(见 NERO_JOINT_MAX_SPD_DEG)。
NERO_RATED_SPD_DEG = [180.0, 180.0, 180.0, 225.0, 225.0, 225.0, 225.0]

# 关节级限制,2026-08-06 从真臂读出来的(read_joint_limits.py,CAN 0x472→0x473/0x47C)。
# 这一层在 CPV 轮廓参数**之上**:
#     电机物理能力(读不到) > 关节级限制(这两行) > CPV 轮廓(vv/ac,下面两行)
NERO_JOINT_MAX_SPD_DEG = [179.9, 179.9, 179.9, 179.9, 224.6, 224.6, 224.6]
NERO_JOINT_MAX_ACC_DEG = [114.6, 114.6, 143.2, 143.2, 143.2, 143.2, 143.2]

# CPV 伺服的**出厂默认**轮廓上限,2026-08-03 从真臂逐关节读出来的(read_cpv_params.py)。
# ⚠ 这是**当前设置值**,不是机械上限 —— 实测恰好是**关节级 max_spd** 的 20.0%
# (36.0/179.9 = 0.2001,44.9/224.6 = 0.1999),七个关节整整齐齐。
# 那个 20% 的吻合顺带把单位定死了:vv 是 rad/s(0.628 = 0.2π,而 180°/s = 1.00π rad/s)。
#
# ⚠ 勘误(2026-08-06):这里原来写"是**额定**速度的 20%",并说"joint4 是例外,只占 16%"。
# **都不对。** 读了关节级限制才发现:joint4 的关节级 max_spd 是 **179.9 而不是手册的
# 225** —— 按臂里实际的设置算,joint4 也是 20.0%,**没有例外**。
# 那个"16%"是拿手册的 225 去除的结果,分母就是错的。
# 要问松灵的问题因此变了:**为什么 joint4 的关节级速度上限设成 180 而不是额定 225?**
NERO_CPV_VV_DEG = [36.0, 36.0, 36.0, 36.0, 44.9, 44.9, 44.9]      # 轮廓速度上限
NERO_CPV_AC_DEG = [114.6, 114.6, 143.2, 143.2, 143.2, 143.2, 143.2]  # 加/减速上限

# ⚠ 加速度**没有余量**:关节级 max_acc 和 CPV 的 ac **逐关节完全相等**(余量 1.00×,
# 2026-08-06 实测)。所以 143.2°/s² 就是我们能拿到的天花板,`set_cpv_acc` 往上调
# 没有意义 —— 上面没有东西。素材超上限只能靠 retime 拉长时间轴(拉 k 倍降 k² 加速度)。
# 之前那句"真实上限可能高好几倍"是**猜的,而且猜错了**,已删。
#
# ⚠⚠ **不要调用 SDK 的 `set_joint_acc_limits()`** —— 它的 scale 差 10 倍:
#     读:decode_47C  raw * 1e-3          → 2000 读成 2.0 rad/s²
#     写:round(v * 1e4)                  → 写 2.0 发出 20000 = 读回 20.0 rad/s²
# 两边都是 16 位字段(ConvertToList_16bit / ConvertToNegative_16bit),20000 装得进去,
# **不报错**。而它的 check() 用同一个错 scale 校验,所以读回校验也会"通过"。
# 想写 2.5 会变成 25.0 —— 十倍的加速度上限。好在余量 1.00× 意味着我们不需要写它。


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _prepare_pyagx_imports() -> None:
    """让本地 pyAgxArm 在 ROS 系统 python 下也能 import。

    ROS Humble 这边要 /usr/bin/python3,而 SDK 的纯 python 依赖可能装在 lerobot
    的 3.10 环境里。把那个 site-packages 作为**兜底**加进 sys.path,并给
    typing_extensions 补一个最小 shim(pyAgxArm 只用到 Literal/Final)。
    """
    if str(PYAGX_ROOT) not in sys.path:
        sys.path.insert(0, str(PYAGX_ROOT))
    if LEROBOT_SITE is None:
        # 不是致命错误(下面有 shim 兜),但要说出来:真机缺包时报错会指向
        # pyAgxArm 内部,不提示很难联想到是这里没找到。
        print("[nero_arm] warn: 未找到 lerobot site-packages,pyAgxArm 的纯 python "
              "依赖可能缺失。可设 LEROBOT_SITE=<env>/lib/python3.10/site-packages。",
              file=sys.stderr, flush=True)
    elif str(LEROBOT_SITE) not in sys.path:
        sys.path.append(str(LEROBOT_SITE))
    try:
        import typing_extensions  # noqa: F401
    except ImportError:
        shim = types.ModuleType("typing_extensions")
        shim.Literal = Literal
        shim.Final = Final
        sys.modules["typing_extensions"] = shim


class NeroArm:
    """NERO 臂封装:mock=正弦摆动;真机=pyAgxArm CAN。接口对齐 InspireHand。"""

    # 接入时等第一帧关节角的上限。臂按 **222Hz** 推送(candump 实测,不是协议标称的
    # 25Hz —— 那个数错了一个量级),正常几毫秒就到;
    # 给到 3s 是覆盖"刚上电还在自检"的情况。
    CONNECT_PROBE_SEC: Final[float] = 3.0
    # 开推送**之前**先白嫖一下已有推送的时间窗。臂在推的话 222Hz,4.5ms 一帧,
    # 0.4s 足够收到几十帧。取短是因为这段等待在冷启动(没人推)时是纯浪费。
    PROBE_PREPUSH_SEC: Final[float] = 0.4

    def __init__(self, mock: bool = True, channel: str = "can0",
                 firmware: str = "default", interface: str = "socketcan") -> None:
        self.mock = mock
        self.channel = channel
        self.interface = interface  # socketcan(Linux) / agx_cando(Windows)
        # "auto" = 用 DEFAULT driver 连一次读 software_version,再按 SDK 自己那套
        # 字符串门限选 driver 重连。这台臂实测是 1.11 → v111,但**不要写死** ——
        # 固件升级后写死的值就是错的,而且错得很安静(某些命令行为变了但不报错)。
        self.firmware = firmware
        self.firmware_detected: str | None = None   # auto 探到的实际值,给遥测用
        # 固件 ≥1.12 上电自动推送,我们不需要开(见 _enable_can_push)
        self.can_push_auto = False
        self.robot = None
        self._t = 0.0
        # mock 起始"就绪位"(非零)—— 这样点『归零』能看到臂真的回到 0
        self._target = [0.3, -0.5, 0.2, -0.8, 0.1, 0.4, 0.0]
        self._frozen = False          # mock 急停:冻结摆动,停在当前位
        self._frozen_pose = None
        # 100 只是"还没设过"时的占位。speed_percent 属性在 _speed_confirmed 之前
        # 返回 None 而不是这个值 —— 理由见该属性的注释(速度只写,读不回来)。
        self._speed = 100
        self._speed_confirmed = False
        self.last_error: str | None = None
        self._enabled = False
        # CPV 伺服是否已经进入(cpv_begin 置上,cpv_end 清掉)。
        # 进入后 auto_set_motion_mode 被关掉,所以**必须**成对调用 ——
        # 不 end 就直接 move_j 会走在 cpv 模式下,行为未定义。
        self._cpv_active = False
        # 接入那一刻的位姿。断开前想"把臂原样交回松灵客户端"就靠它。
        # 不用零位 —— 松灵那边可能把臂停在任意姿态,回零的路径比回这儿远得多。
        self.connect_pose: list[float] | None = None

    def connect(self) -> bool:
        if self.mock:
            self.connect_pose = self.read_angles()
            # mock 下没有可探的固件。显式标成 "mock" 而不是留 None ——
            # 留 None 时 arm_console 会退回报 args.firmware,页面上就显示 "auto",
            # 那是**没有信息**的输出(auto 是意图,不是结果)。
            self.firmware_detected = "mock"
            return True
        _prepare_pyagx_imports()
        from pyAgxArm import (create_agx_arm_config, AgxArmFactory,   # noqa: E402
                              ArmModel, NeroFW)
        fw_map = {"default": NeroFW.DEFAULT, "v111": NeroFW.V111,
                  "v112": NeroFW.V112, "v120": NeroFW.V120}
        fw = self.firmware
        if fw == "auto":
            fw = self._detect_firmware(create_agx_arm_config, AgxArmFactory,
                                       ArmModel, NeroFW)
            self.firmware_detected = fw
        cfg = create_agx_arm_config(robot=ArmModel.NERO,
                                    firmeware_version=fw_map[fw],
                                    interface=self.interface, channel=self.channel,
                                    bitrate=1000000)
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()
        # ⚠ connect() **不等数据** —— 它只建 socket 并起后台读线程,立刻返回。
        # SDK 的 get_joint_angles() 是读 parser 缓存,缓存要等读线程解到第一帧才有值,
        # 在那之前恒回 None。所以这里必须**轮询等**,不能只探一次:
        # 探一次能成只是碰巧赶上了别人已经在推的总线,单机冷启动必然失败。
        # 官方 demo 同样是 while + deadline 的写法。
        # 只读轮询,不在循环里发 enable —— 官方 demo 那样做会给未预期的臂上使能。
        #
        # ⚠ 顺序:**先短探一次,读不到再开推送,然后接着探**。
        # 原来是"探满 8s → 再开推送",那在固件 ≤1.11 上必然超时:1.11 不会自己推,
        # 探的 8s 里总线上压根没有 0x2A5,于是抛"通了但读不到" —— 而开推送的那行
        # 永远走不到。之前能连上只是因为松灵客户端**已经**替我们开过了。
        # 反过来"一上来就发 0x151"也不行:臂不在总线上时发送会让 CAN 控制器
        # error-passive,而且丢掉了"臂在线但没推送"和"臂根本不在"这两种情况的区分。
        # 所以两段:先白嫖已有推送(常见),没有再自己开(冷启动)。
        def _probe(deadline: float):
            while time.monotonic() < deadline:
                p = self.robot.get_joint_angles()
                if p is not None and getattr(p, "msg", None) is not None:
                    return p
                time.sleep(0.05)
            return None

        t0 = time.monotonic()
        probe = _probe(t0 + self.PROBE_PREPUSH_SEC)
        pushed_by_us = False
        if probe is None:
            # 没人在推 —— 自己开。失败只记 last_error(见 _enable_can_push),
            # 因为"发不出去"本身也可能是臂不在线,下面的报错更有信息量。
            pushed_by_us = self._enable_can_push()
            probe = _probe(t0 + self.CONNECT_PROBE_SEC)
        if probe is None:
            self.disconnect()
            raise RuntimeError(
                f"{self.channel} 通了但 {self.CONNECT_PROBE_SEC:.0f}s 内读不到关节角。"
                f"用 `candump {self.channel}` 看有没有 0x2A5/0x2A6/0x2A7/0x2A9:"
                f"没有 → 臂没在推送(检查是否处于 CAN 指令控制模式、"
                f"0x151 byte6 是否开过推送、臂是否上电);"
                f"有 → 固件版本对不上,试 --firmware v120/v112/v111")
        self.connect_pose = list(probe.msg)
        self._target = list(probe.msg)      # 目标位对齐现状,别留着构造时的假值
        # ⚠ 开启 CAN 主动推送(0x151 byte6=0x01)。
        # 该字段默认 0x00 = "无效值" = 本次不改动,所以**不显式开就永远不会开**。
        # 短探阶段能读到值只说明**别人**(松灵客户端)开过 —— 那是它的状态,不是我们的。
        # 它随时可能退出并把推送关掉,所以哪怕白嫖成功也补发一次,让这个状态归我们。
        # set_normal_mode() 是 SDK 里唯一把 enable_can_push 置 ENABLE 的入口。
        # 幂等:重复发只是再置一次 ENABLE,不产生运动。
        if not pushed_by_us:
            self._enable_can_push()
        # ⚠ 读**真实**使能状态,不要用本地默认 False。
        # 臂常态是松灵客户端带电控制着,我们接入时它很可能已经使能。用本地 False
        # 会让自己的使能检查拒掉下发 —— 臂明明能动,页面却说"未使能"。
        # wait=: 使能位在 LowSpd 帧(0x261~0x267),实测 55.4Hz —— 比关节角
        # (222Hz)慢 4 倍,不等就可能读成 False。
        self.read_enabled(wait=self.CONNECT_PROBE_SEC)
        return True

    # SDK 内部的门限(pyAgxArm 工厂里那段),照抄不改进。
    # ⚠ 是**字符串**比较不是数值比较:"1.9" >= "1.20" 字典序为 True,会误判成 v120。
    # 明知有这个坑还照抄,是因为我们必须和 SDK **选同一个 driver** ——
    # 这里聪明反而会和实际生效的 driver 错开,那种不一致比这个坑更难查。
    _FW_GATES: Final[tuple] = (("1.20", "v120"), ("1.12", "v112"), ("1.11", "v111"))
    DETECT_SEC: Final[float] = 3.0

    @classmethod
    def pick_driver(cls, sv: str) -> str:
        for gate, name in cls._FW_GATES:
            if sv >= gate:
                return name
        return "default"

    def _detect_firmware(self, mk_cfg, factory, ArmModel, NeroFW) -> str:
        """用 DEFAULT driver 连一次读 software_version → 返回 driver 名。

        只读:全程不发使能、不发运动、不改模式。0x4AF(固件信息)在 default driver
        里就有,所以拿 DEFAULT 探是安全的 —— 探不到就退回 "default",不猜。
        代价是接入要连两次 CAN,多几百毫秒。值得:写死版本号在固件升级后会静默出错。
        """
        probe_arm = None
        try:
            cfg = mk_cfg(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT,
                         interface=self.interface, channel=self.channel, bitrate=1000000)
            probe_arm = factory.create_arm(cfg)
            probe_arm.connect()
            deadline = time.monotonic() + self.DETECT_SEC
            while time.monotonic() < deadline:
                fw = probe_arm.get_firmware()
                # get_firmware() 直接返回 dict-like(不是 MessageAbstract)——
                # 见 detect_arm_firmware.py 里 `fw.get("software_version")` 的用法。
                sv = fw.get("software_version") if fw is not None else None
                if sv:
                    return self.pick_driver(str(sv).strip())
                time.sleep(0.05)
            self.last_error = f"{self.DETECT_SEC:.0f}s 内读不到固件版本,按 default 走"
        except Exception as e:                      # noqa: BLE001
            self.last_error = f"探固件失败({e}),按 default 走"
        finally:
            if probe_arm is not None:
                try:
                    probe_arm.disconnect()
                except Exception:                   # noqa: BLE001
                    pass
        return "default"

    def _enable_can_push(self) -> bool:
        """开启 0x2A1~0x2A9 周期推送。失败只记 last_error,不抛。

        走 set_normal_mode():在 **default/v111** 上它把 enable_can_push 置 ENABLE
        后发一次 0x151,随即复位成 INVALID(一次性置位),副作用是下发单臂主从配置
        (linkage_config=0x00)。

        ⚠ **固件 ≥1.12 上这是空操作。** SDK v112 的 set_normal_mode 只 warn 一句就返回,
        原文:「No effect (compatibility no-op). Only leader and follower modes remain
        (aligned with Piper). Default is follower; **CAN feedback push is on at
        power-up** — no manual CAN-push setup.」
        也就是说 1.12+ 上电自己就推,压根不需要我们开 —— 这也解释了为什么没有任何
        console 在跑时 candump 3 秒能收到 10280 帧(之前我以为分不清"自动推"和
        "别人开过、状态保留",其实 SDK 文档就写着)。

        所以这里按 driver 分流:≥v112 直接跳过并说明,不去调那个只会 warn 的接口。
        connect() 里"先短探再开推送"的顺序**保留** —— 在 1.21 上是多余的 0.4s,
        降级到 1.11 或换臂时是必需的。
        """
        if self.mock or self.robot is None:
            return True
        drv = self.firmware_detected or self.firmware
        if drv in ("v112", "v120"):
            # 不是失败,是不需要。写进 last_error 会误报成错误,所以只留个标记。
            self.can_push_auto = True
            return True
        try:
            self.robot.set_normal_mode()
            return True
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"开启 CAN 推送失败: {e}"
            return False

    def read_enabled(self, wait: float = 0.0) -> bool:
        """从驱动器读真实使能状态并同步到 _enabled。读不到按 False 处理。

        走 get_joint_enable_status(255):逐关节看 foc_status.driver_enable_status,
        全部使能才返回 True。

        ⚠ 该接口 **"没数据" 和 "未使能" 都返回 False**,自身无法区分。而使能位在
        **LowSpd** 帧(`0x261~0x267`)里,candump 实测 **55.4Hz**,比关节角(222Hz)
        慢 4 倍 —— 刚接入就读可能拿到 False,于是 UI 显示"未使能"而臂其实是使能的。
        (原来这里写 ~2Hz / ~25Hz,那是协议标称值,实测都错了一到两个量级。
        名字容易记反:LowSpd = 0x261+ 带使能位,HighSpd = 0x251+ 带 velocity。)
        所以 wait>0 时先等 get_driver_states(1) 出值(它 None/非 None 能区分有无数据),
        再读使能位。真的未使能时会等满 wait 才返回,所以只在接入时用,轮询里别传。
        """
        if self.mock:
            return self._enabled
        try:
            if wait > 0:
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline:
                    if self.robot.get_driver_states(1) is not None:
                        break
                    time.sleep(0.05)
            self._enabled = bool(self.robot.get_joint_enable_status(255))
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"读使能状态异常: {e}"
            self._enabled = False
        return self._enabled

    def read_angles(self) -> list[float]:
        """7 关节角(rad)。读失败回退上次目标,**不抛** —— 上层在定时循环里调它,
        偶发丢帧不该把进程搞挂(和 InspireHand.read_angles 同约定)。"""
        if self.mock:
            if self._frozen:
                return list(self._frozen_pose)     # 急停:定住不动
            self._t += 1.0 / 30.0
            freqs = [0.30, 0.40, 0.35, 0.50, 0.45, 0.60, 0.55]
            return [self._target[i] + 0.12 * math.sin(freqs[i] * self._t + i)
                    for i in range(7)]
        try:
            ret = self.robot.get_joint_angles()
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"读关节角异常: {e}"
            return list(self._target)
        if ret is None or ret.msg is None:
            self.last_error = "读关节角无回复"
            return list(self._target)
        return list(ret.msg)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def frozen(self) -> bool:
        """急停是否生效中。真机的急停状态在 SDK 里,这里只反映我们发过急停。"""
        return self._frozen

    def enable(self) -> bool:
        self._enabled = True
        if self.mock:
            return True
        return bool(self.robot.enable())

    def disable(self) -> bool:
        self._enabled = False
        if self.mock:
            return True
        return bool(self.robot.disable())

    def reset(self) -> None:
        """退出急停阻尼模式,并**重新使能电机**。

        ⚠ 官方协议流程第 8 条:"急停恢复后需要重新发送 0x471 指令使能电机方可继续运动"。
        只发 0x150 byte0=2 退出阻尼是不够的 —— 电机仍处于失能,运动指令会被静默吞掉,
        而 UI 却因为本地 _enabled 还是 True 而显示"已使能",看起来像"发了没反应"。
        """
        self._frozen = False          # 复位解除急停冻结
        if not self.mock and self.robot is not None:
            self.robot.reset()        # 0x150 byte0=2:退出关节阻尼
        # 重使能对 mock 和真机**都要做** —— 急停里两边都置了 _enabled=False,
        # 这里只在真机分支补使能会让 mock 卡在"复位了但动不了"。
        self.enable()                 # 0x471:急停后必须重发,否则动不了
        if not self.mock:
            self.read_enabled()       # 用真实状态覆盖,别信 enable() 的返回值

    def set_speed_percent(self, pct: float) -> None:
        pct = int(_clamp(pct, 1, 100))
        if not self.mock and self.robot is not None:
            # 先发再记:发失败就别把 _speed 改成没生效的值(否则页面显示的是幻觉)
            self.robot.set_speed_percent(pct)
        self._speed = pct
        self._speed_confirmed = True

    def velocity_is_real(self) -> bool:
        """遥测里的 velocity 是真值还是被 SDK 抹成的 0。

        只有 v120 driver 重写了 `get_motor_states` 且去掉了那行抹零。
        v112 **不算** —— 它没有自己的 get_motor_states,继承 v111 的抹零版。
        判断依据是**实际生效的 driver**(auto 探到的),不是命令行里填的那个。
        """
        return (self.firmware_detected or self.firmware) == "v120"

    @property
    def speed_percent(self) -> int | None:
        """当前速度百分比。**我们没写过就返回 None,不返回猜的值。**

        ⚠ 速度在 nero 上是**只写**的:SDK 没有读回接口(只有 set_speed_percent),
        0x2A1 状态帧里也没有速度字段。所以接入时臂的实际速度是**未知**的 ——
        松灵客户端上次设了多少就是多少。

        原来这里无条件返回 `self._speed`,而它的初值是 `__init__` 里写的 100。
        实测踩到:真机用 `--speed 20` 起的,但因为接入时 read_enabled() 拿到 False,
        速度被挂起没发下去,遥测却报 `speed_percent: 100` —— 那个 100 是**编的**。
        前端 index.html:3573 又会把它同步进滑块,把 20 的安全默认冲掉,
        于是"页面显示 100 / 臂里是未知 / 我们要的是 20"三者全不一致。
        返回 None 让前端的 `!= null` 判断跳过同步,滑块保住安全默认。
        """
        return self._speed if self._speed_confirmed else None

    def move_j(self, rad7: list[float]) -> bool:
        """点位运动。返回是否真的发出去了(急停中会拒发)。"""
        if self._frozen:
            return False          # 急停中:忽略运动指令,等 reset() 解冻
        rad7 = [_clamp(v, *NERO_ARM_LIMITS[i]) for i, v in enumerate(rad7)]
        self._target = rad7
        if self.mock:
            return True
        try:
            self.robot.move_j(rad7)
            return True
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"move_j 异常: {e}"
            return False

    def read_ctrl_mode(self) -> str | None:
        """臂当前的控制模式名(CAN_CTRL / ETHERNET_CONTROL_MODE / …)。读不到回 None。

        为什么单独开这个而不看 telemetry 里的 `arm_status`:那边是 `str(msg)`,
        整条状态挤成一个字符串,程序拿不到单个字段。而 CPV 有**硬前提** ——
        臂必须在 CAN_CTRL。松灵客户端把臂留在 ETHERNET 模式时,CPV 帧发出去
        **不报错也不动**(SDK 的 `_cpv_po_joints_flag` 就是靠 ctrl_mode != CAN_CTRL
        来判断"链路断过、下一帧要重发两遍"的)。回放前必须能查这个。
        """
        if self.mock:
            return "CAN_CTRL"                      # mock 假装已经切好
        try:
            r = self.robot.get_arm_status()
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"读控制模式异常: {e}"
            return None
        m = getattr(r, "msg", None)
        if m is None:
            return None
        cm = getattr(m, "ctrl_mode", None)
        if cm is None:
            return None
        # ctrl_mode 解出来是 int,拿 enum 反查名字。
        # 用 driver 自己挂的 `ARM_STATUS`(SDK 里 `ARM_STATUS = ArmMsgFeedbackStatusEnum`,
        # v112 的 `_cpv_po_joints_flag` 就是这么取的)——**不在这里另抄一份枚举表**。
        # 抄一份的话固件加了模式我们不知道,而且错得很安静。
        # 查不到就回十六进制,别回 None:"读到了但是个没见过的值"和"读不到"是两件事。
        try:
            return self.robot.ARM_STATUS.CtrlMode(int(cm)).name
        except Exception:                          # noqa: BLE001
            return f"0x{int(cm):02X}"

    def cpv_begin(self) -> bool:
        """进入 CPV 逐关节位置伺服。**回放前调一次**,回放完 cpv_end()。

        做两件事:
          1. `set_motion_mode('cpv')` —— 发一次 0x151 把 move_mode 切过去
          2. `set_auto_set_motion_mode_enabled(False)` —— 关掉自动切模式

        ⚠ 第 2 步是**必需的,不是优化**。SDK 的 `_move_cpv` 每次都调
        `_maybe_set_motion_mode('cpv')`,而那个在 auto 开着时会**再发一次 0x151**。
        一个轨迹点要发 7 个关节,于是每点 7 帧 CPV + 7 帧模式设置 = **14 帧**,
        30fps 就是 420 帧/秒,正好是我实测过的 210 帧/秒的两倍。
        抖动测试量的是 7 帧/点那条路径 —— 不关 auto 的话跑的根本不是被测过的东西。
        """
        if self.mock:
            self._cpv_active = True
            return True
        if self.robot is None:
            self.last_error = "cpv_begin: 未连接"
            return False
        try:
            self.robot.set_motion_mode("cpv")
            self.robot.set_auto_set_motion_mode_enabled(False)
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"进入 CPV 异常: {e}"
            return False
        self._cpv_active = True
        return True

    def cpv_end(self) -> None:
        """退出 CPV,把 auto 切模式**恢复**。不抛。

        必须恢复:`move_j` / `move_p` 都靠 auto 把模式切回去,留着 False
        会让之后的点位运动发在 cpv 模式下。这条路径没测过,别留给下一个人踩。
        """
        self._cpv_active = False
        if self.mock or self.robot is None:
            return
        try:
            self.robot.set_auto_set_motion_mode_enabled(True)
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"退出 CPV 异常: {e}"

    @property
    def cpv_active(self) -> bool:
        return self._cpv_active

    def move_cpv_pos(self, rad7: list[float]) -> bool:
        """一个轨迹点:7 个关节各发一帧 CPV 位置。返回是否全发出去了。

        和 `move_j` 的关键差别 —— **重设目标只是给伺服换个目标值,不是抢断轨迹规划**。
        `move_j` 自带规划,30fps 逐帧发等于每 33ms 打断上一条规划,臂永远走不到任何
        一个点(效果 = 对目标轨迹做低通 + 整体滞后)。CPV 是逐关节位置环,
        换目标就是换目标,所以能吃下 30fps 的流。

        ⚠ 需要 `cpv_begin()` 先进模式。没进就发会被 SDK 的 auto 逻辑每帧重切模式
        (见 cpv_begin 的注释),帧数翻倍。这里只警告不拦 —— 拦了的话
        单点调试(比如页面滑块)就得先 begin,那是多余的负担。
        """
        if self._frozen:
            return False              # 急停中:和 move_j 同约定,拒发
        rad7 = [_clamp(v, *NERO_ARM_LIMITS[i]) for i, v in enumerate(rad7)]
        self._target = rad7
        if self.mock:
            return True
        if self.robot is None:
            self.last_error = "move_cpv_pos: 未连接"
            return False
        try:
            for i, v in enumerate(rad7):
                self.robot.move_cpv_pos(i + 1, v)   # SDK 关节号 1-based
            return True
        except AttributeError:
            # v111/default 没有这个方法(v112+ 才有)。说清楚是固件问题,
            # 别让调用方看见一个光秃秃的 AttributeError 去猜。
            self.last_error = ("move_cpv_pos 需要固件 ≥1.12(当前 driver "
                               f"{self.firmware_detected or self.firmware})")
            return False
        except Exception as e:                     # noqa: BLE001
            self.last_error = f"move_cpv_pos 异常: {e}"
            return False

    @property
    def target(self) -> list[float]:
        return list(self._target)

    def estop(self) -> None:
        if self.mock:
            # 定格当前摆动位置,停住 —— 给可见反馈(真机是 SDK 硬急停)。
            # mock 不能真的"下落",但**必须**同样置 _enabled=False,否则 mock 下
            # 走不到"复位要重新使能"这条路径,真机上才第一次撞见。
            self._frozen_pose = self.read_angles()
            self._frozen = True
            self._enabled = False
        elif self.robot is not None:
            self._frozen = True
            self.robot.electronic_emergency_stop()   # 0x150 byte0=1
            # ⚠ 急停**不保位**。官方协议流程第 8 条原文:
            #   "急停状态全部关节电机进入阻尼模式(由于没有关节抱闸,机械臂会缓慢下落)"
            # 所以急停后 _enabled 必须置 False —— 电机进阻尼即失能,reset() 里要重发 0x471。
            # 留着 True 会让 UI 显示"已使能"而实际动不了。
            self._enabled = False

    def telemetry(self) -> dict:
        """调试页要的全量遥测。读不到的项为 None,**不抛**(和 InspireHand.telemetry 同约定)。

        逐关节走 get_motor_states(i):position/velocity/current/torque(HighSpd 帧
        `0x251~0x257`,candump 实测 220.5Hz)。

        ⚠ 原来这里写「SDK 已经补偿 v1.20 之前 velocity 恒为 0」—— **说反了**。
        SDK 不是补偿,是**主动抹零**:v111/default 的 get_motor_states 里有一行
        `motor_state.msg.velocity = 0.0`,注释「corrected in version 1.20」。
        所以 velocity 能不能用**完全取决于 driver**:
          · v120 (固件 ≥1.20) → 真值
          · v111 / default    → **恒 0**
          · v112              → 也恒 0!它没有自己的 get_motor_states,继承 v111 的
        本机 2026-08-03 已升到 1.21 → v120,拿到的是真值。遥测里带 `velocity_valid`
        把这个条件显式说出来,别让下游拿 0 当"臂静止"。

        另外带出 SDK 的硬件时间戳和帧率(`ts_can` / `hz_*`):
        那是内核 SO_TIMESTAMPNS,**CLOCK_REALTIME**,分辨率 2⁻²⁰ 秒。
        ⚠ 不能和我们自己的 `time.monotonic()` 相减 —— 不同时钟。
        而且帧是成批到的(usbip over WSL),单帧戳不代表它在 CAN 线上出现的时刻。
        """
        if self.mock:
            a = self.read_angles()
            return {"mock": True, "position": [round(v, 4) for v in a],
                    "velocity": [0.0] * 7, "velocity_valid": False,
                    "current": [0.0] * 7,
                    "torque": [0.0] * 7, "enabled": self._enabled,
                    "frozen": self._frozen, "speed_percent": self.speed_percent}
        out: dict = {"mock": False, "enabled": self._enabled,
                     "frozen": self._frozen, "speed_percent": self.speed_percent,
                     # 只有 v120 的 get_motor_states 不抹零(见本方法 docstring)
                     "velocity_valid": self.velocity_is_real()}
        pos, vel, cur, tor = [], [], [], []
        ts_can, hz_hs = None, None
        for i in range(1, 8):
            try:
                ms = self.robot.get_motor_states(i)
            except Exception:                      # noqa: BLE001
                ms = None
            m = getattr(ms, "msg", None)
            pos.append(round(m.position, 4) if m else None)
            vel.append(round(m.velocity, 4) if m else None)
            cur.append(round(m.current, 3) if m else None)
            tor.append(round(m.torque, 3) if m else None)
            if ms is not None and ts_can is None:
                # 取第一个有值的关节的戳/帧率代表这一组 —— 7 个 HighSpd 帧同批到,
                # 组内散布实测中位 0.67ms,当成同一时刻够用。
                ts_can = getattr(ms, "timestamp", None)
                hz_hs = getattr(ms, "hz", None)
        out.update(position=pos, velocity=vel, current=cur, torque=tor,
                   ts_can=ts_can, hz_highspd=hz_hs, ts_clock="CLOCK_REALTIME")
        for key, fn in (("arm_status", lambda: self.robot.get_arm_status()),
                        ("enable_status",
                         lambda: self.robot.get_joint_enable_status(255))):
            try:
                r = fn()
                out[key] = str(getattr(r, "msg", r)) if r is not None else None
            except Exception:                      # noqa: BLE001
                out[key] = None
        return out

    def disconnect(self) -> None:
        if not self.mock and self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:                      # noqa: BLE001
                pass
        self.robot = None
        self._enabled = False
