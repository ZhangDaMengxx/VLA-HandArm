#!/usr/bin/env python3
"""sim/detect_arm_firmware.py — 读臂的**主控**固件版本,判 SDK 会选哪个 driver。

**只读。** 连 CAN、读一个版本字符串、断开。不使能、不发运动、不改臂的任何状态。

## 为什么需要这个

`arm_console.py` 现在默认 `--firmware auto`,会先用 DEFAULT driver 读 software_version
再按 SDK 门限重连。写死版本号在固件升级后会**静默**出错,所以不写死。

SDK 按固件版本分了四套 driver,**每一级继承上一级**(MRO: v120 → v112 → v111 → default),
能用的运动命令逐级增加:

| driver  | 有哪些运动命令 |
|---------|--------------------------------------------------|
| default | move_j / move_js / move_l / move_p / move_c / move_mit |
| v111    | 同上(全部继承),**没有** cpv |
| v112    | 上面全部 **+** move_cpv_pos / move_cpv_vel |
| v120    | 同 v112,另外修了 velocity 遥测被抹零的 bug |

> ⚠ 这张表**必须按继承算**。第一版我只 grep 了各版本文件里自己 `def` 的方法,
> 把 v111 写成"只有 move_mit" —— 错的,而且 detect 的输出会自相矛盾。
> 代码里的 `_PRIMS` 和这张 docstring 表**是两份,改一处必须改另一处**
> (这条勘误本身就是因为只改了 `_PRIMS` 忘了这里)。

这直接卡住**逐帧轨迹回放**:`move_j` 自带轨迹规划,30fps 逐帧发等于每 33ms 打断上一条
规划,臂永远走不到任何一个点 —— 变成对目标轨迹做低通滤波(形状被抹平 + 整体滞后)。
能逐帧的:`move_cpv_pos`(v112+,**逐关节位置伺服**,重设目标只是重定向,不是抢断规划)
和 `move_js`(SDK 自评 Risk: EXTREMELY HIGH,无平滑无规划)。

**这台臂 2026-08-03 已升到 1.21 → v120**,所以 `move_cpv_pos` 可用、velocity 是真值。

⚠ 你手上那两个 `*_DRVV2.0.6-*.bin` 是**关节驱动器**固件,和这里读的主控
`software_version` 不是一回事,不能拿来判 driver。

## 用法

    python3 sim/detect_arm_firmware.py            # 默认 can0
    python3 sim/detect_arm_firmware.py --channel can1
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

# ⚠ 版本 → driver 的门限**照抄 SDK 自己的 demo**(demos/detect_nero_series.py):
#     sv >= "1.20" → V120 ;  >= "1.12" → V112 ;  >= "1.11" → V111 ;  else DEFAULT
# 它是**字符串**比较,不是数值比较。所以 "1.9" >= "1.20" 在字典序上为 True,会被判成
# v120 —— 语义上很可能是错的。这里刻意保持和 SDK 一致(工厂最终按同一套规则选类,
# 我们自作聪明反而会和 SDK 的实际行为不符),但**它是个已知的脆弱点**:
# 拿到版本号后请人眼核一下这个映射合不合理,别盲信。
_GATES = (("1.20", "v120"), ("1.12", "v112"), ("1.11", "v111"))

# 各 driver 实际有哪些运动命令。
#
# ⚠ 这张表必须**按继承算**,不能只看各版本文件里自己定义了哪些 `def move_*`。
# 我第一版就是那么写的,结果 v111 被写成"只有 move_mit" —— 错的。实际 MRO 是
#     v120 → v112 → v111 → default
# 每一级都继承上一级,所以 v111 有 default 的全套,v112 是在全套之上**多加**
# cpv 两个,不是替换。写错的后果是判断"能不能逐帧回放"时结论整个反过来。
#
# 用 dir(cls) 实测出来的(见 detect 输出里的 MRO 一行),不是 grep 出来的。
#
# ⚠ 上面 docstring 里那张表和这里是**两份**,改一处必须改另一处。
# 实际踩过:改了 _PRIMS 但 docstring 的 v111 行还写着"只有 move_mit"。
# v120 除了继承 v112 的 cpv,还**重写了 get_motor_states** —— v111/default 里那行
# `motor_state.msg.velocity = 0.0`(注释「corrected in version 1.20」)在 v120 没有。
# 注意 v112 **没有**自己的 get_motor_states,所以它继承 v111 的抹零:只有 v120 是真速度。
_BASE = ["move_j", "move_js", "move_l", "move_p", "move_c", "move_mit"]
_PRIMS = {
    "default": _BASE,
    "v111":    _BASE,                                  # 继承 default 全套
    "v112":    _BASE + ["move_cpv_pos", "move_cpv_vel"],
    "v120":    _BASE + ["move_cpv_pos", "move_cpv_vel"],   # 继承 v112
}


def pick_driver(sv: str) -> str:
    """按 SDK 的门限把 software_version 映射到 driver 名。"""
    for gate, name in _GATES:
        if sv >= gate:
            return name
    return "default"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--wait", type=float, default=8.0,
                    help="等固件帧的秒数。connect() 不等数据,必须轮询")
    args = ap.parse_args()

    from nero_arm import _prepare_pyagx_imports          # 复用它的 sys.path 准备
    _prepare_pyagx_imports()
    from pyAgxArm import (create_agx_arm_config, AgxArmFactory,   # noqa: E402
                          ArmModel, NeroFW)

    # 探测阶段固定用 DEFAULT driver:我们**还不知道**版本,而读版本这条帧所有 driver
    # 都支持。SDK 的 demo 也是先 default 连一次、读到版本再用正确的 driver 重连。
    cfg = create_agx_arm_config(robot=ArmModel.NERO,
                                firmeware_version=NeroFW.DEFAULT,
                                interface="socketcan", channel=args.channel,
                                bitrate=1000000)
    robot = AgxArmFactory.create_arm(cfg)
    try:
        robot.connect()
    except Exception as e:                                        # noqa: BLE001
        print(f"CAN 打开失败({args.channel}): {e}\n"
              f"检查:臂上电、以太网模式已切出、端接电阻、波特率 1Mbps",
              file=sys.stderr)
        return 1

    try:
        # ⚠ 轮询等,不能只探一次:connect() 只建 socket + 起后台读线程就返回,
        # get_firmware() 读的是 parser 缓存,读线程解到帧之前恒为 None。
        # ⚠ 且**不在循环里发 enable** —— SDK 的 demo 那样做(它顺手给臂上使能),
        # 照抄会让一个"只读探测"偷偷改臂的状态。我们只读。
        fw = None
        deadline = time.monotonic() + args.wait
        while time.monotonic() < deadline:
            fw = robot.get_firmware()
            if fw is not None:
                break
            time.sleep(0.1)
        if fw is None:
            print(f"等固件帧超时({args.wait:.0f}s)。CAN 通了但没收到固件帧 —— "
                  f"可能臂没在推送(0x151 byte6 不显式开就没有推送)", file=sys.stderr)
            return 2

        sv = fw.get("software_version")
        drv = pick_driver(str(sv))
        print(f"主控 software_version : {sv}")
        print(f"SDK 会选的 driver     : {drv}"
              f"{'  (= 当前 arm_console 的默认值)' if drv == 'default' else ''}")
        print(f"该 driver 的运动命令   : {' / '.join(_PRIMS.get(drv, ['?']))}")
        for k, v in sorted(fw.items()):
            if k != "software_version":
                print(f"  {k:22s}: {v}")

        print()
        if "move_cpv_pos" in _PRIMS.get(drv, []):
            print("→ 逐帧回放可用 move_cpv_pos(连续位置流,形状正确)。")
            print("  代价:按**单关节**下发,一个轨迹点 = 7 条 CAN 帧,30fps = 210 帧/秒,")
            print("  先确认总线吃得下。同族的 move_cpv_vel 有已知的符号翻转 bug")
            print("  (7 个关节里 6 个被乘 -1,SDK 源码里标着 TODO),别用那个。")
        else:
            print("→ 这个 driver **没有** move_cpv_pos(连续位置流,1.12 才加的)。")
            print("  逐帧回放还剩两条,各有代价:")
            print()
            print("  a) move_js —— 一条命令 7 个关节,形状对。但 SDK 自评")
            print("     Risk: EXTREMELY HIGH(无平滑、无轨迹规划,可能机械冲击/振荡)。")
            print("     我们的轨迹是 MediaPipe 过 IK 出来的、本身带抖,喂给瞬时响应")
            print("     模式正好是触发那几种故障的条件。")
            print("  b) move_mit —— 阻抗控制 T=kp(p_des-p)+kd(v_des-v)+t_ff。")
            print("     它**没有内置轨迹规划**,所以逐帧喂参考位置不会被打断 ——")
            print("     这一点比 move_j 适合跟随。而且阻抗本身就是低通,抖动被")
            print("     柔顺吸收而不是变成冲击。代价:按单关节下发(每点 7 帧)、")
            print("     要调 kp/kd、且是**力矩控制** —— 不给 t_ff 重力补偿的话臂会")
            print("     下垂,稳态误差 ≈ 重力力矩/kp。补偿要动力学模型(pinocchio 有)。")
            print()
            print("  最省事且零新增风险的是 **稀疏路点**:几个关键点交给 move_j 自己")
            print("  规划,手仍逐帧跑在同一条时间轴上。同步粒度从每帧降到每路点。")
        print()
        print(f"⚠ 门限是字符串比较(SDK 自己也是)。'{sv}' → {drv} 这个映射请人眼核一下。")
        return 0
    finally:
        # 臂常态是被松灵客户端带电控制着,我们是临时客人 —— 断开时什么都不改。
        try:
            robot.disconnect()
        except Exception:                                         # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())
