#!/usr/bin/env python3
"""src/nero_arm_bridge.py — 真机桥:NERO 臂(pyAgxArm/CAN)+ RH56DFX 手 → /joint_states。

跑在 **ROS2 系统 python3**。它是"数字孪生"里 /joint_states 的**真机数据源**,
直接替掉 fake_real_arm.py:下游(ros_joint_reader / live_rerun / 网页)一律不改。

  NERO 臂 ─CAN─► pyAgxArm.get_joint_angles() (7, rad) ┐
                                                       ├─► /joint_states (13)
  RH56DFX 手 ─RS485─► InspireHand.read_angles() (6, rad)┘

控制(可选,--enable-control):订阅和仿真同名的话题,但改用 SDK 驱动真机:
  /arm_controller/joint_trajectory  → robot.move_j(最后一个路点)
  /hand_controller/joint_trajectory → hand.set_angles(...)
  /nero/estop (std_msgs/Bool)       → robot.electronic_emergency_stop()

⚠ 默认 --mock:臂发正弦、手回读占位目标,无需 CAN/串口,先把链路跑通。
  真机:--no-mock(臂需 CAN can0 + pyAgxArm;手需手册后去 inspire_hand 的 TODO)。
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))
from inspire_hand import InspireHand, InspireHandConfig, HAND_JOINTS
# NeroArm 抽到 nero_arm.py 了 —— arm_console.py 要在无 ROS 环境用它,
# 而本文件模块级 import rclpy。两处共用一份,别再各写一个。
from nero_arm import NeroArm, NERO_ARM_LIMITS, ARM_JOINTS          # noqa: F401


class NeroBridge(Node):
    def __init__(self, arm_mock: bool, hand_mock: bool, rate: float,
                 enable_control: bool, channel: str, firmware: str) -> None:
        super().__init__("nero_arm_bridge")
        self.arm = NeroArm(mock=arm_mock, channel=channel, firmware=firmware)
        self.hand = InspireHand(InspireHandConfig(mock=hand_mock))
        self.arm.connect()
        self.hand.connect()
        self.get_logger().info(
            f"桥接启动 arm_mock={arm_mock} hand_mock={hand_mock} "
            f"控制={'开' if enable_control else '关(只监控)'} "
            f"发布 /joint_states @ {rate:.0f}Hz")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        if enable_control:
            self.arm.enable()
            qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                     self._on_arm_traj, qos)
            self.create_subscription(JointTrajectory, "/hand_controller/joint_trajectory",
                                     self._on_hand_traj, qos)
            self.create_subscription(Bool, "/nero/estop", self._on_estop, 10)
            self.create_subscription(String, "/nero/arm_command", self._on_arm_cmd, 10)

    def _tick(self) -> None:
        arm = self.arm.read_angles()
        hand = self.hand.read_angles()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + HAND_JOINTS
        msg.position = [float(x) for x in arm] + [float(x) for x in hand]
        self.pub.publish(msg)

    # ---- 控制(--enable-control 时)----
    def _on_arm_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        # 取最后一个路点作为目标(SDK move_j 是点位控制,自带插值)
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        last = msg.points[-1].positions
        target = [last[idx[n]] if n in idx else 0.0 for n in ARM_JOINTS]
        self.arm.move_j(target)

    def _on_hand_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        last = msg.points[-1].positions
        target = [last[idx[n]] if n in idx else 0.0 for n in HAND_JOINTS]
        self.hand.set_angles(target)

    def _on_estop(self, msg: Bool) -> None:
        if msg.data:
            self.arm.estop()
            self.get_logger().warn("收到急停")

    def _on_arm_cmd(self, msg: String) -> None:
        """SDK 级指令(非轨迹):{action: enable|disable|reset|estop|set_speed, value}。"""
        import json as _json
        try:
            cmd = _json.loads(msg.data)
        except _json.JSONDecodeError:
            return
        action = cmd.get("action")
        if action == "enable":
            self.arm.enable()
        elif action == "disable":
            self.arm.disable()
        elif action == "reset":
            self.arm.reset()
        elif action == "estop":
            self.arm.estop()
        elif action == "set_speed":
            self.arm.set_speed_percent(float(cmd.get("value", 100)))
        self.get_logger().info(f"臂指令 {action} {cmd.get('value', '')}")

    def shutdown(self) -> None:
        self.arm.disconnect()
        self.hand.disconnect()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", dest="mock", action="store_true", default=None,
                    help="臂和手都使用无硬件占位模式")
    ap.add_argument("--no-mock", dest="mock", action="store_false",
                    help="臂和手都使用真机模式")
    ap.add_argument("--arm-mock", dest="arm_mock", action="store_true", default=None,
                    help="只让机械臂使用占位模式")
    ap.add_argument("--no-arm-mock", dest="arm_mock", action="store_false",
                    help="只让机械臂使用真机 CAN + pyAgxArm")
    ap.add_argument("--hand-mock", dest="hand_mock", action="store_true", default=None,
                    help="只让灵巧手使用占位模式(当前手未到货时使用)")
    ap.add_argument("--no-hand-mock", dest="hand_mock", action="store_false",
                    help="只让灵巧手使用真机 RS485 驱动")
    ap.add_argument("--rate", type=float, default=100.0, help="/joint_states 发布率 Hz")
    ap.add_argument("--enable-control", dest="enable_control", action="store_true",
                    default=None, help="订阅控制话题(真机默认关需显式开;mock 默认开)")
    ap.add_argument("--no-control", dest="enable_control", action="store_false",
                    help="强制只监控(mock 下也不收控制)")
    ap.add_argument("--channel", default="can0", help="CAN 通道(真机臂)")
    ap.add_argument("--firmware", default="default", choices=["default", "v111", "v112", "v120"],
                    help="NERO SDK 固件适配版本")
    args = ap.parse_args()

    base_mock = True if args.mock is None else args.mock
    arm_mock = base_mock if args.arm_mock is None else args.arm_mock
    hand_mock = base_mock if args.hand_mock is None else args.hand_mock

    # mock 无硬件风险 → 默认开控制,开箱即用;真机默认关,须显式 --enable-control
    enable_control = args.enable_control
    if enable_control is None:
        enable_control = arm_mock and hand_mock

    rclpy.init()
    node = NeroBridge(arm_mock, hand_mock, args.rate, enable_control,
                      args.channel, args.firmware)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
