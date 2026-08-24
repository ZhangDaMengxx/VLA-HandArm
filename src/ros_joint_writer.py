#!/usr/bin/env python3
"""src/ros_joint_writer.py — 控制接口存根:JSON 指令 → JointTrajectory → 控制器。

跑在 **ROS2 系统 python3**。这是"控制"侧的预留实现:接收目标关节角,发到
  /arm_controller/joint_trajectory   (7 臂)
  /hand_controller/joint_trajectory  (6 驱动手)
仿真(Gazebo + ros2_control)现在就能驱动;真臂上线后,只要真臂的控制器也订阅
这两个话题(或在此改成真臂 SDK 调用),上层 app_web.py 不用动。

⚠ 安全:内置关节限位夹取(clamp)——超限指令被裁剪到 URDF 限位内,绝不外发原始越界值。
   真机联调前务必核对 JOINT_LIMITS 与实物一致,并保留急停通道(--estop 发零速停止)。

两种用法:
  1) 单发:python3 ros_joint_writer.py --once '{"arm":[...7],"hand":[...6],"duration":0.5}'
  2) 流式(app_web 调):python3 ros_joint_writer.py    # 逐行读 stdin JSON,每行发一次
     急停:发一行 {"estop": true}
"""
from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_NAMES = [f"joint{i}" for i in range(1, 8)]
HAND_NAMES = ["right_thumb_1_joint", "right_thumb_2_joint",
              "right_index_1_joint", "right_middle_1_joint",
              "right_ring_1_joint", "right_little_1_joint"]

NERO_ARM_LIMITS = {
    "joint1": (-2.705260340591211, 2.705260340591211),   # -155..155 deg
    "joint2": (-1.7453292519943295, 1.7453292519943295), # -100..100 deg
    "joint3": (-2.7576202181510405, 2.7576202181510405), # -158..158 deg
    "joint4": (-1.0122909661567112, 2.1467549799530254), # -58..123 deg
    "joint5": (-2.7576202181510405, 2.7576202181510405), # -158..158 deg
    "joint6": (-0.7330382858376184, 0.9599310885968813), # -42..55 deg
    "joint7": (-1.5707963267948966, 1.5707963267948966), # -90..90 deg
}

# 关节限位(rad)——NERO 来自本地 SDK 文档,手用当前 URDF 驱动关节上限。
JOINT_LIMITS = {
    **NERO_ARM_LIMITS,
    "right_thumb_1_joint": (0.0, 1.246165),
    "right_thumb_2_joint": (0.0, 0.48),
    "right_index_1_joint": (0.0, 1.333),
    "right_middle_1_joint": (0.0, 1.333),
    "right_ring_1_joint": (0.0, 1.333),
    "right_little_1_joint": (0.0, 1.333),
}


def _clamp(names, vals):
    """把每个目标角裁剪进限位;返回 (裁剪后列表, 是否发生裁剪)。"""
    out, clipped = [], False
    for n, v in zip(names, vals):
        lo, hi = JOINT_LIMITS.get(n, (-3.14159, 3.14159))
        cv = max(lo, min(hi, float(v)))
        clipped |= (cv != float(v))
        out.append(cv)
    return out, clipped


def _dur(t: float) -> Duration:
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


# 力控/速度的取值范围。**只做形状和粗范围校验,不做逐通道夹取** ——
# 逐通道上限那张表(FORCE_MAX)在 inspire_hand.py 里,是唯一真源;在这里抄一遍
# 必然和它漂移(拇指到底是 1000 还是 1500 就是这么被两处说法搞混过一次)。
FORCE_SPEED_RANGE = (0, 1000)


def _check_force_like(key: str, val) -> str | None:
    """校验 hand_force / hand_speed 的形状。返回错误串,None 表示通过。

    允许标量(全通道同值)或 6 个值(逐通道,项目顺序)。
    列表长度不对**必须报错而不是补齐**:补齐会让"我以为设了 6 指、其实只设了 3 指"
    悄悄发生,剩下几指沿用上次的阈值,现象是物体被捏歪而不是报错。
    """
    lo, hi = FORCE_SPEED_RANGE
    vals = val if isinstance(val, (list, tuple)) else [val]
    if isinstance(val, (list, tuple)) and len(val) != len(HAND_NAMES):
        return f"{key} 给列表时需要 {len(HAND_NAMES)} 个值(项目顺序),给了 {len(val)}"
    for v in vals:
        try:
            iv = int(v)
        except (TypeError, ValueError):
            return f"{key} 含非数字项: {v!r}"
        if not (lo <= iv <= hi):
            return f"{key} 的 {iv} 超出 {lo}-{hi}"
    return None


class JointWriter(Node):
    def __init__(self) -> None:
        super().__init__("nero_web_joint_writer")
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", qos)
        self.hand_pub = self.create_publisher(
            JointTrajectory, "/hand_controller/joint_trajectory", qos)
        # SDK 级指令通道(使能/复位/调速/急停):真机由 nero_arm_bridge 执行,仿真无副作用
        self.cmd_pub = self.create_publisher(String, "/nero/arm_command", 10)
        self.estop_pub = self.create_publisher(Bool, "/nero/estop", 10)

    def _emit(self, pub, names, positions, duration):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start = _dur(max(0.05, duration))
        msg.points.append(pt)
        pub.publish(msg)

    def send(self, cmd: dict) -> dict:
        """处理一条指令 dict,返回状态 dict(供上层回显)。"""
        if cmd.get("estop"):
            # 真机急停只发急停通道,避免同时下发任何运动目标。
            self.estop_pub.publish(Bool(data=True))
            return {"ok": True, "estop": True}
        if "action" in cmd:
            # SDK 级指令(enable/disable/reset/set_speed)→ /nero/arm_command
            self.cmd_pub.publish(String(data=json.dumps(cmd)))
            return {"ok": True, "action": cmd["action"]}
        dur = float(cmd.get("duration", 0.5))
        res = {"ok": True, "clamped": False}
        # hand_force / hand_speed:**这条路送不到**。JointTrajectory 里没有力控字段,
        # 仿真也没有对应的控制器接口。所以只校验形状,然后**显式报 unsupported**。
        #
        # 为什么不静默忽略:一个「轻捏」技能在真手路径上会设 force=250,在这条路上
        # 力控就还是上一次留下的值 —— 两条路行为不同,而调用方看不出来。宁可让它
        # 在回显里看到"这个字段没生效",也不要让它以为设上了。
        unsup = []
        for key in ("hand_force", "hand_speed"):
            if cmd.get(key) is None:
                continue
            err = _check_force_like(key, cmd[key])
            if err:
                return {"ok": False, "error": err}
            unsup.append(key)
        if unsup:
            res["unsupported"] = unsup
            res["note"] = ("力控/速度只在 console 直连路(hand_console)有效;"
                           "ROS 这条路没有对应通道,本次未下发")
        if "arm" in cmd and cmd["arm"] is not None:
            vals, clipped = _clamp(ARM_NAMES, cmd["arm"])
            self._emit(self.arm_pub, ARM_NAMES, vals, dur)
            res["clamped"] |= clipped
        if "hand" in cmd and cmd["hand"] is not None:
            vals, clipped = _clamp(HAND_NAMES, cmd["hand"])
            self._emit(self.hand_pub, HAND_NAMES, vals, dur)
            res["clamped"] |= clipped
        return res


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", metavar="JSON", help="发一条指令后退出")
    ap.add_argument("--estop", action="store_true", help="立即发急停后退出")
    args = ap.parse_args()

    rclpy.init()
    node = JointWriter()
    try:
        if args.estop:
            rclpy.spin_once(node, timeout_sec=0.5)
            print(json.dumps(node.send({"estop": True})), flush=True)
        elif args.once:
            rclpy.spin_once(node, timeout_sec=0.5)      # 让发布者先建立连接
            print(json.dumps(node.send(json.loads(args.once))), flush=True)
            rclpy.spin_once(node, timeout_sec=0.5)      # 给消息发出去的时间
        else:
            # 流式:逐行读 stdin。
            # ⚠ 用 StdinLines 而不是 select + sys.stdin.readline:后者 select 看 fd、
            #   readline 看用户态缓冲,一次写进多行时会有行卡在缓冲里,要等下一条
            #   命令到了才被处理 = 慢一个命令。见 stdin_lines.py 的模块说明。
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from stdin_lines import StdinLines
            print(json.dumps({"type": "ready"}), flush=True)
            reader = StdinLines()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                for line in reader.poll(0.05):
                    try:
                        cmd = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    print(json.dumps(node.send(cmd), ensure_ascii=False), flush=True)
                if reader.eof:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
