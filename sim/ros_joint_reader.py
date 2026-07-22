#!/usr/bin/env python3
"""sim/ros_joint_reader.py — ROS2 /joint_states → stdout(每帧一行 JSON)。

跑在 **ROS2 系统 python3**(需先 source /opt/ros/humble + 工作区),被 app_web.py 以
子进程方式拉起。它是"实时监控"的数据源前端:订阅唯一真相源 /joint_states
(假真臂 fake_real_arm 或真臂驱动都往这发),把每条消息压成一行 JSON 打到 stdout,
上游 app_web.py(gradio venv)读管道即可 —— 两个 python 环境不互相 import,只靠 stdout 契约。

输出格式(每行):
  {"t": 1721.53, "names": [...13...], "pos": [...13...], "vel": [...] }
  vel 可能为空列表(假臂不发速度)。names 顺序 = 臂 joint1..7 + 手 6 驱动。

用法(一般由 app_web.py 调,手动测试):
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 sim/ros_joint_reader.py
"""
from __future__ import annotations

import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


class JointReader(Node):
    def __init__(self) -> None:
        super().__init__("nero_web_joint_reader")
        # 传感器数据常用 best-effort + KEEP_LAST(1);和 JointStateBroadcaster 默认兼容
        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._cb, qos)
        # 再加一个 reliable 订阅,兼容以 reliable 发布的源(fake_real_arm 用默认 reliable)
        self.create_subscription(JointState, "/joint_states", self._cb,
                                 QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST))
        self._last_key = None
        print(json.dumps({"type": "ready"}), flush=True)

    def _cb(self, msg: JointState) -> None:
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        # 两个订阅可能收到同一条(不同 QoS),用 (t, 首值) 粗去重,避免重复打印
        key = (t, msg.position[0] if msg.position else 0.0)
        if key == self._last_key:
            return
        self._last_key = key
        row = {
            "t": round(t, 4),
            "names": list(msg.name),
            "pos": [round(float(x), 5) for x in msg.position],
            "vel": [round(float(x), 5) for x in msg.velocity] if msg.velocity else [],
        }
        print(json.dumps(row, ensure_ascii=False), flush=True)


def main() -> None:
    rclpy.init()
    node = JointReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({"type": "closed"}), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
