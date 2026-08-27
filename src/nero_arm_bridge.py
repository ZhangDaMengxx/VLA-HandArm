#!/usr/bin/env python3
"""ROS 2 hardware driver for the NERO arm and Inspire RH56DFX hand.

The node is the only process allowed to own the CAN and RS485 devices. It
publishes structured joint/device state and accepts ROS commands. Real hardware
never enables automatically; ``--enable-control`` only creates command
subscriptions and services.

State:
  /joint_states                    sensor_msgs/JointState
  /nero/driver_state               std_msgs/String (JSON, transient local)
  /diagnostics                     diagnostic_msgs/DiagnosticArray

Compatible command topics:
  /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory
  /hand_controller/joint_trajectory trajectory_msgs/JointTrajectory
  /nero/arm_command                std_msgs/String (JSON)
  /nero/estop                      std_msgs/Bool

Services:
  /nero/arm/set_enabled            std_srvs/SetBool
  /nero/arm/reset                  std_srvs/Trigger
  /nero/arm/estop                  std_srvs/Trigger

The trajectory compatibility topics currently execute only the final waypoint.
Full trajectory execution will move to a FollowJointTrajectory action server.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

SRC = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from inspire_hand import HAND_JOINTS, InspireHand, InspireHandConfig  # noqa: E402
from nero_arm import ARM_JOINTS, NeroArm  # noqa: E402
from ros_driver_state import DeviceHealth, DeviceState  # noqa: E402


def _command_qos() -> QoSProfile:
    # Commands must never survive a driver restart or reconnect.
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _trajectory_target(msg: JointTrajectory,
                       expected: list[str]) -> tuple[list[float] | None, str]:
    if not msg.points:
        return None, "trajectory has no points"
    if len(set(msg.joint_names)) != len(msg.joint_names):
        return None, "trajectory contains duplicate joint names"
    point = msg.points[-1]
    if len(point.positions) != len(msg.joint_names):
        return None, "joint_names and positions lengths differ"
    index = {name: i for i, name in enumerate(msg.joint_names)}
    missing = [name for name in expected if name not in index]
    if missing:
        return None, f"trajectory is missing joints: {', '.join(missing)}"
    try:
        target = [float(point.positions[index[name]]) for name in expected]
    except (TypeError, ValueError) as exc:
        return None, f"trajectory contains an invalid position: {exc}"
    if not all(math.isfinite(value) for value in target):
        return None, "trajectory contains a non-finite position"
    return target, ""


class NeroHardwareDriver(Node):
    def __init__(self, *, arm_mock: bool, hand_mock: bool, rate: float,
                 enable_control: bool, channel: str, firmware: str,
                 hand_port: str, retry_initial: float, retry_max: float,
                 read_failure_limit: int) -> None:
        super().__init__("nero_hardware_driver")
        self._accept_commands = enable_control
        self._shutting_down = False

        self.arm = NeroArm(mock=arm_mock, channel=channel, firmware=firmware)
        self.hand = InspireHand(InspireHandConfig(mock=hand_mock, port=hand_port))
        health_args = {
            "retry_initial_s": retry_initial,
            "retry_max_s": retry_max,
            "read_failure_limit": read_failure_limit,
        }
        self.arm_health = DeviceHealth("arm", **health_args)
        self.hand_health = DeviceHealth("hand", **health_args)

        joint_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.joint_pub = self.create_publisher(JointState, "/joint_states", joint_qos)
        self.state_pub = self.create_publisher(String, "/nero/driver_state", _status_qos())
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", QoSProfile(depth=10))

        if enable_control:
            qos = _command_qos()
            self.create_subscription(
                JointTrajectory, "/arm_controller/joint_trajectory", self._on_arm_traj, qos)
            self.create_subscription(
                JointTrajectory, "/hand_controller/joint_trajectory", self._on_hand_traj, qos)
            self.create_subscription(Bool, "/nero/estop", self._on_estop, 10)
            self.create_subscription(String, "/nero/arm_command", self._on_arm_cmd, 10)
            self.create_service(SetBool, "/nero/arm/set_enabled", self._on_set_enabled)
            self.create_service(Trigger, "/nero/arm/reset", self._on_reset)
            self.create_service(Trigger, "/nero/arm/estop", self._on_estop_service)

        self.create_timer(max(0.01, 1.0 / rate), self._publish_joints)
        self.create_timer(0.5, self._maintain_connections)
        self.create_timer(1.0, self._publish_health)
        self._maintain_connections()
        self._publish_health()

        mode = "commands enabled" if enable_control else "monitor only"
        self.get_logger().info(
            f"hardware driver started ({mode}); arm_mock={arm_mock} "
            f"hand_mock={hand_mock}; joint_states={rate:.1f} Hz")
        if enable_control and not (arm_mock and hand_mock):
            self.get_logger().warn(
                "real hardware remains in its current enable state; use "
                "/nero/arm/set_enabled explicitly before motion")

    def _device(self, name: str):
        if name == "arm":
            return self.arm, self.arm_health
        return self.hand, self.hand_health

    def _maintain_connections(self) -> None:
        if self._shutting_down:
            return
        now = time.monotonic()
        for name in ("arm", "hand"):
            device, health = self._device(name)
            if not health.begin_connect(now):
                continue
            self.get_logger().info(f"connecting {name}")
            try:
                device.disconnect()
                connected = bool(device.connect())
                if not connected:
                    raise RuntimeError("connect returned false")
            except Exception as exc:  # noqa: BLE001
                try:
                    device.disconnect()
                except Exception:  # noqa: BLE001
                    pass
                health.mark_fault(str(exc), time.monotonic())
                retry_in = health.snapshot(time.monotonic())["retry_in_s"]
                self.get_logger().error(
                    f"{name} connection failed: {exc}; retry in {retry_in} s")
            else:
                health.mark_ready(time.monotonic())
                self.get_logger().info(f"{name} READY")

    def _fault_device(self, name: str, error: str) -> None:
        device, health = self._device(name)
        try:
            device.disconnect()
        except Exception:  # noqa: BLE001
            pass
        health.mark_fault(error, time.monotonic())
        self.get_logger().error(f"{name} FAULT: {error}")

    def _read_device(self, name: str) -> list[float] | None:
        device, health = self._device(name)
        if not health.ready:
            return None
        try:
            positions = device.read_angles()
            read_ok = bool(getattr(device, "last_read_ok", True))
        except Exception as exc:  # noqa: BLE001
            positions = None
            read_ok = False
            device.last_error = f"read exception: {exc}"
        if not read_ok:
            error = device.last_error or "read failed"
            if health.mark_read_failure(error):
                self._fault_device(name, error)
            return None
        health.mark_read_success(time.monotonic())
        return [float(value) for value in positions]

    def _publish_joints(self) -> None:
        arm = self._read_device("arm")
        hand = self._read_device("hand")
        names: list[str] = []
        positions: list[float] = []
        if arm is not None:
            names.extend(ARM_JOINTS)
            positions.extend(arm)
        if hand is not None:
            names.extend(HAND_JOINTS)
            positions.extend(hand)
        if not names:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.joint_pub.publish(msg)

    def _arm_action(self, action: str, value=None) -> tuple[bool, str]:
        if not self._accept_commands:
            return False, "driver is in monitor-only mode"
        if not self.arm_health.ready:
            return False, f"arm is {self.arm_health.state.value}"
        if action not in {"enable", "disable", "reset", "estop", "set_speed"}:
            return False, f"unknown arm action: {action}"
        if action == "set_speed":
            try:
                value = float(value)
            except (TypeError, ValueError):
                return False, f"invalid speed value: {value!r}"
        try:
            if action == "enable":
                ok = bool(self.arm.enable())
            elif action == "disable":
                ok = bool(self.arm.disable())
            elif action == "reset":
                self.arm.reset()
                ok = bool(self.arm.enabled)
            elif action == "estop":
                self.arm.estop()
                ok = True
            elif action == "set_speed":
                self.arm.set_speed_percent(value)
                ok = True
        except Exception as exc:  # noqa: BLE001
            self._fault_device("arm", f"{action} failed: {exc}")
            return False, str(exc)
        message = f"arm {action} {'succeeded' if ok else 'was rejected'}"
        if not ok:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)
        self._publish_health()
        return ok, message

    def _on_arm_traj(self, msg: JointTrajectory) -> None:
        target, error = _trajectory_target(msg, ARM_JOINTS)
        if error:
            self.get_logger().error(f"rejected arm trajectory: {error}")
            return
        if not self.arm_health.ready:
            self.get_logger().error(
                f"rejected arm trajectory: arm is {self.arm_health.state.value}")
            return
        if not self.arm.enabled or self.arm.frozen:
            reason = "estopped" if self.arm.frozen else "disabled"
            self.get_logger().error(f"rejected arm trajectory: arm is {reason}")
            return
        if not self.arm.move_j(target):
            self._fault_device("arm", self.arm.last_error or "move_j failed")

    def _on_hand_traj(self, msg: JointTrajectory) -> None:
        target, error = _trajectory_target(msg, HAND_JOINTS)
        if error:
            self.get_logger().error(f"rejected hand trajectory: {error}")
            return
        if not self.hand_health.ready:
            self.get_logger().error(
                f"rejected hand trajectory: hand is {self.hand_health.state.value}")
            return
        if not self.hand.set_angles(target):
            self._fault_device("hand", self.hand.last_error or "set_angles failed")

    def _on_estop(self, msg: Bool) -> None:
        if msg.data:
            self._arm_action("estop")

    def _on_arm_cmd(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().error(f"invalid /nero/arm_command JSON: {exc}")
            return
        self._arm_action(str(command.get("action", "")), command.get("value"))

    def _on_set_enabled(self, request: SetBool.Request,
                        response: SetBool.Response) -> SetBool.Response:
        response.success, response.message = self._arm_action(
            "enable" if request.data else "disable")
        return response

    def _on_reset(self, _request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._arm_action("reset")
        return response

    def _on_estop_service(self, _request: Trigger.Request,
                          response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._arm_action("estop")
        return response

    def _state_payload(self) -> dict:
        now = time.monotonic()
        arm = self.arm_health.snapshot(now)
        arm.update({
            "mock": self.arm.mock,
            "enabled": bool(self.arm.enabled),
            "frozen": bool(self.arm.frozen),
            "firmware": self.arm.firmware_detected,
        })
        hand = self.hand_health.snapshot(now)
        hand.update({"mock": self.hand.cfg.mock, "port": self.hand.cfg.port})
        return {
            "stamp": round(self.get_clock().now().nanoseconds * 1e-9, 6),
            "accepting_commands": self._accept_commands,
            "ready": self.arm_health.ready and self.hand_health.ready,
            "arm": arm,
            "hand": hand,
        }

    @staticmethod
    def _diagnostic(name: str, health: DeviceHealth,
                    values: dict) -> DiagnosticStatus:
        msg = DiagnosticStatus()
        msg.name = f"nero_inspire/{name}"
        msg.hardware_id = name
        if health.state is DeviceState.READY:
            msg.level = DiagnosticStatus.OK
            msg.message = "READY"
        elif health.state is DeviceState.FAULT:
            msg.level = DiagnosticStatus.ERROR
            msg.message = health.last_error or "FAULT"
        else:
            msg.level = DiagnosticStatus.WARN
            msg.message = health.state.value
        msg.values = [KeyValue(key=str(key), value=str(value))
                      for key, value in values.items() if value is not None]
        return msg

    def _publish_health(self) -> None:
        payload = self._state_payload()
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [
            self._diagnostic("arm", self.arm_health, payload["arm"]),
            self._diagnostic("hand", self.hand_health, payload["hand"]),
        ]
        self.diagnostics_pub.publish(diagnostics)

    def shutdown(self) -> None:
        self._shutting_down = True
        for name in ("arm", "hand"):
            device, health = self._device(name)
            try:
                device.disconnect()
            except Exception:  # noqa: BLE001
                pass
            health.mark_disconnected("driver shutdown")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mock", dest="mock", action="store_true", default=None)
    parser.add_argument("--no-mock", dest="mock", action="store_false")
    parser.add_argument("--arm-mock", dest="arm_mock", action="store_true", default=None)
    parser.add_argument("--no-arm-mock", dest="arm_mock", action="store_false")
    parser.add_argument("--hand-mock", dest="hand_mock", action="store_true", default=None)
    parser.add_argument("--no-hand-mock", dest="hand_mock", action="store_false")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="joint state polling/publish rate in Hz")
    parser.add_argument("--enable-control", dest="enable_control", action="store_true",
                        default=None, help="accept commands; does not enable real motors")
    parser.add_argument("--no-control", dest="enable_control", action="store_false")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--hand-port", default="/dev/ttyUSB0")
    parser.add_argument("--firmware", default="auto",
                        choices=["auto", "default", "v111", "v112", "v120"])
    parser.add_argument("--retry-initial", type=float, default=1.0)
    parser.add_argument("--retry-max", type=float, default=30.0)
    parser.add_argument("--read-failure-limit", type=int, default=3)
    args = parser.parse_args()

    base_mock = True if args.mock is None else args.mock
    arm_mock = base_mock if args.arm_mock is None else args.arm_mock
    hand_mock = base_mock if args.hand_mock is None else args.hand_mock
    enable_control = args.enable_control
    if enable_control is None:
        enable_control = arm_mock and hand_mock

    rclpy.init()
    node = NeroHardwareDriver(
        arm_mock=arm_mock,
        hand_mock=hand_mock,
        rate=max(1.0, args.rate),
        enable_control=enable_control,
        channel=args.channel,
        firmware=args.firmware,
        hand_port=args.hand_port,
        retry_initial=max(0.1, args.retry_initial),
        retry_max=max(args.retry_initial, args.retry_max),
        read_failure_limit=max(1, args.read_failure_limit),
    )
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
