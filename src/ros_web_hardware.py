#!/usr/bin/env python3
"""ROS2 hardware client for the Web backend.

The Web application runs in a newer Python environment than ROS Humble, so it
starts this JSON-lines worker with the ROS Python runtime.  This process never
opens CAN or serial devices.  It subscribes to the formal Hardware Driver and
uses its acknowledged services for commands.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from nero_inspire_interfaces.srv import SetInt32, SetJointPositions
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

SRC = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from action_sequences import load_default_actions  # noqa: E402
import hand_console  # noqa: E402
from inspire_hand import (HAND_JOINTS, HAND_LIMITS, PROJECT_TO_VENDOR,  # noqa: E402
                          RAW_MAP, InspireHandConfig)
from nero_arm import (ARM_JOINTS, NERO_ARM_LIMITS, NERO_HOME_POSE,  # noqa: E402
                      NERO_TRACKING_READY_POSE)
from ros_combo_playback import RosComboController  # noqa: E402
from stdin_lines import StdinLines  # noqa: E402


_PRINT_LOCK = threading.Lock()


def emit(obj: dict) -> None:
    with _PRINT_LOCK:
        print(json.dumps(obj, ensure_ascii=False), flush=True)


class RosWebError(RuntimeError):
    pass


class RosWebHardware(Node):
    def __init__(self, device: str, *, hz: float, service_timeout: float,
                 startup_timeout: float, speed: int,
                 allow_mock_driver: bool = False) -> None:
        super().__init__(f"nero_web_{device}_backend")
        self.device = device
        self.names = HAND_JOINTS if device == "hand" else ARM_JOINTS
        self.service_timeout = max(0.1, float(service_timeout))
        self._startup_deadline = time.monotonic() + max(1.0, startup_timeout)
        self._lock = threading.RLock()
        self._driver_state: dict = {}
        self._positions: dict[str, float] = {}
        self._target: list[float] | None = None
        self._ready_emitted = False
        self._fatal_emitted = False
        self._started_at = time.monotonic()
        self._speed = int(speed)
        self._allow_mock_driver = bool(allow_mock_driver)
        self.connect_pose: list[float] | None = None
        self.combo_controller: RosComboController | None = None

        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/nero/driver_state", self._on_driver_state, state_qos)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            qos_profile_sensor_data)
        self._service_clients = {
            "arm_joints": self.create_client(
                SetJointPositions, "/nero/arm/set_joints"),
            "arm_tracking_begin": self.create_client(
                Trigger, "/nero/arm/tracking_begin"),
            "arm_tracking_joints": self.create_client(
                SetJointPositions, "/nero/arm/set_tracking_joints"),
            "arm_tracking_end": self.create_client(
                Trigger, "/nero/arm/tracking_end"),
            "arm_enabled": self.create_client(
                SetBool, "/nero/arm/set_enabled"),
            "arm_speed": self.create_client(SetInt32, "/nero/arm/set_speed"),
            "arm_reset": self.create_client(Trigger, "/nero/arm/reset"),
            "arm_estop": self.create_client(Trigger, "/nero/arm/estop"),
            "hand_angles": self.create_client(
                SetJointPositions, "/nero/hand/set_angles"),
            "hand_speed": self.create_client(SetInt32, "/nero/hand/set_speed"),
            "hand_force": self.create_client(SetInt32, "/nero/hand/set_force"),
        }
        self.create_timer(1.0 / max(1.0, hz), self._emit_state)
        self.create_timer(0.1, self._check_startup)

    def _on_driver_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(state, dict):
            with self._lock:
                self._driver_state = state
            self._maybe_emit_ready()

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        with self._lock:
            self._positions.update(
                (str(name), float(value))
                for name, value in zip(msg.name, msg.position)
            )
        self._maybe_emit_ready()

    def device_state(self) -> dict:
        with self._lock:
            value = self._driver_state.get(self.device, {})
            return dict(value) if isinstance(value, dict) else {}

    def driver_accepts_commands(self) -> bool:
        with self._lock:
            return bool(self._driver_state.get("accepting_commands", False))

    def positions(self) -> list[float] | None:
        with self._lock:
            if any(name not in self._positions for name in self.names):
                return None
            return [self._positions[name] for name in self.names]

    def _maybe_emit_ready(self) -> None:
        if self._ready_emitted or self._fatal_emitted:
            return
        state = self.device_state()
        positions = self.positions()
        if state.get("state") != "READY" or positions is None:
            return
        if not self.driver_accepts_commands():
            return
        if bool(state.get("mock", False)) and not self._allow_mock_driver:
            self._emit_fatal(
                "Web 请求真机，但 ROS2 Hardware Driver 当前是 mock 模式")
            return
        self._ready_emitted = True
        self.connect_pose = list(positions)
        with self._lock:
            self._target = list(positions)
        if self.device == "hand":
            sequences = load_default_actions()
            emit({
                "type": "ready",
                "backend": "ros",
                "mock": bool(state.get("mock", False)),
                "port": state.get("port", "ROS2"),
                "joints": list(HAND_JOINTS),
                "vendor_order": list(PROJECT_TO_VENDOR),
                "limits": [list(HAND_LIMITS[name]) for name in HAND_JOINTS],
                "actions": [
                    {"slot": seq.slot, "index": seq.index, "name": seq.name,
                     "steps": len(seq.steps)}
                    for seq in sequences
                ],
            })
        else:
            emit({
                "type": "ready",
                "backend": "ros",
                "mock": bool(state.get("mock", False)),
                "channel": "ROS2",
                "firmware": state.get("firmware"),
                "joints": list(ARM_JOINTS),
                "limits": [list(limit) for limit in NERO_ARM_LIMITS],
                "speed_percent": self._speed,
                "pending_speed": None,
                "connect_pose": [round(value, 4) for value in positions],
                "enabled": bool(state.get("enabled", False)),
                "frozen": bool(state.get("frozen", False)),
                "require_enable": True,
            })

    def _check_startup(self) -> None:
        self._maybe_emit_ready()
        if self._ready_emitted or self._fatal_emitted:
            return
        if time.monotonic() < self._startup_deadline:
            return
        state = self.device_state()
        if not self._driver_state:
            reason = "没有收到 /nero/driver_state，请先启动 Hardware Driver"
        elif not self.driver_accepts_commands():
            reason = (
                "Hardware Driver 是 monitor-only；请用 "
                "nero_hardware_control 重新启动")
        elif state.get("state") != "READY":
            reason = state.get("last_error") or (
                f"ROS2 {self.device} Driver 状态为 "
                f"{state.get('state', 'DISCONNECTED')}")
        else:
            reason = "没有收到完整的 /joint_states"
        self._emit_fatal(reason)

    def _emit_fatal(self, message: str) -> None:
        if self._fatal_emitted:
            return
        self._fatal_emitted = True
        emit({"type": "error", "fatal": True, "backend": "ros",
              "msg": message})

    def _emit_state(self) -> None:
        if not self._ready_emitted:
            return
        positions = self.positions()
        if positions is None:
            return
        state = self.device_state()
        with self._lock:
            target = list(self._target or positions)
        now = time.monotonic()
        row = {
            "type": "state",
            "backend": "ros",
            "t": round(now - self._started_at, 3),
            "names": list(self.names),
            "rad": [round(value, 4) for value in positions],
            "target": [round(value, 4) for value in target],
        }
        if self.device == "hand":
            raw = [self._rad_to_raw(name, value)
                   for name, value in zip(HAND_JOINTS, positions)]
            raw_vendor = [0] * len(raw)
            for index, vendor_index in enumerate(PROJECT_TO_VENDOR):
                raw_vendor[vendor_index] = raw[index]
            row.update({"raw": raw, "raw_vendor": raw_vendor})
        else:
            connect_pose = self.connect_pose
            row.update({
                "enabled": bool(state.get("enabled", False)),
                "frozen": bool(state.get("frozen", False)),
                "speed_percent": self._speed,
                "pose_drift": (
                    None if connect_pose is None else
                    round(max(abs(a - b) for a, b in
                              zip(positions, connect_pose)), 4)
                ),
            })
            if self.combo_controller is not None:
                combo = self.combo_controller.status()
                if combo is not None:
                    row["combo"] = combo
        if state.get("last_error"):
            row["last_error"] = state["last_error"]
        emit(row)

    @staticmethod
    def _rad_to_raw(name: str, value: float) -> int:
        span, invert = RAW_MAP[name]
        lo, hi = HAND_LIMITS[name]
        value = max(lo, min(hi, float(value)))
        fraction = max(0.0, min(1.0, value / span if span else 0.0))
        if invert:
            fraction = 1.0 - fraction
        return int(round(fraction * 1000.0))

    def call(self, name: str, request):
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            raise RosWebError(f"ROS Service 不可用: {client.srv_name}")
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(self.service_timeout):
            future.cancel()
            raise RosWebError(f"ROS Service 超时: {client.srv_name}")
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            raise RosWebError(
                f"ROS Service 调用失败({client.srv_name}): {exc}") from exc
        if response is None:
            raise RosWebError(f"ROS Service 无响应: {client.srv_name}")
        return response

    def set_positions(self, positions: list[float]) -> tuple[bool, str]:
        expected = len(self.names)
        if len(positions) != expected:
            return False, f"angles 需要 {expected} 个值,收到 {len(positions)}"
        try:
            values = [float(value) for value in positions]
        except (TypeError, ValueError) as exc:
            return False, f"angles 含非数字项: {exc}"
        if not all(math.isfinite(value) for value in values):
            return False, "angles 含非有限值"
        request = SetJointPositions.Request()
        request.positions = values
        response = self.call(f"{self.device}_angles" if self.device == "hand"
                             else "arm_joints", request)
        if response.success:
            with self._lock:
                self._target = values
        return bool(response.success), str(response.message)

    def set_tracking_positions(self, positions: list[float]) -> tuple[bool, str]:
        if len(positions) != len(ARM_JOINTS):
            return False, (
                f"tracking_angles 需要 {len(ARM_JOINTS)} 个值,"
                f"收到 {len(positions)}")
        try:
            values = [float(value) for value in positions]
        except (TypeError, ValueError) as exc:
            return False, f"tracking_angles 含非数字项: {exc}"
        if not all(math.isfinite(value) for value in values):
            return False, "tracking_angles 含非有限值"
        request = SetJointPositions.Request()
        request.positions = values
        response = self.call("arm_tracking_joints", request)
        if response.success:
            with self._lock:
                self._target = values
        return bool(response.success), str(response.message)

    def set_int(self, name: str, value: int) -> tuple[bool, str]:
        request = SetInt32.Request()
        request.value = int(value)
        response = self.call(name, request)
        if response.success and name == "arm_speed":
            self._speed = int(value)
        return bool(response.success), str(response.message)

    def set_enabled(self, enabled: bool) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self.call("arm_enabled", request)
        return bool(response.success), str(response.message)

    def trigger(self, name: str) -> tuple[bool, str]:
        response = self.call(name, Trigger.Request())
        return bool(response.success), str(response.message)

    @property
    def speed(self) -> int:
        return self._speed


class RosHandAdapter:
    """Small InspireHand-compatible surface used by the existing player."""

    def __init__(self, backend: RosWebHardware) -> None:
        self.backend = backend
        self.cfg = InspireHandConfig(mock=False)
        self.last_error: str | None = None
        self._target_rad = backend.positions() or [HAND_LIMITS[n][0]
                                                   for n in HAND_JOINTS]

    def set_angles(self, values) -> bool:
        try:
            ok, message = self.backend.set_positions(list(values))
        except RosWebError as exc:
            ok, message = False, str(exc)
        self.last_error = None if ok else message
        if ok:
            self._target_rad = [float(value) for value in values]
        return ok

    def set_speed(self, value: int) -> bool:
        return self._set_int("hand_speed", value)

    def set_force(self, value) -> bool:
        if isinstance(value, (list, tuple)):
            values = [int(item) for item in value]
            if not values or any(item != values[0] for item in values):
                self.last_error = (
                    "ROS2 Driver 暂不支持灵巧手逐通道力控，只能发送一个统一值")
                return False
            value = values[0]
        return self._set_int("hand_force", int(value))

    def _set_int(self, name: str, value: int) -> bool:
        try:
            ok, message = self.backend.set_int(name, int(value))
        except (RosWebError, TypeError, ValueError) as exc:
            ok, message = False, str(exc)
        self.last_error = None if ok else message
        return ok

    def clear_error(self) -> bool:
        self.last_error = "ROS2 Driver 暂未提供灵巧手 clear_error Service"
        return False

    def rad_to_raw(self, name: str, value: float) -> int:
        return self.backend._rad_to_raw(name, value)

    @staticmethod
    def raw_to_rad(name: str, raw: int) -> float:
        span, invert = RAW_MAP[name]
        fraction = max(0.0, min(1.0, float(raw) / 1000.0))
        if invert:
            fraction = 1.0 - fraction
        lo, hi = HAND_LIMITS[name]
        return max(lo, min(hi, fraction * span))

    def write_shorts(self, register: str, vendor_values) -> bool:
        if register != "ANGLE_SET":
            self.last_error = f"ROS2 Driver 不支持原始寄存器写入: {register}"
            return False
        values = list(vendor_values)
        if len(values) != len(HAND_JOINTS):
            self.last_error = "ANGLE_SET 需要 6 个厂商通道值"
            return False
        target = list(self._target_rad)
        for index, name in enumerate(HAND_JOINTS):
            raw = int(values[PROJECT_TO_VENDOR[index]])
            if raw >= 0:
                target[index] = self.raw_to_rad(name, raw)
        return self.set_angles(target)

    def write_shorts_fast(self, register: str, vendor_values) -> bool:
        return self.write_shorts(register, vendor_values)


def _arm_command(backend: RosWebHardware, command: dict,
                 combo: RosComboController) -> dict:
    name = str(command.get("cmd") or "")
    state = backend.device_state()
    moving = name in ("angles", "home", "goto_connect_pose",
                      "goto_tracking_ready", "tracking_begin",
                      "tracking_angles", "combo_prepare", "combo_start")
    if moving and not state.get("enabled", False):
        return {"type": "error", "cmd": name,
                "token": command.get("token"),
                "msg": "臂未使能,运动指令被拒"}
    if moving and state.get("frozen", False):
        return {"type": "error", "cmd": name,
                "token": command.get("token"),
                "msg": "急停生效中,运动指令被拒"}
    if name.startswith("combo_"):
        return combo.handle(command)
    if combo.active and name in ("angles", "home", "goto_connect_pose",
                                 "goto_tracking_ready", "tracking_begin",
                                 "tracking_angles"):
        return {"type": "error", "cmd": name,
                "msg": "联合回放进行中,先停止回放"}
    try:
        if name == "angles":
            ok, message = backend.set_positions(list(command.get("rad") or []))
        elif name == "home":
            ok, message = backend.set_positions(list(NERO_HOME_POSE))
        elif name == "goto_connect_pose":
            if backend.connect_pose is None:
                return {"type": "error", "cmd": name, "msg": "没有记录接入位姿"}
            ok, message = backend.set_positions(list(backend.connect_pose))
        elif name == "goto_tracking_ready":
            ok, message = backend.set_positions(list(NERO_TRACKING_READY_POSE))
        elif name == "speed":
            ok, message = backend.set_int("arm_speed", int(command.get("value", 20)))
        elif name == "enable":
            ok, message = backend.set_enabled(True)
        elif name == "disable":
            ok, message = backend.set_enabled(False)
        elif name == "estop":
            ok, message = backend.trigger("arm_estop")
        elif name == "reset":
            ok, message = backend.trigger("arm_reset")
        elif name == "tracking_begin":
            ok, message = backend.trigger("arm_tracking_begin")
        elif name == "tracking_angles":
            ok, message = backend.set_tracking_positions(
                list(command.get("rad") or []))
        elif name == "tracking_end":
            ok, message = backend.trigger("arm_tracking_end")
        else:
            return {"type": "error", "cmd": name, "msg": f"未知指令: {name}"}
    except (RosWebError, TypeError, ValueError) as exc:
        if name == "tracking_angles":
            return {
                "type": "ack", "cmd": name, "ok": False,
                "tracking_token": command.get("tracking_token"),
                "frame_id": command.get("frame_id"), "msg": str(exc),
            }
        return {"type": "error", "cmd": name, "msg": str(exc)}
    result = {"type": "ack", "cmd": name, "ok": ok,
              "msg": message or None}
    if name == "tracking_angles":
        result.update({
            "tracking_token": command.get("tracking_token"),
            "frame_id": command.get("frame_id"),
        })
    if name == "estop":
        result["warn"] = (
            "急停已下发:机械臂无抱闸，可能缓慢下落，注意下方净空")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("arm", "hand"), required=True)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--service-timeout", type=float, default=3.0)
    parser.add_argument("--startup-timeout", type=float, default=5.0)
    parser.add_argument("--allow-mock-driver", action="store_true",
                        help="test only: accept a mock ROS2 Hardware Driver")
    args = parser.parse_args()

    rclpy.init()
    node = RosWebHardware(
        args.device,
        hz=args.hz,
        service_timeout=args.service_timeout,
        startup_timeout=args.startup_timeout,
        speed=args.speed,
        allow_mock_driver=args.allow_mock_driver,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    stdin_lines = StdinLines()
    sequences = load_default_actions() if args.device == "hand" else []
    hand = RosHandAdapter(node) if args.device == "hand" else None
    combo = RosComboController(node, emit) if args.device == "arm" else None
    node.combo_controller = combo
    player_interval = 1.0 / 200.0
    next_player_tick = time.monotonic()
    combo_interval = 1.0 / 200.0
    next_combo_tick = time.monotonic()
    try:
        while rclpy.ok():
            player = hand_console._player if hand is not None else None
            timeout = 0.1
            if player is not None:
                timeout = max(0.0, min(timeout, next_player_tick - time.monotonic()))
            if combo is not None and combo.active:
                timeout = max(0.0, min(timeout, next_combo_tick - time.monotonic()))
            for line in stdin_lines.poll(timeout):
                try:
                    command = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if command.get("cmd") == "quit":
                    if (hand is not None and command.get("home", True)
                            and node._ready_emitted):
                        hand.set_angles([HAND_LIMITS[name][0]
                                         for name in HAND_JOINTS])
                    raise KeyboardInterrupt
                if not node._ready_emitted:
                    emit({"type": "error", "cmd": command.get("cmd"),
                          "msg": "ROS2 Hardware Driver 尚未就绪"})
                    continue
                if hand is not None:
                    service_started_ns = time.perf_counter_ns()
                    event = hand_console.handle(hand, command, sequences)
                    if command.get("cmd") == "angles":
                        service_completed_ns = time.perf_counter_ns()
                        metadata = command.get("_perf") or {}
                        try:
                            enqueued_ns = int(
                                metadata.get("enqueued_ns") or service_started_ns)
                        except (TypeError, ValueError):
                            enqueued_ns = service_started_ns
                        event["perf"] = {
                            "id": metadata.get("id", command.get("perf_id")),
                            "ack_token": metadata.get("ack_token"),
                            "source": metadata.get("source"),
                            "stdin_queue_ms": round(
                                (service_started_ns - enqueued_ns) / 1e6, 2),
                            # Includes ROS scheduling and the Driver's RS485 write.
                            "serial_ms": round(
                                (service_completed_ns - service_started_ns) / 1e6, 2),
                            "enqueue_to_serial_ms": round(
                                (service_completed_ns - enqueued_ns) / 1e6, 2),
                        }
                else:
                    assert combo is not None
                    if command.get("cmd") in ("disable", "estop", "reset"):
                        combo.abort(f"{command.get('cmd')} 中止联合回放")
                    event = _arm_command(node, command, combo)
                emit(event)
            if stdin_lines.eof:
                break
            player = hand_console._player if hand is not None else None
            now = time.monotonic()
            if player is not None and now >= next_player_tick:
                next_player_tick = now + player_interval
                event = player.tick()
                if event:
                    emit(event)
                if player.done:
                    emit({"type": "action_done", "slot": player.seq.slot,
                          "index": player.seq.index, "name": player.seq.name,
                          **player.summary()})
                    hand_console._player = None
            if combo is not None and now >= next_combo_tick:
                next_combo_tick = now + combo_interval
                combo.tick()
    except KeyboardInterrupt:
        pass
    finally:
        if combo is not None:
            combo.abort("ROS Web worker 正在退出", notify=False)
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        emit({"type": "closed", "backend": "ros"})


if __name__ == "__main__":
    main()
