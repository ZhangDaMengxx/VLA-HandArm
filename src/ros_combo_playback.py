"""ROS-backed arm side of Web combo playback.

This module deliberately has no rclpy imports so the timing and state machine
can be tested without a ROS installation.  The backend object is the small
service surface implemented by :mod:`ros_web_hardware`.
"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable

from combo_player import ArmTrajPack, ArmWaypoint, ComboPlayer
from nero_arm import ARM_JOINTS, NERO_ARM_LIMITS


class RosArmAdapter:
    """Expose the arm methods used by ``ComboPlayer`` through ROS services."""

    mock = False

    def __init__(self, backend) -> None:
        self.backend = backend
        self.last_error: str | None = None
        self.cpv_active = False

    @property
    def enabled(self) -> bool:
        return bool(self.backend.device_state().get("enabled", False))

    @property
    def frozen(self) -> bool:
        return bool(self.backend.device_state().get("frozen", False))

    @property
    def speed_percent(self) -> int:
        return int(self.backend.speed)

    def set_speed_percent(self, value: int) -> None:
        ok, message = self.backend.set_int("arm_speed", int(value))
        if not ok:
            raise RuntimeError(message or "ROS2 Driver 拒绝设置机械臂速度")

    def move_j(self, values) -> bool:
        return self._move(self.backend.set_positions, values)

    def move_cpv_pos(self, values) -> bool:
        return self._move(self.backend.set_tracking_positions, values)

    def _move(self, method, values) -> bool:
        try:
            ok, message = method(list(values))
        except Exception as exc:  # ROS errors are normalized for ComboPlayer.
            ok, message = False, str(exc)
        self.last_error = None if ok else (message or "ROS2 Driver 拒绝目标")
        return bool(ok)

    def cpv_begin(self) -> bool:
        try:
            ok, message = self.backend.trigger("arm_tracking_begin")
        except Exception as exc:
            ok, message = False, str(exc)
        self.cpv_active = bool(ok)
        self.last_error = None if ok else (message or "进入 CPV 失败")
        return bool(ok)

    def cpv_end(self) -> bool:
        if not self.cpv_active:
            return True
        try:
            ok, message = self.backend.trigger("arm_tracking_end")
        except Exception as exc:
            ok, message = False, str(exc)
        if ok:
            self.cpv_active = False
        self.last_error = None if ok else (message or "退出 CPV 失败")
        return bool(ok)

    def read_angles(self) -> list[float]:
        return list(self.backend.positions() or [])

    @staticmethod
    def read_ctrl_mode() -> str:
        # The Hardware Driver's acknowledged services are the command gate.
        return "CAN_CTRL"

    @staticmethod
    def velocity_is_real() -> bool:
        return True


class RosComboController:
    """Prepare, synchronize and play the arm half of a combo pack."""

    def __init__(self, backend, emit: Callable[[dict], None]) -> None:
        self.backend = backend
        self.arm = RosArmAdapter(backend)
        self.emit = emit
        self.player: ComboPlayer | None = None
        self.phase = "idle"
        self.token: str | None = None
        self._lock = threading.RLock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self.player is not None

    def handle(self, command: dict) -> dict:
        name = str(command.get("cmd") or "")
        if name == "combo_prepare":
            return self._prepare(command)
        if name == "combo_start":
            return self._start(command)
        if name in ("combo_pause", "combo_resume", "combo_stop"):
            return self._control(name, command)
        return {"type": "error", "cmd": name, "msg": f"未知联合回放指令: {name}"}

    def _prepare(self, command: dict) -> dict:
        token = str(command.get("token") or "")
        base = {"cmd": "combo_prepare", "token": token}
        if not token:
            return {"type": "error", **base, "msg": "token 为空"}
        with self._lock:
            if self.player is not None:
                return {"type": "error", **base,
                        "msg": "已有联合回放,先发 combo_stop"}
        try:
            points = self._parse_waypoints(command.get("waypoints") or [])
        except (KeyError, TypeError, ValueError) as exc:
            return {"type": "error", **base, "msg": f"waypoints 不合法: {exc}"}
        mode = "stream" if command.get("mode") == "stream" else "waypoints"
        pack = ArmTrajPack(
            name=str(command.get("name") or "联合回放"),
            mode=mode,
            waypoints=points,
            approach_rad=list(points[0].rad),
        )
        player = ComboPlayer(pack, self.arm, None, None,
                             skip_arm=(mode == "stream"))
        bad = player.preflight()
        if bad:
            return {"type": "error", **base,
                    "msg": "preflight 不通过: " + "; ".join(bad)}
        ok, message = player.begin_approach()
        if not ok:
            return {"type": "error", **base,
                    "msg": f"approach 失败: {message}"}
        with self._lock:
            self.player = player
            self.phase = "approaching"
            self.token = token
        return {
            "type": "ack", **base, "ok": True, "name": pack.name,
            "waypoints": len(points), "mode": mode,
            "duration_s": round(pack.dur_ns / 1e9, 2), "phase": self.phase,
        }

    @staticmethod
    def _parse_waypoints(raw: list) -> list[ArmWaypoint]:
        if not raw:
            raise ValueError("waypoints 为空")
        points: list[ArmWaypoint] = []
        previous_t = -1
        for index, item in enumerate(raw):
            values = [float(value) for value in item["rad"]]
            t_ns = int(item["t_ns"])
            if len(values) != len(ARM_JOINTS):
                raise ValueError(
                    f"waypoint[{index}] 需要 {len(ARM_JOINTS)} 个角,收到 {len(values)}")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"waypoint[{index}] 含非有限值")
            if t_ns < 0 or t_ns <= previous_t:
                raise ValueError(f"waypoint[{index}] t_ns={t_ns} 不是严格递增")
            for joint, value in enumerate(values):
                lo, hi = NERO_ARM_LIMITS[joint]
                if value < lo - 1e-4 or value > hi + 1e-4:
                    raise ValueError(
                        f"waypoint[{index}] {ARM_JOINTS[joint]}={value:.4f} 超限位")
            points.append(ArmWaypoint(t_ns=t_ns, rad=values))
            previous_t = t_ns
        return points

    def _start(self, command: dict) -> dict:
        token = str(command.get("token") or "")
        with self._lock:
            player = self.player
            if player is None or self.phase != "ready":
                return {"type": "error", "cmd": "combo_start", "token": token,
                        "msg": "机械臂尚未完成 approach"}
            if token != self.token:
                return {"type": "error", "cmd": "combo_start", "token": token,
                        "msg": "combo token 不匹配"}
            try:
                start_at = float(command.get("start_at") or 0.0)
            except (TypeError, ValueError):
                start_at = 0.0
            if start_at <= time.monotonic():
                return {"type": "error", "cmd": "combo_start", "token": token,
                        "msg": "start_at 必须是未来时刻"}
            player.start(start_at=start_at)
            self.phase = "playing"
            return {"type": "ack", "cmd": "combo_start", "ok": True,
                    "token": token, "name": player.pack.name,
                    "start_at": start_at}

    def _control(self, name: str, command: dict) -> dict:
        with self._lock:
            player = self.player
            if player is None:
                return {"type": "error", "cmd": name, "msg": "没有在回放"}
            requested_token = str(command.get("token") or "")
            if requested_token and requested_token != self.token:
                return {"type": "error", "cmd": name,
                        "token": requested_token, "msg": "combo token 不匹配"}
            if name == "combo_stop":
                player.stop()
                result = {"type": "ack", "cmd": name, "ok": True,
                          "token": self.token, "stopped": True,
                          "name": player.pack.name}
                self._finish_locked(stopped=True)
                return result
            if self.phase != "playing":
                return {"type": "error", "cmd": name,
                        "msg": "approach 阶段只能停止,不能暂停或恢复"}
            if name == "combo_pause":
                player.pause()
            else:
                player.resume()
            return {"type": "ack", "cmd": name, "ok": True,
                    "token": self.token, "paused": player.paused}

    def tick(self) -> None:
        with self._lock:
            player = self.player
            if player is None:
                return
            state = self.backend.device_state()
            if (state.get("state") != "READY"
                    or not self.backend.driver_accepts_commands()
                    or not state.get("enabled", False)
                    or state.get("frozen", False)):
                self._fail_locked("回放期间机械臂失能、急停或 Driver 断线")
                return
            if self.phase == "approaching":
                phase, detail = player.poll_approach(
                    current=self.backend.positions(), now=time.monotonic())
                if phase == "failed":
                    self._fail_locked(detail)
                elif phase == "ready":
                    if not self.arm.cpv_begin():
                        self._fail_locked(f"进入 CPV 失败: {self.arm.last_error}")
                    else:
                        self.phase = "ready"
                        self.emit({"type": "combo_ready", "name": player.pack.name,
                                   "token": self.token, "detail": detail})
                return
            if self.phase != "playing":
                return
            event = player.tick()
            if event and event.get("arm_ok") is False:
                self._fail_locked(f"ROS2 Driver 拒绝轨迹帧: {self.arm.last_error}")
                return
            if player.done:
                self._finish_locked(stopped=player.stopped)

    def status(self) -> dict | None:
        with self._lock:
            player = self.player
            if player is None:
                return None
            playing = self.phase == "playing"
            return {
                "name": player.pack.name, "phase": self.phase,
                "progress": round(player.progress(), 4) if playing else 0.0,
                "elapsed_ms": max(0, player.elapsed_ns // 1_000_000) if playing else 0,
                "total_ms": player.total_ns // 1_000_000,
                "paused": player.paused if playing else False,
                "i": player.i_arm, "n": len(player.pack.waypoints),
                "fail": player.fail_arm,
            }

    def abort(self, reason: str, *, notify: bool = True) -> None:
        with self._lock:
            if self.player is None:
                return
            if notify:
                self._fail_locked(reason)
            else:
                self._clear_locked()

    def _finish_locked(self, *, stopped: bool) -> None:
        player = self.player
        if player is None:
            return
        name, token = player.pack.name, self.token
        sent, failed = player.sent_arm, player.fail_arm
        end_ok = self.arm.cpv_end()
        self._clear_locked(end_cpv=False)
        self.emit({"type": "combo_done", "name": name, "token": token,
                   "stopped": stopped, "sent": sent, "fail": failed,
                   "tracking_end_ok": end_ok})

    def _fail_locked(self, message: str) -> None:
        player = self.player
        name = player.pack.name if player is not None else None
        token = self.token
        self._clear_locked()
        self.emit({"type": "combo_failed", "name": name, "token": token,
                   "msg": message})

    def _clear_locked(self, *, end_cpv: bool = True) -> None:
        player = self.player
        if player is not None:
            player.cancel_approach()
        if end_cpv:
            self.arm.cpv_end()
        self.player = None
        self.phase = "idle"
        self.token = None
