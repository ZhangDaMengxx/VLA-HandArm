#!/usr/bin/env python3
"""通用灵巧手可行域探测器：资产标称角 + raw 真机边界 + 可审计报告。

默认使用 Mock。真机只读预检需要 ``--adapter inspire --hardware --phase preflight``；
任何运动阶段还必须提供 ``--allow-motion CONFIRM_HAND_MOTION``。

示例：
  python3 src/hand_feasibility.py --phase all
  python3 src/hand_feasibility.py --adapter inspire --hardware --phase preflight
  python3 src/hand_feasibility.py --adapter inspire --hardware --phase single \
    --joint right_index_1_joint --allow-motion CONFIRM_HAND_MOTION

设计边界：
  * URDF/datasheet 定义跨设备共享的资产标称 rad，不在这里人工拟合物理角。
  * 真机探测始终在 raw/归一量 u 域执行，输出条件化安全证据。
  * STATUS 不作为到位/接触判据；使用位置误差、稳态力、错误位和温度组合判断。
  * VLA 可以消费 profile 做残差学习，但最终动作仍必须经过确定性安全投影。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable


SCHEMA_SPEC = "hand_model_spec/1"
SCHEMA_PROFILE = "hand_feasibility_profile/1"
MOTION_CONFIRMATION = "CONFIRM_HAND_MOTION"
REPO = Path(__file__).resolve().parents[1]
DEFAULT_SPEC = REPO / "configs/hands/inspire_rh56dfx_right.json"
DEFAULT_REPORT_DIR = REPO / "reports/hand_feasibility"


class SpecError(ValueError):
    """资产规范无效或与 URDF 冲突。"""


class ProbeError(RuntimeError):
    """探测流程无法继续。"""


class SafetyAbort(ProbeError):
    """触发硬安全中止：尝试软冻结，但不自动追加回张开动作。"""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(value, f, ensure_ascii=False, indent=2, sort_keys=True)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_name, path)
    except BaseException:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
        raise


def _need_number(raw: dict[str, Any], key: str, *, lower: float | None = None) -> float:
    value = raw.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SpecError(f"{key} 必须是有限数值")
    value = float(value)
    if lower is not None and value < lower:
        raise SpecError(f"{key} 必须 >= {lower}")
    return value


@dataclass(frozen=True)
class JointSpec:
    name: str
    raw_open: int
    raw_closed: int
    model_lower_rad: float
    model_upper_rad: float
    nominal_source: str

    def raw_from_u(self, u: float) -> int:
        u = _clamp(float(u), 0.0, 1.0)
        return int(round(self.raw_open + u * (self.raw_closed - self.raw_open)))

    def u_from_raw(self, raw: float) -> float:
        span = self.raw_closed - self.raw_open
        if span == 0:
            raise SpecError(f"{self.name} raw_open 与 raw_closed 不能相等")
        return _clamp((float(raw) - self.raw_open) / span, 0.0, 1.0)

    def nominal_rad_from_u(self, u: float) -> float:
        u = _clamp(float(u), 0.0, 1.0)
        return self.model_lower_rad + u * (self.model_upper_rad - self.model_lower_rad)


@dataclass(frozen=True)
class InteractionSpec:
    id: str
    description: str
    conditioning_joints: tuple[str, ...]
    condition_operator: str
    condition_levels: tuple[float, ...]
    probe_joint: str


@dataclass(frozen=True)
class ProbePolicy:
    speed: int = 15
    force: int = 250
    single_step_u: float = 0.1
    interaction_step_u: float = 0.1
    boundary_resolution_u: float = 0.01
    min_settle_s: float = 2.0
    settle_timeout_s: float = 6.0
    sample_interval_s: float = 0.08
    stable_samples: int = 2
    stable_delta_raw: int = 5
    tracking_tolerance_raw: int = 60
    contact_force_abs: int | None = 200
    max_current_abs: int | None = 2000
    max_temp_c: int = 55
    max_missing_samples: int = 3
    stall_error_mask: int = 1
    fatal_error_mask: int = 254
    preflight_samples: int = 5

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "ProbePolicy":
        known = set(cls.__dataclass_fields__)
        unknown = set(raw) - known
        if unknown:
            raise SpecError(f"probe_policy 未知字段: {sorted(unknown)}")
        values: dict[str, Any] = {}
        for name, fld in cls.__dataclass_fields__.items():
            value = raw.get(name, fld.default)
            if name in {"contact_force_abs", "max_current_abs"} and value is None:
                values[name] = None
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise SpecError(f"probe_policy.{name} 必须是数值")
            values[name] = int(value) if name not in {
                "single_step_u", "interaction_step_u", "boundary_resolution_u",
                "min_settle_s", "settle_timeout_s", "sample_interval_s",
            } else float(value)
        policy = cls(**values)
        for name in ("single_step_u", "interaction_step_u", "boundary_resolution_u"):
            if not 0 < getattr(policy, name) <= 1:
                raise SpecError(f"probe_policy.{name} 必须在 (0,1] 内")
        if policy.stable_samples < 1 or policy.max_missing_samples < 0:
            raise SpecError("stable_samples 必须 >=1，max_missing_samples 必须 >=0")
        if (policy.min_settle_s < 0 or policy.settle_timeout_s <= policy.min_settle_s
                or policy.sample_interval_s < 0):
            raise SpecError(
                "min_settle_s 必须 >=0 且小于 settle_timeout_s；sample_interval_s 必须 >=0")
        return policy


@dataclass(frozen=True)
class HandModelSpec:
    path: Path
    model_id: str
    display_name: str
    adapter: str
    asset_urdf: Path
    asset_revision: str
    asset_sha256: str
    angle_semantics: str
    source_priority: tuple[str, ...]
    joints: tuple[JointSpec, ...]
    interactions: tuple[InteractionSpec, ...]
    policy: ProbePolicy
    mock_constraints: tuple[dict[str, Any], ...] = field(default_factory=tuple)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.name for j in self.joints)

    @property
    def joint_index(self) -> dict[str, int]:
        return {name: i for i, name in enumerate(self.joint_names)}

    def joint(self, name: str) -> JointSpec:
        try:
            return self.joints[self.joint_index[name]]
        except KeyError as exc:
            raise SpecError(f"未知关节 {name!r}") from exc

    def raw_from_targets(self, targets_u: dict[str, float]) -> list[int]:
        unknown = set(targets_u) - set(self.joint_names)
        if unknown:
            raise SpecError(f"目标含未知关节: {sorted(unknown)}")
        return [j.raw_from_u(targets_u.get(j.name, 0.0)) for j in self.joints]

    def u_from_raw(self, raw: Iterable[float]) -> list[float]:
        values = list(raw)
        if len(values) != len(self.joints):
            raise SpecError(f"raw 需要 {len(self.joints)} 项，收到 {len(values)}")
        return [j.u_from_raw(value) for j, value in zip(self.joints, values)]

    @classmethod
    def load(cls, path: str | Path) -> "HandModelSpec":
        path = Path(path).resolve()
        raw = json.loads(path.read_text(encoding="utf-8"))
        if raw.get("schema_version") != SCHEMA_SPEC:
            raise SpecError(f"schema_version 必须是 {SCHEMA_SPEC}")
        asset = raw.get("asset")
        if not isinstance(asset, dict):
            raise SpecError("asset 必须是对象")
        urdf_value = asset.get("urdf")
        if not isinstance(urdf_value, str) or not urdf_value:
            raise SpecError("asset.urdf 必须是相对规范文件的路径")
        urdf = (path.parent / urdf_value).resolve()
        if not urdf.is_file():
            raise SpecError(f"URDF 不存在: {urdf}")
        limits = _urdf_joint_limits(urdf)

        joints_raw = raw.get("joints")
        if not isinstance(joints_raw, list) or not joints_raw:
            raise SpecError("joints 必须是非空数组")
        joints: list[JointSpec] = []
        seen: set[str] = set()
        for entry in joints_raw:
            if not isinstance(entry, dict):
                raise SpecError("joints 每项必须是对象")
            name = entry.get("name")
            if not isinstance(name, str) or not name or name in seen:
                raise SpecError(f"关节名无效或重复: {name!r}")
            seen.add(name)
            if name not in limits:
                raise SpecError(f"URDF 中没有带 limit 的关节 {name}")
            raw_open = entry.get("raw_open")
            raw_closed = entry.get("raw_closed")
            if isinstance(raw_open, bool) or not isinstance(raw_open, int):
                raise SpecError(f"{name}.raw_open 必须是整数")
            if isinstance(raw_closed, bool) or not isinstance(raw_closed, int):
                raise SpecError(f"{name}.raw_closed 必须是整数")
            if raw_open == raw_closed:
                raise SpecError(f"{name} raw_open 与 raw_closed 不能相等")
            lo, hi = limits[name]
            nominal_source = entry.get("nominal_source", "vendor_urdf")
            if not isinstance(nominal_source, str) or not nominal_source:
                raise SpecError(f"{name}.nominal_source 必须是非空字符串")
            nominal_range = entry.get("nominal_range_rad")
            if nominal_range is not None:
                if (not isinstance(nominal_range, list) or len(nominal_range) != 2
                        or any(isinstance(v, bool) or not isinstance(v, (int, float))
                               for v in nominal_range)):
                    raise SpecError(f"{name}.nominal_range_rad 必须是两个数值")
                lo, hi = float(nominal_range[0]), float(nominal_range[1])
                if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
                    raise SpecError(f"{name}.nominal_range_rad 必须是递增有限区间")
            elif nominal_source != "vendor_urdf":
                raise SpecError(
                    f"{name} nominal_source={nominal_source!r} 时必须显式给 nominal_range_rad")
            joints.append(JointSpec(
                name, raw_open, raw_closed, lo, hi, str(nominal_source)))

        names = {j.name for j in joints}
        interactions: list[InteractionSpec] = []
        interaction_ids: set[str] = set()
        for entry in raw.get("interactions", []):
            if not isinstance(entry, dict):
                raise SpecError("interactions 每项必须是对象")
            iid = entry.get("id")
            if not isinstance(iid, str) or not iid or iid in interaction_ids:
                raise SpecError(f"interaction id 无效或重复: {iid!r}")
            interaction_ids.add(iid)
            conditions = entry.get("conditioning_joints")
            operator = entry.get("condition_operator", "min")
            levels = entry.get("condition_levels")
            probe = entry.get("probe_joint")
            if not isinstance(conditions, list) or not conditions:
                raise SpecError(f"{iid}.conditioning_joints 必须非空")
            if not isinstance(levels, list) or not levels:
                raise SpecError(f"{iid}.condition_levels 必须非空")
            if any(name not in names for name in conditions) or probe not in names:
                raise SpecError(f"{iid} 引用了未知关节")
            if probe in conditions:
                raise SpecError(f"{iid} probe_joint 不能同时是 conditioning_joint")
            if operator not in {"min", "max", "mean"}:
                raise SpecError(f"{iid}.condition_operator 只能是 min/max/mean")
            level_values = tuple(float(v) for v in levels)
            if any(not math.isfinite(v) or v < 0 or v > 1 for v in level_values):
                raise SpecError(f"{iid}.condition_levels 必须在 [0,1]")
            interactions.append(InteractionSpec(
                id=iid,
                description=str(entry.get("description", "")),
                conditioning_joints=tuple(str(v) for v in conditions),
                condition_operator=str(operator),
                condition_levels=level_values,
                probe_joint=str(probe),
            ))

        source_priority = asset.get("source_priority", [])
        if not isinstance(source_priority, list) or not all(isinstance(v, str) for v in source_priority):
            raise SpecError("asset.source_priority 必须是字符串数组")
        model_id = raw.get("model_id")
        adapter_name = raw.get("adapter")
        if not isinstance(model_id, str) or not model_id:
            raise SpecError("model_id 必须是非空字符串")
        if not isinstance(adapter_name, str) or not adapter_name:
            raise SpecError("adapter 必须是非空字符串")
        return cls(
            path=path,
            model_id=model_id,
            display_name=str(raw.get("display_name", raw.get("model_id", ""))),
            adapter=adapter_name,
            asset_urdf=urdf,
            asset_revision=str(asset.get("revision", "unknown")),
            asset_sha256=_sha256(urdf),
            angle_semantics=str(asset.get("angle_semantics", "asset_nominal_rad")),
            source_priority=tuple(source_priority),
            joints=tuple(joints),
            interactions=tuple(interactions),
            policy=ProbePolicy.from_dict(raw.get("probe_policy", {})),
            mock_constraints=tuple(raw.get("mock_constraints", [])),
        )


def _urdf_joint_limits(path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    out: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        name = joint.get("name")
        if not name or limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        if lower is None or upper is None:
            continue
        out[name] = (float(lower), float(upper))
    return out


@dataclass
class Observation:
    angle_raw: list[int] | None
    force: list[int] | None
    current: list[int] | None
    temp: list[int] | None
    error: list[int] | None
    status: list[int] | None
    timestamp_s: float = field(default_factory=time.monotonic)


class HandProbeAdapter(ABC):
    """厂商适配边界。所有数组均使用 HandModelSpec 的项目关节顺序。"""

    def __init__(self, spec: HandModelSpec) -> None:
        self.spec = spec

    @abstractmethod
    def connect(self, *, read_only: bool) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def configure(self, *, speed: int, force: int) -> None: ...

    @abstractmethod
    def command_raw(self, project_raw: list[int]) -> None: ...

    @abstractmethod
    def observe(self) -> Observation: ...

    @abstractmethod
    def freeze(self) -> bool: ...

    @abstractmethod
    def identity(self) -> dict[str, Any]: ...


class MockHandAdapter(HandProbeAdapter):
    """确定性 Mock：执行 raw 命令，并按规范中的声明式约束模拟自碰撞。"""

    def __init__(self, spec: HandModelSpec, *, temperature_c: int = 30,
                 missing_observations: int = 0) -> None:
        super().__init__(spec)
        self.raw = spec.raw_from_targets({})
        self.target = list(self.raw)
        self.force = [0] * len(spec.joints)
        self.current = [0] * len(spec.joints)
        self.error = [0] * len(spec.joints)
        self.temperature_c = int(temperature_c)
        self.missing_observations = int(missing_observations)
        self.connected = False
        self.speed = spec.policy.speed
        self.force_set = spec.policy.force

    def connect(self, *, read_only: bool) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def configure(self, *, speed: int, force: int) -> None:
        self.speed, self.force_set = int(speed), int(force)

    def command_raw(self, project_raw: list[int]) -> None:
        if not self.connected:
            raise ProbeError("Mock adapter 未连接")
        if len(project_raw) != len(self.spec.joints):
            raise ProbeError("raw 命令长度不匹配")
        self.target = [int(v) for v in project_raw]
        desired_u = self.spec.u_from_raw(self.target)
        actual_u = list(desired_u)
        self.force = [0] * len(actual_u)
        self.current = [0] * len(actual_u)
        self.error = [0] * len(actual_u)
        index = self.spec.joint_index
        for constraint in self.spec.mock_constraints:
            when = constraint.get("when_all_u_at_least", {})
            if not isinstance(when, dict):
                continue
            if not all(desired_u[index[name]] >= float(limit) for name, limit in when.items()):
                continue
            joint = str(constraint.get("joint"))
            if joint not in index:
                continue
            i = index[joint]
            max_u = float(constraint.get("max_u", 1.0))
            if desired_u[i] > max_u:
                actual_u[i] = max_u
                self.force[i] = int(constraint.get("force_abs", 300))
                self.current[i] = int(constraint.get("current_abs", 300))
                self.error[i] |= self.spec.policy.stall_error_mask
        self.raw = [j.raw_from_u(u) for j, u in zip(self.spec.joints, actual_u)]

    def observe(self) -> Observation:
        if self.missing_observations > 0:
            self.missing_observations -= 1
            return Observation(None, None, None, None, None, None)
        n = len(self.spec.joints)
        return Observation(
            angle_raw=list(self.raw), force=list(self.force), current=list(self.current),
            temp=[self.temperature_c] * n, error=list(self.error), status=[2] * n,
        )

    def freeze(self) -> bool:
        self.target = list(self.raw)
        self.force = [0] * len(self.raw)
        self.current = [0] * len(self.raw)
        return True

    def identity(self) -> dict[str, Any]:
        return {"adapter": "mock", "model_id": self.spec.model_id, "mock": True}


class InspireRH56Adapter(HandProbeAdapter):
    """RH56DFX Adapter。绕过 rad 映射，直接以项目序 raw 探测。"""

    def __init__(self, spec: HandModelSpec, *, port: str, baudrate: int = 115200,
                 device_id: str | None = None, firmware: str | None = None) -> None:
        super().__init__(spec)
        self.port = port
        self.baudrate = int(baudrate)
        self.device_id = device_id
        self.firmware = firmware
        self.hand = None

    def connect(self, *, read_only: bool) -> None:
        from inspire_hand import (HAND_JOINTS, InspireHand, InspireHandConfig)

        if tuple(HAND_JOINTS) != self.spec.joint_names:
            raise SpecError("资产规范关节顺序与 Inspire 驱动不一致，拒绝连接")
        cfg = InspireHandConfig(
            port=self.port, baudrate=self.baudrate, mock=False,
            init_speed=self.spec.policy.speed, init_force=self.spec.policy.force,
            initialize_runtime=not read_only,
        )
        self.hand = InspireHand(cfg)
        if not self.hand.connect():
            raise ProbeError("InspireHand.connect() 返回 False")

    def close(self) -> None:
        if self.hand is not None:
            self.hand.disconnect()
        self.hand = None

    def configure(self, *, speed: int, force: int) -> None:
        if self.hand is None:
            raise ProbeError("Inspire adapter 未连接")
        if not self.hand.set_speed(int(speed)):
            raise ProbeError("写 SPEED_SET 失败")
        if not self.hand.set_force(int(force)):
            raise ProbeError("写 FORCE_SET 失败")

    def command_raw(self, project_raw: list[int]) -> None:
        from inspire_hand import PROJECT_TO_VENDOR

        if self.hand is None:
            raise ProbeError("Inspire adapter 未连接")
        if len(project_raw) != len(PROJECT_TO_VENDOR):
            raise ProbeError("raw 命令必须是 6 项")
        vendor = [0] * len(PROJECT_TO_VENDOR)
        for project_i, vendor_i in enumerate(PROJECT_TO_VENDOR):
            vendor[vendor_i] = int(project_raw[project_i])
        if not self.hand.write_shorts("ANGLE_SET", vendor):
            raise ProbeError("写 ANGLE_SET 失败")

    def observe(self) -> Observation:
        if self.hand is None:
            raise ProbeError("Inspire adapter 未连接")
        data = self.hand.telemetry()
        return Observation(
            angle_raw=data.get("angle_act"), force=data.get("force_act"),
            current=data.get("current"), temp=data.get("temp"),
            error=data.get("error"), status=data.get("status"),
        )

    def freeze(self) -> bool:
        """把当前 ANGLE_ACT 原样写回 ANGLE_SET，消除位置误差后再断串口。"""
        if self.hand is None:
            return False
        from inspire_hand import PROJECT_TO_VENDOR

        values = self.hand.read_regs("ANGLE_ACT", 12, "6h")
        if values is None:
            return False
        project_raw = [int(values[PROJECT_TO_VENDOR[i]])
                       for i in range(len(PROJECT_TO_VENDOR))]
        self.command_raw(project_raw)
        return True

    def identity(self) -> dict[str, Any]:
        result = {
            "adapter": "inspire_rh56dfx", "model_id": self.spec.model_id,
            "mock": False, "port": self.port, "baudrate": self.baudrate,
        }
        if self.device_id:
            result["device_id"] = self.device_id
        if self.firmware:
            result["firmware"] = self.firmware
        return result


class ProfileRecorder:
    """原子保存可恢复报告；每个探测点落盘，异常退出也保留证据。"""

    def __init__(self, path: Path, spec: HandModelSpec, adapter: HandProbeAdapter,
                 policy: ProbePolicy, *, resume: bool = False) -> None:
        self.path = path.resolve()
        if resume:
            self.data = json.loads(self.path.read_text(encoding="utf-8"))
            if self.data.get("schema_version") != SCHEMA_PROFILE:
                raise ProbeError("恢复文件 schema 不兼容")
            if self.data.get("model_id") != spec.model_id:
                raise ProbeError("恢复文件 model_id 与当前规范不同")
            if self.data.get("asset", {}).get("sha256") != spec.asset_sha256:
                raise ProbeError("恢复文件的资产 SHA-256 与当前 URDF 不同")
            if self.data.get("adapter") != adapter.identity():
                raise ProbeError("恢复文件的设备/Adapter 身份与本次不同")
            old = self.data.get("conditions", {})
            if old != asdict(policy):
                raise ProbeError("恢复文件的完整探测条件与本次不同")
            self.data["status"] = "building"
            self.data["resumed_at"] = _utc_now()
        else:
            self.data = {
                "schema_version": SCHEMA_PROFILE,
                "run_id": str(uuid.uuid4()),
                "status": "building",
                "created_at": _utc_now(),
                "model_id": spec.model_id,
                "asset": {
                    "urdf": str(spec.asset_urdf), "revision": spec.asset_revision,
                    "sha256": spec.asset_sha256,
                    "angle_semantics": spec.angle_semantics,
                    "source_priority": list(spec.source_priority),
                    "joint_limits_rad": {
                        j.name: {
                            "range": [j.model_lower_rad, j.model_upper_rad],
                            "source": j.nominal_source,
                        } for j in spec.joints
                    },
                },
                "adapter": adapter.identity(),
                "conditions": asdict(policy),
                "results": {"preflight": None, "single_joint": {}, "interactions": {}},
                "events": [],
            }
        self.save()

    def save(self) -> None:
        _atomic_json(self.path, self.data)

    def event(self, kind: str, **payload: Any) -> None:
        self.data["events"].append({"time": _utc_now(), "kind": kind, **payload})
        self.save()

    def set_preflight(self, result: dict[str, Any]) -> None:
        self.data["results"]["preflight"] = result
        self.save()

    def start_item(self, bucket: str, key: str, metadata: dict[str, Any]) -> None:
        current = self.data["results"][bucket].get(key)
        if current and current.get("status") == "complete":
            return
        self.data["results"][bucket][key] = {
            "status": "building", "metadata": metadata, "points": []
        }
        self.save()

    def point(self, bucket: str, key: str, value: dict[str, Any]) -> None:
        self.data["results"][bucket][key]["points"].append(value)
        self.save()

    def finish_item(self, bucket: str, key: str, summary: dict[str, Any]) -> None:
        item = self.data["results"][bucket][key]
        item["status"] = "complete"
        item["summary"] = summary
        self.save()

    def is_complete(self, bucket: str, key: str) -> bool:
        return self.data["results"][bucket].get(key, {}).get("status") == "complete"

    def complete(self) -> None:
        self.data["status"] = "complete"
        self.data["completed_at"] = _utc_now()
        self.save()

    def abort(self, reason: str) -> None:
        self.data["status"] = "aborted"
        self.data["aborted_at"] = _utc_now()
        self.data["abort_reason"] = reason
        self.save()


class FeasibilityEnvelope:
    """把归一动作投影到已完成 Profile 的确定性安全包络。"""

    def __init__(self, spec: HandModelSpec, profile: dict[str, Any], *,
                 allow_mock: bool = False) -> None:
        if profile.get("schema_version") != SCHEMA_PROFILE:
            raise ProbeError("Profile schema 不兼容")
        if profile.get("status") != "complete":
            raise ProbeError("Profile 尚未完成，不能用于运行时投影")
        asset = profile.get("asset", {})
        if asset.get("sha256") != spec.asset_sha256:
            raise ProbeError("Profile 与当前 URDF SHA-256 不一致")
        if profile.get("model_id") != spec.model_id:
            raise ProbeError("Profile model_id 与当前规范不一致")
        if profile.get("adapter", {}).get("mock") and not allow_mock:
            raise ProbeError("Mock Profile 不能用于真实运行时投影")
        self.spec = spec
        self.profile = profile

    @classmethod
    def load(cls, spec: HandModelSpec, path: str | Path, *,
             allow_mock: bool = False) -> "FeasibilityEnvelope":
        profile = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(spec, profile, allow_mock=allow_mock)

    @staticmethod
    def _condition_value(operator: str, values: list[float]) -> float:
        if operator == "min":
            return min(values)
        if operator == "max":
            return max(values)
        return sum(values) / len(values)

    @staticmethod
    def _conservative_boundary(rows: list[dict[str, Any]], condition: float) -> float:
        valid = sorted(
            (float(row["condition_level_u"]), float(row["safe_max_u"]))
            for row in rows if row.get("evaluation_status") == "complete")
        if not valid:
            raise ProbeError("interaction 没有完整边界")
        if condition <= valid[0][0]:
            return valid[0][1]
        if condition >= valid[-1][0]:
            return valid[-1][1]
        for (x0, y0), (x1, y1) in zip(valid, valid[1:]):
            if x0 <= condition <= x1:
                # 不对未知区间做乐观插值，使用两端更保守的已验证边界。
                return min(y0, y1)
        raise ProbeError("无法定位 interaction 条件区间")

    def project(self, targets_u: dict[str, float], *, margin_u: float = 0.02) -> dict[str, Any]:
        if not 0 <= margin_u < 1:
            raise ProbeError("margin_u 必须在 [0,1) 内")
        unknown = set(targets_u) - set(self.spec.joint_names)
        if unknown:
            raise ProbeError(f"目标含未知关节: {sorted(unknown)}")
        requested: dict[str, float] = {}
        for name in self.spec.joint_names:
            value = targets_u.get(name, 0.0)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProbeError(f"{name} 的目标必须是数值")
            value = float(value)
            if not math.isfinite(value):
                raise ProbeError(f"{name} 的目标必须是有限数值")
            requested[name] = _clamp(value, 0.0, 1.0)
        projected = dict(requested)
        changes: list[dict[str, Any]] = []
        single = self.profile.get("results", {}).get("single_joint", {})
        for name in self.spec.joint_names:
            item = single.get(name)
            if not item or item.get("status") != "complete":
                raise ProbeError(f"缺少 {name} 的完整单关节证据")
            summary = item.get("summary", {})
            if summary.get("evaluation_status") != "complete":
                raise ProbeError(f"{name} 单关节证据为 inconclusive")
            allowed = max(0.0, float(summary["safe_max_u"]) - margin_u)
            if projected[name] > allowed:
                changes.append({"joint": name, "reason": "single_joint",
                                "requested_u": projected[name], "allowed_u": allowed})
                projected[name] = allowed

        interactions = self.profile.get("results", {}).get("interactions", {})
        for interaction in self.spec.interactions:
            item = interactions.get(interaction.id)
            if not item or item.get("status") != "complete":
                raise ProbeError(f"缺少 {interaction.id} 的完整联合证据")
            summary = item.get("summary", {})
            if summary.get("evaluation_status") != "complete":
                raise ProbeError(f"{interaction.id} 联合证据为 inconclusive")
            values = [projected[name] for name in interaction.conditioning_joints]
            condition = self._condition_value(interaction.condition_operator, values)
            boundary = self._conservative_boundary(summary.get("boundaries", []), condition)
            allowed = max(0.0, boundary - margin_u)
            probe = interaction.probe_joint
            if projected[probe] > allowed:
                changes.append({
                    "joint": probe, "reason": f"interaction:{interaction.id}",
                    "condition_u": condition, "requested_u": projected[probe],
                    "allowed_u": allowed,
                })
                projected[probe] = allowed

        raw = self.spec.raw_from_targets(projected)
        nominal_rad = {
            joint.name: joint.nominal_rad_from_u(projected[joint.name])
            for joint in self.spec.joints
        }
        return {
            "model_id": self.spec.model_id,
            "profile_run_id": self.profile.get("run_id"),
            "conditions": self.profile.get("conditions"),
            "margin_u": margin_u,
            "requested_u": requested,
            "projected_u": projected,
            "projected_raw": dict(zip(self.spec.joint_names, raw)),
            "projected_nominal_rad": nominal_rad,
            "changed": bool(changes),
            "changes": changes,
        }


def _levels(step: float) -> list[float]:
    count = int(math.floor(1.0 / step + 1e-9))
    values = [round(i * step, 10) for i in range(1, count + 1)]
    if not values or values[-1] < 1.0 - 1e-9:
        values.append(1.0)
    else:
        values[-1] = 1.0
    return values


class FeasibilityProbe:
    def __init__(self, spec: HandModelSpec, adapter: HandProbeAdapter,
                 recorder: ProfileRecorder, *, policy: ProbePolicy | None = None,
                 sleep_fn: Callable[[float], None] = time.sleep,
                 monotonic_fn: Callable[[], float] = time.monotonic) -> None:
        self.spec = spec
        self.adapter = adapter
        self.recorder = recorder
        self.policy = policy or spec.policy
        self.sleep = sleep_fn
        self.monotonic = monotonic_fn

    def _validate_shape(self, values: list[int] | None, name: str) -> bool:
        if values is None:
            return False
        if len(values) != len(self.spec.joints):
            raise SafetyAbort(f"{name} 通道数 {len(values)} 与规范不一致")
        return True

    def _hard_fault(self, obs: Observation) -> str | None:
        for values, name in ((obs.temp, "TEMP"), (obs.error, "ERROR")):
            if values is not None and len(values) != len(self.spec.joints):
                return f"{name} 通道数与规范不一致"
        if obs.temp is not None and max(obs.temp) >= self.policy.max_temp_c:
            return f"温度达到 {max(obs.temp)}C（阈值 {self.policy.max_temp_c}C）"
        if obs.error is not None:
            fatal = [(self.spec.joint_names[i], int(value))
                     for i, value in enumerate(obs.error)
                     if int(value) & self.policy.fatal_error_mask]
            if fatal:
                return f"ERROR 命中 fatal mask: {fatal}"
        if self.policy.max_current_abs is not None and obs.current is not None:
            peak = max(abs(int(value)) for value in obs.current)
            if peak >= self.policy.max_current_abs:
                return f"电流绝对值达到 {peak}（阈值 {self.policy.max_current_abs}）"
        return None

    def preflight(self) -> dict[str, Any]:
        valid = 0
        last: Observation | None = None
        missing = 0
        for _ in range(self.policy.preflight_samples):
            obs = self.adapter.observe()
            fault = self._hard_fault(obs)
            if fault:
                raise SafetyAbort(f"预检失败: {fault}")
            critical = all(self._validate_shape(values, name) for values, name in (
                (obs.angle_raw, "ANGLE_ACT"), (obs.temp, "TEMP"), (obs.error, "ERROR")))
            if critical:
                valid += 1
                last = obs
            else:
                missing += 1
            self.sleep(self.policy.sample_interval_s)
        if valid == 0:
            raise SafetyAbort("预检没有取得一组完整 ANGLE_ACT/TEMP/ERROR")
        assert last is not None
        nonzero_errors = [(self.spec.joint_names[i], int(value))
                          for i, value in enumerate(last.error or []) if int(value) != 0]
        if nonzero_errors:
            raise SafetyAbort(f"预检 ERROR 非零，先人工诊断/清错: {nonzero_errors}")
        result = {
            "status": "pass", "time": _utc_now(), "samples": self.policy.preflight_samples,
            "valid_samples": valid, "missing_samples": missing,
            "angle_raw": last.angle_raw, "temp": last.temp, "error": last.error,
            "force": last.force, "current": last.current,
            "status_register": last.status,
            "note": "STATUS 仅记录，不用于可行性判定",
        }
        self.recorder.set_preflight(result)
        return result

    def _wait_target(self, target_raw: list[int], active: set[int], *,
                     judge_contact: bool = True) -> dict[str, Any]:
        start = self.monotonic()
        previous: list[int] | None = None
        stable = 0
        missing = 0
        samples = 0
        peak_force = 0
        peak_current = 0
        peak_temp = 0
        last: Observation | None = None
        while self.monotonic() - start <= self.policy.settle_timeout_s:
            obs = self.adapter.observe()
            samples += 1
            fault = self._hard_fault(obs)
            if fault:
                raise SafetyAbort(fault)
            if not all(self._validate_shape(values, name) for values, name in (
                    (obs.angle_raw, "ANGLE_ACT"), (obs.temp, "TEMP"), (obs.error, "ERROR"))):
                missing += 1
                if missing > self.policy.max_missing_samples:
                    raise SafetyAbort("连续遥测缺失超过上限，停止运动")
                self.sleep(self.policy.sample_interval_s)
                continue
            missing = 0
            last = obs
            assert obs.angle_raw is not None
            if obs.force is not None:
                peak_force = max(peak_force, *(abs(int(obs.force[i])) for i in active))
            if obs.current is not None:
                peak_current = max(peak_current, *(abs(int(obs.current[i])) for i in active))
            if obs.temp is not None:
                peak_temp = max(peak_temp, max(int(v) for v in obs.temp))
            if previous is not None:
                movement = max(abs(obs.angle_raw[i] - previous[i]) for i in active)
                stable = stable + 1 if movement <= self.policy.stable_delta_raw else 0
            previous = list(obs.angle_raw)
            elapsed = self.monotonic() - start
            if stable >= self.policy.stable_samples and elapsed >= self.policy.min_settle_s:
                errors = [abs(obs.angle_raw[i] - target_raw[i]) for i in active]
                tracking = max(errors, default=0)
                reasons: list[str] = []
                # 恢复动作也必须真的到位；judge_contact 只控制力/堵转是否算边界。
                if tracking > self.policy.tracking_tolerance_raw:
                    reasons.append(f"tracking_error_raw={tracking}")
                if judge_contact and self.policy.contact_force_abs is not None:
                    steady_force = max((abs(int(obs.force[i])) for i in active), default=0) \
                        if obs.force is not None else None
                    if steady_force is not None and steady_force >= self.policy.contact_force_abs:
                        reasons.append(f"steady_force_abs={steady_force}")
                if judge_contact and obs.error is not None:
                    stalled = [self.spec.joint_names[i] for i in active
                               if int(obs.error[i]) & self.policy.stall_error_mask]
                    if stalled:
                        reasons.append(f"stall_error={stalled}")
                return {
                    "verdict": "infeasible" if reasons else "feasible",
                    "reasons": reasons,
                    "actual_raw": list(obs.angle_raw),
                    "tracking_error_raw": tracking,
                    "steady_force": obs.force,
                    "peak_force_abs": peak_force,
                    "peak_current_abs": peak_current,
                    "peak_temp_c": peak_temp,
                    "error": obs.error,
                    "status_register": obs.status,
                    "samples": samples,
                    "elapsed_s": round(elapsed, 6),
                }
            self.sleep(self.policy.sample_interval_s)
        actual = last.angle_raw if last is not None else None
        return {
            "verdict": "inconclusive", "reasons": ["settle_timeout"],
            "actual_raw": actual, "peak_force_abs": peak_force,
            "peak_current_abs": peak_current, "peak_temp_c": peak_temp,
            "samples": samples, "elapsed_s": round(self.monotonic() - start, 6),
        }

    def _command_targets(self, targets_u: dict[str, float], *,
                         active_names: Iterable[str], label: str,
                         judge_contact: bool = True) -> dict[str, Any]:
        target_raw = self.spec.raw_from_targets(targets_u)
        active = {self.spec.joint_index[name] for name in active_names}
        if not active:
            active = set(range(len(self.spec.joints)))
        self.adapter.command_raw(target_raw)
        result = self._wait_target(target_raw, active, judge_contact=judge_contact)
        result.update({
            "label": label,
            "target_u": {name: round(float(value), 6) for name, value in targets_u.items()},
            "target_raw": target_raw,
        })
        return result

    def return_open(self) -> dict[str, Any]:
        return self._command_targets({}, active_names=self.spec.joint_names,
                                     label="return_open", judge_contact=False)

    def _candidate(self, fixture: dict[str, float], probe_joint: str,
                   probe_u: float, *, label: str) -> dict[str, Any]:
        home = self.return_open()
        if home["verdict"] != "feasible":
            raise SafetyAbort(f"无法确认回到张开位: {home.get('reasons')}")
        if fixture:
            fixed = self._command_targets(
                fixture, active_names=fixture, label=f"{label}:fixture", judge_contact=True)
            if fixed["verdict"] != "feasible":
                fixed["fixture_failed"] = True
                return fixed
        targets = dict(fixture)
        targets[probe_joint] = probe_u
        return self._command_targets(
            targets, active_names=set(fixture) | {probe_joint}, label=label, judge_contact=True)

    def _refine(self, fixture: dict[str, float], probe_joint: str,
                low: float, high: float, *, bucket: str, key: str,
                label_prefix: str) -> tuple[float, float]:
        while high - low > self.policy.boundary_resolution_u + 1e-12:
            mid = round((low + high) / 2.0, 10)
            point = self._candidate(fixture, probe_joint, mid,
                                    label=f"{label_prefix}:refine:{mid:.4f}")
            self.recorder.point(bucket, key, point)
            if point["verdict"] == "feasible":
                low = mid
            elif point["verdict"] == "infeasible":
                high = mid
            else:
                break
        return low, high

    def run_single(self, joint_names: Iterable[str] | None = None) -> dict[str, Any]:
        selected = list(joint_names or self.spec.joint_names)
        unknown = set(selected) - set(self.spec.joint_names)
        if unknown:
            raise ProbeError(f"未知单关节: {sorted(unknown)}")
        summaries: dict[str, Any] = {}
        for name in selected:
            if self.recorder.is_complete("single_joint", name):
                summaries[name] = self.recorder.data["results"]["single_joint"][name]["summary"]
                continue
            joint = self.spec.joint(name)
            self.recorder.start_item("single_joint", name, {
                "model_limit_rad": [joint.model_lower_rad, joint.model_upper_rad],
                "raw_open": joint.raw_open, "raw_closed": joint.raw_closed,
            })
            safe, failed = 0.0, None
            status = "complete"
            for u in _levels(self.policy.single_step_u):
                point = self._candidate({}, name, u, label=f"single:{name}:{u:.4f}")
                self.recorder.point("single_joint", name, point)
                if point["verdict"] == "feasible":
                    safe = u
                    continue
                if point["verdict"] == "infeasible":
                    failed = u
                    safe, failed = self._refine({}, name, safe, failed,
                                                bucket="single_joint", key=name,
                                                label_prefix=f"single:{name}")
                else:
                    status = "inconclusive"
                break
            summary = {
                "evaluation_status": status,
                "safe_max_u": round(safe, 6),
                "first_infeasible_u": None if failed is None else round(failed, 6),
                "safe_max_raw": joint.raw_from_u(safe),
                "safe_max_nominal_rad": round(joint.nominal_rad_from_u(safe), 9),
            }
            self.recorder.finish_item("single_joint", name, summary)
            summaries[name] = summary
        return summaries

    def run_interactions(self, ids: Iterable[str] | None = None) -> dict[str, Any]:
        by_id = {item.id: item for item in self.spec.interactions}
        selected = list(ids or by_id)
        unknown = set(selected) - set(by_id)
        if unknown:
            raise ProbeError(f"未知 interaction: {sorted(unknown)}")
        out: dict[str, Any] = {}
        for iid in selected:
            interaction = by_id[iid]
            if self.recorder.is_complete("interactions", iid):
                out[iid] = self.recorder.data["results"]["interactions"][iid]["summary"]
                continue
            self.recorder.start_item("interactions", iid, asdict(interaction))
            boundaries: list[dict[str, Any]] = []
            evaluation_status = "complete"
            for level in interaction.condition_levels:
                fixture = {name: level for name in interaction.conditioning_joints}
                safe, failed = 0.0, None
                condition_status = "complete"
                for u in _levels(self.policy.interaction_step_u):
                    point = self._candidate(
                        fixture, interaction.probe_joint, u,
                        label=f"interaction:{iid}:condition={level:.4f}:probe={u:.4f}")
                    point["condition_level_u"] = level
                    self.recorder.point("interactions", iid, point)
                    if point.get("fixture_failed"):
                        condition_status = "fixture_infeasible"
                        evaluation_status = "inconclusive"
                        break
                    if point["verdict"] == "feasible":
                        safe = u
                        continue
                    if point["verdict"] == "infeasible":
                        failed = u
                        safe, failed = self._refine(
                            fixture, interaction.probe_joint, safe, failed,
                            bucket="interactions", key=iid,
                            label_prefix=f"interaction:{iid}:condition={level:.4f}")
                    else:
                        condition_status = "inconclusive"
                        evaluation_status = "inconclusive"
                    break
                probe = self.spec.joint(interaction.probe_joint)
                boundaries.append({
                    "condition_level_u": level,
                    "condition_raw": {
                        name: self.spec.joint(name).raw_from_u(level)
                        for name in interaction.conditioning_joints
                    },
                    "probe_joint": interaction.probe_joint,
                    "safe_max_u": round(safe, 6),
                    "first_infeasible_u": None if failed is None else round(failed, 6),
                    "safe_max_raw": probe.raw_from_u(safe),
                    "safe_max_nominal_rad": round(probe.nominal_rad_from_u(safe), 9),
                    "evaluation_status": condition_status,
                })
            summary = {"evaluation_status": evaluation_status, "boundaries": boundaries}
            self.recorder.finish_item("interactions", iid, summary)
            out[iid] = summary
        return out


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    parser.add_argument("--adapter", choices=["mock", "inspire"], default="mock")
    parser.add_argument("--phase", choices=["preflight", "single", "interactions", "all"],
                        default="preflight")
    parser.add_argument("--joint", action="append", help="single 阶段只测指定关节，可重复")
    parser.add_argument("--interaction", action="append",
                        help="interactions 阶段只测指定 interaction，可重复")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--speed", type=int, help="覆盖 spec 的测试速度")
    parser.add_argument("--force", type=int, help="覆盖 spec 的测试力阈值")
    parser.add_argument("--port", default=os.environ.get("INSPIRE_HAND_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--device-id", help="测试设备序列号/资产编号，写入 Profile")
    parser.add_argument("--firmware", help="厂家固件版本，写入 Profile")
    parser.add_argument("--hardware", action="store_true",
                        help="确认使用真实硬件 Adapter；本开关本身不授权运动")
    parser.add_argument("--allow-motion", metavar="TOKEN",
                        help=f"运动阶段必须精确填写 {MOTION_CONFIRMATION}")
    parser.add_argument("--project-profile", type=Path,
                        help="不连接设备；用完成的 Profile 投影 --target-u")
    parser.add_argument("--target-u",
                        help="待投影的归一动作 JSON 对象，例如 '{\"right_index_1_joint\":1}'")
    parser.add_argument("--margin-u", type=float, default=0.02)
    parser.add_argument("--allow-mock-profile", action="store_true",
                        help="仅用于离线演示；默认拒绝 Mock Profile")
    return parser


def _policy_with_overrides(policy: ProbePolicy, args: argparse.Namespace) -> ProbePolicy:
    values: dict[str, Any] = {}
    if args.speed is not None:
        if not 0 <= args.speed <= 1000:
            raise ProbeError("--speed 必须在 0..1000")
        values["speed"] = args.speed
    if args.force is not None:
        if not 0 <= args.force <= 1000:
            raise ProbeError("--force 必须在 0..1000")
        values["force"] = args.force
    return replace(policy, **values)


def _default_output(spec: HandModelSpec, adapter_name: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_REPORT_DIR / f"{spec.model_id}_{adapter_name}_{stamp}.json"


def _load_resume_for_gate(path: Path, spec: HandModelSpec, policy: ProbePolicy,
                          adapter: HandProbeAdapter) -> dict[str, Any]:
    if not path.is_file():
        raise ProbeError(f"--resume 文件不存在: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != SCHEMA_PROFILE:
        raise ProbeError("恢复文件 schema 不兼容")
    if data.get("model_id") != spec.model_id:
        raise ProbeError("恢复文件 model_id 与当前规范不同")
    if data.get("asset", {}).get("sha256") != spec.asset_sha256:
        raise ProbeError("恢复文件的资产 SHA-256 与当前 URDF 不同")
    if data.get("conditions") != asdict(policy):
        raise ProbeError("恢复文件的完整探测条件与本次不同")
    if data.get("adapter") != adapter.identity():
        raise ProbeError("恢复文件的设备/Adapter 身份与本次不同")
    return data


def _freeze_after_failure(adapter: HandProbeAdapter, recorder: ProfileRecorder) -> bool:
    try:
        ok = bool(adapter.freeze())
        recorder.event("soft_freeze", ok=ok)
        return ok
    except Exception as exc:  # noqa: BLE001 - failure path must preserve the original error
        try:
            recorder.event("soft_freeze", ok=False,
                           error=f"{type(exc).__name__}: {exc}")
        except Exception:  # noqa: BLE001 - best-effort evidence during teardown
            pass
        return False


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        spec = HandModelSpec.load(args.spec)
        policy = _policy_with_overrides(spec.policy, args)
        if args.project_profile is not None:
            if not args.target_u:
                raise ProbeError("--project-profile 必须同时提供 --target-u")
            target = json.loads(args.target_u)
            if not isinstance(target, dict):
                raise ProbeError("--target-u 必须是 JSON 对象")
            envelope = FeasibilityEnvelope.load(
                spec, args.project_profile, allow_mock=args.allow_mock_profile)
            print(json.dumps(envelope.project(target, margin_u=args.margin_u),
                             ensure_ascii=False, indent=2))
            return 0
        motion = args.phase in {"single", "interactions", "all"}
        if args.adapter == "inspire" and not args.hardware:
            raise ProbeError("真实 Inspire Adapter 必须显式添加 --hardware")
        if args.adapter == "inspire" and motion and args.allow_motion != MOTION_CONFIRMATION:
            raise ProbeError(
                f"运动阶段必须添加 --allow-motion {MOTION_CONFIRMATION}；"
                "该口令只授权本次所选 phase")
        if args.adapter == "inspire" and args.phase == "all":
            raise ProbeError("真机禁止 --phase all；必须按 preflight -> single -> interactions 分阶段")
        if args.resume and args.output is None:
            raise ProbeError("--resume 必须显式指定原 --output 文件")
        if args.adapter == "mock":
            # Mock 命令即时到位，不模拟机械时间；报告明确记录该条件覆盖。
            policy = replace(policy, min_settle_s=0.0)
            adapter: HandProbeAdapter = MockHandAdapter(spec)
        else:
            adapter = InspireRH56Adapter(
                spec, port=args.port, device_id=args.device_id, firmware=args.firmware)
        output = (args.output or _default_output(spec, args.adapter)).resolve()
        resume_data = (_load_resume_for_gate(output, spec, policy, adapter)
                       if args.resume else None)
        if args.adapter == "inspire" and args.phase == "interactions":
            if not args.resume:
                raise ProbeError("真机 interactions 必须用 --resume 复用单关节 Profile")
            assert resume_data is not None
            selected_ids = args.interaction or [item.id for item in spec.interactions]
            interaction_map = {item.id: item for item in spec.interactions}
            required: set[str] = set()
            for iid in selected_ids:
                item = interaction_map.get(iid)
                if item is None:
                    raise ProbeError(f"未知 interaction: {iid}")
                required.update(item.conditioning_joints)
                required.add(item.probe_joint)
            single = resume_data.get("results", {}).get("single_joint", {})
            incomplete = [name for name in sorted(required)
                          if single.get(name, {}).get("status") != "complete"]
            if incomplete:
                raise ProbeError(f"interactions 前缺少相关单关节证据: {incomplete}")
        try:
            adapter.connect(read_only=not motion)
            recorder = ProfileRecorder(output, spec, adapter, policy, resume=args.resume)
        except BaseException:
            adapter.close()
            raise
        probe = FeasibilityProbe(
            spec, adapter, recorder, policy=policy,
            sleep_fn=(lambda _: None) if args.adapter == "mock" else time.sleep)
        try:
            preflight = probe.preflight()
            print(f"preflight: {preflight['status']} ({preflight['valid_samples']} valid)")
            if motion:
                adapter.configure(speed=policy.speed, force=policy.force)
                probe.return_open()
            if args.phase in {"single", "all"}:
                result = probe.run_single(args.joint)
                print(json.dumps({"single_joint": result}, ensure_ascii=False, indent=2))
            if args.phase in {"interactions", "all"}:
                result = probe.run_interactions(args.interaction)
                print(json.dumps({"interactions": result}, ensure_ascii=False, indent=2))
            if motion:
                probe.return_open()
            recorder.complete()
            print(f"profile: {output}")
            return 0
        except SafetyAbort as exc:
            frozen = _freeze_after_failure(adapter, recorder) if motion else None
            recorder.abort(str(exc))
            print(f"SAFETY ABORT: {exc}; soft_freeze={frozen}", file=sys.stderr)
            return 3
        except KeyboardInterrupt:
            frozen = _freeze_after_failure(adapter, recorder) if motion else None
            recorder.abort("keyboard_interrupt")
            print(f"interrupted; soft_freeze={frozen}", file=sys.stderr)
            return 130
        except Exception as exc:
            if motion:
                _freeze_after_failure(adapter, recorder)
            recorder.abort(f"{type(exc).__name__}: {exc}")
            raise
        finally:
            adapter.close()
    except (SpecError, ProbeError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
