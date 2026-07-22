#!/usr/bin/env python3
"""sim/inspire_hand.py — 因时 Inspire RH56DFX 六自由度灵巧手驱动(占位)。

⚠ 占位说明:RH56DFX 走 RS485 Modbus-RTU,控制量是每自由度 0–1000 的整数
(不是弧度)。真实寄存器地址 / 从站 ID / 6 自由度顺序 **必须以因时官方手册为准**,
本文件把这些做成配置项并在真实读写处留 TODO —— 拿到手册填数即可,不改结构。

对上层(nero_arm_bridge / ros_joint_writer)暴露统一的**弧度**接口:
  read_angles() -> list[6]  (rad)      当前手指角
  set_angles(rad6)                     目标手指角(内部弧度→0-1000→写寄存器)
两端用线性映射 + 限位夹取转换,方向可按手册配置(RH56 常见 1000=张开)。

mock 模式(无硬件):set_angles 存目标,read_angles 回读目标 —— 让整条管线
(bridge→/joint_states→Rerun/网页)在没接真手时也能端到端跑通。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

# 6 自由度顺序 = schema/URDF 的 HAND 顺序(和 ros_joint_writer 一致)。
# ⚠ 真手的物理通道顺序可能不同,用 CHANNEL_ORDER 重排(拿手册确认后填)。
HAND_JOINTS = ["thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
               "index_proximal_joint", "middle_proximal_joint",
               "ring_proximal_joint", "pinky_proximal_joint"]
# 各自由度弧度限位(rad),和 URDF <mimic> 驱动关节一致。
HAND_LIMITS = {
    "thumb_proximal_yaw_joint": (0.0, 1.308),
    "thumb_proximal_pitch_joint": (0.0, 0.6),
    "index_proximal_joint": (0.0, 1.47),
    "middle_proximal_joint": (0.0, 1.47),
    "ring_proximal_joint": (0.0, 1.47),
    "pinky_proximal_joint": (0.0, 1.47),
}
RAW_MIN, RAW_MAX = 0, 1000        # RH56 驱动量范围


@dataclass
class InspireHandConfig:
    port: str = "/dev/ttyUSB0"        # RS485 串口(拿手册/实物确认)
    baudrate: int = 115200            # 因时默认常见 115200
    slave_id: int = 1                 # Modbus 从站 ID(手册确认)
    # 方向:True = raw 越大手指越张开(RH56 常见);False = raw 越大越握紧
    raw_open_high: bool = True
    # 物理通道→HAND_JOINTS 的重排索引(默认顺序一致);拿手册确认后改
    channel_order: List[int] = field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    mock: bool = True                 # 无硬件时 True


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class InspireHand:
    """RH56DFX 驱动封装(占位)。真实读写留 TODO;mock 下可全链路运行。"""

    def __init__(self, cfg: InspireHandConfig | None = None) -> None:
        self.cfg = cfg or InspireHandConfig()
        self._client = None
        # mock 目标缓存:初始张开(按方向映射成弧度下/上限)
        self._target_rad = [HAND_LIMITS[n][0] for n in HAND_JOINTS]

    # ---- 弧度 ⇄ 0-1000 ----
    def rad_to_raw(self, name: str, rad: float) -> int:
        lo, hi = HAND_LIMITS[name]
        rad = _clamp(rad, lo, hi)
        frac = (rad - lo) / (hi - lo) if hi > lo else 0.0     # 0=张开(弧度下限)..1=握紧
        if self.cfg.raw_open_high:
            frac = 1.0 - frac                                 # raw 大=张开
        return int(round(RAW_MIN + frac * (RAW_MAX - RAW_MIN)))

    def raw_to_rad(self, name: str, raw: int) -> float:
        lo, hi = HAND_LIMITS[name]
        frac = (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)
        if self.cfg.raw_open_high:
            frac = 1.0 - frac
        return lo + frac * (hi - lo)

    # ---- 连接 ----
    def connect(self) -> bool:
        if self.cfg.mock:
            return True
        # TODO(真机):以下按因时手册实现 —— 打开串口 + Modbus 客户端
        #   import minimalmodbus / pymodbus
        #   self._client = minimalmodbus.Instrument(self.cfg.port, self.cfg.slave_id)
        #   self._client.serial.baudrate = self.cfg.baudrate
        raise NotImplementedError(
            "RH56DFX 真机读写未实现:需因时手册的寄存器地址后填入 connect/read/set。"
            "当前请用 mock=True 跑占位链路。")

    def disconnect(self) -> None:
        if self._client is not None:
            try:
                self._client.serial.close()
            except Exception:
                pass
        self._client = None

    # ---- 读 / 写(弧度接口)----
    def read_angles(self) -> List[float]:
        """返回 6 自由度当前角(rad,HAND_JOINTS 顺序)。"""
        if self.cfg.mock:
            return list(self._target_rad)                     # 占位:回读目标
        # TODO(真机):读 ANGLE_ACT 寄存器 → raw6 → channel_order 重排 → raw_to_rad
        raise NotImplementedError("RH56DFX read_angles 需手册寄存器地址")

    def set_angles(self, rad6: List[float]) -> bool:
        """设置 6 自由度目标角(rad,HAND_JOINTS 顺序),内部转 0-1000 写寄存器。"""
        rad6 = [_clamp(r, *HAND_LIMITS[n]) for n, r in zip(HAND_JOINTS, rad6)]
        if self.cfg.mock:
            self._target_rad = rad6
            return True
        # raw = [self.rad_to_raw(n, r) for n, r in zip(HAND_JOINTS, rad6)]
        # raw = [raw[i] for i in self.cfg.channel_order]      # 重排到物理通道
        # TODO(真机):把 raw 写 ANGLE_SET 寄存器
        raise NotImplementedError("RH56DFX set_angles 需手册寄存器地址")

