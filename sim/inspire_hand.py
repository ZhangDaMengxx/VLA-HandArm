#!/usr/bin/env python3
"""sim/inspire_hand.py — 因时 Inspire RH56DFX 六自由度灵巧手 RS485 驱动。

协议**不是 Modbus-RTU**(旧占位注释的说法是错的),是因时自家包帧:
  写请求  EB 90 | id | len=payload+3 | 0x12 | addr_lo | addr_hi | payload... | cs
  读请求  EB 90 | id | 0x04         | 0x11 | addr_lo | addr_hi | read_len   | cs
  回复头  90 EB (与请求相反)
  cs = 从 id 起(不含帧头)累加取低 8 位
帧实现依据 handarm_notes.md:1490-1527(手册 V1.09 2.2,逐条复算校验和),
并已在真手实测通过(2026-07-31, /dev/ttyUSB0)。

对上层(nero_arm_bridge / hand_console)暴露**弧度**接口:
  read_angles() -> list[6] (rad)   当前手指角
  set_angles(rad6)                 目标手指角
两处易错点已按手册坐实,不要改回去:
  · 通道顺序是**完全逆序**:厂商 m=0 是小拇指,项目顺序 0 是拇指侧摆
  · 方向**逐通道**不同:四指 raw 大=张开,拇指弯曲 raw 大=弯曲

mock 模式(无硬件):set_angles 存目标,read_angles 回读目标,整条链路可空跑。
"""
from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass, field
from typing import List

# 6 自由度顺序 = schema/URDF 的 HAND 顺序(和 ros_joint_writer 一致)。
# 2026-08-10 更新：使用厂商2025-04-18新URDF的关节命名
HAND_JOINTS = ["right_thumb_1_joint", "right_thumb_2_joint",
               "right_index_1_joint", "right_middle_1_joint",
               "right_ring_1_joint", "right_little_1_joint"]

# URDF 驱动关节限位(rad)。**安全夹取用这套**,和 ros_joint_writer.JOINT_LIMITS 一致。
#
# ✅ 2026-08-10 更新：真机已到位，采用厂商2025-04-18新URDF的限位值作为标准。
# 新URDF限位来自SolidWorks导出，应该更接近机械真实值。
#
# 关节名对应（新→旧）：
#   right_thumb_1_joint (yaw侧摆)  ← thumb_proximal_yaw_joint   [0, 1.246165]
#   right_thumb_2_joint (pitch弯曲) ← thumb_proximal_pitch_joint [0, 0.48]
#   right_index_1_joint            ← index_proximal_joint        [0, 1.333]
#   right_middle_1_joint           ← middle_proximal_joint       [0, 1.333]
#   right_ring_1_joint             ← ring_proximal_joint         [0, 1.333]
#   right_little_1_joint           ← pinky_proximal_joint        [0, 1.333]
#
# 限位变化（旧→新）：
#   thumb_1 (yaw):    1.308 → 1.246165  (-4.7%)
#   thumb_2 (pitch):  0.6   → 0.48      (-20%)
#   四指 (MCP):       1.47  → 1.333     (-9.3%)
#
# ⚠ 旧手势包兼容性：已录制的gesture pack中超限的帧会被自动夹到新上限，
# 抓握动作可能变弱。需要时可重新录制或手动调整force参数补偿。
HAND_LIMITS = {
    "right_thumb_1_joint": (0.0, 1.246165),    # 拇指侧摆（yaw）
    "right_thumb_2_joint": (0.0, 0.48),        # 拇指弯曲（pitch）
    "right_index_1_joint": (0.0, 1.333),       # 食指MCP
    "right_middle_1_joint": (0.0, 1.333),      # 中指MCP
    "right_ring_1_joint": (0.0, 1.333),        # 无名指MCP
    "right_little_1_joint": (0.0, 1.333),      # 小指MCP
}
RAW_MIN, RAW_MAX = 0, 1000        # ANGLE_SET / ANGLE_ACT 量程

# 逐通道力控上限(g),**项目顺序**,与 HAND_JOINTS 对齐。
#
# 拇指为什么也是 1000 —— **实测结论,别再改回 1500**(2026-08-06,verify_force_limits.py):
#   手册两个寄存器容易混:
#     FORCE_SET(1498)          运行期写的就是它 —— 标 0-1000,全通道
#     DEFAULT_FORCE_SET(1044)  上电默认值      —— 标 0-1000,**拇指 0-1500**
#   "拇指 1500" 只对后者成立。真手上写 FORCE_SET=1500,读回来是 1000;写 800 回 800、
#   写 1000 回 1000。所以手内部就夹在 1000,拇指不例外。
#   要真拿到拇指那 1/3 行程,只能写 DEFAULT_FORCE_SET(1044) 并 SAVE(1005) —— 但那是
#   改上电默认值(掉电保存),和运行期调力控是两件事,而且 handarm_notes 明确写了
#   初始化里**不要**写 SAVE。想做得单独设计,不要顺手塞进 set_force。
#
# 力控语义是**阈值**不是目标力:位置控制到碰到这个力就停住保持。而且手册 2.4.12
# 说这是**指尖**握力,接触点不在指尖时力臂变短、实际握力更大 —— 调这个值要记得。
#
# 2026-08-10 更新：关节名改为新URDF命名
FORCE_MAX = {
    "right_thumb_1_joint": 1000,      # 拇指侧摆
    "right_thumb_2_joint": 1000,      # 拇指弯曲
    "right_index_1_joint": 1000,      # 食指
    "right_middle_1_joint": 1000,     # 中指
    "right_ring_1_joint": 1000,       # 无名指
    "right_little_1_joint": 1000,     # 小指
}
SPEED_MAX = 1000                  # SPEED_SET 全通道同量程,没有逐通道差异


def _need6_force(vals) -> list:
    """力控入参必须是 6 个。**不补齐、不截断** —— 少给几个就报错。

    补齐会让"我以为设了 6 指、其实只设了 3 指"这种事悄悄发生:剩下 3 指沿用
    上一次的阈值,握持力一半强一半弱,现象是物体被捏歪而不是报错。
    """
    out = list(vals)
    if len(out) != 6:
        raise ValueError(f"力控需要 6 个值(项目顺序),给了 {len(out)} 个")
    return out

# 寄存器地址(手册 V1.09 2.4,唯一真源;见 handarm_notes.md:1441-1479)。
# ⚠ 波特率寄存器是 1002,厂商 demo_485.py 写的 1001 是 bug。
REG = {
    "HAND_ID": 1000, "REDU_RATIO": 1002, "CLEAR_ERROR": 1004, "SAVE": 1005,
    "FORCE_CLB": 1009, "CURRENT_LIMIT": 1020, "VOLTAGE": 1472,
    "POS_SET": 1474, "ANGLE_SET": 1486, "FORCE_SET": 1498, "SPEED_SET": 1522,
    "POS_ACT": 1534, "ANGLE_ACT": 1546, "FORCE_ACT": 1582, "CURRENT": 1594,
    "ERROR": 1606, "STATUS": 1612, "TEMP": 1618,
}

# 项目顺序索引 → 厂商通道 m。厂商 m: 0 小拇指 1 无名指 2 中指 3 食指 4 拇指弯曲 5 拇指旋转。
# 正好完全逆序,是自逆置换(读回来用同一个数组反向索引即可)。
PROJECT_TO_VENDOR = [5, 4, 3, 2, 1, 0]

# raw(0-1000) ↔ rad 的逐通道映射,端点取自官方 xls《关节角与0-1000 对应关系》。
# 只有这三类驱动关节是**严格线性**的(xls 实测偏离直线 0.0000),可以直接线性换算;
# 耦合的远端关节(P2/指腹/指尖)非线性,最多偏 11°,要精确得查表 —— 那是可视化的事,
# 不影响这里的驱动量换算。span 单位 rad,invert=True 表示 raw 越大关节角越小。
#   四指     xls  90°→170° (raw 0→1000) = 张开方向,URDF 0=张开 ⇒ 反向
#   拇指弯曲 raw 0 = 弯曲 / raw 1000 = 张开 ⇒ 反向(实机反馈订正)
#   拇指侧摆 raw 0 = 垂直掌面(对掌位) / raw 1000 = 完全打开 ⇒ 反向
#
# 拇指弯曲方向 —— 曾经是 invert=False,实机反馈证明反了,别改回去:
#   原推断依据 handarm_notes.md:1602 的 xls 端点「大拇指弯曲关节角 170°→130°」,
#   按"内角变小=弯曲"读成 raw↑=弯曲。漏洞在于那一列**没写角度怎么量的**(哪两根连杆、
#   内角还是外角);四指列 90°→170° 读成"变大=张开",拇指列反着读,两列基准未必一致。
#   实机现象:模型与真手角度反馈相反。渲染已验证忠实于 URDF(rad↑ 时拇指尖朝食指靠近,
#   140mm→65mm),所以错的是这一步的方向。翻转后六个通道统一为 raw 1000=张开,
#   符合因时产品约定 —— 原先拇指弯曲是唯一的例外,本身就可疑。
#
# 拇指侧摆方向已由 URDF 几何定死,不要再改回 False:
#   URDF yaw=0.000 → 拇指躺在掌面里(出平面 14°),拇指尖↔食指尖最近 47mm,捏不上
#   URDF yaw=1.246 → 最大外展(出平面 ~60°),拇指尖↔食指尖最近 ~8mm,能捏
# 所以 yaw 上限=对掌位=raw 0,yaw 下限=完全打开=raw 1000,和四指同为 invert=True。
# span 取 URDF 上限而非 xls 行程 1.39626(80°):这样 raw 0/1000 正好打在 URDF
# 两端,两个方向都跑满。用 xls 行程的话 rad_to_raw(上限)=63,最后 6% 对掌行程发不出去。
# 代价是中段 rad 与物理角差 ~2.5°(把 80° 物理行程压到 ~71°),可视化和控制都看不出来。
#
# 2026-08-10 更新：span全部同步新URDF限位（真机已到位，采用厂商最新值）
RAW_MAP = {
    "right_thumb_1_joint":   (1.246165, True),   # 拇指侧摆: raw 0=垂直掌面, 1000=完全打开
    "right_thumb_2_joint":   (0.48, True),       # 拇指弯曲: raw 0=弯曲, 1000=张开
    "right_index_1_joint":   (1.333, True),      # 食指: 新URDF限位
    "right_middle_1_joint":  (1.333, True),      # 中指
    "right_ring_1_joint":    (1.333, True),      # 无名指
    "right_little_1_joint":  (1.333, True),      # 小指
}


@dataclass
class InspireHandConfig:
    """默认值来自 Python 上位机 + ROS1 launch,已实测(handarm_notes.md:717-725)。"""
    port: str = field(default_factory=lambda: os.environ.get(
        "INSPIRE_HAND_PORT", "/dev/ttyUSB0"))
    baudrate: int = 115200
    hand_id: int = 1                  # 手 ID,默认 1
    timeout: float = 0.5              # 串口底层超时(_txn 不依赖它,留作兜底)
    # 单次事务(发一帧+收一帧)的截止时间。实测手回复在 ~3ms 内,60ms 有 20 倍余量。
    # 这个值直接决定"手掉线时"的降级速度:读不到就等满它,不会更久。
    txn_timeout: float = 0.06
    mock: bool = True                 # 无硬件时 True
    # 上电初始化的速度/力控。手册: 运行期 SPEED_SET/FORCE_SET 不断电保存,每次连上要重设,
    # 否则沿用 flash 里的 DEFAULT_* —— 不设可能出现"发了角度但几乎不动"。
    init_speed: int = 500
    init_force: int = 500
    # True: 弧度按 URDF 限位夹取(与 ros_joint_writer 一致,安全优先)。
    # 代价: 拇指弯曲 URDF 上限 0.6 < xls 实际行程 0.698,写角度只能到实际行程的 86%,
    # 回读满弯也在 0.6 处饱和。要跑满行程就置 False,但那是放宽一条安全限位。
    clamp_to_urdf: bool = True


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# 包帧(与 selfcheck_hardware.py:23-43 同一实现,已实测)
# ---------------------------------------------------------------------------
def build_read(addr: int, length: int, hand_id: int = 1) -> bytes:
    body = bytes([hand_id, 0x04, 0x11, addr & 0xFF, (addr >> 8) & 0xFF, length])
    return b"\xeb\x90" + body + bytes([sum(body) & 0xFF])


def build_write(addr: int, payload: bytes, hand_id: int = 1) -> bytes:
    body = (bytes([hand_id, len(payload) + 3, 0x12, addr & 0xFF, (addr >> 8) & 0xFF])
            + payload)
    return b"\xeb\x90" + body + bytes([sum(body) & 0xFF])


def parse_reply(raw: bytes, want_addr: int | None = None) -> bytes | None:
    """回复帧头 90 EB。校验和不符/帧不完整返回 None;成功返回 payload(跳过 cmd+addr)。

    want_addr 给了就**核对回复里的寄存器地址**,不匹配就跳过这一帧继续往后找。

    ⚠ 为什么必须核对地址:手会持续回帧,`reset_input_buffer()` 清不干净(串口驱动
    缓冲之外还有在途字节),于是**上一次查询的迟到回复会被当成本次的结果**。
    实测症状:连读三次 FORCE_SET 得到三个不同结果,其中两次是 ANGLE_ACT 的值 ——
    校验和全对,因为那确实是合法帧,只是答的是别的问题。
    只验校验和无法发现这类错误:它不是"数据坏了",是"答非所问"。而后果比坏数据
    更糟 —— 调试页会把角度当力显示,而且看不出异常。

    多帧共存时逐帧扫:地址不符就前进到下一个 90 EB,而不是直接放弃 —— 缓冲里
    往往是「若干迟到帧 + 本次回复」,放弃就永远读不到本次的。
    """
    pos = 0
    while True:
        i = raw.find(b"\x90\xeb", pos)
        if i < 0 or len(raw) < i + 8:
            return None
        body = raw[i + 2:]
        need = 2 + body[1] + 1
        if len(body) < need:
            # 帧不完整:可能还在传。**不要**继续往后找 —— 后面没有更多帧了。
            return None
        frame, cs = body[:need - 1], body[need - 1]
        if len(frame) < 5 or sum(frame) & 0xFF != cs:
            pos = i + 2
            continue
        if want_addr is not None and (frame[3] | (frame[4] << 8)) != want_addr:
            pos = i + 2                     # 答的是别的寄存器,跳过
            continue
        return frame[5:]


class InspireHand:
    """RH56DFX RS485 驱动。mock=True 时不碰串口,整条链路可空跑。"""

    def __init__(self, cfg: InspireHandConfig | None = None) -> None:
        self.cfg = cfg or InspireHandConfig()
        self._sp = None
        # mock 目标缓存:按 URDF 下限起(四指=张开)
        self._target_rad = [HAND_LIMITS[n][0] for n in HAND_JOINTS]
        self.last_error: str | None = None

    # ---- 弧度 ⇄ raw(0-1000),逐通道方向 ----
    def rad_to_raw(self, name: str, rad: float) -> int:
        span, invert = RAW_MAP[name]
        if self.cfg.clamp_to_urdf:
            rad = _clamp(rad, *HAND_LIMITS[name])
        rad = _clamp(rad, 0.0, span)
        frac = rad / span if span > 0 else 0.0
        if invert:
            frac = 1.0 - frac
        return int(round(RAW_MIN + frac * (RAW_MAX - RAW_MIN)))

    def raw_to_rad(self, name: str, raw: int) -> float:
        span, invert = RAW_MAP[name]
        frac = _clamp((raw - RAW_MIN) / (RAW_MAX - RAW_MIN), 0.0, 1.0)
        if invert:
            frac = 1.0 - frac
        rad = frac * span
        if self.cfg.clamp_to_urdf:
            rad = _clamp(rad, *HAND_LIMITS[name])
        return rad

    # ---- 连接 ----
    def connect(self) -> bool:
        if self.cfg.mock:
            return True
        import serial                                  # 只在真机路径 import
        self._sp = serial.Serial(self.cfg.port, self.cfg.baudrate,
                                 timeout=self.cfg.timeout)
        time.sleep(0.1)                                # CH340 上电稳定
        # 首次握手用宽截止时间:CH340 刚上电 + 手可能还没就绪,用 60ms 会误判成掉线。
        # 稳态事务仍走 cfg.txn_timeout。
        if self.read_regs("HAND_ID", 1, "B", deadline=0.5) is None:
            self.disconnect()
            raise RuntimeError(
                f"{self.cfg.port} 打开了但读 HAND_ID 无响应:确认 24V 供电、"
                f"RS485 A/B 未接反、手 ID={self.cfg.hand_id}")
        # 速度/力控不断电保存,每次连上都要重设(否则沿用 flash 的 DEFAULT_*)
        self.set_speed(self.cfg.init_speed)
        self.set_force(self.cfg.init_force)
        return True

    def disconnect(self) -> None:
        if self._sp is not None:
            try:
                self._sp.close()
            except Exception:                          # noqa: BLE001
                pass
        self._sp = None

    @property
    def connected(self) -> bool:
        return self.cfg.mock or (self._sp is not None and self._sp.is_open)

    # ---- 串口事务(所有读写走这里)----
    def _txn(self, frame: bytes, want: int = 0, deadline: float | None = None):
        """发一帧,收到**完整可解析**的回复就立刻返回,不傻等。

        ⚠ 这里曾经写 `time.sleep(0.012)` + `read(64)`,是延迟和"报文卡住"的根源:
        pyserial 的 read(n) 会等到攒够 n 字节**或超时**才返回。回复帧只有 nbytes+8
        字节(读 6 个 short = 20 字节),永远攒不到 64,于是每次事务都白等满
        timeout=0.5s。一次全量遥测 8 个事务 ≈ 4s,20Hz 主循环直接被拖死 ——
        表现就是"下发有延迟、容易卡住阻塞"。
        改成按 in_waiting 增量收、解析成功即返回:单次事务从 ~512ms 降到 ~3ms。
        """
        dl = self.cfg.txn_timeout if deadline is None else deadline
        # 从**发出去的帧**里取寄存器地址,回复必须答同一个地址才算本次的结果。
        # 读帧和写帧的 addr 都在 frame[5:7](帧头 2 + hand_id/len/cmd 3)。
        want_addr = frame[5] | (frame[6] << 8) if len(frame) >= 7 else None
        self._sp.reset_input_buffer()                   # 丢弃上次超时后迟到的回复,
        self._sp.write(frame)                           # 不清会和本次回复串味
        self._sp.flush()
        end = time.monotonic() + dl
        buf = b""
        while time.monotonic() < end:
            n = self._sp.in_waiting
            if n:
                buf += self._sp.read(n)
                payload = parse_reply(buf, want_addr)
                if payload is not None and len(payload) >= want:
                    return payload
            else:
                time.sleep(0.0005)                     # 0.5ms,别空转烧 CPU
        return None

    # ---- 寄存器读写(原始层,调试页直接用)----
    def read_regs(self, reg: str, nbytes: int, fmt: str, deadline: float | None = None):
        """读寄存器 → struct.unpack 结果(小端)。失败返回 None(不抛,调试页要能显示"读不到")。"""
        if self._sp is None:
            return None
        try:
            payload = self._txn(build_read(REG[reg], nbytes, self.cfg.hand_id),
                                want=nbytes, deadline=deadline)
            if payload is None or len(payload) < nbytes:
                self.last_error = f"读 {reg} 无有效回复"
                return None
            return struct.unpack("<" + fmt, payload[:nbytes])
        except Exception as e:                         # noqa: BLE001
            self.last_error = f"读 {reg} 异常: {e}"
            return None

    def write_shorts(self, reg: str, vals: List[int]) -> bool:
        """写 6 short 寄存器组(ANGLE_SET / SPEED_SET / FORCE_SET)。-1 = 该通道不动作。"""
        if self._sp is None:
            return False
        try:
            payload = b"".join(struct.pack("<h", int(v)) for v in vals)
            # 写回复是空 payload(8 字节),_txn 收到就返回;收不到也不算失败 ——
            # 手对写指令偶发不回,但动作照做,这里返回 False 会让上层误判掉线。
            self._txn(build_write(REG[reg], payload, self.cfg.hand_id))
            return True
        except Exception as e:                         # noqa: BLE001
            self.last_error = f"写 {reg} 异常: {e}"
            return False

    def write_byte(self, reg: str, val: int) -> bool:
        if self._sp is None:
            return False
        try:
            self._txn(build_write(REG[reg], bytes([val & 0xFF]), self.cfg.hand_id))
            return True
        except Exception as e:                         # noqa: BLE001
            self.last_error = f"写 {reg} 异常: {e}"
            return False

    # ---- 弧度接口(bridge / 上层用)----
    def read_angles(self) -> List[float]:
        """当前 6 驱动关节角(rad,项目顺序)。读失败回退上次目标,不抛 —— bridge 在
        100Hz 定时器里调它,偶发丢帧不该把节点搞挂。"""
        if self.cfg.mock:
            return list(self._target_rad)
        vals = self.read_regs("ANGLE_ACT", 12, "6h")
        if vals is None:
            return list(self._target_rad)
        # 厂商通道 → 项目顺序(PROJECT_TO_VENDOR 自逆,同一数组即可)
        return [self.raw_to_rad(n, vals[PROJECT_TO_VENDOR[i]])
                for i, n in enumerate(HAND_JOINTS)]

    def set_angles(self, rad6: List[float]) -> bool:
        """设目标角(rad,项目顺序)→ raw → 重排到厂商通道 → 写 ANGLE_SET。"""
        if self.cfg.clamp_to_urdf:
            rad6 = [_clamp(r, *HAND_LIMITS[n]) for n, r in zip(HAND_JOINTS, rad6)]
        self._target_rad = list(rad6)
        if self.cfg.mock:
            return True
        raw_proj = [self.rad_to_raw(n, r) for n, r in zip(HAND_JOINTS, rad6)]
        vendor = [0] * 6
        for i, m in enumerate(PROJECT_TO_VENDOR):
            vendor[m] = raw_proj[i]
        return self.write_shorts("ANGLE_SET", vendor)

    def set_speed(self, speed: int) -> bool:
        """全通道速度 0-1000。不断电保存,每次连上要重设。"""
        if self.cfg.mock:
            return True
        return self.write_shorts("SPEED_SET", [int(_clamp(speed, 0, 1000))] * 6)

    def set_force(self, force: int | list | tuple) -> bool:
        """力控阈值(g)。给标量 = 全通道同值;给 6 个值 = **逐通道**,项目顺序。

        逐通道夹取按 FORCE_MAX。现在六个通道上限都是 1000(实测手内部就夹在这儿,
        见 FORCE_MAX 注释),所以逐通道**暂时看不出差别** —— 保留这条路是因为
        「不同手指用不同力」本身是真需求(捏鸡蛋要拇指+食指轻、其余不动),
        而底层 write_shorts 一直是 6 通道的,是这层 API 之前把它拍平成一个标量。

        ⚠ 顺序:入参是**项目顺序**(同 HAND_JOINTS),写下去前才转厂商顺序。
        逐通道限制一旦不同,顺序错了就是"给小指发了拇指的上限" —— 而且不会报错,
        只会悄悄用错的力。PROJECT_TO_VENDOR 恰好是自逆置换,这掩盖了顺序错误:
        转两次等于没转,测试也照样过。所以这里**显式转一次**,不依赖它自逆。
        """
        if self.cfg.mock:
            return True
        return self.write_shorts("FORCE_SET", self._force_vendor(force))

    @staticmethod
    def _force_vendor(force: int | list | tuple) -> List[int]:
        """力控入参 → 厂商顺序的 6 个值,逐通道夹取。None → -1(该通道不动作)。"""
        if isinstance(force, (list, tuple)):
            vals = _need6_force(force)
        else:
            vals = [force] * 6
        proj: List[int] = []
        for name, v in zip(HAND_JOINTS, vals):
            if v is None:
                proj.append(-1)
                continue
            proj.append(int(_clamp(int(v), 0, FORCE_MAX[name])))
        out = [-1] * 6
        for i, v in enumerate(proj):
            out[PROJECT_TO_VENDOR[i]] = v
        return out

    def clear_error(self) -> bool:
        if self.cfg.mock:
            return True
        return self.write_byte("CLEAR_ERROR", 1)

    def telemetry(self) -> dict:
        """调试页要的全量遥测。读不到的项为 None,不抛。

        **全部 6 通道数组都是项目顺序**(同 HAND_JOINTS / read_angles),不是厂商顺序。

        ⚠ 2025-08-06 修:这里原来直接 `list(v)` **不换序**,于是同一个页面上
        关节滑块是项目序(拇指在前)、力/温度/电流是厂商序(小指在前) —— 显示时
        又只是 join 成一串数字,没有逐指标签,所以看不出错位。后果是把小指的力
        读成拇指的,而且判据、阈值全跟着偏。mock 分支反而是项目序的(它从
        _target_rad 算),所以改之前 mock 和真机互相矛盾,拿 mock 测不出这个 bug。

        FORCE_ACT 必须按 **signed** 解析:手册标 0-1000g,但实测空载有负偏置
        (handarm_notes.md:1433),用 unsigned 会得到 65000 这种假值。
        """
        if self.cfg.mock:
            raw = [self.rad_to_raw(n, r) for n, r in zip(HAND_JOINTS, self._target_rad)]
            return {"angle_act": raw, "force_act": [0] * 6, "temp": [30] * 6,
                    "status": [0] * 6, "error": [0] * 6, "current": [0] * 6,
                    "pos_act": [0] * 6, "voltage": 470, "mock": True}
        out: dict = {"mock": False}
        for key, reg, nb, fmt in (
            ("angle_act", "ANGLE_ACT", 12, "6h"),
            ("pos_act",   "POS_ACT",   12, "6h"),
            ("force_act", "FORCE_ACT", 12, "6h"),   # signed,空载有负偏置
            ("current",   "CURRENT",   12, "6h"),
            ("temp",      "TEMP",       6, "6B"),
            ("status",    "STATUS",     6, "6B"),
            ("error",     "ERROR",      6, "6B"),
        ):
            v = self.read_regs(reg, nb, fmt)
            out[key] = ([int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)]
                        if v is not None else None)
        v = self.read_regs("VOLTAGE", 2, "h")
        out["voltage"] = v[0] if v is not None else None
        # 给前端一份权威通道名,免得它自己猜顺序(猜错不会报错,只会静默错位)
        out["channels"] = list(HAND_JOINTS)
        return out
