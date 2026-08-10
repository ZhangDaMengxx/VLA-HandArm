#!/usr/bin/env python3
"""sim/action_sequences.py — 解析 Inspire 手默认动作序列文件。"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass
class ActionStep:
    """一步动作：6角+6速+6力+延时(ms)。字段为 None = 不设置(对应原文件空白)。"""
    angles: list[int | None]      # 6 通道角度 0-1000 或 None
    speeds: list[int | None]      # 6 通道速度 0-1000 或 None
    forces: list[int | None]      # 6 通道力控 0-1000 或 None
    delay_ms: int                 # 延时 ms(厂商格式里就是这个,保留)
    # 相对序列起点的**绝对时刻**,整数纳秒。None = 没有(厂商 DefaultAction.txt 就没有)。
    #
    # ⚠ 为什么要加这个而不是继续累加 delay_ms:30fps 的真周期是 33.3333…ms,
    # 存成整数 33 每帧少 0.333ms,**单向不抵消**。600 帧(20s)累计 200ms,
    # 2400 帧(MAX_FRAMES)累计 800ms。臂按绝对时刻走、手按累加走,末尾就错开了 ——
    # 抓取动作里手已经合上而臂还没到位。
    # 整数纳秒对齐 ROS 的 builtin_interfaces/Duration(int32 sec + uint32 nanosec),
    # 残差 0.333ns/帧,2400 帧才 0.0008ms,比臂的控制周期(4.5ms)细 5600 倍。
    # 播放器按绝对时刻**定位**而不是累加,所以这点残差也不会被放大。
    t_ns: int | None = None


@dataclass
class ActionSequence:
    """一个动作序列：索引号 + 最多 8 步(实际可能少于 8,空行跳过)。

    ⚠ index **不唯一**:DefaultAction.txt 里 13 个序列的索引号是
    2,3,4,4,4,3,3,4,4,3,5,5,3(手册说这文件是导出快照,不是索引表)。
    所以定位必须用 slot(文件里的第几个,0 起),index 只作参考。
    """
    index: int                    # 厂商动作库索引号(可重复!)
    name: str                     # 显示名
    steps: list[ActionStep]
    slot: int = -1                # 文件里的位置,唯一,前端/播放器用它定位


def parse_field(s: str) -> int | None:
    """解析一个字段：空白(逗号间无值)→ None,否则→ int。"""
    s = s.strip()
    return None if not s else int(s)


def parse_action_file(path: Path) -> list[ActionSequence]:
    """解析 DefaultAction.txt:每 9 行一组(索引+8步),13 组。"""
    lines = path.read_text(encoding="utf-8").splitlines()
    seqs = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line or line.startswith("#"):
            i += 1
            continue
        # 第 i 行:索引号(单独一行)
        try:
            idx = int(line)
        except ValueError:
            i += 1
            continue
        # 接下来最多 8 行步骤(遇到空行或下一个索引号就停)
        steps = []
        for j in range(i + 1, min(i + 9, len(lines))):
            step_line = lines[j].strip()
            if not step_line:                        # 空行:该步骤为空(文件里是占位)
                continue
            # 尝试解析是否是下一个索引号(单个数字且无逗号)
            if "," not in step_line:
                break
            parts = step_line.split(",")
            if len(parts) < 19:
                continue                             # 格式不对,跳过
            try:
                angles = [parse_field(parts[k]) for k in range(6)]
                speeds = [parse_field(parts[k]) for k in range(6, 12)]
                forces = [parse_field(parts[k]) for k in range(12, 18)]
                delay = int(parts[18]) if parts[18].strip() else 0
            except (ValueError, IndexError):
                continue
            # 文件里每组固定占 8 行,不足的用 "    ,    ,..." 空白占位。这种行 strip 后
            # 非空(全是逗号和空格),会解析成"全 None + 延时 0"的空步 —— 得跳掉,
            # 否则每个序列都报 8 步,播放时前面几步瞬间空转。
            if all(v is None for v in angles) and delay == 0:
                continue
            steps.append(ActionStep(angles, speeds, forces, delay))
        if steps:                                    # 只保留有有效步骤的序列
            seqs.append(ActionSequence(idx, f"动作 {idx}", steps))
        i += 9                                       # 跳到下一组(即使步骤少于8行,原文件也占9行)
    return seqs


# 按厂商索引号给的粗略名字。⚠ 这些名字是**猜的** —— 厂商文件只给了索引号,没给名称,
# 而且同一个索引号下有多个不同内容的序列(见 ActionSequence.index 的注释)。
# 实机跑一遍看手做什么动作,再把名字改准。
ACTION_HINTS = {
    2: "开合",
    3: "捏取",
    4: "抓握",
    5: "侧捏",
}

DEFAULT_ACTION_FILE = Path.home() / "ros2_ws/inspire_hand/动作序列文件/DefaultAction.txt"


def load_default_actions(path: Path | None = None) -> list[ActionSequence]:
    """加载默认动作序列。名字带 slot 后缀区分同索引号的多个序列。"""
    f = path or DEFAULT_ACTION_FILE
    if not f.exists():
        return []
    seqs = parse_action_file(f)
    # index 重复,显示名必须唯一,否则列表上分不清哪个是哪个
    for i, s in enumerate(seqs):
        s.slot = i
        hint = ACTION_HINTS.get(s.index, "动作")
        s.name = f"{i + 1}. {hint}(idx {s.index})"
    return seqs
