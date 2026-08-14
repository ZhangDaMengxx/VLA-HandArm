# 开发手册 (HANDBOOK)

NERO-7 + 灵巧手项目的完整开发指南。每次修改代码前从这里开始。

---

## 📚 文档索引

| 文档 | 用途 | 位置 |
|------|------|------|
| **HANDBOOK.md** | 开发手册（本文档） | `lerobotTest/` |
| **HARDWARE.md** | 硬件说明文档 | `lerobotTest/` |
| **CHANGELOG.md** | 更新日志 | `lerobotTest/` |
| **PROJECT_STATUS.md** | 项目进度 | `lerobotTest/` |

---

## 🏗️ 项目结构

```
ros2_ws/
├── lerobotTest/                    # Python主仓库
│   ├── assets/                     # 资产文件（URDF + meshes）
│   │   ├── hand/                   # 灵巧手（新标准）
│   │   │   ├── urdf/inspire_hand_right.urdf  ← 新URDF（2025-04-18）
│   │   │   └── meshes/*.STL
│   │   ├── arm/                    # 机械臂
│   │   │   ├── urdf/nero_description.urdf
│   │   │   └── meshes/*.STL
│   │   ├── assembled/              # 装配体URDF
│   │   │   └── nero_inspire_right.urdf  ← 臂+手装配
│   │   ├── viz/                    # 浏览器可视化
│   │   │   ├── arm/
│   │   │   ├── hand/
│   │   │   └── combo/nero_inspire_right_viz.urdf ← Web用
│   │   ├── hand_legacy/            # 旧版备份
│   │   └── arm_legacy/
│   │
│   ├── sim/                        # Python代码
│   │   ├── paths.py                # ⭐ 路径唯一真源
│   │   ├── inspire_hand.py         # ⭐ 灵巧手驱动
│   │   ├── nero_arm_bridge.py      # 机械臂桥接
│   │   ├── app_web.py              # Web服务器
│   │   ├── schema.py               # 数据schema
│   │   │
│   │   ├── build_nero_inspire.py   # ⭐ 生成装配URDF
│   │   ├── build_combo_viz.py      # ⭐ 生成Web viz
│   │   ├── build_arm_viz.py
│   │   └── build_hand_viz.py
│   │
│   ├── configs/                    # 配置文件
│   ├── data/                       # 数据集
│   ├── outputs/                    # 输出文件
│   │
│   └── 文档/
│       ├── HANDBOOK.md             # 本文档
│       ├── HARDWARE.md             # 硬件文档
│       ├── CHANGELOG.md            # 更新日志
│       └── PROJECT_STATUS.md       # 项目状态
│
└── src/nero_inspire_ros2/          # ROS2仓库
    ├── nero_inspire_description/
    ├── nero_inspire_sim/
    └── nero_vla_bridge/
```

---

## 🎯 核心代码说明

### 路径管理 (`sim/paths.py`)

**这是所有路径的唯一真源。** 所有脚本从这里import路径常量。

```python
from paths import (
    REPO,           # 仓库根目录
    ASSETS,         # assets/
    HAND_ROOT,      # assets/hand/
    ARM_ROOT,       # assets/arm/
    INSPIRE_URDF,   # assets/hand/urdf/inspire_hand_right.urdf
    ASSEMBLY_URDF,  # assets/assembled/nero_inspire_right.urdf
    VIZ,            # assets/viz/
)
```

**修改路径时**：只改 `paths.py`，所有脚本自动生效。

---

### 灵巧手驱动 (`sim/inspire_hand.py`)

**用途**：因时RH56DFX灵巧手的RS485通信驱动

**关键配置**（第38-63行）：
```python
HAND_JOINTS = [          # 6个驱动关节（新命名）
    "right_thumb_1_joint",   # 拇指侧摆
    "right_thumb_2_joint",   # 拇指弯曲
    "right_index_1_joint",   # 食指MCP
    "right_middle_1_joint",  # 中指MCP
    "right_ring_1_joint",    # 无名指MCP
    "right_little_1_joint",  # 小指MCP
]

HAND_LIMITS = {          # 限位（新URDF值）
    "right_thumb_1_joint": (0.0, 1.246165),
    "right_thumb_2_joint": (0.0, 0.48),
    # ...
}

PROJECT_TO_VENDOR = [5, 4, 3, 2, 1, 0]  # 厂商通道映射（逆序）
```

**关键方法**：
- `set_angles(rad6)` - 设置6个关节角度（弧度）
- `read_angles()` - 读取当前角度
- `set_force(force6)` - 设置6个关节力控
- `set_speed(speed6)` - 设置速度

**厂商通道顺序**（不要改）：
```
0=小指, 1=无名指, 2=中指, 3=食指, 4=拇指弯曲, 5=拇指旋转
```

**修改时注意**：
- 关节名必须与URDF一致
- 限位值来自URDF，不要随意改
- `PROJECT_TO_VENDOR`映射是固定的（厂商寄存器顺序）

---

### 装配体生成 (`sim/build_nero_inspire.py`)

**用途**：组装 NERO-7臂 + 适配法兰 + 灵巧手 → 完整装配URDF

**输入**：
- `NERO_FLANGE_URDF` - 机械臂URDF（含法兰）
- `INSPIRE_URDF` - 灵巧手URDF
- 装配参数（第48-54行）

**输出**：
- `assets/assembled/nero_inspire_right.urdf` (MuJoCo/Pinocchio用)

**装配参数**（第48-54行）：
```python
FLANGE_MOUNT_XYZ = "0 0 0.016489"          # link8 → 法兰
FLANGE_MOUNT_RPY = "0 0 1.570796"

MOUNT_XYZ = "0.000042 0.005962 0.002158"   # 法兰 → 手
MOUNT_RPY = "-1.570790 -0.000000 -1.570799"
```

**如何修改装配位置**：
1. 编辑 `MOUNT_XYZ` 和 `MOUNT_RPY`（单位：米和弧度）
2. 运行 `python3 sim/build_nero_inspire.py`
3. 运行 `python3 sim/build_combo_viz.py`（更新Web版本）
4. 重启web服务器查看效果

**验证**：脚本运行后自动用MuJoCo验证加载

---

### Web可视化生成 (`sim/build_combo_viz.py`)

**用途**：将装配URDF转换为浏览器可用的glb版本

**输入**：
- `assets/assembled/nero_inspire_right.urdf`

**输出**：
- `assets/viz/combo/nero_inspire_right_viz.urdf`
- `assets/viz/combo/meshes/*.glb` (23个mesh)

**工作流程**：
1. 读取装配URDF
2. STL → glb转换（或从已有glb拷贝）
3. 路径改为相对路径（浏览器可访问）
4. 删除collision（浏览器不需要）

**修改装配后必须运行**：
```bash
python3 sim/build_combo_viz.py
```

---

### Web服务器 (`sim/app_web.py`)

**用途**：FastAPI Web 工作台（控制台 + 3D可视化 + 实时摄像头手部追踪）

**启动**：
```bash
conda activate lerobot
python sim/app_web.py
```

实时手部追踪依赖 `lerobot` 环境中的 MediaPipe/dex-retargeting 和
Uvicorn WebSocket 支持。不要使用默认 Python 或缺少 `websockets`/`wsproto`
的环境验证 `/ws/hand/mimic`。

**功能**：
- 手部调试页面（单指控制）
- Combo页面（臂+手联合）
- 技能测试
- 3D实时可视化
- 浏览器 MediaPipe 摄像头追踪
- `/ws/hand/mimic` 低延迟重定向，失败时降级到 `/api/hand/mimic`

**关键配置**：
- 第810行：`_COMBO_ASSETS = REPO / "assets/viz/combo"`
- 使用 `nero_inspire_right_viz.urdf` 显示3D模型

**更新3D模型后需要重启服务器**

---

## 🔧 常用开发任务

### 1. 修改灵巧手关节限位

**文件**：`sim/inspire_hand.py` 第50-58行

**步骤**：
1. 编辑 `HAND_LIMITS` 字典
2. 同步URDF：编辑 `assets/hand/urdf/inspire_hand_right.urdf` 中的 `<limit>` 标签
3. 重新生成装配：`python3 sim/build_nero_inspire.py`
4. 重新生成viz：`python3 sim/build_combo_viz.py`
5. 测试

### 2. 修改装配位置（手相对法兰）

**文件**：`sim/build_nero_inspire.py` 第53-54行

**步骤**：
1. 编辑 `MOUNT_XYZ` 和 `MOUNT_RPY`
2. 运行 `python3 sim/build_nero_inspire.py`
3. 运行 `python3 sim/build_combo_viz.py`
4. 重启web服务器
5. 浏览器 `Ctrl+Shift+R` 强制刷新

**参数说明**：
- XYZ单位：米（0.001 = 1mm）
- RPY单位：弧度（1.5708 ≈ 90°）

### 3. 添加新的手势技能

**文件**：`sim/skills/gestures.yaml` 或代码中的技能定义

**步骤**：
1. 定义关节角度序列
2. 添加到技能注册表
3. 测试并调整
4. 更新文档

### 4. 更新URDF mesh路径

**不要直接改URDF！** 应该：
1. 调整 `build_nero_inspire.py` 中的路径逻辑
2. 重新生成URDF

---

## 📁 参考资料位置

### 厂商资料

位于项目外部，详见 `HARDWARE.md`

### 已提取的关键代码

| 用途 | 原始位置 | 项目位置 |
|------|---------|----------|
| 灵巧手Python上位机 | `基于Python灵巧手的上位机软件/` | `sim/inspire_hand.py` |
| 灵巧手C SDK | `灵巧手_SDK/C_Linux/` | 参考用 |
| 机械臂SDK | `pyAgxArm-master/` | `sim/nero_arm_bridge.py` |
| 灵巧手URDF | `urdf_right_2025_4_18/` | `assets/hand/urdf/` |
| 机械臂URDF | ROS包 | `assets/arm/urdf/` |

---

## 🔄 工作流程

### 典型开发流程

```bash
# 1. 查看当前状态
cat PROJECT_STATUS.md

# 2. 查看硬件规格
cat HARDWARE.md

# 3. 修改代码
# ...

# 4. 重新生成装配（如果改了URDF或装配参数）
python3 sim/build_nero_inspire.py
python3 sim/build_combo_viz.py

# 5. 测试
python3 sim/hand_console.py --no-mock  # 真机测试
# 或
conda run -n lerobot python sim/app_web.py  # Web/摄像头/WebSocket测试

# 6. 提交
git add .
git commit -m "feat: 描述变更"
git push origin main

# 7. 更新文档
# 更新 CHANGELOG.md
# 更新 PROJECT_STATUS.md（如果完成了任务）
```

---

## 🐛 调试技巧

### 1. MuJoCo加载URDF失败

**检查**：
- mesh文件路径是否正确（绝对路径）
- joint/link名称是否有冲突
- 限位值是否合理

**验证**：
```python
import mujoco
m = mujoco.MjModel.from_xml_path('assets/assembled/nero_inspire_right.urdf')
print(f"nq={m.nq}, njnt={m.njnt}")
```

### 2. Web 3D不显示

**检查**：
- `assets/viz/combo/nero_inspire_right_viz.urdf` 是否最新
- mesh路径是否为相对路径
- glb文件是否存在

**强制刷新**：
```bash
# 重新生成
python3 sim/build_combo_viz.py

# 重启服务器
pkill -f app_web.py
conda run -n lerobot python sim/app_web.py

# 浏览器 Ctrl+Shift+R
```

### 3. 灵巧手通信失败

参见 `HARDWARE.md` 的"故障排除"章节

---

## 📖 推荐阅读顺序

新手入门：
1. 本文档（HANDBOOK.md）- 了解项目结构
2. HARDWARE.md - 了解硬件
3. PROJECT_STATUS.md - 了解当前进度
4. CHANGELOG.md - 了解历史变更

开发前：
1. PROJECT_STATUS.md - 确认当前任务
2. 本文档 - 找到相关代码位置
3. HARDWARE.md - 确认硬件限制
4. 修改代码
5. 更新 CHANGELOG.md 和 PROJECT_STATUS.md

---

## 🆘 获取帮助

1. **查看文档**：本手册 + HARDWARE.md
2. **查看代码注释**：关键文件都有详细注释
3. **查看历史**：`git log` 和 CHANGELOG.md
4. **查看原始资料**：厂商文档（见HARDWARE.md）

---

**最后更新**：2026-08-14

**维护者**：项目团队  
**反馈**：遇到文档错误或不清楚的地方，请提issue或直接修改
