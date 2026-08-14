# 开发手册

本手册说明 `lerobotTest` 的当前开发边界。硬件参数看 [HARDWARE.md](HARDWARE.md)，
部署 MCP/Bridge 看 `/home/zhang123/ros2_ws/robot-mcp-server`。

## 仓库边界

```text
ros2_ws/
├── lerobotTest/              本仓库：驱动、Web、仿真、标定和 VLA 数据管线
├── robot-mcp-server/         现行 MCP Server + 可部署 robot-bridge
└── src/nero_inspire_ros2/    ROS2 包（独立仓库）
```

本仓库根目录 `bridge.py` 与 `mcp_server/` 是拆仓前快照。允许用于历史比较，但新功能、
部署修复和文档应先落到 `robot-mcp-server`，再按需同步共享驱动。

## 关键模块

| 文件 | 职责 |
|------|------|
| `sim/paths.py` | 本仓资产路径常量 |
| `sim/inspire_hand.py` | RH56DFX RS485 驱动和运行时限位 |
| `sim/nero_arm.py` | NERO CAN/SDK 封装和运行时限位 |
| `sim/nero_arm_bridge.py` | ROS2 硬件桥；真机默认只监控 |
| `sim/hand_console.py` | 灵巧手调试和动作播放 |
| `sim/arm_console.py` | 机械臂调试，默认 mock 和低速 |
| `sim/app_web.py` | Web 工作台和 WebSocket 后端 |
| `sim/skills/` | 技能清单、执行后端和安全闸 |
| `sim/build_nero_inspire.py` | 生成臂、法兰、手装配 URDF |
| `sim/build_combo_viz.py` | 生成本地 Web 合体模型 |
| `sim/build_canonical.py` | 视频到规范层 |
| `sim/derive_embodiment.py` | 规范层到本体轨迹/数据集 |

## 开发环境

Web、retargeting 与 ROS Python ABI 依赖当前 `lerobot` Python 3.10 环境：

```bash
conda activate lerobot
python --version
python sim/app_web.py
```

仅做纯 Python mock 单测时，可按测试文件要求使用系统 Python。不要用是否能 import
来推断真机驱动已经可用。

## 常见改动

### 修改灵巧手映射或限位

同步检查：

1. `sim/inspire_hand.py` 的 `HAND_JOINTS`、`HAND_LIMITS`、`RAW_MAP`
2. `sim/skills/hand_pose.py` 的复制表
3. `assets/hand/urdf/inspire_hand_right.urdf`
4. 浏览器模型和 retargeting 配置
5. `HARDWARE.md`

当前基线校验会失败，不得忽略：

```bash
python3 sim/skills/hand_pose.py --verify
```

已知差异是拇指弯曲 `0.48` 对 `0.69813/0.6`，以及四指 `1.333` 对
`1.39626/1.47`，共 10 项。修复必须基于硬件/URDF/动作安全决策，而不是为了让测试变绿
随意选一组数。

### 修改装配位置

当前参数定义在 `sim/build_nero_inspire.py`：

```python
FLANGE_MOUNT_XYZ = "0 0 0.016489"
FLANGE_MOUNT_RPY = "0 0 1.570796"
MOUNT_XYZ = "0.000042 0. 0.002158"
MOUNT_RPY = "0 0 1.570796"
```

修改后执行：

```bash
python3 sim/build_nero_inspire.py
python3 sim/build_combo_viz.py
```

不要依赖旧迁移报告里的代码行号或 `MOUNT_XYZ/MOUNT_RPY`；那些文档保留的是当时状态。

### 修改 Web 摄像头链路

现行前端使用本地 vendored MediaPipe Tasks。传输采用单帧在途和
latest-frame-wins，WebSocket 失败时降级 HTTP。修改后至少运行：

```bash
node sim/web/tests/hand_tracker_tasks.test.mjs
node sim/web/tests/hand_mimic_transport.test.mjs
```

浏览器实测还应覆盖：启动、停止、重复启动、权限拒绝、WebSocket 断线恢复和页面切换。

### 修改 MCP 或部署 Bridge

在独立仓库工作：

```bash
cd /home/zhang123/ros2_ws/robot-mcp-server
```

现行 MCP 入口为 `/mcp`，能力为手和臂，不包含 combo、视觉 mimic 或通用 `/execute`。
共享驱动变更应分别核对 `robot-bridge/sim/` 与本仓 `sim/`，不要假定它们自动同步。

## 验证层级

按风险逐层执行，文档审查期间不运行真实运动命令：

```bash
# 只读/静态
python3 sim/skills/hand_pose.py --verify

# Web 前端
node sim/web/tests/hand_tracker_tasks.test.mjs
node sim/web/tests/hand_mimic_transport.test.mjs

# 独立 MCP 仓库
cd /home/zhang123/ros2_ws/robot-mcp-server
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s mcp_server/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robot-bridge/tests -v
```

真机验证必须显式安排，记录硬件、固件、速度、初始姿态和急停条件。mock 通过只能说明
软件链路可运行，不能证明真机移动、安全限位或接线正确。

## 提交前检查

- `git diff` 中没有凭据、私钥、生成缓存或无关用户改动。
- 行为变化有对应测试；硬件变化有明确的真机验证状态。
- 更新 `CHANGELOG.md`、`PROJECT_STATUS.md` 和相关操作文档。
- 新 MCP 事实来自独立仓库代码，而不是本仓内嵌快照。
- 历史文档只加状态说明，不重写历史事实。

---

**最后核对**：2026-08-14
