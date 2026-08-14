# VLA-HandArm

NERO 七自由度机械臂、因时 RH56DFX 灵巧手、Web 调试工作台和 VLA 数据管线。

本仓库用于本体开发、仿真、标定、数据处理和 Web 调试。对外部署的 MCP Server
及轻量硬件 Bridge 已拆分到独立仓库：

- 本地目录：`/home/zhang123/ros2_ws/robot-mcp-server`
- GitHub：`git@github.com:ZhangDaMengxx/Moshu-robot-mcp-server.git`
- 现行基准：`main` 分支；本页最后核对时为 `f4e1c7e`（2026-08-14）

```text
[MCP client] -- JSON-RPC /mcp --> [MCP Server] -- HTTP --> [robot-bridge]
                                                           | RS485 / CAN
                                                           v
                                                     [灵巧手 / 机械臂]
```

## 从哪里开始

| 目标 | 入口 |
|------|------|
| 部署 MCP Server 或硬件 Bridge | `/home/zhang123/ros2_ws/robot-mcp-server/README.md` |
| 查看本项目文档状态 | [README_DOCS.md](README_DOCS.md) |
| 调试硬件 | [HARDWARE.md](HARDWARE.md) |
| 修改本项目代码 | [HANDBOOK.md](HANDBOOK.md) |
| 查看当前进度和已知问题 | [PROJECT_STATUS.md](PROJECT_STATUS.md) |
| 部署完整 Web/ROS2 真机主机 | [deploy/README.md](deploy/README.md) |

> `mcp_server/` 和根目录 `bridge.py` 是拆仓前的内嵌快照，包含曾经试验过的
> combo、视觉 mimic 和 `/execute` 能力。它们不是当前 MCP 部署基准；不要从这些
> 文件推断线上接口，也不要与独立仓库混合部署。

## Web 工作台

```bash
conda activate lerobot
python sim/app_web.py
```

服务监听 `0.0.0.0:7860`。若本机存在 `ssl/key.pem` 和 `ssl/cert.pem` 会启用 HTTPS，
否则使用 HTTP。`localhost` 开发可以使用 HTTP；局域网摄像头访问需要可信 HTTPS。

主要能力：

- 机械臂、灵巧手和合体 3D 状态与调试
- 浏览器 MediaPipe Tasks Hand Landmarker；GPU 失败转 CPU，Tasks 失败转 Legacy
- `/ws/hand/mimic` 低延迟重定向和 latest-target 真机控制；最多一个待发目标和一个 ACK 在途
- retarget 后的真手目标使用六关节 One Euro 自适应滤波和 0.0005rad 分辨率门限；3D 预览不滤波
- HTTP 为断线时的 retarget/3D 预览降级路径，WebSocket 恢复前不驱动真手
- 联合动作录制和回放（这是 Web 工作台能力，不等于现行 MCP combo 工具）

实时手部控制保持 MediaPipe 21 点与 dex-retargeting 后端协议不变。浏览器不再在
WebSocket 返回后逐帧追加硬件 HTTP 请求；后端以 30Hz 投递最新目标并等待
`hand_console` 的真实 RS485 ACK。滤波状态按 WebSocket 隔离，超过 200ms 无有效目标、
硬件离线或连接断开时重置。实现与验收说明见
[sim/web/MEDIAPIPE_TASKS_MIGRATION.md](sim/web/MEDIAPIPE_TASKS_MIGRATION.md)。

## VLA 数据管线

```bash
pip install -r requirements.txt
python sim/build_nero_inspire.py
python sim/build_canonical.py
python sim/derive_embodiment.py --emit-traj
python sim/replay_rerun.py --serve
```

管线将人手视频转换为规范层，再映射到 NERO + Inspire 的关节轨迹和
LeRobotDataset。具体约定见 [sim/CANONICAL_SPEC.md](sim/CANONICAL_SPEC.md)。

## 目录

```text
bridge.py              拆仓前的硬件代理快照，不是部署基准
mcp_server/            拆仓前的 MCP 快照和历史文档
sim/                   驱动、Web、技能、标定、仿真和数据管线
assets/                URDF、mesh 和浏览器模型
data/                  动作包、标定和数据集
deploy/                完整 Web/ROS2 真机主机部署
```

## 当前安全约束

- 真机运动前必须确认工作区、低速、使能状态和急停可达。
- `sim/skills/hand_pose.py --verify` 当前有 10 项与 `sim/inspire_hand.py` 不一致，
  在修复并完成真机验证前，不得把手势可行域表视为已对齐。
- Web 的 7860 端口没有应用层鉴权，不应直接暴露到公网。
- `ssl/key.pem` 曾被 Git 跟踪。现有私钥不得继续作为共享或生产凭据使用。

---

**最后核对**：2026-08-14
