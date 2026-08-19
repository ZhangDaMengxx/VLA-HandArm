# VLA-HandArm

> AI 与自动化工具首先阅读 [README_DOCS.md](README_DOCS.md)；根目录
> [AGENTS.md](AGENTS.md) 定义文档优先级和仓库边界。

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
python src/app_web.py
```

服务监听 `0.0.0.0:7860`。若本机存在 `ssl/key.pem` 和 `ssl/cert.pem` 会启用 HTTPS，
否则使用 HTTP。`localhost` 开发可以使用 HTTP；局域网摄像头访问需要可信 HTTPS。

主要能力：

- 机械臂、灵巧手和合体 3D 状态与调试；实时视频跟随只在“实时 Live · 合体”页提供
- 浏览器 MediaPipe Tasks Hand Landmarker；GPU 失败转 CPU，Tasks 失败转 Legacy
- `/ws/hand/mimic` 同时输出 7 轴机械臂与 6 轴灵巧手目标；Mock 和真机共用协议与 IK 链
- 单一按钮完成当前手腕位置/姿态的联合锚定、冻结和重新锚定；首个有效手帧即可点击，随后固定采集 12 帧并做离群点剔除
- 锚点使用机械臂当前关节 FK，页面显示采样进度与位置/姿态抖动，避免启动跳变或无限等待稳定
- latest-target 真机控制最多一个待发目标和一个 ACK 在途；真臂实时跟随默认不授权
- retarget 后的真手目标使用六关节 One Euro 自适应滤波和 0.0005rad 分辨率门限；3D 预览不滤波
- HTTP 为断线时的 retarget/3D 预览降级路径，WebSocket 恢复前不驱动真手
- 联合动作录制和回放（这是 Web 工作台能力，不等于现行 MCP combo 工具）

实时手部控制保持 MediaPipe 21 点与 dex-retargeting 后端协议不变。浏览器不再在
WebSocket 返回后逐帧追加硬件 HTTP 请求；后端以 30Hz 投递最新目标并等待
`hand_console` 的真实 RS485 ACK。滤波状态按 WebSocket 隔离，超过 200ms 无有效目标、
硬件离线或连接断开时重置。实现与验收说明见
[src/web/MEDIAPIPE_TASKS_MIGRATION.md](src/web/MEDIAPIPE_TASKS_MIGRATION.md)。

合体跟随同时传输 MediaPipe world landmarks 和 image landmarks：前者用于手型重定向与
手掌姿态，后者通过手掌表观尺度估计腕部相对位置。该单目位置明确标记为
`monocular_scale`，只适合锚定后的有限范围相对控制，不是绝对米制真值。Mock 模式已完成
WebSocket、IK 和 Three.js 臂手联动验收；真实机械臂尚未验证。丢手、左右手变化、连续
IK 失败、急停/冻结、未使能或断线都会停止机械臂目标投递并冻结跟随。

## VLA 数据管线

```bash
pip install -r requirements.txt
python src/build_nero_inspire.py
python src/build_canonical.py
python src/derive_embodiment.py --emit-traj
python src/replay_rerun.py --serve
```

管线将人手视频转换为规范层，再映射到 NERO + Inspire 的关节轨迹和
LeRobotDataset。具体约定见 [src/CANONICAL_SPEC.md](src/CANONICAL_SPEC.md)。

## 目录

```text
bridge.py              拆仓前的硬件代理快照，不是部署基准
mcp_server/            拆仓前的 MCP 快照和历史文档
src/                   驱动、Web、技能、标定、仿真和数据管线
src/test/              离线测试；hardware/ 为需显式运行的真机脚本
assets/                URDF、mesh 和浏览器模型
data/                  动作包、标定和数据集
third_party/            上游源码、厂商 SDK、外部数据和项目 overlay
deploy/                完整 Web/ROS2 真机主机部署
```

第三方内容的来源边界和 Git 策略见
[third_party/README.md](third_party/README.md)。上游资产保留原始内部目录结构；只有
`third_party/overlays/` 由本项目维护并进入 Git。

## 当前安全约束

- 真机运动前必须确认工作区、低速、使能状态和急停可达。
- `src/skills/hand_pose.py --verify` 当前有 10 项与 `src/inspire_hand.py` 不一致，
  在修复并完成真机验证前，不得把手势可行域表视为已对齐。
- Web 的 7860 端口没有应用层鉴权，不应直接暴露到公网。
- `ssl/key.pem` 曾被 Git 跟踪。现有私钥不得继续作为共享或生产凭据使用。

---

**最后核对**：2026-08-19
