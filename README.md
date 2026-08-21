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
| 建立灵巧手可行域 Profile | [src/HAND_FEASIBILITY_AUTOMATION.md](src/HAND_FEASIBILITY_AUTOMATION.md) |
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
- 浏览器 MediaPipe Tasks Hand Landmarker；页面按本机能力选择自动、CPU/WASM 或 GPU/WebGL，macOS 显示 Apple GPU（WebGL/Metal）
- 切换功能页或关闭浏览器时，灵巧手张开、机械臂按安全条件回伸直位，随后释放串口和 CAN
- `/ws/hand/mimic` 同时输出 7 轴机械臂与 6 轴灵巧手目标；Mock 和真机共用协议与 IK 链
- 单一按钮完成当前手腕位置/姿态的联合锚定、冻结和重新锚定；首个有效手帧即可点击，随后固定采集 12 帧并做离群点剔除
- 锚点使用机械臂当前关节 FK，页面显示采样进度与位置/姿态抖动，避免启动跳变或无限等待稳定
- 灵巧手与机械臂 IK 已解耦；各链路 latest-only，最多一个待处理目标和一个在途操作，过期 IK 不下发
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
灵巧手目标在 retarget 后立即进入独立 30Hz mailbox，不等待机械臂 IK；IK 使用单 worker
和深度 1 的 pending 槽，响应携带最近完成机械臂结果的 `source_frame_id`。合体页默认
灵巧手速度 `1000`、机械臂速度 `50%`。

页面内切换会等待复位和断开完成；浏览器关闭或刷新则通过 `pagehide/sendBeacon` 尽力通知
后端执行同一释放流程。浏览器进程被强制结束、主机断电或网络中断时，该通知无法保证送达；
在服务端心跳/租约 watchdog 完成前，不能把它视为硬件通道的绝对释放保证。

## VLA 数据管线

```bash
pip install -r requirements.txt
python src/build_nero_inspire.py
python src/build_canonical.py                         # 新建 Capture，Ego 写入 <capture>/ego/
python src/derive_embodiment.py --emit-traj           # 默认读取最新 ready Capture
python src/replay_rerun.py --serve                    # 默认回放最新 ready Capture
```

管线将人手视频转换为规范层，再映射到 NERO + Inspire 的关节轨迹和
LeRobotDataset。正式数据默认保存在
`datasets/captures/capture_<YYYYMMDD>_<sequence>_<uuid>/`：`ego/` 是独立 Ego
LeRobotDataset，机器人数据集位于
`robot_datasets/<target>/target_revision_v001/retarget_v001/`。一次 Web 管线运行也固定使用
同一个 Capture，避免规范层、轨迹和验收报告串到不同批次。
Source 层会保留原视频、处理结果原文件或参与构建的原分辨率 RGB/对齐深度，并记录
Source -> Ego 帧映射；缺失的硬件时间戳保持为空，不会用 FPS 推算值冒充。
每次构建还会把所选版本化质量口径固化为 `source/quality_profile.json`。默认 profile 按
RGB 视频、旧 960×540 RGB-D 帧集或外部处理结果区分；未来固定相机 60 Hz 数采显式使用
`--quality-profile ego_fixed_rgbd_60hz_v1`。验收读取 Capture 快照，不按当前代码中的新阈值
重解释旧数据，并把 LeRobot 内部帧间隔一致性与真实 RGB/Depth 硬件同步分开报告。验收项还
显式标记绝对精度、稳定性代理和连续性；没有逐帧真值时，手腕绝对误差保持不可测，骨长波动
或深度连续性不会被当成绝对精度通过。
Ego `meta/coordinate_system.json` 使用 2.0 契约逐字段声明坐标语义：普通固定相机 RGB 的
`wrist_pose` 为 `episode0_camera`，带 `camera_to_world` 标定的 RGB-D 为 `scene_world`；
消费者读取该文件，不根据目录或输入类型猜坐标系。
每个新 Ego/RobotDataset 还会按 episode 建立 `annotations/episode_*.json`；默认状态是
`unreviewed`，后续构建不会覆盖人工审核。RobotDataset 的 `qa/episode_*.json` 记录帧索引、
state/action 有限值等自动检查，缺少碰撞、限位或指尖真值时明确标为 `not_evaluated`。
整个 Capture 可用 `src/verify_dataset.py --capture-bundle --capture-root <capture>` 校验 Source、
环境快照、严格 v3、血缘、sidecar 覆盖和 SHA-256。

旧 `src/out/` 不会自动移动或删除；只有显式传 `--legacy-out`（Web 使用
`VLA_LEGACY_OUT=1`）才会读取旧产物。当前迁移只改变路径并保留既有 `xyzw` 四元数和数值
运算。数据集专用 Python 3.12.13 + `lerobot[dataset]==0.6.1` 环境和严格 v3 校验已通过；
完整 Source RGB-D 原始流与真实硬件微秒时间戳仍需由后续采集设备提供。目录说明见
[datasets/captures/README.md](datasets/captures/README.md)，字段约定见
[src/CANONICAL_SPEC.md](src/CANONICAL_SPEC.md)。

## 目录

```text
bridge.py              拆仓前的硬件代理快照，不是部署基准
mcp_server/            拆仓前的 MCP 快照和历史文档
src/                   驱动、Web、技能、标定、仿真和数据管线
src/test/              离线测试；hardware/ 为需显式运行的真机脚本
assets/                URDF、mesh 和浏览器模型
data/                  动作包、标定和数据集
datasets/captures/     Capture Bundle 根目录；真实 Capture 默认不进入 Git
configs/quality_profiles/  版本化数采能力与验收阈值
configs/hands/             灵巧手型号、资产标称角和自动探测策略
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
- 通用可行域探测器已经能按 datasheet/URDF 标称角生成条件化 Profile，但尚未运行真机
  commissioning，也尚未接入 Web、Bridge 或 `hand_pose` 的运行时安全投影。
- Web 的 7860 端口没有应用层鉴权，不应直接暴露到公网。
- 浏览器关闭释放是尽力而为；无人值守真机运行不得依赖它替代急停或服务端租约。
- `ssl/key.pem` 曾被 Git 跟踪。现有私钥不得继续作为共享或生产凭据使用。

---

**最后核对**：2026-08-21
