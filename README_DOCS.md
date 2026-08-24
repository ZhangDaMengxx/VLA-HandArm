<!-- AI_DOCUMENTATION_ENTRYPOINT: READ_THIS_FIRST -->

# AI / 项目文档导航

本文是 `lerobotTest` 文档清单和时效性入口。文档分为“现行”“专题参考”“历史归档”
三类；历史文档保留当时事实，不作为当前命令、路径或接口依据。

> **AI 必读入口。** 在分析或修改本仓库前，先完整阅读本页；判断冲突时，现行文档
> 优先于专题参考，专题参考优先于历史归档。根目录 `AGENTS.md` 会把支持该约定的
> AI 工具直接引导到本页。

## 现行文档

| 文档 | 内容 | 当前基准 |
|------|------|----------|
| [README.md](README.md) | 项目入口和仓库边界 | 2026-08-21 |
| [HANDBOOK.md](HANDBOOK.md) | 开发入口、关键模块和验证命令 | 2026-08-21 |
| [HARDWARE.md](HARDWARE.md) | 真机规格、运行时参数和安全约束 | 2026-08-21 |
| [PROJECT_STATUS.md](PROJECT_STATUS.md) | 当前进度、风险和近期任务 | 2026-08-21 |
| [TODO.md](TODO.md) | 可执行待办 | 2026-08-21 |
| [CHANGELOG.md](CHANGELOG.md) | 本仓库变更历史 | 持续维护 |
| [GIT_GUIDE.md](GIT_GUIDE.md) | 三个仓库的 Git 约定 | 2026-08-14 |
| [deploy/README.md](deploy/README.md) | 完整 Web/ROS2 真机主机部署 | 现行 |
| [WEB_ACCESS.md](WEB_ACCESS.md) | Web、Windows、WSL、摄像头与 HTTPS | 2026-08-18 |

## MCP 与 Bridge

现行部署代码不在本仓库，权威来源是：

```text
/home/zhang123/ros2_ws/robot-mcp-server
git@github.com:ZhangDaMengxx/Moshu-robot-mcp-server.git
```

其中 `README.md`、`frp_deploy.md`、`robot-bridge/README.md` 和
`robot-bridge/WINDOWS_DEPLOY.md` 是部署入口。现行标准接口是 JSON-RPC
`POST /mcp`，旧 REST 兼容接口位于 `/mcp_rest/tools/*`。

本仓库的 `mcp_server/` 是拆仓前快照：

| 文档 | 状态 |
|------|------|
| [mcp_server/README.md](mcp_server/README.md) | 当前边界和 10 个工具的接口速查 |
| [mcp_server/DEPLOY.md](mcp_server/DEPLOY.md) | 跳转到独立仓库的部署说明 |
| [mcp_server/DOCKER_BUILD.md](mcp_server/DOCKER_BUILD.md) | 旧镜像构建参考，部署前核对独立仓库 |
| [mcp_server/SERVER_DEPLOY.md](mcp_server/SERVER_DEPLOY.md) | 旧服务器部署参考，部署前核对独立仓库 |
| `API_DESIGN.md`、`DYNAMIC_COMBO.md`、`TEST_REPORT.md` | 已归档，不代表现行服务 |

## 专题技术文档

这些文档描述仍存在的本地开发模块。涉及真机时，仍以代码、硬件手册和当前状态页为准。

### 四元数兼容说明（必须遵守）

本仓库当前的兼容数据契约是 **`xyzw`**：

```text
[qx, qy, qz, qw]
```

外部 EGO 规范或其他工具可能使用 **`wxyz`**：

```text
[qw, qx, qy, qz]
```

两者只是同一个四元数的字段排列不同。边界转换仅移动 `w` 的位置：

```python
q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
```

只要读取端按对应顺序解析，旋转矩阵、平移、齐次变换和坐标系语义都不改变。它不能与
`XYZ/ZYX` 等 Euler 轴序、主动/被动旋转或 `c2w/w2c` 变换方向混淆；那些属于不同的
旋转/变换约定，不能通过简单换列解决。现有 Capture、Canonical、IK、回放和校验器继续
使用 `xyzw`，禁止仅修改文档字段顺序或原地改写旧数据；如需 `wxyz`，必须使用显式版本化
导出/导入适配器并通过旋转矩阵往返测试。

| 文档 | 主题 |
|------|------|
| [src/README.md](src/README.md) | Web/技能/回放模块总览 |
| [src/ARM_DEBUG.md](src/ARM_DEBUG.md) | 机械臂调试记录与现状 |
| [src/HAND_DEBUG.md](src/HAND_DEBUG.md) | 灵巧手调试、实时摄像头控制和真机性能记录 |
| [src/COMBO_DEBUG.md](src/COMBO_DEBUG.md) | 本地 Web combo 调试；不是 MCP combo |
| [src/HAND_SAFETY_PLAN.md](src/HAND_SAFETY_PLAN.md) | 灵巧手安全方案和未完成项 |
| [src/HAND_LIMIT_AUDIT_2026_08_21.md](src/HAND_LIMIT_AUDIT_2026_08_21.md) | 灵巧手 span/limit、Bridge 安全闸、动作包影响与真机校验准入 |
| [src/HAND_FEASIBILITY_AUTOMATION.md](src/HAND_FEASIBILITY_AUTOMATION.md) | 多手型资产规范、自动可行域探测、Profile 和安全投影 |
| [src/camera/README.md](src/camera/README.md) | 相机 Adapter 与 NERO 臂上相机手眼标定 |
| [src/lerobot_v3/README.md](src/lerobot_v3/README.md) | Python 3.12 主运行时与 ROS Humble 3.10 薄桥边界 |
| [EGO_DATA_STANDARD.md](EGO_DATA_STANDARD.md) | 眼镜主视角 EGO 采集、质量阈值、坐标时间和交付规范 |
| [src/CANONICAL_SPEC.md](src/CANONICAL_SPEC.md) | VLA 规范层数据契约 |
| [datasets/captures/README.md](datasets/captures/README.md) | Capture Bundle、quality profile 快照和旧路径兼容边界 |
| [VISUALIZER_SPEC.md](VISUALIZER_SPEC.md) | 可视化约定 |
| [src/build_urdf/README.md](src/build_urdf/README.md) | URDF 装配分析工具 |
| [src/web/MEDIAPIPE_TASKS_MIGRATION.md](src/web/MEDIAPIPE_TASKS_MIGRATION.md) | MediaPipe Tasks、引擎降级、latest-target 和验收契约 |
| [src/web/vendor/mediapipe-tasks/README.md](src/web/vendor/mediapipe-tasks/README.md) | 本地模型/WASM 来源和校验值 |
| [third_party/README.md](third_party/README.md) | 第三方资产边界、目录和 Git 策略 |
| [third_party/overlays/dex-retargeting/example/vector_retargeting/VISUALIZER_ARCH.md](third_party/overlays/dex-retargeting/example/vector_retargeting/VISUALIZER_ARCH.md) | overlay 可视化结构参考 |

## 历史归档

下列文档记录 2026-08-10 至 2026-08-11 的迁移、阶段性验证或已撤回实验。文中的
旧目录、代码行号、通过结论和接口不保证仍有效：

- [MIGRATION_2026_08_10.md](MIGRATION_2026_08_10.md)
- [mcp_server/TEST_REPORT.md](mcp_server/TEST_REPORT.md)
- [更新日志.md](更新日志.md)（旧日志，后续记录看 `CHANGELOG.md`）
- [assets/nero_official/README.md](assets/nero_official/README.md)（厂商资产来源记录）
- [assets/arm_legacy/nero_official/README.md](assets/arm_legacy/nero_official/README.md)（重复的旧资产来源记录）
- `user-package/README.md`、`user-package/USER_GUIDE.md`（拆仓前用户包）

`MIGRATION_2026_08_10.md` 已合并同日资产方案、装配报告、Web Combo 更新和快速入口。
`mcp_server/DYNAMIC_COMBO.md` 保留已撤回的动态 Combo 实验详情。`PROJECT_PLAN.md` 是
阶段性方案，使用其中结论前先对照本页和当前代码。

## 审查范围

本次审查覆盖项目团队维护的入口、硬件、Web、MCP、部署、迁移和调试文档。以下内容按
上游或生成物处理，不改写其技术正文：

- `third_party/dex-retargeting/`、`third_party/dex-urdf/`、`third_party/egozero/`
- `third_party/pinocchio-kinematics-lite/`、`third_party/pyAgxArm/`、`third_party/unitree-sdk2/`
- `third_party/kinect2-middle/`（第三方 RGB-D 数据和工具）
- `.pytest_cache/` 与各测试缓存中的 README

引用这些文档时应同时记录上游版本；它们不能替代本项目的当前状态和安全约束。

## 维护规则

- 修改运行方式、接口或目录时，同一提交更新 `README.md`、`HANDBOOK.md` 和本页。
- 修改硬件参数时更新 `HARDWARE.md`，并运行相关只读/模拟校验。
- 历史报告不改写当时事实，只在顶部注明归档状态和当前入口。
- MCP 部署变化首先更新独立 `robot-mcp-server`，本仓只维护边界和链接。
- 文档不得记录私钥、API Key、个人 SSH 指纹或可复用凭据。

## 当前未闭环文档风险

1. 本仓驱动、手势安全表、ROS writer、正式 URDF及独立 Bridge 的资产标称映射已统一，
   离线一致性校验通过。2026-08-24 新真机 Profile 已完成六关节空载单关节全行程扫描；
   interaction commissioning 及 Web/Bridge 条件化安全投影仍未闭环，详见
   `src/HAND_LIMIT_AUDIT_2026_08_21.md`。
2. Git 历史中存在 `ssl/key.pem`；应轮换并另行处理跟踪/历史清理。
3. 摄像头真手链已有延迟样本及 GPU/CPU/Apple GPU 选择控件，One Euro 大死区台阶已修正；新门限真机复测、跨浏览器/macOS 和同幅度速度对照仍未闭环。
4. 合体腕姿跟随已完成 Mock、IK 和 Three.js 验收；灵巧手与机械臂 IK 已按 latest-only 异步解耦，但双链路性能和真实机械臂尚未验证，单目位置也不是绝对米制真值。
5. 页面内切换会等待硬件复位和断开；浏览器关闭依赖 `pagehide/sendBeacon` 尽力通知，
   服务端 heartbeat/lease watchdog 和多标签页通道所有权尚未实现。
6. Capture 路径、生命周期、Source 基础留存、版本化 quality profile、绝对/代理质量语义、
   坐标契约、Python 3.12 + LeRobot 0.6.1 完整离线链、严格 v3、episode sidecar 和 Capture 完整性校验已完成；
   设备原生 RGB-D/raw depth、真实硬件时间戳以及限位/碰撞/指尖误差的物理 QA 证据仍未闭环。

---

**最后全面审查**：2026-08-24
