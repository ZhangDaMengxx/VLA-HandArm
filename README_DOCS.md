# 项目文档导航

本文是 `lerobotTest` 文档清单和时效性入口。文档分为“现行”“专题参考”“历史归档”
三类；历史文档保留当时事实，不作为当前命令、路径或接口依据。

## 现行文档

| 文档 | 内容 | 当前基准 |
|------|------|----------|
| [README.md](README.md) | 项目入口和仓库边界 | 2026-08-14 |
| [HANDBOOK.md](HANDBOOK.md) | 开发入口、关键模块和验证命令 | 2026-08-14 |
| [HARDWARE.md](HARDWARE.md) | 真机规格、运行时参数和安全约束 | 2026-08-14 |
| [PROJECT_STATUS.md](PROJECT_STATUS.md) | 当前进度、风险和近期任务 | 2026-08-14 |
| [TODO.md](TODO.md) | 可执行待办 | 2026-08-14 |
| [CHANGELOG.md](CHANGELOG.md) | 本仓库变更历史 | 持续维护 |
| [GIT_GUIDE.md](GIT_GUIDE.md) | 三个仓库的 Git 约定 | 2026-08-14 |
| [deploy/README.md](deploy/README.md) | 完整 Web/ROS2 真机主机部署 | 现行 |
| [HTTPS_SETUP.md](HTTPS_SETUP.md) | Web 摄像头 HTTPS 和证书安全 | 2026-08-14 |
| [WSL_CAMERA_SETUP.md](WSL_CAMERA_SETUP.md) | WSL + Windows 浏览器摄像头验证 | 2026-08-14 |

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

| 文档 | 主题 |
|------|------|
| [sim/README.md](sim/README.md) | Web/技能/回放模块总览 |
| [sim/ARM_DEBUG.md](sim/ARM_DEBUG.md) | 机械臂调试记录与现状 |
| [sim/HAND_DEBUG.md](sim/HAND_DEBUG.md) | 灵巧手调试记录与现状 |
| [sim/COMBO_DEBUG.md](sim/COMBO_DEBUG.md) | 本地 Web combo 调试；不是 MCP combo |
| [sim/HAND_SAFETY_PLAN.md](sim/HAND_SAFETY_PLAN.md) | 灵巧手安全方案和未完成项 |
| [sim/CANONICAL_SPEC.md](sim/CANONICAL_SPEC.md) | VLA 规范层数据契约 |
| [VISUALIZER_SPEC.md](VISUALIZER_SPEC.md) | 可视化约定 |
| [sim/build_urdf/README.md](sim/build_urdf/README.md) | URDF 装配分析工具 |
| [adjust_hand_mount_example.md](adjust_hand_mount_example.md) | 当前装配参数调整流程 |
| [WINDOWS_DEPLOY.md](WINDOWS_DEPLOY.md) | 本仓库 Windows 开发参考；MCP 部署看独立仓库 |
| [sim/web/MEDIAPIPE_TASKS_MIGRATION.md](sim/web/MEDIAPIPE_TASKS_MIGRATION.md) | 当前 MediaPipe Tasks 迁移和兼容契约 |
| [sim/web/vendor/mediapipe-tasks/README.md](sim/web/vendor/mediapipe-tasks/README.md) | 本地模型/WASM 来源和校验值 |
| [overlays/dex-retargeting/example/vector_retargeting/VISUALIZER_ARCH.md](overlays/dex-retargeting/example/vector_retargeting/VISUALIZER_ARCH.md) | overlay 可视化结构参考 |

## 历史归档

下列文档记录 2026-08-10 至 2026-08-11 的迁移、阶段性验证或已撤回实验。文中的
旧目录、代码行号、通过结论和接口不保证仍有效：

- [MIGRATION_2026_08_10.md](MIGRATION_2026_08_10.md)
- [MIGRATION_README.md](MIGRATION_README.md)
- [ASSET_MIGRATION_PLAN.md](ASSET_MIGRATION_PLAN.md)
- [ASSEMBLY_SUCCESS.md](ASSEMBLY_SUCCESS.md)
- [WEB_COMBO_UPDATE.md](WEB_COMBO_UPDATE.md)
- [sim/URDF_FIX_2026-08-10.md](sim/URDF_FIX_2026-08-10.md)
- [DOC_SUMMARY.md](DOC_SUMMARY.md)
- [DYNAMIC_SCAN_SUMMARY.md](DYNAMIC_SCAN_SUMMARY.md)
- [mcp_server/TEST_REPORT.md](mcp_server/TEST_REPORT.md)
- [更新日志.md](更新日志.md)（旧日志，后续记录看 `CHANGELOG.md`）
- [assets/nero_official/README.md](assets/nero_official/README.md)（厂商资产来源记录）
- [assets/arm_legacy/nero_official/README.md](assets/arm_legacy/nero_official/README.md)（重复的旧资产来源记录）
- `user-package/README.md`、`user-package/USER_GUIDE.md`（拆仓前用户包）

`FILE_LIST.md`、`PROJECT_PLAN.md` 和 `QUICKSTART.md` 是阶段性清单/方案，顶部已标出
时效性；使用其中命令前先对照本页和当前代码。

## 审查范围

本次审查覆盖项目团队维护的入口、硬件、Web、MCP、部署、迁移和调试文档。以下内容按
上游或生成物处理，不改写其技术正文：

- `dex-retargeting-main/`、`dex-urdf-main/`、`egozero-main/`
- `pinocchio-kinematics-lite-main/`、`pyAgxArm-master/`、`unitree_sdk2-main/`
- `.pytest_cache/` 与各测试缓存中的 README

引用这些文档时应同时记录上游版本；它们不能替代本项目的当前状态和安全约束。

## 维护规则

- 修改运行方式、接口或目录时，同一提交更新 `README.md`、`HANDBOOK.md` 和本页。
- 修改硬件参数时更新 `HARDWARE.md`，并运行相关只读/模拟校验。
- 历史报告不改写当时事实，只在顶部注明归档状态和当前入口。
- MCP 部署变化首先更新独立 `robot-mcp-server`，本仓只维护边界和链接。
- 文档不得记录私钥、API Key、个人 SSH 指纹或可复用凭据。

## 当前未闭环文档风险

1. `sim/skills/hand_pose.py --verify` 有 10 项参数不一致。
2. `verify_migration.py` 和 `final_summary.py` 仍引用旧资产路径，不能作为当前验收工具。
3. Git 历史中存在 `ssl/key.pem`；应轮换并另行处理跟踪/历史清理。

---

**最后全面审查**：2026-08-14
