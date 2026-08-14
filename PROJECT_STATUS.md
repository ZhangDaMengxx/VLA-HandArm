# 项目状态

**核对日期**：2026-08-14
**阶段**：Web 实时遥操作收尾、真机安全对齐、MCP 拆仓后的文档治理

## 当前结论

| 模块 | 状态 | 说明 |
|------|------|------|
| 硬件驱动 | 开发中 | 臂/手驱动已具备，仍需系统真机验收 |
| Web 手部追踪 | 主链已通 | MediaPipe Tasks、WebSocket 和 HTTP 降级已接入 |
| Web 传输 | 单测通过 | 单帧在途、latest-frame-wins、停止后不重连已实现 |
| VLA 数据管线 | 可开发 | 规范层、映射和回放工具已存在，训练闭环未完成 |
| MCP/Bridge | 已拆仓 | 现行代码在 `/home/zhang123/ros2_ws/robot-mcp-server` |
| ROS2 | 待复核 | 独立仓库的关节命名和真机控制仍需验证 |
| 文档 | 已全面审查 | 现行与历史文档已分层，仍有代码级风险待处理 |

## MCP 现行基准

权威仓库：`/home/zhang123/ros2_ws/robot-mcp-server`，本次核对提交为
`f4e1c7e`（`feat: add bridge heartbeat monitoring`）。

现行能力：

- 标准 MCP：JSON-RPC `POST /mcp`，并支持相应 GET/DELETE 会话语义
- 兼容 REST：`/mcp_rest/tools/list`、`/mcp_rest/tools/call`，已弃用
- HTTP API：`/api/v1/hand/*`、`/api/v1/arm/*`
- 10 个 MCP 工具：4 个灵巧手工具、6 个机械臂工具
- Bridge 心跳、断线状态、恢复检测和降级健康检查
- 灵巧手真机连接失败时 Bridge 启动失败；机械臂失败时可 hand-only 运行

不属于现行 MCP 的能力：combo、视觉 mimic 和 Bridge `/execute`。这些只存在于本仓
内嵌实验快照或 Web 工作台，不应出现在部署能力清单中。

## 最近完成

### 2026-08-14

- 完成文档与本地硬件代码的首轮对齐
- 确认 MCP 独立仓库、分支、远端和最新提交
- 验证 MCP 心跳单测 3 项通过、机械臂 mock 单测 1 项通过
- Web 手势传输增加单帧在途、latest-frame-wins 和生命周期保护
- MediaPipe Tasks 改用本地资源并补充 Node 单测

### 2026-08-10 至 2026-08-13

- 迁移到 2025-04-18 灵巧手 URDF 和新关节命名
- 重组 `assets/` 并生成臂手装配与 Web 模型
- 接通浏览器摄像头、手部关键点、retargeting 和 WebSocket 后端
- 完成 NERO 驱动、控制台、技能和联合动作的一批 mock/离线开发

阶段性成功报告已经归档；它们不再作为当前验收依据。

## 当前风险

### Critical

1. **手势安全表与驱动不一致**

   `python3 sim/skills/hand_pose.py --verify` 报 10 项不一致。Bridge 在执行手势和角度
   可行域检查时依赖该表，因此在完成参数决策和真机验证前，不能宣称安全表已对齐。

2. **仓库中存在已跟踪 TLS 私钥**

   `ssl/key.pem` 与证书已进入 Git。该私钥必须轮换，不应继续用于共享或生产场景；
   移出跟踪和历史处理需要单独执行。

### Major

3. `verify_migration.py` 仍引用 `assets/inspire_hand/`，当前为 `assets/hand/`。
4. `final_summary.py` 在路径失效时仍可能输出“全部完成”并退出 0。
5. Web 摄像头仍缺真实浏览器的 FPS、延迟和断线恢复验收数据。
6. 真机运动、急停、复位、限位和 MCP 断线行为尚未形成完整验收记录。

## 验证记录

已通过：

```bash
node sim/web/tests/hand_tracker_tasks.test.mjs
node sim/web/tests/hand_mimic_transport.test.mjs

cd /home/zhang123/ros2_ws/robot-mcp-server
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s mcp_server/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robot-bridge/tests -v
```

当前预期失败：

```bash
python3 sim/skills/hand_pose.py --verify
python3 verify_migration.py
```

文档审查没有运行任何真实硬件运动命令。

## 下一步

1. 统一手势安全表与驱动参数，并同步两个仓库。
2. 轮换和停止跟踪 TLS 私钥。
3. 修复迁移验收脚本的路径及退出码。
4. 完成浏览器与真机验收。
5. 复核 ROS2 独立仓库的关节命名和控制链。

详细行动项见 [TODO.md](TODO.md)。
