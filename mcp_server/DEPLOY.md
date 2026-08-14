# MCP 与 Robot Bridge 部署入口

> 本仓 `mcp_server/` 和根目录 `bridge.py` 是拆仓前快照，不再作为部署源。

现行部署仓库：

```bash
cd /home/zhang123/ros2_ws/robot-mcp-server
git status
```

文档入口：

| 场景 | 文档 |
|------|------|
| 总体安装、Linux/Windows 启动 | `README.md` |
| Bridge 依赖、参数、路由 | `robot-bridge/README.md` |
| Windows 灵巧手 + 机械臂 | `robot-bridge/WINDOWS_DEPLOY.md` |
| 公网 FRP、TLS、鉴权和心跳 | `frp_deploy.md` |
| MCP 容器配置 | `mcp_server/docker-compose.yml`、`docker-compose.frp.yml` |

## 本地 mock 验证

在独立仓库中分别启动：

```bash
# 终端 1
cd /home/zhang123/ros2_ws/robot-mcp-server/robot-bridge
python bridge.py --mock --host 127.0.0.1 --port 9000

# 终端 2
cd /home/zhang123/ros2_ws/robot-mcp-server/mcp_server
ROBOT_BRIDGE_URL=http://127.0.0.1:9000 MCP_SECURITY_MODE=lan \
  python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

验证健康状态和标准 MCP：

```bash
curl http://127.0.0.1:8000/health
curl -X POST http://127.0.0.1:8000/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}'
```

## 关键语义

- 不加 `--mock` 时，灵巧手连接失败会阻止 Bridge 启动。
- 真机模式下机械臂连接失败会告警并降级为仅灵巧手运行。
- MCP Server 可在 Bridge 不可用时降级启动，并通过心跳检测恢复。
- public 模式必须配置 API Key；公网部署使用独立仓库的 FRP/TLS 方案。
- 现行 MCP 不包含 combo、视觉 mimic 或 `/execute`。
- `hand_pose.py` 与驱动参数当前仍有 10 项不一致，真机手势测试前必须处理。

---

**最后核对**：2026-08-14
