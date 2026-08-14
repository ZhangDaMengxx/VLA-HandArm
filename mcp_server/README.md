# MCP Server 快照说明

> **状态：非部署基准。** 本目录是拆仓前的内嵌快照，并包含 combo 和视觉 mimic 等
> 分叉实验。现行代码、测试和部署文件位于
> `/home/zhang123/ros2_ws/robot-mcp-server/mcp_server`。

## 现行架构

```text
MCP client --> MCP Server --> robot-bridge --> RH56DFX / NERO
              独立仓库        硬件所在主机
```

标准 MCP 端点：

- `POST /mcp`：JSON-RPC initialize、tools/list、tools/call
- `GET /mcp`、`DELETE /mcp`：Streamable HTTP 会话相关请求
- `/mcp_rest/tools/*`：旧 REST 兼容接口，已弃用

`/mcp/tools/list` 和 `/mcp/tools/call` **不是当前路由**。

## 当前工具

| 工具 | 作用 |
|------|------|
| `hand_list_gestures` | 查询手势 ID |
| `hand_gesture` | 执行预设手势 |
| `hand_set_angles` | 设置 6 个灵巧手关节角 |
| `hand_status` | 查询灵巧手状态 |
| `arm_status` | 查询机械臂连接、使能、急停和关节状态 |
| `arm_set_joints` | 设置 7 个机械臂关节角 |
| `arm_enable` | 使能机械臂 |
| `arm_disable` | 下使能机械臂 |
| `arm_estop` | 机械臂急停 |
| `arm_reset` | 退出急停并重新使能 |

现行服务不提供 MCP combo、视觉 mimic 或通用 `/execute` 工具。

## 协议验证

从独立仓库启动服务后：

```bash
curl -X POST http://127.0.0.1:8000/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26","capabilities":{},"clientInfo":{"name":"curl","version":"1"}}}'
```

public 安全模式还需要 `X-API-Key`。客户端配置、FRP、TLS、心跳和服务器部署请看独立
仓库的 `README.md` 与 `frp_deploy.md`。

## 健康状态与心跳

现行 Server 支持：

- `ROBOT_HEARTBEAT_INTERVAL`，默认 5 秒
- `ROBOT_HEARTBEAT_TIMEOUT`，默认 2 秒
- Bridge 断线、恢复、连续失败次数、最后成功时间和最后错误状态
- Bridge 不可用时 Server 降级启动
- 运动请求断线后不自动重试，避免重复运动

## 本目录能否继续使用

只建议用于历史比较或本仓实验复现。部署、发布和问题修复应在独立仓库进行；若确需
同步共享驱动，必须显式对比两个目录并分别测试。

---

**最后核对**：2026-08-14，独立仓库提交 `f4e1c7e`
