# Windows 部署入口

本仓曾同时维护 Bridge 和 MCP Server 的 Windows 指南。现行实现已经迁移到独立仓库，
不要从本仓根目录 `bridge.py` 或 `mcp_server/` 组合部署。

使用：

```text
/home/zhang123/ros2_ws/robot-mcp-server/README.md
/home/zhang123/ros2_ws/robot-mcp-server/robot-bridge/WINDOWS_DEPLOY.md
```

现行 Windows 支持：

- RH56DFX 灵巧手：RS485 转 USB，显式指定 `COM` 端口
- NERO 机械臂：松灵 CAN 适配器和 `agx_cando`
- 只用灵巧手，或灵巧手 + 机械臂
- 机械臂连接失败时降级为仅灵巧手
- mock 模式验证完整 HTTP/MCP 链路
- 标准 MCP `/mcp` 和 10 个工具

## 边界提示

- Windows 与 WSL 会争用 USB 设备；通过 usbipd attach 到 WSL 后，Windows 原生进程
  不能同时打开它。
- 真机灵巧手连接失败不会静默回退 mock。
- 机械臂上电后默认未使能，运动前必须确认现场安全。
- 急停后无抱闸的机械臂可能缓慢下落。
- `arm_reset` 是退出急停并重新使能，不等同于“7 个关节回零”。
- 现行 MCP 不提供 `arm_move` 技能、combo 工具或 `/execute`。

## 本仓 Web 工作台

如果目标是在 Windows 浏览器访问 WSL 中的本仓 Web 工作台，而不是部署 MCP，请看：

- [WSL_CAMERA_SETUP.md](WSL_CAMERA_SETUP.md)
- [HTTPS_SETUP.md](HTTPS_SETUP.md)
- [deploy/README.md](deploy/README.md)

---

**最后核对**：2026-08-14
