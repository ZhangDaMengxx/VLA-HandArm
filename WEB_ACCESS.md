# Web、Windows 与 WSL 访问

本文统一说明本仓 Web 工作台的 Windows/WSL 访问、浏览器摄像头、HTTPS，以及与独立
MCP Bridge 的边界。

## 边界

本仓 `src/app_web.py` 是完整 Web/ROS2 开发工作台。MCP Server 和可部署的 Windows
Robot Bridge 位于独立仓库：

```text
/home/zhang123/ros2_ws/robot-mcp-server/README.md
/home/zhang123/ros2_ws/robot-mcp-server/robot-bridge/WINDOWS_DEPLOY.md
```

不要从本仓根目录 `bridge.py` 或 `mcp_server/` 组合部署。Windows 与 WSL 会争用 USB
设备；通过 usbipd attach 到 WSL 后，Windows 原生进程不能同时打开该设备。

## 数据路径

摄像头由 Windows 浏览器读取，WSL 后端不直接访问摄像头设备。浏览器本地运行
MediaPipe Tasks Hand Landmarker，只把 21 个世界坐标关键点发送给 WSL：

```text
Windows 浏览器                         WSL2
getUserMedia                           src/app_web.py :7860
MediaPipe Tasks 21 点  -- WebSocket --> /ws/hand/mimic
                         HTTP fallback  /api/hand/mimic
```

视频帧不会发送给 WSL，但关键点仍属于行为或生物特征数据，应按项目数据策略处理。

## 本机启动

```bash
cd /home/zhang123/ros2_ws/lerobotTest
conda activate lerobot
python src/app_web.py
```

服务监听 `0.0.0.0:7860`。Windows 浏览器在本机开发时优先访问：

```text
http://localhost:7860
```

`localhost` 属于浏览器安全上下文例外，通常可以在 HTTP 下使用摄像头。只有其他设备
通过局域网 IP 访问时才需要可信 HTTPS 和受限的防火墙入站规则。

## HTTPS

浏览器 `getUserMedia()` 只允许可信 HTTPS，以及 `localhost`/`127.0.0.1` 的本机例外。
局域网证书应包含所有实际访问名。以 `mkcert` 为例：

```bash
mkdir -p ssl
mkcert -key-file ssl/key.pem -cert-file ssl/cert.pem \
  localhost 127.0.0.1 192.168.1.189
chmod 600 ssl/key.pem
```

IP 只是示例。客户端必须信任签发 CA；跳过浏览器警告不等于可信部署。

仓库中的 `ssl/key.pem` 和 `ssl/cert.pem` 曾被 Git 跟踪，现有私钥应视为已泄露：

- 不得继续用于共享或生产环境。
- 新私钥必须保持未跟踪，不能放入聊天、Issue、日志或提交。
- 移出当前跟踪不能消除 Git 历史副本；历史清理应单独评估。

`src/app_web.py` 仅在 `ssl/key.pem` 和 `ssl/cert.pem` 同时存在时启用 HTTPS，不负责
验证证书是否可信、过期或匹配主机名。

生产或公网环境不能直接暴露无鉴权的 `7860` 端口。应使用 Caddy/Nginx，并配置证书
续期、身份认证、来源限制、审计和 WebSocket 代理。MCP 公网部署应使用独立仓库的
`frp_deploy.md`，不要与 Web 工作台证书或 Bridge 凭据混用。

证书检查：

```bash
openssl x509 -in ssl/cert.pem -noout -subject -issuer -dates -ext subjectAltName
```

## 前端实现与自动测试

- MediaPipe Tasks、WASM 和模型位于 `src/web/vendor/mediapipe-tasks/`，不依赖运行时 CDN。
- 同一时刻只允许一帧请求在途，采集更快时只保留最新待发帧。
- WebSocket 断线时使用 HTTP 降级；`stop()` 后不会继续重连。

```bash
node src/test/web/hand_tracker_tasks.test.mjs
node src/test/web/hand_mimic_transport.test.mjs
```

自动测试不访问摄像头，也不证明浏览器权限、GPU/WASM 性能或真机运动正确。

## 浏览器验收

1. 打开 Replay/实时摄像头入口并授权摄像头。
2. 确认画面、关键点覆盖和 Network 中的 `/ws/hand/mimic`。
3. 人为断开 WebSocket，确认 HTTP 降级且没有旧帧堆积。
4. 恢复网络，确认只创建一个新连接。
5. 停止摄像头或切换页面，确认媒体轨道、定时器和连接均停止。
6. 记录采集 FPS、发送 FPS、后端处理时间和端到端延迟。

## 常见问题

### Windows 无法访问 localhost

```bash
ss -ltnp | rg ':7860'
```

确认后端监听成功和 Windows 到 WSL 的 localhost 转发正常。

### 摄像头权限被拒绝

检查 Windows 隐私设置、浏览器站点权限和页面是否为安全上下文。摄像头被其他应用
独占也会导致失败。

### 模型或 WASM 加载失败

检查浏览器控制台和 `src/web/vendor/mediapipe-tasks/SHA256SUMS`。当前资源来自本地，
排查重点不是 CDN。

### 有关键点但没有真手动作

确认 Web API 返回值和 UI 的 mock/连接状态。3D 模型变化不代表真机已经运动；同一串口
不能同时被 Web、Bridge 和手控制台占用。

---

**最后核对**：2026-08-18
