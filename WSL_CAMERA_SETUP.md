# WSL 摄像头验证指南

摄像头由 Windows 浏览器读取，WSL 后端不直接访问摄像头设备。浏览器本地执行
MediaPipe Tasks Hand Landmarker，只把 21 个世界坐标关键点发给 WSL。

```text
Windows 浏览器                         WSL2
getUserMedia                           sim/app_web.py :7860
MediaPipe Tasks 21 点  -- WebSocket --> /ws/hand/mimic
                         HTTP fallback  /api/hand/mimic
```

视频帧不会发送给 WSL 后端，但关键点仍属于行为/生物特征数据，应按项目数据策略处理。

## 启动

```bash
cd /home/zhang123/ros2_ws/lerobotTest
conda activate lerobot
python sim/app_web.py
```

服务实际监听 `0.0.0.0:7860`。Windows 浏览器优先访问：

```text
http://localhost:7860
```

localhost 可以在 HTTP 下使用摄像头。通过局域网 IP 访问必须配置可信 HTTPS，见
[HTTPS_SETUP.md](HTTPS_SETUP.md)。

## 当前前端实现

- MediaPipe Tasks 代码、WASM 和 hand landmarker 模型保存在
  `sim/web/vendor/mediapipe-tasks/`，不依赖运行时 CDN。
- 同一时刻只允许一帧请求在途；采集更快时只保留最新待发帧。
- WebSocket 断线时使用 HTTP 降级，并在控制器仍 active 时尝试恢复。
- `stop()`、启动代次和定时器均参与生命周期判断，停止后不会继续重连。

## 自动化验证

```bash
node sim/web/tests/hand_tracker_tasks.test.mjs
node sim/web/tests/hand_mimic_transport.test.mjs
```

这些测试不访问摄像头，也不证明浏览器权限、GPU/WASM 性能或真机运动正确。

## 浏览器验收

1. 打开工作台的 Replay/实时摄像头入口。
2. 允许摄像头权限，确认画面和关键点覆盖正常。
3. 在 Network 中确认优先连接 `/ws/hand/mimic`。
4. 人为断开 WebSocket，确认 HTTP 降级且没有旧帧堆积。
5. 恢复网络，确认只创建一个新连接。
6. 停止摄像头或切换页面，确认媒体轨道、定时器和连接都停止。
7. 记录采集 FPS、发送 FPS、后端处理时间和端到端延迟。

目前尚未形成真实浏览器的性能验收记录，因此不要引用旧文档中的固定 10 FPS、
`<150 ms` 等数值作为当前保证。

## 常见问题

### Windows 无法访问 localhost

检查服务是否监听以及 Windows 到 WSL 的 localhost 转发：

```bash
ss -ltnp | rg ':7860'
```

只有需要从其他设备访问时才添加 Windows 防火墙入站规则，并限制来源网络；localhost
本机访问通常不需要开放公网入站。

### 摄像头权限被拒绝

检查 Windows 隐私设置、浏览器站点权限和页面是否为安全上下文。清除站点权限后重新
授权。摄像头被其他应用独占也会导致失败。

### 模型或 WASM 加载失败

检查浏览器控制台和 `sim/web/vendor/mediapipe-tasks/SHA256SUMS`。现行实现使用本地资源，
排查重点不是 CDN 连通性。

### 有关键点但没有真手动作

先确认 Web API 返回值和 UI 的 mock/连接状态。不要把 3D 模型变化视为真机运动证据；
同一串口不能同时被 Web、Bridge 和手控制台占用。

---

**最后核对**：2026-08-14
