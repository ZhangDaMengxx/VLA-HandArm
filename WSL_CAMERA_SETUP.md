# WSL 摄像头功能测试指南

## 架构说明

```
┌────────────────────────────┐
│  Windows (浏览器)           │
│  ├─ 摄像头访问 (getUserMedia)│  ← 在 Windows 侧
│  ├─ MediaPipe WASM 推理    │  ← 浏览器本地
│  └─ HTTP POST 关键点       │
└──────────┬─────────────────┘
           │ localhost:7860
           ↓
┌────────────────────────────┐
│  WSL2 (Python FastAPI)     │
│  └─ /api/hand/mimic        │  ← 只接收 JSON
└────────────────────────────┘
```

**关键点**：
- ✅ 摄像头在 Windows 侧，WSL 不直接访问硬件
- ✅ 只传输关键点数据（63 个浮点数），不传视频流
- ✅ 通过 HTTP JSON 通信，与普通 API 无异

## 前置条件

### 1. 确认 WSL2 网络配置

```powershell
# 在 Windows PowerShell 中
wsl -l -v
# 确认是 WSL2（不是 WSL1）
```

### 2. 确认服务监听正确地址

```bash
# 在 WSL 中启动服务
cd /home/zhang123/ros2_ws/lerobotTest
python sim/app_web.py

# 确认监听在 0.0.0.0:7860（不是 127.0.0.1）
# 输出应包含：
# INFO:     Uvicorn running on http://0.0.0.0:7860
```

如果监听在 127.0.0.1，修改启动命令：
```bash
uvicorn sim.app_web:app --host 0.0.0.0 --port 7860
```

### 3. 检查 Windows 防火墙

```powershell
# 在 Windows PowerShell (管理员) 中
# 允许 WSL2 的端口转发
New-NetFirewallRule -DisplayName "WSL2 Port 7860" -Direction Inbound -LocalPort 7860 -Protocol TCP -Action Allow
```

## 测试步骤

### Step 1: 启动服务

```bash
cd /home/zhang123/ros2_ws/lerobotTest
python sim/app_web.py
```

### Step 2: 浏览器访问

在 **Windows 浏览器** 中打开：
```
http://localhost:7860
```

**注意事项**：
- ✅ 使用 `localhost`（不要用 `127.0.0.1` 或 WSL IP）
- ✅ Chrome/Edge 推荐（Firefox 某些版本对 WSL localhost 支持不佳）
- ✅ 不需要 HTTPS（localhost 例外）

### Step 3: 测试摄像头

1. 点击 **Replay** 页面
2. 点击 **"📷 实时摄像头"** 按钮
3. **浏览器会弹出摄像头权限请求** → 点击"允许"
4. 应该看到：
   - 摄像头画面
   - 绿色骨骼线叠加在手上
   - 左下角状态显示识别到的手势

### Step 4: 验证通信

打开浏览器开发者工具 (F12) → Network 标签：
- 应该看到每隔 100ms 发送一次 `POST /api/hand/mimic`
- Response 应该包含 `{"ok": true, "gesture": "..."}`

在 WSL 终端查看日志：
```
[app_web] POST /api/hand/mimic - 识别到: OK手势
[app_web] POST /api/hand/mimic - 识别到: 握拳
```

## 常见问题

### Q1: 浏览器提示"无法访问 localhost:7860"

**原因**: WSL2 网络转发问题

**解决**:
```bash
# 在 WSL 中检查服务是否运行
netstat -tuln | grep 7860

# 应该看到：
# tcp  0.0.0.0:7860  0.0.0.0:*  LISTEN
```

如果没有，确认启动命令使用了 `--host 0.0.0.0`。

### Q2: 摄像头权限被拒绝

**原因**: Windows 隐私设置或浏览器权限

**解决**:
1. Windows 设置 → 隐私 → 摄像头 → 允许应用访问摄像头
2. 浏览器设置 → 隐私 → 网站权限 → 摄像头 → 允许 `http://localhost`
3. 清除浏览器对该站点的权限设置，重新请求

### Q3: 摄像头画面正常，但没有骨骼线

**原因**: MediaPipe 加载失败或手没有正确检测

**解决**:
1. 打开浏览器控制台 (F12) 查看错误信息
2. 确认能访问 CDN: `https://cdn.jsdelivr.net/npm/@mediapipe`
3. 手要完整出现在画面中，手指张开更容易识别
4. 光线要充足

### Q4: 识别到手势但灵巧手没有动作

**原因**: 后端逻辑或灵巧手连接问题

**检查**:
```bash
# WSL 终端查看日志
# 应该看到：
[app_web] 识别到手势: OK手势
[app_web] 调用技能包: gestures/ok_hand.json
```

如果没有 `/api/hand/mimic` 的日志，说明请求没到达后端（网络问题）。
如果有日志但没有动作，检查灵巧手连接状态。

### Q5: 延迟很高

**原因**: 发送频率过高或网络瓶颈

**调整**:
修改 `sim/web/hand_mimic.js` 的节流间隔：
```javascript
this.sendInterval = 200; // 改为 5 FPS（降低服务器负载）
```

## 性能参考

| 项目 | 指标 |
|------|------|
| 摄像头分辨率 | 640×480 |
| MediaPipe 推理 | ~60 FPS (浏览器端) |
| 发送频率 | 10 FPS (节流) |
| 网络延迟 | < 5ms (localhost) |
| 端到端延迟 | < 150ms |

## WSL1 vs WSL2

| 特性 | WSL1 | WSL2 |
|------|------|------|
| localhost 转发 | ✅ 原生支持 | ✅ 自动转发 |
| 网络架构 | 共享 Windows 网络栈 | 虚拟网络 (NAT) |
| 推荐 | ⚠️ 已过时 | ✅ 推荐 |

**结论**: 两者都支持，但 WSL2 性能更好。

## 技术栈

- **前端推理**: MediaPipe Hands 0.4 (WASM)
- **摄像头 API**: `navigator.mediaDevices.getUserMedia()`
- **通信协议**: HTTP POST (JSON)
- **后端**: FastAPI (Python 3.10)
- **网络**: WSL2 localhost 自动转发

## 安全说明

1. **摄像头数据不离开本机**
   - 视频流只在浏览器处理
   - 不上传到任何远程服务器

2. **关键点数据**
   - 只包含手部关键点坐标（无法还原视频）
   - 仅发送到 localhost (WSL)

3. **权限控制**
   - 浏览器每次都会请求摄像头权限
   - 用户可以随时拒绝或撤销
