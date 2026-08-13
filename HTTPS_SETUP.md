# HTTPS 配置指南 — 启用远程摄像头访问

## 问题背景

浏览器的 `getUserMedia()` API（摄像头/麦克风访问）需要**安全上下文**：
- ✅ `https://任何域名` — 允许
- ✅ `http://localhost` 或 `http://127.0.0.1` — 允许（本地开发例外）
- ❌ `http://192.168.x.x` — **禁止**（非安全上下文）

当客户端通过局域网 IP 访问服务器时，必须使用 HTTPS。

---

## 已配置内容

### 1. 自签名证书（已生成）

```
ssl/
├── cert.pem  # 证书（1.8KB）
└── key.pem   # 私钥（3.2KB）
```

- 有效期：365 天
- 证书主体：CN=192.168.1.189
- 加密算法：RSA 4096 位

### 2. FastAPI/Uvicorn 配置（已启用）

`app_web.py` 会自动检测 `ssl/` 目录：
- 如果证书存在 → 启动 HTTPS
- 如果证书不存在 → 降级到 HTTP

---

## 使用指南

### 启动服务器

```bash
cd /home/zhang123/ros2_ws/lerobotTest
python sim/app_web.py
```

应该看到：
```
========================================================================
  回放工作台启动中。浏览器打开:  https://192.168.1.189:7860
  ✅ HTTPS 已启用（支持摄像头访问）
  ⚠️  首次访问需在浏览器中信任自签名证书
========================================================================
```

### 客户端浏览器访问

**1. 打开 URL**
```
https://192.168.1.189:7860
```

**2. 信任证书**

首次访问会看到安全警告：

**Chrome/Edge:**
- 显示："您的连接不是私密连接"
- 点击 **"高级"**
- 点击 **"继续前往 192.168.1.189（不安全）"**

**Firefox:**
- 显示："警告：潜在的安全风险"
- 点击 **"高级"**
- 点击 **"接受风险并继续"**

**Safari:**
- 显示："此连接不是私密连接"
- 点击 **"显示详细信息"**
- 点击 **"访问此网站"**

**3. 测试摄像头**

进入 **Replay** 页面 → 点击 **📷 Hand Mimic** 按钮：
- 浏览器会弹出摄像头权限请求
- 点击 **"允许"**
- 应该看到实时摄像头画面 + 绿色骨骼点

---

## 故障排查

### 问题 1：证书警告无法跳过

**症状**：浏览器一直阻止访问，没有"继续"按钮

**解决**：
- Chrome: 在警告页面输入 `thisisunsafe`（不会显示，直接输入即可）
- Firefox: 更新到最新版本
- 或使用 `mkcert` 生成本地受信任的证书（见下文）

### 问题 2：仍然无法访问摄像头

**检查清单**：
1. ✅ 确认使用 `https://` 而不是 `http://`
2. ✅ 确认 URL 中的 IP 与证书 CN 一致（192.168.1.189）
3. ✅ 浏览器已允许摄像头权限
4. ✅ 摄像头未被其他应用占用
5. ✅ F12 控制台无报错

### 问题 3：多客户端都需要信任证书

**症状**：每台新设备访问都需要手动信任

**原因**：自签名证书不在系统信任链中

**解决**：使用 `mkcert` 生成本地 CA（可选，见下文）

---

## 高级选项：使用 mkcert（可选）

`mkcert` 可以生成**系统信任的本地证书**，避免每次手动接受警告。

### 安装 mkcert

**WSL/Ubuntu:**
```bash
sudo apt install libnss3-tools
wget https://github.com/FiloSottile/mkcert/releases/download/v1.4.4/mkcert-v1.4.4-linux-amd64
chmod +x mkcert-v1.4.4-linux-amd64
sudo mv mkcert-v1.4.4-linux-amd64 /usr/local/bin/mkcert
```

### 生成证书

**服务器端（WSL）:**
```bash
cd /home/zhang123/ros2_ws/lerobotTest
mkcert -install  # 安装本地 CA
mkcert -key-file ssl/key.pem -cert-file ssl/cert.pem 192.168.1.189 localhost
```

**客户端（每台设备）:**
```bash
# 需要在每台客户端设备上运行：
mkcert -install
```

然后将服务器端的 `~/.local/share/mkcert/rootCA.pem` 复制到客户端并导入。

**注意**：这对于多客户端场景仍然比较麻烦。生产环境建议使用 Let's Encrypt 或企业 CA 签发的证书。

---

## 生产环境建议

对于正式部署：
1. 使用 **Let's Encrypt** 免费证书（需要公网域名）
2. 使用 **Nginx/Caddy** 作为反向代理处理 HTTPS
3. 配置 HSTS、CSP 等安全头
4. 定期更新证书（Let's Encrypt 90 天有效期）

---

## 技术原理

### 为什么 `getUserMedia()` 需要安全上下文？

1. **隐私保护**：防止恶意网站偷偷录音/录像
2. **中间人攻击防护**：未加密的 HTTP 可能被劫持
3. **W3C 标准要求**：Secure Contexts 规范强制要求

### 自签名证书的局限性

- ❌ 浏览器不信任（需手动接受）
- ❌ 不适合公网部署
- ✅ 适合开发/测试/局域网环境
- ✅ 功能上等同于正式证书

---

## 参考资料

- [W3C Secure Contexts](https://w3c.github.io/webappsec-secure-contexts/)
- [MDN: getUserMedia()](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)
- [mkcert GitHub](https://github.com/FiloSottile/mkcert)
