# Web 摄像头 HTTPS 配置

浏览器 `getUserMedia()` 只允许安全上下文：可信 `https://`，以及本机开发例外
`http://localhost` / `http://127.0.0.1`。通过局域网 IP 访问 Web 工作台时应使用 HTTPS。

## 重要安全状态

仓库中的 `ssl/key.pem` 和 `ssl/cert.pem` 曾被 Git 跟踪。私钥一旦进入版本库就应视为
已经泄露：

- 不要继续把现有 `key.pem` 用于共享、局域网长期运行或生产环境。
- 需要生成新的私钥和证书，并让私钥保持未跟踪状态。
- 移出当前 Git 跟踪不能消除历史副本；是否清理历史应单独评估和执行。
- 不要通过聊天、Issue、日志或提交发送私钥内容。

`sim/app_web.py` 的实际行为是：仅当 `ssl/key.pem` 和 `ssl/cert.pem` 同时存在时启用
HTTPS，否则启动 HTTP；它不会验证证书是否可信、是否过期或是否匹配访问主机名。

## 本机开发

Windows 浏览器访问 WSL 服务时优先使用：

```text
http://localhost:7860
```

localhost 属于安全上下文例外，通常不需要证书。启动命令：

```bash
conda activate lerobot
python sim/app_web.py
```

## 局域网开发

推荐使用团队或设备信任的本地 CA。以 `mkcert` 为例，生成文件时应包含所有实际访问名：

```bash
mkdir -p ssl
mkcert -key-file ssl/key.pem -cert-file ssl/cert.pem \
  localhost 127.0.0.1 192.168.1.189
chmod 600 ssl/key.pem
```

IP 只是示例，应替换为当前主机地址。客户端还必须信任签发该证书的 CA；单纯点击浏览器
风险提示不等于建立了可复用的可信部署。

建议在 `.gitignore` 中忽略至少：

```gitignore
ssl/*.key
ssl/key.pem
```

证书公钥是否入库取决于部署方式，私钥永远不应入库。

## 生产或公网

不要直接暴露 `sim/app_web.py:7860`。该 Web 工作台包含运动控制端点且没有应用层鉴权。
应使用 Caddy/Nginx 等反向代理，并至少具备：

- 公开信任或组织信任的证书与自动续期
- 身份认证和最小权限
- 来源网络限制
- 请求和异常审计
- 明确的 WebSocket 代理配置

MCP 的公网部署是另一条链路，请使用独立 `robot-mcp-server/frp_deploy.md`，不要把 Web
工作台证书与 MCP/Bridge 凭据混用。

## 检查

```bash
openssl x509 -in ssl/cert.pem -noout -subject -issuer -dates -ext subjectAltName
```

浏览器侧确认：页面是安全上下文、证书主机名匹配、摄像头权限已允许，WebSocket 使用
`wss://`。证书警告、权限拒绝和 WebSocket 失败应分别排查。

---

**最后核对**：2026-08-14
