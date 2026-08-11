# MCP Server 云端部署指南

## 架构

```
[Claude AI] → [云端 MCP Server] → [隧道] → [用户本地 Bridge] → [硬件]
```

## 云端部署

### 1. 构建镜像

```bash
cd mcp_server
docker build -t robot-mcp-server .
```

### 2. 运行容器

```bash
docker run -d \
  --name robot-mcp \
  -p 8000:8000 \
  -e ROBOT_BRIDGE_URL="https://xxx.trycloudflare.com" \
  -e ROBOT_BRIDGE_TOKEN="your-bridge-token-here" \
  -e MCP_SECURITY_MODE="public" \
  -e MCP_API_KEYS="key1,key2,key3" \
  robot-mcp-server
```

**环境变量说明**：

- `ROBOT_BRIDGE_URL` — 用户的隧道 URL（用户提供）
- `ROBOT_BRIDGE_TOKEN` — bridge 认证 token（和用户共享）
- `MCP_SECURITY_MODE` — 必须 `public`（云端强制鉴权）
- `MCP_API_KEYS` — 逗号分隔的 API Key 列表（给 AI 客户端用）

### 3. 验证部署

```bash
curl http://your-server:8000/health
# 应该返回 {"status": "degraded", "error": "..."}  ← 正常，因为还没连 bridge

# 测试 MCP 协议
curl -X POST http://your-server:8000/mcp \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}'
```

---

## 用户侧部署

### 1. 下载 bridge 运行时

```bash
# 下载 release 包（或者 clone 仓库）
wget https://github.com/yourrepo/releases/latest/bridge-runtime.tar.gz
tar xzf bridge-runtime.tar.gz
cd bridge-runtime
```

### 2. 安装依赖

```bash
pip install fastapi uvicorn pyserial
# 如果用真机械臂还需要: pip install pyAgxArm
```

### 3. 生成 bridge token（一次性）

```bash
python3 -c "import secrets; print(secrets.token_urlsafe(32))"
# 输出类似: a8f3k2j9d8s7f6h5g4j3k2l1m0n9b8v7
# 把这个 token 同时给云端和本地用
```

### 4. 启动 bridge

```bash
# 设置 token
export BRIDGE_TOKEN="a8f3k2j9d8s7f6h5g4j3k2l1m0n9b8v7"

# 启动（mock 模式测试）
python bridge.py --mock --host 127.0.0.1 --port 9000

# 真机模式（串口号根据实际情况改）
# python bridge.py --host 127.0.0.1 --port 9000
```

### 5. 开启隧道

```bash
# 安装 cloudflared
# Ubuntu/WSL: 
curl -L https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64 -o cloudflared
chmod +x cloudflared
sudo mv cloudflared /usr/local/bin/

# 开启快速隧道
cloudflared tunnel --url http://localhost:9000
```

**输出示例**：
```
Your quick Tunnel has been created! Visit it at:
https://random-words.trycloudflare.com
```

### 6. 把隧道 URL 和 token 给云端管理员

- 隧道 URL: `https://random-words.trycloudflare.com`
- Bridge Token: `a8f3k2j9d8s7f6h5g4j3k2l1m0n9b8v7`

云端会用这两个值更新容器环境变量。

---

## 连接 Claude AI

### 方式 1: Claude Desktop（推荐）

编辑 `~/Library/Application Support/Claude/claude_desktop_config.json` (Mac)
或 `%APPDATA%\Claude\claude_desktop_config.json` (Windows):

```json
{
  "mcpServers": {
    "robot": {
      "url": "http://your-cloud-server:8000/mcp",
      "headers": {
        "X-API-Key": "your-api-key"
      }
    }
  }
}
```

重启 Claude Desktop，工具会自动出现。

### 方式 2: MCP Inspector（测试用）

```bash
npm install -g @modelcontextprotocol/inspector
mcp-inspector http://your-server:8000/mcp
```

---

## 故障排查

### Bridge 连不上

```bash
# 检查 bridge 是否在跑
curl http://localhost:9000/health

# 检查 token 是否正确
curl -H "X-Bridge-Token: wrong-token" http://localhost:9000/hand/status
# 应该返回 401 Unauthorized
```

### 隧道不通

```bash
# 从外网测试隧道
curl https://your-tunnel-url.trycloudflare.com/health

# 应该返回 bridge 的 health 响应（带 mock 字段）
```

### MCP Server 连不上 bridge

检查云端容器日志：
```bash
docker logs robot-mcp
```

应该看到 `硬件代理已连接` 或连接失败的详细错误。

---

## 安全注意事项

1. **Bridge Token 必须设置** — 不设就是裸奔
2. **隧道 URL 不要公开分享** — 拿到的人能控制硬件
3. **API Key 定期轮换** — 一个客户端一把 key
4. **免费隧道 URL 会变** — 重启 cloudflared 就要通知云端更新

生产环境建议用 Cloudflare 命名隧道（免费，固定域名）：
https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/
