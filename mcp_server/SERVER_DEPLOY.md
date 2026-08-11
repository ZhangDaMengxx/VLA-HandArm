# 云服务器部署指南 - 开箱即用

## 📦 部署包内容

- `robot-mcp-server.tar` — Docker 镜像文件（67MB）
- `docker-compose.yml` — 容器编排配置
- `.env.example` — 环境变量模板
- `DEPLOY.md` — 详细文档

---

## 🚀 快速部署（3 步）

### 第 1 步：上传部署包

```bash
# 上传 deploy-package.tar.gz 到服务器
scp deploy-package.tar.gz user@your-server:/tmp/
```

### 第 2 步：解压并加载镜像

```bash
# SSH 登录服务器
ssh user@your-server

# 解压
cd /tmp
tar xzf deploy-package.tar.gz

# 加载 Docker 镜像
docker load -i robot-mcp-server.tar

# 验证镜像
docker images | grep robot-mcp-server
# 应该显示: robot-mcp-server   latest   ...   67.0MB
```

### 第 3 步：配置并启动

```bash
# 创建工作目录
mkdir -p ~/mcp-server
cd ~/mcp-server

# 复制配置文件
cp /tmp/docker-compose.yml .
cp /tmp/.env.example .env

# 编辑 .env 配置（重要！）
nano .env
```

**必须修改的配置**：
```bash
# .env 文件内容
ROBOT_BRIDGE_URL=http://example.com        # 等用户提供隧道 URL 后填
ROBOT_BRIDGE_TOKEN=placeholder              # 等用户提供 token 后填
MCP_SECURITY_MODE=public                    # 云端必须是 public
MCP_API_KEYS=your-key-1,your-key-2          # 给 AI 客户端用的 key
```

**生成 API Key 的方法**：
```bash
python3 -c "import secrets; print(secrets.token_urlsafe(32))"
# 输出示例: a8f3k2j9d8s7f6h5g4j3k2l1m0n9b8v7
```

**启动服务**：
```bash
docker compose up -d
```

**验证部署**：
```bash
# 检查容器状态
docker ps | grep mcp-robot-server

# 检查健康
curl http://localhost:8000/health

# 检查 MCP 协议
curl -X POST http://localhost:8000/mcp \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-key-1" \
  -d '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}'
```

---

## 🔄 用户连接后更新配置

当用户提供了隧道 URL 和 token：

```bash
# 编辑 .env
nano .env

# 修改这两行
ROBOT_BRIDGE_URL=https://user-tunnel.trycloudflare.com
ROBOT_BRIDGE_TOKEN=user-provided-token-here

# 重启服务生效
docker compose restart

# 验证连接成功
curl http://localhost:8000/health
# 应该返回: {"status": "ok", "hand": {...}, "arm": {...}}
```

---

## 🌐 开放公网访问

### 配置防火墙

```bash
# Ubuntu/Debian
sudo ufw allow 8000/tcp

# CentOS/RHEL
sudo firewall-cmd --permanent --add-port=8000/tcp
sudo firewall-cmd --reload
```

### 云厂商安全组

在云厂商控制台添加入站规则：
- 端口：8000
- 协议：TCP
- 来源：0.0.0.0/0（或限制为 Claude API 的 IP 段）

### 测试公网访问

```bash
# 从外网测试
curl http://your-server-ip:8000/health
```

---

## 📱 连接 Claude Desktop

编辑 Claude Desktop 配置文件：

**Mac**: `~/Library/Application Support/Claude/claude_desktop_config.json`  
**Windows**: `%APPDATA%\Claude\claude_desktop_config.json`

```json
{
  "mcpServers": {
    "robot": {
      "command": "npx",
      "args": [
        "-y", "mcp-remote",
        "http://your-server-ip:8000/mcp",
        "--header", "X-API-Key:your-key-1"
      ]
    }
  }
}
```

⚠️ `mcpServers` 走 stdio，**不认 `url` 字段** —— 连 HTTP 端点要靠 `mcp-remote`
转换，所以运行 Claude Desktop 的机器需要装 Node.js。`--header` 的冒号后不要加空格。

重启 Claude Desktop，工具会自动出现。

---

## 🛠️ 常用命令

```bash
# 查看日志
docker logs -f mcp-robot-server

# 重启服务
docker compose restart

# 停止服务
docker compose down

# 更新镜像（新版本）
docker load -i new-robot-mcp-server.tar
docker compose down
docker compose up -d

# 查看资源占用
docker stats mcp-robot-server
```

---

## ⚠️ 故障排查

### 容器无法启动

```bash
docker logs mcp-robot-server
```

常见问题：
- 端口被占用 → `lsof -i :8000` 找到进程并停止
- 环境变量格式错误 → 检查 `.env` 语法
- 镜像未加载 → `docker images` 确认镜像存在

### 连不上 bridge

健康检查返回 `"status": "degraded"`：
- 用户的 bridge 还没启动
- 用户的隧道还没开启
- `ROBOT_BRIDGE_URL` 或 `ROBOT_BRIDGE_TOKEN` 配置错误

### API Key 无效

返回 401 Unauthorized：
- 检查 `.env` 中的 `MCP_API_KEYS`
- 确认 Claude 配置文件中的 key 一致
- 注意逗号分隔多个 key，不要有空格

---

## 🔒 安全建议

1. **定期轮换 API Key**
2. **使用 HTTPS**（建议用 nginx 反向代理 + Let's Encrypt 证书）
3. **限制 IP 白名单**（如果 AI 客户端 IP 固定）
4. **监控异常请求**（`docker logs` 定期检查）
5. **备份 .env 文件**（但不要提交到 git）

---

## 📊 性能监控

```bash
# 实时监控
docker stats mcp-robot-server

# 查看资源限制
docker inspect mcp-robot-server | grep -A 5 "Memory"
```

如需限制资源：
```yaml
# docker-compose.yml 中添加
services:
  mcp-server:
    # ... 其他配置
    deploy:
      resources:
        limits:
          cpus: '1'
          memory: 512M
```

---

## ✅ 部署检查清单

- [ ] 镜像已加载（`docker images`）
- [ ] `.env` 已配置（`MCP_SECURITY_MODE=public`, `MCP_API_KEYS` 已设置）
- [ ] 容器已启动（`docker ps` 看到 `mcp-robot-server`）
- [ ] 健康检查正常（`curl http://localhost:8000/health`）
- [ ] MCP 协议可用（`tools/list` 返回 6 个工具）
- [ ] 防火墙已开放（公网能访问 `:8000`）
- [ ] Claude Desktop 已配置并重启
- [ ] 等待用户提供隧道 URL 和 token

完成后就可以让用户开始使用了！
