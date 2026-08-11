# Docker 构建指南

## 问题：本地网络限制

如果遇到以下错误：
```
failed to do request: Head "https://docker.mirrors.ustc.edu.cn/...": 
dial tcp: lookup docker.mirrors.ustc.edu.cn: no such host
```

**原因**：Docker Desktop 配置了不可用的镜像源，且网络无法访问官方 Docker Hub。

---

## 解决方案 1：在云服务器上构建（推荐）

云服务器通常网络正常，直接在那边构建：

```bash
# 在云服务器上
git clone <你的仓库>
cd <仓库>/mcp_server

# 构建镜像
docker build -t robot-mcp-server .

# 或使用 docker compose
docker compose build

# 启动
docker compose up -d
```

---

## 解决方案 2：修改 Docker Desktop 镜像源

### Windows 上操作：

1. 打开 Docker Desktop
2. 点击设置（Settings）→ Docker Engine
3. 找到 `registry-mirrors` 配置：
   ```json
   {
     "registry-mirrors": [
       "https://docker.mirrors.ustc.edu.cn/"
     ]
   }
   ```
4. **删除或替换**为可用的镜像源：
   ```json
   {
     "registry-mirrors": []
   }
   ```
   或使用其他可用源（如果有）。

5. 点击 "Apply & Restart"
6. 回到 WSL 重试构建

---

## 解决方案 3：本地 Python 直接运行（临时测试）

如果急需测试功能，跳过 Docker：

```bash
cd mcp_server
pip install -r requirements.txt

# 配置环境变量
export ROBOT_BRIDGE_URL=http://localhost:9000
export ROBOT_BRIDGE_TOKEN=test-token
export MCP_SECURITY_MODE=lan

# 启动
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

测试通过后再在云端用 Docker 正式部署。

---

## 解决方案 4：导出镜像文件传输

如果云服务器也无法访问 Docker Hub（内网环境），可以在有网络的机器上构建后传输：

```bash
# 有网络的机器A上
docker build -t robot-mcp-server .
docker save robot-mcp-server -o robot-mcp-server.tar

# 传输到目标服务器B
scp robot-mcp-server.tar user@server-b:/tmp/

# 服务器B上
docker load -i /tmp/robot-mcp-server.tar
docker compose up -d
```

---

## 验证构建成功

```bash
# 查看镜像
docker images | grep robot-mcp-server

# 测试运行
docker run --rm -p 8000:8000 \
  -e ROBOT_BRIDGE_URL=http://host.docker.internal:9000 \
  -e MCP_SECURITY_MODE=lan \
  robot-mcp-server

# 另一个终端测试
curl http://localhost:8000/health
```

---

## 推荐流程

**开发阶段**：本地用 Python 直接跑（方案 3）快速迭代

**部署阶段**：云服务器上用 Docker（方案 1）保证环境一致

**离线部署**：用镜像文件传输（方案 4）
