# 机械臂 + 灵巧手 MCP 服务

将机器人控制能力通过 MCP (Model Context Protocol) 协议开放给 AI（如 Claude），实现自然语言控制机械臂和灵巧手。

---

## 🚀 快速开始

### 用户侧（本地运行 Bridge）

**⚠️ 完整 clone 会下载 2.8GB，推荐使用精简方式**

#### 方式 1：精简 clone（推荐，<50MB）

```bash
# Git 2.25+ 支持
git clone --filter=blob:none --no-checkout https://github.com/ZhangDaMengxx/VLA-HandArm.git
cd VLA-HandArm
git sparse-checkout init --cone
git sparse-checkout set bridge.py sim mcp_server/DEPLOY.md

# 检出需要的文件
git checkout
```

#### 方式 2：完整 clone（开发用，2.8GB）

```bash
git clone https://github.com/ZhangDaMengxx/VLA-HandArm.git
```

#### 启动 Bridge

```bash
# 安装依赖
pip install fastapi uvicorn pyserial

# 启动 bridge（mock 模式测试）
python bridge.py --mock --host 127.0.0.1 --port 9000
```

**详细说明**：[mcp_server/DEPLOY.md](mcp_server/DEPLOY.md)

---

### 服务器侧（部署 MCP Server）

从 [Releases](https://github.com/ZhangDaMengxx/VLA-HandArm/releases) 下载 Docker 镜像部署包，或自行构建：

```bash
cd mcp_server
docker compose up -d
```

**详细说明**：[mcp_server/SERVER_DEPLOY.md](mcp_server/SERVER_DEPLOY.md)

---

## 🏗️ 架构

```
┌──────────────┐      MCP 协议      ┌────────────────┐
│ Claude       │ ←──────────────→   │ 云端 MCP       │
│ Desktop      │                     │ Server         │
└──────────────┘                     └────────┬───────┘
                                              │
                                        HTTP + 隧道
                                              │
                                     ┌────────▼───────┐
                                     │ 本地 Bridge    │
                                     │ (用户电脑)      │
                                     └────────┬───────┘
                                              │
                                      串口/CAN 驱动
                                              │
                                     ┌────────▼───────┐
                                     │ 机械臂+灵巧手   │
                                     └────────────────┘
```

---

## 📚 文档

| 文档 | 说明 |
|------|------|
| [DEPLOY.md](mcp_server/DEPLOY.md) | 用户完整部署指南（本地 bridge + 隧道） |
| [SERVER_DEPLOY.md](mcp_server/SERVER_DEPLOY.md) | 云端 MCP Server 部署指南 |
| [DOCKER_BUILD.md](mcp_server/DOCKER_BUILD.md) | Docker 构建问题排查 |

---

## 🛠️ 开发相关（VLA 数据管线）

把一段人手视频转成 NERO(7-DoF)+ inspire 手的关节轨迹，打包成 LeRobotDataset，验证这套数据能否用来训 VLA。

### 运行数据管线

```bash
pip install -r requirements.txt
python sim/build_nero_inspire.py     # 生成 NERO+inspire 装配 URDF
python sim/build_canonical.py        # 视频 → 规范层
python sim/derive_embodiment.py --emit-traj  # 规范层 → 本体数据集
python sim/replay_rerun.py --serve   # Rerun 回放
```

详细说明见原 README。

---

## 📦 仓库结构

```
├── bridge.py              # 硬件代理（用户本地运行）
├── mcp_server/            # MCP Server（云端部署）
│   ├── DEPLOY.md          # 用户部署指南
│   ├── SERVER_DEPLOY.md   # 服务器部署指南
│   └── app/               # MCP 服务代码
├── sim/                   # 驱动和技能库
│   ├── inspire_hand.py    # 灵巧手驱动
│   ├── nero_arm.py        # 机械臂驱动
│   └── skills/            # 技能定义和执行
├── assets/                # URDF 模型和网格（开发用）
└── data/                  # 数据和标定（开发用）
```

---

## 🔗 相关链接

- **仓库**：https://github.com/ZhangDaMengxx/VLA-HandArm
- **问题反馈**：[Issues](https://github.com/ZhangDaMengxx/VLA-HandArm/issues)
