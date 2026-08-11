# VLA-HandArm

机械臂 + 灵巧手的 MCP 服务，让 AI 用自然语言驱动硬件。另含一条 VLA 数据管线：
人手视频 → 关节轨迹 → LeRobotDataset。

```
[Claude] --MCP--> [MCP Server] --HTTP--> [bridge] --RS485/CAN--> [硬件]
                   本地或云端             用户本机
```

MCP Server 不碰硬件，只发 HTTP；驱动、技能表、可行域校验全在 bridge。

## 只想跑硬件

用独立的轻量仓库，**别 clone 这个** —— 这里带着仿真资产和第三方库，2.9GB：

```bash
git clone https://github.com/ZhangDaMengxx/robot-bridge.git
cd robot-bridge
pip install -r requirements.txt
python bridge.py --mock --host 127.0.0.1 --port 9000
```

[robot-bridge](https://github.com/ZhangDaMengxx/robot-bridge) 只有运行时需要的
东西，260KB，从本仓库拆出去的，驱动和标定数据同源。

### 或者从本仓库精确检出

不想另开仓库就用 sparse-checkout。别写 `sim` 整个目录（422MB），下面这 7 个文件
合计 120KB：

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/ZhangDaMengxx/VLA-HandArm.git
cd VLA-HandArm
git sparse-checkout init --no-cone
git sparse-checkout set \
  /bridge.py /sim/inspire_hand.py /sim/skills/schema.py \
  /sim/skills/hand_pose.py /sim/skills/registry.yaml \
  /sim/skills/gestures.yaml /mcp_server/DEPLOY.md
git checkout main
```

接真机械臂再补 `/sim/nero_arm.py`。

### 启动 bridge

```bash
pip install fastapi uvicorn pyserial pyyaml

# mock：不连硬件，先跑通链路
python bridge.py --mock --host 127.0.0.1 --port 9000

# 真机 Linux
python bridge.py --hand-port /dev/ttyUSB0 --host 127.0.0.1 --port 9000

# 真机 Windows
python bridge.py --hand-port COM5 --host 127.0.0.1 --port 9000
```

不加 `--mock` 时连不上硬件会**直接启动失败**，不会静默退到 mock。

详见 [mcp_server/DEPLOY.md](mcp_server/DEPLOY.md)。

## 单机验证（不用云服务器）

bridge 和 MCP Server 同一台机器，不用隧道、不用防火墙：

```bash
# 终端 1
python bridge.py --mock --host 127.0.0.1 --port 9000

# 终端 2（在 mcp_server/ 下）
ROBOT_BRIDGE_URL=http://127.0.0.1:9000 MCP_SECURITY_MODE=lan \
  python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

Claude Desktop 配置（`mcpServers` 走 stdio，需要 mcp-remote 代理）：

```json
{
  "mcpServers": {
    "robot": {
      "command": "npx",
      "args": ["-y", "mcp-remote", "http://127.0.0.1:8000/mcp"]
    }
  }
}
```

重启 Claude Desktop，问它"列出可用的手势"验证链路。

## 云端部署 MCP Server

从 [Releases](https://github.com/ZhangDaMengxx/VLA-HandArm/releases) 下载
Docker 镜像部署包，或自行构建：

```bash
cd mcp_server
docker compose up -d
```

详见 [mcp_server/SERVER_DEPLOY.md](mcp_server/SERVER_DEPLOY.md)。

## 完整 clone（开发用）

```bash
git clone https://github.com/ZhangDaMengxx/VLA-HandArm.git
```

2.9GB，含仿真资产（URDF、网格）、第三方库、数据集。

## 文档

| 文档 | 说明 |
|------|------|
| [DEPLOY.md](mcp_server/DEPLOY.md) | 用户部署指南（本地 bridge + 隧道） |
| [SERVER_DEPLOY.md](mcp_server/SERVER_DEPLOY.md) | 云端 MCP Server 部署 |
| [DOCKER_BUILD.md](mcp_server/DOCKER_BUILD.md) | Docker 构建问题排查 |

## VLA 数据管线（开发）

把一段人手视频转成 NERO（7-DoF）+ Inspire 手的关节轨迹，打包成 LeRobotDataset，
验证这套数据能否用来训 VLA。

```bash
pip install -r requirements.txt
python sim/build_nero_inspire.py     # NERO + inspire 装配 URDF
python sim/build_canonical.py        # 视频 → 规范层
python sim/derive_embodiment.py --emit-traj  # 规范层 → 本体数据集
python sim/replay_rerun.py --serve   # Rerun 回放
```

## 仓库结构

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

**仓库**：https://github.com/ZhangDaMengxx/VLA-HandArm  
**问题反馈**：[Issues](https://github.com/ZhangDaMengxx/VLA-HandArm/issues)
