# Combo 动态扫描机制

> **实验功能归档。** 现行 `robot-mcp-server` 不包含 combo router、combo MCP 工具或
> Bridge `/combo/*`。本文只用于理解本仓内嵌分叉，不能作为部署说明。

## 改动背景

**之前**：硬编码的 `COMBO_PRESETS` 映射表
```python
COMBO_PRESETS = {
    "伸手": {"file": "伸手.json", "desc": "..."},
    "挥手": {"file": "shake_hand.json", "desc": "..."},
}
```

**问题**：
1. 新录制的动作需要手动添加到代码
2. 不支持子目录
3. 文件重命名后映射失效
4. 和 `app_web.py` 的动态扫描逻辑不一致

**现在**：动态扫描 `data/combos/` 目录

---

## 架构

```
┌──────────────────────────────────────────┐
│  MCP Server (controller.py)             │
│  - list_combos()  → GET /combo/list     │
│  - play_combo(name) → POST /combo/play  │
└──────────────────────────────────────────┘
               ↓ HTTP
┌──────────────────────────────────────────┐
│  Bridge (bridge.py)                      │
│  - GET /combo/list                       │
│    → combo_pack.list_packs()            │
│  - POST /combo/play                      │
│    → combo_pack.load_pack()             │
└──────────────────────────────────────────┘
               ↓ 文件系统
┌──────────────────────────────────────────┐
│  data/combos/                            │
│  ├── 伸手.json                           │
│  ├── 挥手.json                           │
│  ├── 点赞.json                           │
│  └── 常用/                               │
│      └── 三指抓握.json                   │
└──────────────────────────────────────────┘
```

---

## Bridge 端点

### 1. GET /combo/list

**功能**：列出所有可用的联合动作包

**返回**：
```json
{
  "ok": true,
  "root": "/home/zhang123/ros2_ws/lerobotTest/data/combos",
  "packs": [
    {
      "name": "挥手",
      "path": "挥手.json",
      "mode": "keyframe",
      "frames": 15,
      "duration_ms": 3000,
      "desc": "左右摆动招手"
    },
    {
      "name": "点赞",
      "path": "点赞.json",
      "mode": "keyframe",
      "frames": 8,
      "duration_ms": 1500,
      "desc": "竖起大拇指"
    }
  ]
}
```

**实现**：
- 调用 `combo_pack.list_packs()` 扫描目录
- 支持递归子目录
- 自动跳过损坏的文件（带 `error` 字段）

---

### 2. POST /combo/play

**功能**：按名称或路径播放联合动作

**请求**（二选一）：
```json
// 方式 1：按名称（自动查找）
{
  "name": "挥手"
}

// 方式 2：按路径（直接加载）
{
  "path": "常用/挥手.json"
}
```

**返回**：
```json
{
  "ok": true,
  "name": "挥手",
  "path": "挥手.json",
  "mode": "keyframe",
  "frames": 15,
  "duration_ms": 3000,
  "pack": {
    "schema": "combo_pack/1",
    "name": "挥手",
    "frames": [...]
  }
}
```

**错误处理**：
- `404` - 未找到（附带可用列表）
- `409` - 找到多个同名（附带路径列表）
- `400` - 格式错误或 stream 模式包

---

## Controller 逻辑

### list_combos()

```python
async def list_combos(self):
    """动态获取列表"""
    await self.ensure_connected()
    
    resp = await self.client.get("/combo/list")
    resp.raise_for_status()
    data = resp.json()
    
    return {
        "presets": [
            {
                "name": p["name"],
                "path": p["path"],
                "description": p.get("desc", ""),
                "frames": p.get("frames", 0),
                "duration_ms": p.get("duration_ms", 0)
            }
            for p in data.get("packs", []) if not p.get("error")
        ]
    }
```

**特点**：
- 每次调用都重新扫描（实时反映文件变化）
- 过滤掉损坏的包
- Bridge 端点未实现时返回空列表

---

### play_combo(name)

```python
async def play_combo(self, name: str):
    """按名称播放"""
    await self.ensure_connected()
    
    try:
        resp = await self.client.post("/combo/play", json={"name": name})
        resp.raise_for_status()
        return resp.json()
    
    except httpx.HTTPStatusError as e:
        if e.response.status_code == 404:
            # 提供可用列表
            combos = await self.list_combos()
            available = [c["name"] for c in combos["presets"]]
            raise ValueError(f"未找到: {name}（可用: {', '.join(available)}）")
        
        elif e.response.status_code == 409:
            # 重名冲突
            detail = e.response.json().get("detail", "")
            raise ValueError(f"多个同名: {detail}")
```

**特点**：
- Bridge 负责查找和加载
- 失败时提供友好的错误信息
- 支持重名检测

---

## 使用示例

### MCP 工具调用

```python
# AI Agent 通过 MCP 调用
{
  "name": "play_combo",
  "arguments": {
    "name": "挥手"
  }
}

# 结果
{
  "ok": true,
  "name": "挥手",
  "frames": 15,
  "duration_ms": 3000
}
```

### Web API 调用

```bash
# 列出可用动作
curl http://localhost:8000/api/v1/combo/list

# 执行动作
curl -X POST http://localhost:8000/api/v1/combo/play \
  -H "Content-Type: application/json" \
  -d '{"name": "挥手"}'
```

---

## 优势

### 1. 自动发现新动作
录制完成后立即可用，无需修改代码：

```bash
# 在 app_web.py 录制
POST /api/combo/save {"name": "新动作", ...}

# MCP Server 立即可用
POST /api/v1/combo/play {"name": "新动作"}
```

### 2. 支持子目录组织

```
data/combos/
├── 基础/
│   ├── 伸手.json
│   └── 收手.json
├── 交互/
│   ├── 挥手.json
│   └── 点赞.json
└── 抓取/
    └── 三指抓握.json
```

所有文件自动发现，路径为 `基础/伸手.json`。

### 3. 支持重命名

文件重命名后，只要 JSON 内的 `name` 字段不变，就能继续按名称播放。

### 4. 一致性

MCP Server 和 app_web.py 使用相同的扫描逻辑，保证行为一致。

---

## 注意事项

### 1. 性能

每次调用 `list_combos()` 都会扫描目录：
- **开销**：几十个文件时 <10ms
- **优化**：可在 Bridge 层添加缓存

### 2. 重名处理

同名文件会返回 409 错误：
```json
{
  "detail": "找到多个同名动作: 挥手，请使用路径指定: ['挥手.json', '备份/挥手.json']"
}
```

**解决**：调用方改用 `path` 而不是 `name`。

### 3. Bridge 依赖

Controller 依赖 Bridge 的 `/combo/list` 和 `/combo/play` 端点。
如果 Bridge 未实现：
- `list_combos()` 返回空列表
- `play_combo()` 抛出连接错误

---

## 迁移指南

### 从硬编码迁移

**之前**：
```python
# 添加新动作需要改代码
COMBO_PRESETS["新动作"] = {"file": "新动作.json", "desc": "..."}
```

**现在**：
```bash
# 直接录制保存即可
# 无需修改任何代码
```

### 兼容性

录制的文件格式完全兼容：
- Schema 版本不变（`combo_pack/1`）
- 文件路径不变（`data/combos/`）
- 旧文件自动发现，无需迁移

---

## 测试

### 1. 列出动作

```bash
curl http://localhost:9000/combo/list
```

预期：返回所有可用动作列表

### 2. 按名称播放

```bash
curl -X POST http://localhost:9000/combo/play \
  -H "Content-Type: application/json" \
  -d '{"name": "挥手"}'
```

预期：返回动作详情和完整数据

### 3. 按路径播放

```bash
curl -X POST http://localhost:9000/combo/play \
  -H "Content-Type: application/json" \
  -d '{"path": "常用/挥手.json"}'
```

预期：直接加载该路径的文件

### 4. 错误处理

```bash
# 不存在的动作
curl -X POST http://localhost:9000/combo/play \
  -d '{"name": "不存在"}'

# 预期：404 + 可用列表
```

---

## 总结

改为动态扫描后：
- ✅ 无需手动维护映射表
- ✅ 自动发现新录制的动作
- ✅ 支持子目录和重命名
- ✅ 和 app_web.py 行为一致
- ✅ 更好的错误提示
