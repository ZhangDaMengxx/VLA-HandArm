# 待办事项 (TODO)

## 灵巧手安全计划 - 暂停（2026-08-11）

**已完成：**
- ✅ 第 0 步：TrajectoryBackend 加载期预检
- ✅ 第 1 步：读 Kilohertz-Safe 论文
- ✅ 第 3 步：几何碰撞检查器（标定参数：span2=0.525, k3=1.075）

**待办：**
- [ ] **第 4 步：路径检查**（~40 行，按 SPEED_SET 推中间轨迹查碰撞）
  - 计划：`HAND_SAFETY_PLAN.md` 第 190 行
  - 验收：能离线复现"握拳时堵转"
  
- [ ] **第 5 步：约束进 dex_retargeting**
  - 加 `add_inequality_constraint` 到 derive_embodiment
  - 修复 `derive_embodiment.py:366` 的裸 clip
  
- [ ] **第 6 步：换手复用**
  - 把 1-5 的输入收成"手型档案"

- [ ] **遗留：统一 thumb_2 limit**
  - URDF: 0.48 → 0.525（让网页端读到正确值）
  - hand_pose.LIMIT_HI: 0.6 → 0.525
  - collision_checker: 已是 0.525 ✓
  - 影响：网页滑块范围、实际发送限制

- [ ] **旧表的二维问题**
  - 发现：(yaw=600, pitch=100) 会碰，但旧表说 T=600 安全
  - 旧表只在对角线 (T,T) 测过，离对角线的点未验证
  - 考虑：是否需要二维表，或者改用几何实时检查

---

## 当前任务：MCP Server（2026-08-11 启动）

**目标**：让大模型（Claude 等）通过 MCP 协议调用现有技能包，控制真实硬件。

**技术栈**：FastAPI + MCP protocol

**需求**：
- 配置 `control_address`（局域网/公网可切换）
- 封装技能包为 MCP tools
- 公网模式需鉴权（API key）

**参考**：
- MCP 规范：https://modelcontextprotocol.io
- 现有技能包：`sim/skills/registry.yaml` + `backend.py`
- 控制层：`hand_console.py`, `app_web.py`
