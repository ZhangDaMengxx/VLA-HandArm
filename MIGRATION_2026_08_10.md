# 灵巧手URDF迁移记录 - 2026-08-10

> **历史归档。** 本文记录 2026-08-10 当时的迁移过程，目录随后从
> `assets/inspire_hand/` 调整为 `assets/hand/`，部分验证脚本现已失效。当前路径和
> 验证状态见 [README_DOCS.md](README_DOCS.md) 与 [PROJECT_STATUS.md](PROJECT_STATUS.md)。

## 背景

真机到位后，决定采用厂商2025-04-18新URDF作为项目标准，替换之前的旧版本URDF。

## 迁移内容

### 1. 文件结构变更

#### 备份旧版本
```
assets/inspire_hand/ → assets/inspire_hand_legacy/
```

#### 新标准位置
```
assets/inspire_hand/
├── urdf/
│   └── inspire_hand_right.urdf    (新URDF，重命名自urdf_right_2025_4_18.urdf)
├── meshes/
│   ├── R_base_link.STL
│   ├── right_thumb_1.STL
│   ├── right_thumb_2.STL
│   ├── right_index_1.STL
│   ├── right_middle_1.STL
│   ├── right_ring_1.STL
│   └── right_little_*.STL
├── config/
├── launch/
└── textures/
```

### 2. 关节命名变更

| 旧关节名 (inspire_hand_legacy) | 新关节名 (inspire_hand) | 含义 |
|------------------------------|----------------------|------|
| `thumb_proximal_yaw_joint` | `right_thumb_1_joint` | 拇指侧摆(yaw) |
| `thumb_proximal_pitch_joint` | `right_thumb_2_joint` | 拇指弯曲(pitch) |
| `index_proximal_joint` | `right_index_1_joint` | 食指MCP |
| `middle_proximal_joint` | `right_middle_1_joint` | 中指MCP |
| `ring_proximal_joint` | `right_ring_1_joint` | 无名指MCP |
| `pinky_proximal_joint` | `right_little_1_joint` | 小指MCP |

### 3. 限位变更（采用新URDF SolidWorks导出值）

| 关节 | 旧限位 (rad) | 新限位 (rad) | 变化 |
|-----|------------|------------|------|
| thumb_1 (yaw) | [0, 1.308] | [0, 1.246165] | -4.7% |
| thumb_2 (pitch) | [0, 0.6] | [0, 0.48] | -20% |
| 四指 (MCP) | [0, 1.47] | [0, 1.333] | -9.3% |

### 4. 代码更新

#### 核心驱动 `sim/inspire_hand.py`
- ✅ 更新 `HAND_JOINTS` - 6个关节名
- ✅ 更新 `HAND_LIMITS` - 6个限位值
- ✅ 更新 `RAW_MAP` - 6个span值（同步新限位）
- ✅ 更新 `FORCE_MAX` - 6个力控上限键名
- ✅ 添加详细注释说明限位变更原因

#### 其他受影响文件（使用自动迁移脚本`migrate_hand_joints.py`）
1. `sim/schema.py` - 数据schema定义
2. `sim/ros_joint_writer.py` - ROS2控制器
3. `sim/hand_rerun.py` - 手部3D可视化
4. `sim/live_rerun.py` - 实时3D可视化
5. `sim/build_inspire_from_vendor.py` - 厂商数据转换
6. `sim/skills/backend.py` - 技能后端
7. `sim/skills/hand_pose.py` - 手部姿态技能
8. `sim/skills/test_hand_pose.py` - 手部姿态测试
9. `sim/test_combo_page.py` - 联合页面测试

**统计**：9个文件，50处替换

### 5. 文档更新

#### `handarm_notes.md`
- ✅ 更新"RH56DFX 可用资料"章节，添加迁移说明
- ✅ 更新"当前项目手部顺序"为新关节名
- ✅ 更新标准URDF路径引用

## 关键技术点

### 厂商通道映射（保持不变）

```python
# 厂商寄存器顺序 (ANGLE_SET 1486起)
# m=0 小拇指 (little)
# m=1 无名指 (ring)
# m=2 中指   (middle)
# m=3 食指   (index)
# m=4 拇指弯曲 (thumb_pitch / thumb_2)
# m=5 拇指旋转 (thumb_yaw / thumb_1)

# 项目顺序 → 厂商通道（完全逆序，自逆置换）
PROJECT_TO_VENDOR = [5, 4, 3, 2, 1, 0]
```

### 方向配置（保持不变）

所有6个通道统一为 `invert=True`（raw 1000 = 完全张开）

## 兼容性影响

### ⚠️ 已录制手势包
限位收紧后，已录制的gesture pack中超出新上限的帧会被自动夹到新限位：
- 拇指弯曲收紧20%，闭合动作会变弱
- 四指收紧9.3%，抓握力度可能降低

**解决方案**：
1. 重新录制受影响的手势包
2. 手动调整`force`参数补偿
3. 如需恢复旧行程，修改`inspire_hand.py`中的`HAND_LIMITS`

### ✅ 代码兼容性
- 自动迁移脚本确保所有代码中的关节名已更新
- `PROJECT_TO_VENDOR`映射保持不变，无需修改厂商通信层
- 方向配置保持统一，无需调整控制逻辑

## 验证步骤

### 已完成
- [x] 文件结构迁移
- [x] 代码自动更新（9个文件，50处）
- [x] 核心驱动手动更新（inspire_hand.py）
- [x] 文档更新（handarm_notes.md）

### 待执行
- [ ] 运行单元测试：`python3 -m pytest sim/test_*.py`
- [ ] 启动hand_console真机验证：`python3 sim/hand_console.py --no-mock`
- [ ] 验证Web界面手部控制：`~/gradio_venv/bin/python sim/app_web.py`
- [ ] 测试已录制手势包回放（观察限位夹取效果）
- [ ] 更新COMBO装配URDF（如果引用了手部）

## 工具脚本

### `migrate_hand_joints.py`
自动替换代码中的旧关节名为新关节名。

用法：
```bash
# 检查模式（不修改）
python3 migrate_hand_joints.py --check

# 应用模式（实际修改）
python3 migrate_hand_joints.py --apply
```

## 回滚方案

如果需要回滚到旧版本：

```bash
# 1. 恢复旧URDF
mv assets/inspire_hand assets/inspire_hand_2025_04_18
mv assets/inspire_hand_legacy assets/inspire_hand

# 2. Git回滚代码修改
git checkout HEAD -- sim/

# 3. 恢复handarm_notes.md
git checkout HEAD -- handarm_notes.md
```

## 参考资料

- 旧URDF来源：项目早期使用的dex-urdf版本
- 新URDF来源：厂商提供 `urdf_right_2025_4_18/` (SolidWorks导出)
- 官方xls：`关节与角度对应关系/关节角与0-1000 对应关系.xls`
- 用户手册：`因时机器人仿人五指灵巧手--RH56用户手册V1.09cn.pdf`

## 维护者

- 迁移执行：2026-08-10
- 文档编写：2026-08-10
- 联系方式：见 `handarm_notes.md` 中的团队信息
