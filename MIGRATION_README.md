# 灵巧手URDF迁移 README

## 概述

本次迁移（2026-08-10）将项目使用的灵巧手URDF从旧版本更新为厂商2025-04-18提供的最新版本。真机已到位，采用新URDF作为项目标准。

## 快速开始

### 查看迁移总结
```bash
python3 final_summary.py
```

### 验证迁移结果
```bash
python3 verify_migration.py
```

### 查看详细迁移记录
```bash
cat MIGRATION_2026_08_10.md
```

## 主要变更

### 关节名变更
- 旧：`thumb_proximal_yaw_joint` → 新：`right_thumb_1_joint`
- 旧：`thumb_proximal_pitch_joint` → 新：`right_thumb_2_joint`
- 旧：`index_proximal_joint` → 新：`right_index_1_joint`
- 旧：`middle_proximal_joint` → 新：`right_middle_1_joint`
- 旧：`ring_proximal_joint` → 新：`right_ring_1_joint`
- 旧：`pinky_proximal_joint` → 新：`right_little_1_joint`

### 限位变更
- thumb_1 (yaw): 1.308 → 1.246165 rad (-4.7%)
- thumb_2 (pitch): 0.6 → 0.48 rad (-20%)
- 四指 (MCP): 1.47 → 1.333 rad (-9.3%)

## 文件位置

### 当前标准URDF
```
assets/inspire_hand/urdf/inspire_hand_right.urdf
```

### 旧版本备份
```
assets/inspire_hand_legacy/
```

## 已更新文件

### 核心驱动
- `sim/inspire_hand.py` - 手动更新关节名、限位、映射配置

### 批量更新（自动迁移脚本）
共9个文件，50处替换：
- `sim/schema.py`
- `sim/ros_joint_writer.py`
- `sim/hand_rerun.py`
- `sim/live_rerun.py`
- `sim/build_inspire_from_vendor.py`
- `sim/skills/backend.py`
- `sim/skills/hand_pose.py`
- `sim/skills/test_hand_pose.py`
- `sim/test_combo_page.py`

### 文档
- `handarm_notes.md` - 更新URDF路径和关节说明

## 工具脚本

### migrate_hand_joints.py
批量替换代码中的旧关节名。

```bash
# 检查模式（不修改）
python3 migrate_hand_joints.py --check

# 应用模式（实际修改）
python3 migrate_hand_joints.py --apply
```

### verify_migration.py
验证迁移是否成功。

```bash
python3 verify_migration.py
```

### final_summary.py
显示迁移总结报告。

```bash
python3 final_summary.py
```

## 兼容性说明

### ⚠️ 已录制手势包
限位收紧后，超出新上限的帧会被自动夹到新限位：
- 拇指pitch动作可能变弱（收紧20%）
- 四指抓握力度可能降低（收紧9.3%）

**建议**：重新录制受影响的手势包

### ✅ 代码兼容性
- 厂商通道映射保持不变（`PROJECT_TO_VENDOR = [5,4,3,2,1,0]`）
- 方向配置保持统一（所有通道`invert=True`）
- 无需修改通信协议层

## 待验证清单

- [ ] 运行单元测试：`python3 -m pytest sim/test_*.py`
- [ ] 启动手部控制台（真机）：`python3 sim/hand_console.py --no-mock`
- [ ] 测试Web界面：`~/gradio_venv/bin/python sim/app_web.py`
- [ ] 测试已录制手势包回放
- [ ] 更新COMBO装配URDF（如果引用了手部）

## 回滚方案

如需回滚到旧版本：

```bash
# 1. 恢复旧URDF
mv assets/inspire_hand assets/inspire_hand_2025_04_18
mv assets/inspire_hand_legacy assets/inspire_hand

# 2. Git回滚代码修改
git checkout HEAD -- sim/

# 3. 恢复文档
git checkout HEAD -- handarm_notes.md
```

## 参考文档

- `MIGRATION_2026_08_10.md` - 完整迁移记录
- `handarm_notes.md` - 技术手册
- 新URDF来源：`assets/urdf_right/urdf_right_2025_4_18/` （厂商SolidWorks导出）

## 联系方式

如有问题，请参考`handarm_notes.md`中的项目信息。
