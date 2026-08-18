# 快速开始 - 灵巧手URDF迁移后

> **历史迁移快速参考。** 文中提到的迁移、验证、总结和提交脚本已于 2026-08-18
> 删除，下列命令仅保留当时记录，不能执行。现行启动与验证见 [README.md](README.md)、
> [HANDBOOK.md](HANDBOOK.md) 和 [PROJECT_STATUS.md](PROJECT_STATUS.md)。

## 迁移已完成 ✅

新URDF已成为项目标准，所有代码已更新。查看完整信息：

```bash
python3 final_summary.py
```

## 立即验证

### 1. 自动验证迁移
```bash
python3 verify_migration.py
```

### 2. 查看详细记录
```bash
cat MIGRATION_2026_08_10.md
# 或
cat MIGRATION_README.md  # 快速参考版本
```

## 手动测试步骤

### 真机测试（灵巧手）
```bash
# 启动手部控制台（真机模式）
python3 sim/hand_console.py --no-mock

# 预期结果：
# - 能识别新关节名 right_thumb_1_joint 等
# - 限位自动应用新值（thumb_pitch 0.48, 四指 1.333）
# - 串口通信正常（厂商映射保持不变）
```

### Web界面测试
```bash
# 启动Web服务
~/gradio_venv/bin/python sim/app_web.py

# 浏览器打开 http://<WSL_IP>:7860
# 测试：
# 1. 手部调试页面能正常显示关节名
# 2. 3D可视化正常（新URDF meshes）
# 3. 手势包回放功能正常（注意限位夹取效果）
```

### 单元测试
```bash
# 运行所有测试
python3 -m pytest sim/test_*.py -v

# 或只测试手部相关
python3 -m pytest sim/test_*hand*.py -v
```

## 关键变更一览

### 关节名
```
旧 → 新
thumb_proximal_yaw_joint   → right_thumb_1_joint
thumb_proximal_pitch_joint → right_thumb_2_joint
index_proximal_joint       → right_index_1_joint
middle_proximal_joint      → right_middle_1_joint
ring_proximal_joint        → right_ring_1_joint
pinky_proximal_joint       → right_little_1_joint
```

### 限位收紧
```
thumb_1 (yaw):   1.308 → 1.246 rad (-4.7%)
thumb_2 (pitch): 0.6 → 0.48 rad (-20%)  ⚠️ 明显收紧
四指 (MCP):      1.47 → 1.333 rad (-9.3%)
```

### 不变部分
- ✅ 厂商通道映射 `PROJECT_TO_VENDOR = [5,4,3,2,1,0]`
- ✅ 方向配置（所有通道 `invert=True`）
- ✅ 串口通信协议

## 常见问题

### Q: 手势包播放时动作变弱了？
A: 限位收紧导致。解决方案：
1. 重新录制手势包（推荐）
2. 调高 `force` 参数补偿
3. 修改 `sim/inspire_hand.py` 中的 `HAND_LIMITS` 恢复旧值

### Q: 需要重新录制所有手势包吗？
A: 不一定。只有需要用到拇指深度闭合和四指极限张开的手势包会受影响。
测试回放时观察，如果动作符合预期就不需要重录。

### Q: 如何回滚到旧版本？
A: 见 `MIGRATION_2026_08_10.md` 的"回滚方案"章节。

### Q: COMBO装配URDF需要更新吗？
A: 如果 `nero_inspire` 装配引用了手部，需要检查路径是否正确指向新URDF。

## Git提交建议

```bash
# 查看Git提交脚本（推荐分步提交）
./git_commit_migration.sh

# 或直接查看变更
git status
git diff sim/
```

## 文件位置

```
新URDF:    assets/inspire_hand/urdf/inspire_hand_right.urdf
旧备份:    assets/inspire_hand_legacy/
核心驱动:  sim/inspire_hand.py
文档:      MIGRATION_2026_08_10.md
```

## 工具脚本

```bash
final_summary.py          # 迁移总结
verify_migration.py       # 自动验证
migrate_hand_joints.py    # 迁移工具（已运行）
git_commit_migration.sh   # Git提交建议
```

## 支持

遇到问题请查看：
1. `MIGRATION_2026_08_10.md` - 完整技术细节
2. `handarm_notes.md` - 项目技术手册
3. 或提交issue到项目仓库

---

**迁移完成日期**: 2026-08-10  
**新URDF来源**: 厂商2025-04-18 SolidWorks导出  
**代码变更**: 9个文件，50处更新
