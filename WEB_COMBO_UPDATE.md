# Web端Combo页面更新完成 - 2026-08-10

> **历史归档。** 本文记录当时的 Web 合体模型修复。Web combo 仍是本地工作台能力，
> 但现行独立 MCP Server 不提供 combo 工具或接口。当前边界见 [README.md](README.md)。

## ✅ 问题解决

**问题**：Web端combo页面显示的还是旧模型（旧关节名）

**原因**：
1. `assets/viz/combo/nero_inspire_right_viz.urdf` 使用的是17:05生成的旧版本
2. 在我们17:35重新生成assembled URDF后，viz版本没有同步更新

## ✅ 已执行的修复

### 1. 更新了 `build_combo_viz.py` 脚本
修改了mesh查找逻辑，支持新的目录结构：
- 优先查找已有的glb（arm/手legacy）
- 找不到就从新手部STL转换
- 支持 `HAND_ROOT/meshes/*.STL` 路径

### 2. 重新生成了 Combo Viz URDF
```bash
python3 sim/build_combo_viz.py
```

**输出**：`assets/viz/combo/nero_inspire_right_viz.urdf` (11.6 MB, 23个mesh)

### 3. 验证新关节名 ✅
```bash
$ grep "joint.*name=.*thumb" assets/viz/combo/nero_inspire_right_viz.urdf | head -5
  <joint name="right_thumb_1_joint" type="revolute">
  <joint name="right_thumb_2_joint" type="revolute">
  <joint name="right_thumb_3_joint" type="revolute">
  <joint name="right_thumb_4_joint" type="revolute">
```

**所有关节名已更新为新版！** ✅

## 🔄 需要重启Web服务器

Web服务器当前正在运行（PID: 2035080, 2037346），但加载的是旧URDF。

### 重启步骤

```bash
# 方法1：找到并杀死进程
pkill -f app_web.py

# 等待几秒
sleep 3

# 重新启动
~/gradio_venv/bin/python sim/app_web.py &

# 或者如果在screen/tmux里，直接Ctrl+C后重启
```

### 方法2：强制刷新浏览器

如果Web服务器能自动检测文件变更，也可以尝试：
1. 浏览器打开 combo 页面
2. 按 `Ctrl+Shift+R` 强制刷新（清除缓存）
3. 如果还不行，清空浏览器缓存后再试

## 📊 完整装配体组成

重新生成后的combo viz包含：

### 机械臂（8个mesh）
- base_link.glb
- Link1.glb ~ Link7.glb
- Link8.glb (手腕末节，新转换)

### 适配法兰（1个）
- rh56df_adapter_flange.glb (新转换)

### 灵巧手（13个mesh，新关节名）
根link: R_base_link.glb

**驱动关节（6个）：**
1. right_thumb_1.glb - 拇指侧摆
2. right_thumb_2.glb - 拇指弯曲
3. right_index_1.glb - 食指MCP
4. right_middle_1.glb - 中指MCP
5. right_ring_1.glb - 无名指MCP
6. right_little_1.glb - 小指MCP

**耦合关节（6个）：**
- right_thumb_3.glb, right_thumb_4.glb
- right_index_2.glb, right_middle_2.glb
- right_ring_2.glb, right_little_2.glb

## ✅ 验证清单

- [x] `build_combo_viz.py` 脚本已更新
- [x] combo viz URDF已重新生成（17:38）
- [x] 新URDF包含新关节名（right_*_joint）
- [x] 所有23个mesh已生成到 `assets/viz/combo/meshes/`
- [ ] Web服务器已重启（需要手动操作）
- [ ] 浏览器已强制刷新
- [ ] Combo页面显示正确的新模型

## 🚀 提交变更

新生成的文件需要提交到GitHub：

```bash
git add sim/build_combo_viz.py
git add assets/viz/combo/nero_inspire_right_viz.urdf
git add assets/viz/combo/meshes/  # 如果meshes有更新
git commit -m "fix: 更新combo viz URDF使用新灵巧手关节名

- 修复build_combo_viz.py支持新目录结构
- 重新生成combo viz URDF（新关节名）
- 支持从手部STL转换glb（新手部URDF）
"
git push origin main
```

---
更新时间: 2026-08-10 17:38
脚本: sim/build_combo_viz.py
输出: assets/viz/combo/nero_inspire_right_viz.urdf
文件大小: 11.6 MB (23个mesh)
