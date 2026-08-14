# Assets 目录重组方案 - 2026-08-10

> **历史归档。** 本文是迁移方案及当时的命令，不代表当前目录状态；不要重新执行其中
> 的移动/复制命令。当前目录边界见 [HANDBOOK.md](HANDBOOK.md)。

## 目标

1. 将分散在 `sim/assets/` 和 `assets/` 的资源统一到 `assets/` 下
2. 按用途分层:源文件(arm/hand) / 装配体(assembled) / 浏览器产物(viz)
3. `paths.py` 成为路径唯一真源
4. 保留厂商包的 config/launch/textures(完整性 + 参考价值)

## 新布局

```
lerobotTest/
└── assets/
    ├── arm/                              # 臂 URDF + mesh(from nero_description)
    │   ├── urdf/
    │   │   ├── nero_description.urdf
    │   │   └── nero_with_hand_flange_description.urdf
    │   ├── meshes/                       # STL/obj
    │   ├── config/
    │   │   └── joint_names_nero_description.yaml
    │   └── launch/                       # ROS1 launch(参考用)
    │       ├── display.launch
    │       └── gazebo.launch
    │
    ├── hand/                             # 灵巧手 URDF + mesh(from urdf_right_2025_4_18)
    │   ├── urdf/
    │   │   └── inspire_hand_right.urdf
    │   ├── meshes/                       # STL(13 个)
    │   ├── config/
    │   │   └── joint_names_urdf_right_2025_4_18.yaml
    │   ├── launch/                       # ROS1 launch(参考用)
    │   │   ├── display.launch
    │   │   └── gazebo.launch
    │   └── textures/                     # (空,保留结构完整性)
    │
    ├── hand_legacy/                      # 旧手(dex-urdf 版,备份)
    │   └── ...
    │
    ├── arm_legacy/                       # 旧版臂(备份/参考)
    │   ├── nero_old/                     # 64M
    │   └── nero_official/                # 1.6M
    │
    ├── assembled/                        # 装配体 URDF(pinocchio/MuJoCo 用,绝对路径 mesh)
    │   ├── nero_inspire_right.urdf           (臂+法兰+手,build_nero_inspire.py 生成)
    │   ├── nero_gripper_right.urdf           (臂+夹爪)
    │   └── inspire_hand_absolute.urdf        (手单体,绝对路径版,hand_rerun.py 用)
    │
    └── viz/                              # 浏览器可视化产物(glb + 相对路径 URDF)
        ├── arm/
        │   ├── nero_arm_viz.urdf
        │   └── meshes/*.glb                  (8 个:base_link + Link1..7)
        │
        ├── hand/
        │   ├── inspire_hand_right_viz.urdf   (新建,从 combo/meshes 挑手的 13 个 glb)
        │   └── meshes/*.glb                  (13 个:R_base_link + right_*_[1-4].glb)
        │
        └── combo/
            ├── nero_inspire_right_viz.urdf
            └── meshes/*.glb                  (33 个:臂 8 + 法兰 + 手 13 + ...)
```

## 删除 / 不迁移

- `assets/urdf_right/` — 与 `assets/inspire_hand/` 同源重复,删
- `sim/assets/inspire_hand_viz.urdf` — 重新生成,旧文件删

## 迁移到 arm_legacy(保留作为参考)

- `assets/nero_old/` → `assets/arm_legacy/nero_old/`
- `assets/nero_official/` → `assets/arm_legacy/nero_official/`

## 操作顺序(无 git 退路,用复制→校验→删除)

### 阶段 1:备份
```bash
cd /home/zhang123/ros2_ws/lerobotTest
tar -czf ~/lerobotTest_assets_backup_$(date +%Y%m%d_%H%M%S).tar.gz assets/ sim/assets/
```

### 阶段 2:新建目标结构
```bash
mkdir -p assets/{arm,hand,assembled,viz/{arm,hand,combo}}/{urdf,meshes,config,launch,textures} 2>/dev/null
```

### 阶段 3:复制源文件

#### 3.1 臂(nero_description → assets/arm/)
```bash
cp -r assets/nero_description/urdf assets/arm/
cp -r assets/nero_description/meshes assets/arm/
cp -r assets/nero_description/config assets/arm/
cp -r assets/nero_description/launch assets/arm/
```

#### 3.2 手(inspire_hand → assets/hand/)
```bash
# URDF:从子目录提到一级
cp assets/inspire_hand/urdf/inspire_hand_right.urdf assets/hand/urdf/
# 其余整目录拷
cp -r assets/inspire_hand/meshes assets/hand/
cp -r assets/inspire_hand/config assets/hand/
cp -r assets/inspire_hand/launch assets/hand/
cp -r assets/inspire_hand/textures assets/hand/
```

#### 3.3 装配体(sim/assets/*.urdf → assets/assembled/)
```bash
cp sim/assets/nero_inspire_right.urdf assets/assembled/
cp sim/assets/nero_gripper_right.urdf assets/assembled/
```

#### 3.4 viz 产物(sim/assets/*_viz/ → assets/viz/)
```bash
cp -r sim/assets/arm_viz/nero_arm_viz.urdf assets/viz/arm/
cp -r sim/assets/arm_viz/meshes assets/viz/arm/

cp -r sim/assets/combo_viz/nero_inspire_right_viz.urdf assets/viz/combo/
cp -r sim/assets/combo_viz/meshes assets/viz/combo/
```

#### 3.5 手 viz(从 combo 里挑出手的 13 个 glb)
```bash
# 新建 hand viz URDF(见阶段 4)
# 复制 glb:
for f in R_base_link right_{thumb,index,middle,ring,little}_{1,2,3,4}.glb; do
  [ -f sim/assets/combo_viz/meshes/$f ] && cp sim/assets/combo_viz/meshes/$f assets/viz/hand/meshes/
done
```

### 阶段 4:生成新产物

#### 4.1 手部浏览器 viz URDF
```python
# sim/build_hand_viz.py(改写)
# 输入:assets/hand/urdf/inspire_hand_right.urdf(STL 相对路径)
# 输出:assets/viz/hand/inspire_hand_right_viz.urdf(glb 相对路径)
```

#### 4.2 手部 pinocchio viz URDF
```python
# 已有 sim/build_hand_viz.py 的旧逻辑(STL 绝对路径)
# 输出:assets/assembled/inspire_hand_absolute.urdf
```

### 阶段 5:更新 `sim/paths.py`
```python
REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "sim"
ASSETS = REPO / "assets"
DATA = REPO / "data"
CONFIGS = REPO / "configs"

# 源 URDF + mesh
ARM_ROOT = ASSETS / "arm"
HAND_ROOT = ASSETS / "hand"
HAND_LEGACY = ASSETS / "hand_legacy"

NERO_URDF = ARM_ROOT / "urdf/nero_description.urdf"
NERO_FLANGE_URDF = ARM_ROOT / "urdf/nero_with_hand_flange_description.urdf"
INSPIRE_URDF = HAND_ROOT / "urdf/inspire_hand_right.urdf"

# 装配体(pinocchio/MuJoCo)
ASSEMBLED = ASSETS / "assembled"
ASSEMBLY_URDF = ASSEMBLED / "nero_inspire_right.urdf"
GRIPPER_URDF = ASSEMBLED / "nero_gripper_right.urdf"
HAND_ABSOLUTE_URDF = ASSEMBLED / "inspire_hand_absolute.urdf"

# 浏览器 viz
VIZ = ASSETS / "viz"
ARM_VIZ_URDF = VIZ / "arm/nero_arm_viz.urdf"
HAND_VIZ_URDF = VIZ / "hand/inspire_hand_right_viz.urdf"
COMBO_VIZ_URDF = VIZ / "combo/nero_inspire_right_viz.urdf"

# dex_retargeting
RETARGET_CONFIG = CONFIGS / "inspire_hand_right_local.yml"
RETARGET_URDF_DIR = ASSETS  # urdf_path 相对它解析

OUT = SIM / "out"
```

### 阶段 6:更新所有引用(~20 个文件)

按新 `paths.py` 逐个改,改完跑一遍验证脚本能跑。

### 阶段 7:更新前端

#### 7.1 `sim/app_web.py` 静态挂载
```python
# L797-799: hand_assets
_HAND_ASSETS = REPO / "assets/hand"
if _HAND_ASSETS.is_dir():
    app.mount("/hand_assets", StaticFiles(directory=str(_HAND_ASSETS)), name="hand_assets")

# L804-806: arm_assets
_ARM_ASSETS = ASSETS / "viz/arm"
...
# L810-813: combo_assets
_COMBO_ASSETS = ASSETS / "viz/combo"
...
```

#### 7.2 `sim/web/hand3d.js` 关节名
```javascript
// L18-25: MIMIC 的 key 改成新名(right_thumb_3_joint / right_index_2_joint ...)
export const MIMIC = {
  right_thumb_3_joint:  ["right_thumb_2_joint", 1.1425, 0.0],
  right_thumb_4_joint:  ["right_thumb_2_joint", 0.857789, 0.0],  // 链式展平
  right_index_2_joint:  ["right_index_1_joint",  1.1169, 0.0],
  right_middle_2_joint: ["right_middle_1_joint", 1.1169, 0.0],
  right_ring_2_joint:   ["right_ring_1_joint",   1.1169, 0.0],
  right_little_2_joint: ["right_little_1_joint",  1.1169, 0.0],
};

// L27-29: DRIVEN 改成新名
export const DRIVEN = ["right_thumb_1_joint", "right_thumb_2_joint",
                       "right_index_1_joint", "right_middle_1_joint",
                       "right_ring_1_joint", "right_little_1_joint"];
```

#### 7.3 `sim/web/index.html:2324` URDF 加载路径
```javascript
// 原:const n = await v.load("/hand_assets/inspire_hand_right.urdf");
// 新:
const n = await v.load("/hand_assets/urdf/inspire_hand_right.urdf");
```
或者在手 viz 里平铺一份到根目录,保持 `/hand_assets/inspire_hand_right_viz.urdf`。

### 阶段 8:清理

校验新布局工作正常后:
```bash
rm -rf sim/assets/
rm -rf assets/nero_description assets/inspire_hand assets/urdf_right
# nero_old / nero_official 待用户确认
```

## 验证清单

- [ ] `python3 sim/build_nero_inspire.py` — 装配体重建
- [ ] `python3 sim/hand_rerun.py --no-serve` — pinocchio 加载手
- [ ] `python3 sim/live_rerun.py --no-serve` — pinocchio 加载装配体
- [ ] `~/gradio_venv/bin/python sim/app_web.py` 启动,访问灵巧手调试页,3D 正常显示
- [ ] 合体页 3D 正常
- [ ] `python3 sim/build_hand_viz.py` 能跑且产物路径正确

## 风险点

- ❌ 无 git 退路:lerobotTest 未跟踪,移动错了只能靠备份 tar 恢复
- ⚠ retarget config 的 `urdf_path: inspire_hand/urdf/inspire_hand_right.urdf` 要改成 `hand/urdf/inspire_hand_right.urdf`
- ⚠ 前端关节名不匹配 → 3D 手指不动(已在阶段 7.2 修复)
- ⚠ `build_combo_viz.py:54` 硬编的 `_HAND_GLB` 路径要改

## 下一步

用户确认后,按阶段逐步执行。每个阶段完成后校验文件数量 / md5,确认无误再进入下一阶段。
