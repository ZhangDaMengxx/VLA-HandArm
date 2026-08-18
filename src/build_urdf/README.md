# build_urdf — 装配 URDF 的几何推导脚本

这些**不是运行时代码**,是一次性的几何推导:从装配体 STL 反解出
`link7 → link8 → 适配法兰 → 手` 这条链上的固定变换,结果填进
`../build_nero_inspire.py` 顶部的常量块。

什么时候需要重跑:换适配法兰、换手、换臂,或怀疑挂接位姿不对。
平时改 URDF 不需要动这里 —— 直接跑 `../build_nero_inspire.py` 就行。

**数据源**(都在 `assets/nero_description/meshes/`,毫米制)

- `nero_RH56DF.stl` — 装配体,245 万三角面,含臂 + link8 + 适配法兰 + 手。
  这是唯一的真值来源。
- `rh56df_adapter_flange.stl` — 单独的适配法兰。原文件名 `NERO+因时RH56DF_适配法兰.stl`
  带中文和 `+`,转 `package://` URI 会出问题,故另存了 ASCII 名副本。

## 跑法

必须**在本目录内**跑(中间缓存写 `_cache/`),用 conda lerobot 环境
(需 numpy / scipy / trimesh):

```bash
cd ~/ros2_ws/lerobotTest/src/build_urdf
P=~/miniconda3/envs/lerobot/bin/python

$P reg1.py          # 零位 FK + 装配体点云 → _cache/asm_v.npy, arm_v.npy
$P reg4.py          # 装配体按连通分量拆件 → _cache/asm_big.npz
$P reg12.py         # 关节7轴线圆拟合
$P eps_test.py      # ε(绕工具轴 180° 二义性)判定
$P flange_align.py  # 法兰自身位姿:48 种轴对齐变换穷举
$P reg15.py         # 手平移精修 + 法兰质量惯量
$P remount.py       # 汇总成可直接填的 xyz/rpy 字符串  ← 权威值出处
$P final_check.py   # 端到端:解析生成好的 URDF 跑 FK,与装配体比残差
$P finger_check.py  # 核对 dex-urdf 手指布局 vs 装配体(屈曲无关的那一轴)
```

诊断残差用(想知道"到底差在哪"时跑):

```bash
$P residual_breakdown.py  # 拆解:度量假象 / 点→顶点 / 点→面
$P residual_tail.py       # 长尾是真实差异还是目标连通体取窄了(改用区域取目标)
$P tail_source.py         # 定位长尾的具体位置与来源
$P plane_levels.py        # 量法兰区域的 z 向平面高度与面积(定案内部面问题)
```

前两步有先后依赖(`reg4` 吃 `reg1` 的缓存),之后各步只依赖缓存,可单跑。
`_cache/` 约 100MB,删了重跑 `reg1`+`reg4` 即可复原。

> **注意** `eps_test.py` 末尾打印的 `link7 -> adapter` 是**法兰朝向修正之前**的旧值
> (它写在 `flange_align.py` 之前)。填 URDF 只认 `remount.py` 的输出。

中间诊断脚本,按需单跑:`reg2`(z 向切片对比)、`reg3`(连通体拆分原型)、
`reg5`(左右手判别)、`reg6`(零件签名匹配)、`reg8`(平面面元普查)、
`reg9`(圆柱轴线检测)、`reg10`/`reg13`/`reg14`(ε 的几个旁证)、
`validate`、`verify_flange`、`flange_shape`、`slice_flange`、`mount_values`。

公共库:`stl_probe`(二进制 STL 读取)、`urdf_fk`(URDF 解析 + FK)、
`icp`(配准,含定旋转变体)、`feat`(平面提取 / 表面采样)、
`cyl`(法向投票找圆柱轴线)。

## 结论

| 量 | 值 | 来源 / 残差 |
|---|---|---|
| 关节7轴线 | 装配系 y=+0.20, z=+42.489, r=22.378 mm | Ø44.8 圆柱面圆拟合,残差均值 0.485mm |
| ε(绕工具轴朝向) | +1 | link8 拟合中位 0.357mm,反向 0.528mm |
| 法兰自身位姿 | 绕 Y 转 180° + 沿 z 平移 −5mm | 48 变换穷举 rmse 0.119mm(单位变换 1.432mm) |
| 手基座平移 | (−0.042, +5.962, −7.158) mm | 手掌 ICP rmse 0.3555mm |
| 法兰质量 | 22511 mm³ → 铝 60.8 g | 0.4mm 体素填充 |

填进 `../build_nero_inspire.py` 的最终值:

```
link8 → 法兰      xyz="0 0 0.016489"            rpy="0 0 1.570796"
法兰  → 手根 base  xyz="0.000042 0.005962 0.002158"  rpy="0 0 1.570796"
```

端到端校验:三个件对装配体的残差(中位)。**两种度量差 6 倍以上,别混用**:

| 件 | 点→顶点 | 点→面 | 度量假象下限 |
|---|---|---|---|
| link8 | 0.267 | **0.081** | 0.90 |
| 适配法兰 | 0.627 | **0.079** | 1.55 |
| hand_base_link | 0.300 | **0.101** | 0.92 |

`final_check.py` 报的是**点→顶点**,且装配体顶点还降采样到 20%。装配体三角面本身
有大小(边长 0.6~1.4mm),落在面中心的采样点离最近顶点天生就有距离 —— 这是**度量
假象**,与贴合好坏无关。"假象下限"一列是装配体**自己的表面采样点到自己的顶点**
(件根本不参与)的中位距离,可见 link8 与法兰报出来的数几乎全是这个假象
(0.267 vs 0.90、0.627 vs 1.55)。法兰数字看着最差只因它三角面最粗。

真实贴合(点→面,`residual_breakdown.py` / `residual_tail.py`)三件都是
**0.08~0.10mm**,量级一致。

**长尾**(点→面 >1mm 的比例)有两处,各有明确来源,都不是错位:
- 法兰 4.5%,全在装配系 z=−5 单个平面。独立 STL 是多个**未布尔**的重叠实体
  (Rhino 常见),主板顶面与凸台底面在此共面、两者都在网格里;装配体做过布尔,
  凸台底面成了内部面。我的采样点落在这张内部面上,故读出 ≈1.7mm。该平面上
  独立 STL 有 1197mm²、装配体只有 795mm²,差的 402mm² 正是那张内部面。
- 手掌 13.4%，在装配系 y∈[13,31]（即手的掌侧）。这是**真实形状差异**：同一薄层
  里 URDF 掌壳伸到 z=−133、装配体只到 −126。dex-urdf 的掌壳与原厂 CAD 形状不同,
  最大差 ~6mm。**只影响碰撞体,不影响运动学** —— 坐标系(0.3mm)和手指布局
  (≤0.05mm)都已单独验过。

**手是右手。** 装配体拇指凸台占 x∈[15,90]、y∈[8,71],只有右手映射能落进去,
左手会落到 y<0;手掌 ICP 也是右手更优(0.355 vs 0.737mm)。

**q=0(臂竖直)时的朝向**:手心朝 world −x、手指朝 +z(沿工具轴伸出)、
小指在 +y 侧、拇指在 −y 侧。

**手指内部几何也核过了**(`finger_check.py`)。装配体里手指是未知屈曲角,不能直接
比位置;但屈曲绕各指关节轴转、该轴 ∥ 手的 z 轴 → 映射到装配系 x 轴,故**沿 x_asm
的位置与屈曲角无关**,可直接比。结果:四个 proximal 段中心差 ≤0.05mm、四个
intermediate 段 ≤0.36mm(中节偏差略大合理,还受各指 tilt 角影响),宽度差全部 ≈0。
结论:dex-urdf 的手指布局与真实 CAD 一致,retarget 的指尖精度可用。

## 两个坑

**别用 bbox 判法兰朝向。** 法兰 z 范围 [−15, 10] 关于 −2.5 反对称,导致
"单位变换"和"绕 Y 180° + z−5"两种摆法的 bbox **六个面全部吻合到 0.0005mm**。
只看 bbox 会得出"法兰在装配体里就是单位变换"的错误结论(我第一遍就栽在这)。
要看切片轮廓:独立 STL 是 z−15..−2 方板 / z+1..+10 圆段,装配体里正好相反
(`slice_flange.py` 就是干这个的)。

**臂的视觉网格必须是 STL。** 臂原本用 `.dae`,而缺 `pycollada` 的 trimesh 系
查看器、以及 VSCode URDF Visualizer,都会**静默跳过整条臂** —— 症状是
"URDF 里手和法兰都有,就是没有臂"。臂的 link / 关节 / 惯量其实一直都在。
`../build_nero_inspire.py` 的 `dae_visual_to_stl()` 负责换成同名 `.STL`,
并补 URDF material(STL 不带颜色信息,否则 RViz 里整条臂是一坨灰)。

## 上游 joint8 的 1mm(已修)

臂 URDF `nero_with_hand_flange_description.urdf` 里 joint8 的 x 原写 **0.032**,
官方 xacro(`nero_with_revo2_flange` / `nero_with_gripper_flange`)是 **0.031**。

判据有两条,互相独立:
- `Link8.STL`、`stand_v2.STL`、`nero_old/revo2_flange.stl` 三者 **md5 完全相同**
  —— 同一零件三个名字,不该有两个安装值。
- 装配体实测:按 0.031 摆 `Link8.STL`,对装配体中 link8 实体的中位残差
  **0.311mm**;按 0.032 是 0.557mm。

**2026-08-04 已改为 0.031**,同时把 `FLANGE_MOUNT_XYZ` 的 Z 从 0.015489
(含 1mm 补偿)改回 **0.016489**(不含补偿)。两处必须同改,只改一处法兰和手
的绝对位姿就会漂 1mm。

改完的效果:法兰与手的世界位姿、指尖坐标**逐位不变**,只有 link8 自己下移 1mm,
其残差中位 0.522 → **0.267mm**(mean 0.630 → 0.437)。

> 仓库里另外三个臂 URDF(`nero_description.urdf`、`nero_description_stand_v2.urdf`、
> `nero_description_with_{left,right}_hand.xacro`)的对应值是 **0.033**,同属这个
> 问题,但它们走夹爪/旧手路线、不在本装配链上,未改动。
