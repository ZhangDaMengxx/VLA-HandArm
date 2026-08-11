# 手的可行域 / 自碰撞:方案与计划

**这份文档解决的不是硬件安全,是标签质量。**

数据集的 `action[t]` 是下一帧的绝对关节目标。若那些目标里有物理上不可能的构型,
policy 学到的就是"在这个观测下把食指压到 raw 0 同时拇指对掌" —— 一个**在这台机器上
不存在的动作**。训练时梯度往不可实现的方向推,评估时你分不清"policy 学得不好"和
"标签本来就不可达"。所以它在 EGO→VLA 的关键路径上,不是运维琐事。

## 一、活着的风险(2026-08-10 查出)

`replay_rgb_demo`(`voice_enabled: true`)指向 `sim/out/robot_traj_nero_inspire_rgb.npz`,
逐帧过 `check_feasible`:

| npz | 不可行帧 | 占比 | 是否登记成技能 |
|---|---|---|---|
| `robot_traj_nero_inspire_rgb.npz` | **78 / 766** | 10.2% | ✅ `replay_rgb_demo` |
| `robot_traj_raw.npz` | **362 / 710** | 51.0% | ❌ 未登记 |
| `robot_traj_nero_inspire.npz` | 0 | — | ❌ |
| `robot_traj_nero_inspire_rgbd.npz` | 0 | — | ✅ `replay_rgbd_demo`(干净) |

78 帧分 10 个连续块,最长 **帧 589..630 = 42 帧 @30fps = 1.40 秒持续顶**;
最深帧 416 命令食指 **raw 0** 而该处下界 **225**。

**为什么发得出去**:`check_feasible` 只在 `sim/skills/schema.py:299` 被调用 ——
那是 pose/action 那条路。`TrajectoryBackend.steps()` 直接 yield npz 原始帧,
**零可行域检查**。说一句「回放 RGB 示教」+ 确认,就是 1.4 秒堵转 →
过温位(ERROR Bit1,**不可清除**)。

## 二、根因

`sim/derive_embodiment.py:366`:

```python
hand12 = np.clip(savgol_filter(hand12, spec.savgol_win, spec.savgol_poly, axis=0), 0.0, 1.55)
```

重定向输出只做了**标量范围夹取**,没有任何自碰撞约束。

**这不是我们写错了。**`dex_retargeting` 本身就没有碰撞约束 —— 见第三节的外部确认。

## 三、外部工作:别重复发明

### `dex_retargeting`(已在用,`derive_embodiment.py:36` import)

查过它的优化器,结论是**架构能装约束,只是没装**:

- `optimizer.py:41` 用 **NLopt `LD_SLSQP`** —— 该求解器**原生支持
  `add_inequality_constraint`**
- 但 `optimizer.py:59-60` **只调了盒式限位**(`set_lower_bounds`/`set_upper_bounds`)
- `kinematics_adaptor.py:33` 的 `MimicJointKinematicAdaptor.backward_jacobian`
  能把梯度**穿过 mimic 关节**传回驱动关节 —— 约束的导数可以正确回传
- `optimizer.py:309` 的 `DexPilotOptimizer` 已有**指尖距离项**,但它在
  **目标函数**里(软的),不是约束

### Kilohertz-Safe(论文,`~/ros2_ws/KilohertzSafe-*.pdf`)

Tian / Yang / Zhao / Kan,USTC。把重定向改写成 **Δq 空间的凸 QP**,碰撞进硬约束:

- 胶囊碰撞模型,安全函数 `h(q) = ‖p_A(q) − p_B(q)‖₂ − (r_A + r_B)`,
  `p_A`/`p_B` 是骨架最近见证点,`h < 0` 即碰撞
- 距离雅可比(Danskin 定理)`J_dist(q) = n̂ᵀ[J_v,A − J_v,B]`,把 `ḣ` 变成
  `q̇` 的**线性**映射 —— QP 成立的关键
- CBF 条件 `ḣ(q) ≥ −α·h(q)` 保证安全集前向不变性

**它明确批评"碰撞写进目标函数"这种做法**:

> Existing nonlinear optimization based retargeting approaches typically encode
> collision avoidance by adding distance or collision related penalties **to the
> objective function**. Such treatments [are effective only] in a probabilistic or
> empirical sense, **rather than providing formal guarantees**.

**独立确认我们那 10.2% 是已知行为**:该文以 Dex-Retargeting 为基线,报告它
> show **sharp degradations during finger adduction** and other
> **interpenetration-prone phases**, due to the **absence of explicit
> collision-aware constraints**.

`finger adduction`(手指内收)就是拇指-食指并到一起。

延迟对比(该文 TABLE I):Ours 9.05ms / Dex-Retargeting 15.59ms / GeoRT 34.49ms;
RT@100Hz 分别 85.82% / 34.41% / 0.19%。

**三处别照搬**:

1. **它的「95%」比听起来弱**。口径是 `S_safe = clip(D_self / D_safe, 0, 1)`,
   结论是"95% 的控制步 safety score > **0.8**",**不是 95% 无碰撞**;
   且论文自称该指标是 *relative indicator ... rather than a complete physical
   safety model*。提取的部分里**未给出 `D_safe` 数值**。
   → **所以加载期硬检查必须长期保留**,不能指望约束后就干净。
2. 指标只算**非相邻 link 对**,mimic 链内部相邻对不在其中。
3. 硬件是 **Wuji Hand**,不是 Inspire;胶囊半径怎么定该文没给通用办法。

**未确认**:没找到代码开源声明(arxiv 正文 fetch 被拦,靠本地 PDF 提取)。

### 行业通行做法(训练知识,未逐条查证,用前自己核)

- **一次性生成碰撞对矩阵**(MoveIt Setup Assistant:采样上万构型,标出"永远碰"
  和"永不碰"的 link 对写进 SRDF)—— 产物是**哪些对要查**,不是可行域表
- **在线逐构型查碰撞**是常态(MoveIt/FCL、Drake、pinocchio+coal)
- **球/胶囊近似 + 可微距离**用于放进优化器(NVIDIA curobo,GPU kHz 级)
- **没人预计算 3+ 自由度子系统的可行域表** —— 维度爆炸,且在线查已够快

## 四、我们要什么 / 不要什么

**关键判断:我们是离线数据集生成,没有延迟约束。**Kilohertz-Safe 的核心机制
(Δq 空间、凸 QP、CBF)全是为了 9ms 实时遥操作 —— **我们不需要**。

| 论文的做法 | 它为什么要 | 我们要吗 |
|---|---|---|
| Δq 空间 + 凸 QP | 9ms 实时 | ❌ 离线可以慢慢解 |
| 胶囊近似 | 让约束可微且快 | ❌ 离线用**精确网格-网格**,毫米间隙下更准 |
| CBF `ḣ ≥ −αh` | 连续时间前向不变性 | ⚠️ 二期(限制相邻帧朝边界靠近的速度,降低硬件插值穿越概率) |
| **`h(q) ≥ 0` 作为硬不等式约束** | 形式保证 | ✅ **这就是我们要的那一条** |

**产物是「检查器」,不是「表」。**`is_feasible(q) -> (bool, penetration_mm)`。
理由:表是采样密度的函数(换网格要重算)、丢掉穿透深度(那个量正好拿来对账力偏移)、
且只能查静态点(查路径要另一套)。`FEASIBLE` 可保留成**检查器的导出缓存**给加载期
快速用,但真相来源是检查器 + 标定参数。

**库选 `trimesh` + `python-fcl`,不选 pybullet**:pybullet 默认把凹网格转凸包做碰撞,
手指网格 1400-3400 面、指间间隙毫米级,凸包会让手指"变胖",系统性偏保守且偏多少未知。
FCL 给精确网格-网格 + 穿透深度。FK 自己写(~60 行,串链累乘 URDF origin)。

## 五、两个概念陷阱(想清楚再动手)

### 陷阱一:几何和实测量的不是同一个量

- **几何**给的是**接触起始点** —— 网格刚碰上那个角度。二值、精确、**与力无关**
- **实测**给的是**某个力/速度下的堵转位置** —— 已经过了首次接触(肌腱拉伸 + 接触变形)

`HAND_DEBUG.md:646` 自己写着:表里的 **225 是「卡住的位置」,不是「安全到位的位置」**。

所以拿几何值去比 225,**应该期望几何值 > 225**,不是等于。validation 判据必须**不对称**
(假阴性危险,假阳性只是保守):

1. 实测堵转 ≤ 几何接触点(方向必须对)
2. 力越大,两者差越大(要在两个力档各扫一次才验得出)
3. 各扫描点的**排序**一致(哪个位置最紧,几何和实测得说同一个)
4. **几何说不碰的地方,实测必须真能过去** —— 这条最强,几何漏判会直接让校验放过撞的姿态

### 陷阱二:实测的第一份工作是「标定几何的输入」,不是「验证几何的输出」

"实物和 xlsx 不同"更准确说是 **URDF 和 xlsx 不同** —— 两个都是模型,实物是第三方。
而碰撞几何活在 URDF 的关节角空间里:`rad_to_raw` 把 rad 夹到 `LIMIT_HI`(URDF 的 0.600),
实际行程是 xlsx 的 0.698。**raw 0 到底对应哪个弧度?** 搞错的话几何算的是错构型,
输出再密也没意义。

所以顺序是:**实测标定 raw↔rad → 几何算密集可行集 → 实测抽查(4 条判据) → 输出**。
实测从"验证者"变成"标定源 + 抽查者",点数因此从 20-30 降到 5-8。

## 六、计划(按依赖排,每步独立验收)

### 第 0 步:`TrajectoryBackend` 加载期全帧预检 —— 离线,~15 行,现在就能做

拿现成 `check_feasible` 在 `_load()` 里过一遍全部帧,不可行 = **加载失败**
(和 pose 那条路同一个判据、同一个态度:`HAND_DEBUG.md:642` 宁可清单加载不过)。

**不会白做**:后面检查器落地后,这里换成调用新检查器即可,位置和语义不变。

验收:`replay_rgb_demo` 加载即报 78 帧不可行并指出帧号;`replay_rgbd_demo` 照常加载。

### 第 1 步:读 Kilohertz-Safe ✅ 已完成(2026-08-10)

收益:第 5 步的约束形式不用自己设计,`h(q)` / `J_dist` 公式直接可用;
并确认了不需要 QP/CBF/胶囊那套(第四节)。

### 第 2 步:t5 对角线标定 —— ~5min 真机,5 个点,**要你点头**

`test_thumb_index_collision.py --only t5`。脚本已改好(2026-08-10):
`SCAN_SPEED=50`(顶死力 272g 而非 >941g)、每点前 `go_open`、每点后
`check_faults` 查 ERROR/TEMP、过温或 ≥55℃ 立即中止。

**定位是标定,不是验证** —— 产出真实行程端点给几何用,顺带验旧表可不可信
(`compare_feasible` 已实现,4 种情形离线验过)。

验收:能否重现 `FEASIBLE` 的 (300,225)/(450,52)/(600,0)。重现不了 → 先搞清楚
哪个不对,别往二维扩。

### 第 3 步:几何碰撞检查器 —— 装 `trimesh` + `python-fcl`,~150 行

URDF + 29 个碰撞网格 + 第 2 步标定参数 → `is_feasible(q) -> (bool, penetration_mm)`。
**先只做拇指-食指对**(重点),碰撞对矩阵留接口。

已知结构:`thumb_intermediate` mimic pitch ×1.334、`thumb_distal` mimic pitch ×0.667、
`index_intermediate` mimic index ×1.06399 offset −0.04545(`inspire_hand_right_glb.urdf`)。
所以拇指-食指的真实可行域是**三维** `(yaw, pitch, index)`,不是现在压成的一维。

验收:几何预测 vs t5 实测满足第五节那 4 条不对称判据。

**2026-08-11 实施:`sim/collision_checker.py`(~330 行)。检查器可用,但 URDF 不可信。**

三个已修的坑:

1. `fcl.BVHModel(verts, faces)` 带参构造 **segfault**,必须走 `beginModel/addSubModel/endModel`
2. FCL 的 mesh-mesh `penetration_depth` **不可用** —— 拿已知重叠的方块验证,
   重叠 0.5mm 和 10mm 都报 20mm(= 方块边长)。改用 `fcl.distance` 报最小间距;
   布尔判定(`fcl.collide` 返回值)可信
3. 正运动学必须从 URDF 读:`thumb_1` 轴是 `0 0 -1`、`index_1` 是斜轴
   `0 −0.0698 −0.9976`,多个 origin 带 rpy。原先"全部绕 Z"的硬编码是错的

本文档上面写的 mimic 系数(×1.334 / ×0.667 / ×1.06399)来自 `inspire_hand_right_glb.urdf`,
该文件已移到 `assets/inspire_hand_legacy/`。**在用的**
`assets/hand/urdf/inspire_hand_right.urdf` 是另一套:`thumb_3 = thumb_2 ×1.1425`、
`thumb_4 = thumb_3 ×0.7508`、`index_2 = index_1 ×1.1169`,关节命名也不同
(与 `hand_pose.HAND_JOINTS` 对得上)。

判据结果(旧表三点,`yaw=pitch=T`):

| T | 实测堵转 index | 几何首次接触 | 判据一 |
|---|---|---|---|
| 300 | 225 | 820 | 几何更保守 ✓ |
| 450 | 52 | 610 | 几何更保守 ✓ |
| 600 | 0 | 全程不碰 | 几何更保守 ✓ |

判据一(实测堵转 ≤ 几何接触点)、判据三(排序一致)、判据四(几何说不碰的实测能过去)
在这三点成立。判据二要两个力档的真机数据,没做。

**但这三行都不算数** —— URDF 自身不自洽:

- mimic 把从动关节顶出**它自己的限位**:`thumb_2` 取到自己的上限 0.48 时,
  `thumb_3` = 0.548(声明上限 0.3578,超 53%)、`thumb_4` = 0.412(上限 0.2775,超 48%)。
  所以主动关节合法时从动关节已经违规
- 因此表里每个碰撞点都至少有一个拇指关节越界。新增 `CollisionResult.out_of_limit`
  把这件事显式暴露,不让虚构构型静默产出结论
- **在完全不越界的构型里,拇指-食指几何上从不碰撞**。而真机确实会互顶
  → URDF 限位偏紧,不是实物行程

`thumb_2` 上限有三个互斥值:URDF 0.48 / `hand_pose.LIMIT_HI` 0.6 / xlsx span 0.698。
raw < 312 全落在争议区,而**这正是唯一会发生碰撞的区域**。这就是陷阱二本身:
raw↔rad 定不下来,几何算的是错构型。

顺带查清 `hand_pinch` / `hand_ok`:命令 (yaw 108, pitch 141, index 0) 几何判碰
(`thumb_2`×`index_2`),而实测堵转位置 (583, 602, 383) 几何判可行、间距 7.2mm。
即**手能做出这个动作是靠堵转,不是靠姿态安全** —— 检查器报警是对的,
之前临时放宽成 `index: 0.24` 是把真实的堵转盖住了(命令侧那条判定同样落在争议区)。

**结论:第 3 步的机械部分完成(FK + 碰撞 + 距离,自检通过),验收未过。
卡在第 2 步 —— raw↔rad 真机标定没做,而计划里第 2 步要你点头。**

### 第 4 步:路径检查 —— ~40 行,复用第 3 步

按各通道 `SPEED_SET` 推中间轨迹,逐个中间构型查碰撞。

**验收:能离线复现握拳那次卡死。** 这是最硬的验收 —— 那个故障**终点合法、路径不合法**
(拇指 yaw 走 802 counts、食指 680、其余 1000,同速不同距,半路相遇),
静态表永远看不见。同时能判定那 78 帧是"终点越界"还是"帧间穿越",两者修法不同。

### 第 5 步:约束进 `dex_retargeting`

`add_inequality_constraint`(NLopt SLSQP 原生支持),约束用第三节的 `h(q) ≥ 0`。
根治 `derive_embodiment.py:366` 那个裸 clip。

验收:重跑 `derive_embodiment`,新 npz 不可行帧数**大幅下降**。
⚠ 不写"= 0" —— 论文自己的数据说约束后仍有残留,第 0 步的闸要一直在。

### 第 6 步:换手复用 —— 把 1-5 的输入收成「手型档案」

| 输入 | 换手时 |
|---|---|
| URDF(关节树 + origin + limits + mimic) | 跟新手资料来 |
| 碰撞网格 | 跟新手资料来 |
| 驱动关节 → 通道映射(`PROJECT_TO_VENDOR`) | 跟新手资料来 |
| raw 约定(`RAW_MAP`,哪头是 0、是否 invert) | 跟新手资料来 |
| **真实行程(标定)** | **每只手实测一次(第 2 步,5-8 点)** |

`hand_pose.py --verify` 已经用 importlib 按路径核对 `inspire_hand` 的表 ——
"手型档案"这个概念隐含存在了,第 6 步是把它显式化。

**明确不做**:t6 二维网格扫描(20-30 点真机 —— 几何算得更密更快,把便宜的事做贵了)、
预计算三维可行域表当真相来源、自己写重定向优化器。

## 七、待定

1. **装不装 `trimesh` + `python-fcl`** —— 第 3 步的前提,pip 纯 Python 轮子
2. **`hand_pose` API 改不改** —— 一维 `index_min_raw` → 按碰撞对查表。
   不改就只能继续 `max()` 压一维,几何算出的三维信息浪费掉
3. **t5 什么时候跑** —— 要真串口,按只读约定等你点头
4. **Kilohertz-Safe 有没有开源 repo** —— 有的话 `J_dist` 那部分可直接读

## 八、相关记录

- `HAND_DEBUG.md:619-653` 干涉实测原始结论、`LIMIT_MARGIN` 由来、对角线外插限制
- `HAND_DEBUG.md:982` 早已列了"补可行域二维扫描"这条待办 —— 本文档把它**否决**了
  (改走几何),否决理由见第四、六节
- `更新日志.md` 2026-08-10 条:握拳拆两阶段、`params_passthrough`、`hold` 语义为何先不做
- **`FEASIBLE` 的出处是错的**:`hand_pose.py:107` 标注"来自 T3",但 T3 把 pitch 固定
  `OPEN` → `T=max(yaw,1000)` 恒为 1000,且测点是 1000/800/…/0,**产不出 300/450/600**。
  真实来源是一次对角线扫描,代码不在该文件里。已在
  `test_thumb_index_collision.py` 顶部和 `sweep()` docstring 记下,t5 负责补回。
  **教训:产物必须自带 provenance(手型名 / URDF 哈希 / 哪些点实测 / 力速条件 / 残差),
  写进文件而不是注释** —— 注释会和代码分叉,这次就是。
