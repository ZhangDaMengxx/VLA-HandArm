"""RobotSpec:把「一台机器人」需要的全部参数收成一个规格对象。

规范层(canonical_ds)本体无关;`derive_embodiment.py` 拿 canonical_ds + 一个 RobotSpec →
按这台机器人的 URDF/重定向配置派生出它的 LeRobotDataset。**换机器人 = 加一个 RobotSpec**,
采集与规范层一字不动。这是「一次采集、多本体复用」的落地点。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import numpy as np

from paths import REPO, RETARGET_CONFIG, RETARGET_URDF_DIR, ASSEMBLY_URDF, NERO_URDF, GRIPPER_URDF
from schema import ARM_JOINTS, HAND_ACTUATED


@dataclass
class RobotSpec:
    name: str
    # --- 手:dex-retargeting ---
    retarget_cfg: Path              # 重定向配置(.yml)
    urdf_dir: Path                  # 配置里 urdf_path 的解析根
    hand_actuated: List[str]        # 进入 state/action 的驱动手关节(dex:retarget 12 输出的子集;gripper:["gripper_joint"])
    # --- 臂:IK ---
    arm_urdf: Path
    ee_frame: str
    q_home: np.ndarray              # (nq,) home 姿态(法兰朝向 + 位置锚点)
    arm_joint_names: List[str]      # 进入 state/action 的臂关节名
    # hand_mode: dex=灵巧手(retarget 12→取 6 驱动);gripper=平行夹爪(拇指-食指捏合距离→1 标量开合)。
    # 臂部分两模式完全一致(IK 只认 link7);区别只在"手"这几列 + 可视化 URDF。
    hand_mode: str = "dex"
    # 平行夹爪的开口宽度(m,两指夹持面间距),gripper 模式下把捏合宽度线性映射到 [0, gripper_open_width_m]。
    # 语义对齐官方 SDK pyAgxArm 的 move_gripper_m(value=开口宽度,m) 与官方 URDF 主关节 gripper
    # 的 limit[0,0.1];单指行程 = 该值的一半(URDF 里两个 prismatic 各 [0,0.05])。
    gripper_open_width_m: float = 0.1
    # 可视化 URDF(replay_rerun 按本体加载;IK 仍用 arm_urdf/nero_description)。默认 inspire 装配。
    viz_urdf: Path = ASSEMBLY_URDF
    ee_frame_correction_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # human physical wrist body frame -> robot ee body frame.
    # Columns encode where human X/Y/Z axes land in robot ee coordinates.
    wrist_motion_basis_R: np.ndarray = field(default_factory=lambda: np.eye(3))
    wrist_position_basis_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wrist_rotation_compose: str = "left"  # left=dR @ home; right=home @ dR,用于诊断动态旋转在 world/local 轴上的差异
    arm_position_mode: str = "relative"  # fixed=锁 home/anchor; relative=legacy 相对位移; absolute=metric 绝对米制位置
    # --- 绝对锚定模式(frame_mode="anchored")---
    # R_hand_ee: 人手 physical wrist body frame -> robot ee body frame 的固定装配旋转。
    # 列 = robot ee 轴在 human wrist 系里的方向;数值上 = wrist_motion_basis_R.T,
    # 但语义是"装配关系"(不再对 delta 做相似变换)。锚定模式只用它 + 首帧锚定,
    # 不用 wrist_motion_basis_R / wrist_position_basis_rpy / wrist_rotation_compose 这三个 legacy 旋钮。
    R_hand_ee: np.ndarray = field(default_factory=lambda: np.eye(3))
    frame_mode: str = "legacy"  # metric=固定 base 绝对映射; anchored=首帧锚到 home; legacy=旧 relative/compose
    # --- 度量模式(frame_mode="metric")---
    # 固定的世界->base 旋转(与数据无关,物理摆放定死;把世界向量转到 base 系)。
    R_base_world: np.ndarray = field(default_factory=lambda: np.eye(3))
    # 人手工作空间质心映射到的 base 系锚点(m)。centroid 每段按数据算,再平移到这里。
    p_base_anchor: tuple[float, float, float] = (0.1, -0.1, 0.5)
    metric_scale: float = 1.0            # 人臂展->机器人臂展缩放;1.0=米制原样
    arm_position_gain: float = 1.0
    arm_position_limit_m: float = 0.05   # 相对 home 的最大末端位移半径,避免视觉跳点甩飞 IK
    # --- 稳定化 / 平滑(见 wrist_stabilize.py + build_robot_traj)---
    # ⚠ 勘误(2026-08-03):下面三行的注释原来和值**全不一致** —— 注释写 8°/40%/11 帧,
    # 而值是 25/1.0/9。注释没跟着调参更新,读代码的人会得到完全错误的印象。
    # 尤其 oop_alpha:注释说"只保留 40%",实际 1.0,而 attenuate_out_of_plane 第一行是
    #     if alpha is None or alpha >= 1.0: return Rs
    # 也就是**整个出平面衰减是空转的**,一点作用都没有。
    # 这次只把注释改成实话,不动值 —— 改值是调参,要先有依据(见下面各行的说明)。
    gate_deg: float = 25.0
    """帧间旋转增量限幅(度)。超过就沿测地线截到这个值。<=0 关闭。

    实测(RGB 素材,710 帧)帧间增量 p95 只有 4.75°、max 14.23° —— **全在 25 以下,
    所以这个门限几乎不触发**,等于没设。原值是 8°,被放宽到 25 大概是因为快转被截。
    真要让它起作用得回到 10° 上下,但那会截掉真实快转,得先分清哪些是跳变哪些是真动作。
    """
    oop_alpha: float = 1.0
    """出平面(绕相机 X/Y)朝向衰减系数。1.0 = **完全不衰减,即功能关闭**。

    ⚠ 这个 1.0 意味着 attenuate_out_of_plane 直接 return,没有任何效果。
    为什么现在不改回 0.4:
      · 实测 RGB 素材上出平面占姿态误差能量 **98.7%**(相对首帧漂移 p95 39°),
        面内只有 5°。所以衰减方向是对的,工具也对
      · 但出平面里**混着真信号** —— 手心朝下、手背翻上来都是绕 X/Y 的真动作。
        一刀切按幅度衰减会把它们一起压小(原注释里的抱怨就是这个)
      · 而且我们最终用 **RGB-D**,深度进来后出平面歧义本身就小:实测 RGB-D 素材
        姿态抖动只有 4.1~4.8°,是 RGB 的 1/4~1/5。可能压根不需要衰减
    要改的话正确做法是**按时间尺度分**(慢漂移=噪声偏置,压;快变=真动作,留),
    不是按幅度一刀切。前提是"慢漂移都是噪声",这一条**还没验证**。
    """
    savgol_win: int = 9
    savgol_poly: int = 3
    """
        - savgol_poly=2：用二次曲线，适合保留加速/减速动作，比线性平滑自然。
        - savgol_poly=3：能保留更多细节，但对噪声更敏感。
        - savgol_poly=1：更像局部直线平均，动作会更钝。
    """


    k_null: float = 0.0
    repo_id: str = ""

    def __post_init__(self):
        if not self.repo_id:
            self.repo_id = f"local/{self.name}_handdemo"

    @property
    def out_root(self) -> Path:
        return REPO / f"src/out/lerobot_ds_{self.name}"


# ============================================================================
# 两个 NERO 规格,按数据源分开,互不污染:
#   nero_inspire_rgb  —— 普通 RGB(无深度/无世界系)。legacy 相对路径 + 原始静止 home。
#                        RGB 没有 T_base_world,只能待在相机系相对区,这是它唯一能可达的配置。
#   nero_inspire_rgbd —— kinect RGB-D。anchored + 几何正确 R_hand_ee + 重摆 home + 位置锁定。
#                        朝向物理正确(555/557),是通往固定 base 度量摆放的过渡态。
# 别名 nero_inspire -> _rgb,保持旧命令(普通 RGB)开箱即用。
# ============================================================================

# ---------- 普通 RGB:legacy 相对,原始 home,不含任何 anchored 改动 ----------
NERO_INSPIRE_RGB = RobotSpec(
    name="nero_inspire_rgb",
    retarget_cfg=RETARGET_CONFIG,
    urdf_dir=RETARGET_URDF_DIR,
    hand_actuated=HAND_ACTUATED,
    arm_urdf=NERO_URDF,
    ee_frame="link7",
    # 手腕轴朝 +Y 横向;绕 X +90° 映射到 +Z,手心更接近朝向相机。
    ee_frame_correction_rpy=(np.pi / 2.0, -np.pi / 2.0, 0.0),
    # human physical wrist body frame -> NERO link7/ee body frame(legacy 相似变换用)。
    wrist_motion_basis_R=np.array([
        [0.0,  0.0, -1.0],
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
    ]),
    # 相对 wrist 平移 -> NERO base 相对平移的临时轴映射。
    wrist_position_basis_rpy=(0.0, 0.0, -np.pi / 2.0),
    frame_mode="legacy",           # 旧 relative/compose 路径
    arm_position_mode="relative",  # 跟随 wrist 相对位移
    # 原始静止 home(anchored 重摆前的值),RGB legacy 路径用它。
    q_home=np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]),
    arm_joint_names=ARM_JOINTS,
)

# 几何正确装配(rgbd 两个规格共用)= --r-hand-ee -Y,-Z,+X:
#   human Z(指向)->ee+X(link7 approach)、human X(掌法向)->ee-Y(palm normal)、human Y->ee-Z。
# 由 estimate_wrist 人手系约定 + URDF link7->inspire 装配链推导而来,非试出。
_RGBD_R_HAND_EE = np.array([
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
    [1.0,  0.0,  0.0],
])
# 自然静止 home:metric 下仅作 IK 参考;实际种子由 derive 按数据 bootstrap(见 _solve_arm_metric)。
_RGBD_NATURAL_HOME = np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634])

# ---------- kinect RGB-D(主路径):metric 固定 base 绝对映射 ----------
# 世界-上=-Z_world(相机侧滚1.1°确认)、机器人正对人 => R_base_world=[[0,1,0],[1,0,0],[0,0,-1]]。
# 缩放1.0(人手 0.49m 对角工作空间远小于 NERO 0.8m+ 可达)。full 位置可达实测 550/557。
# 与 anchored 的本质区别:R_base_world 是物理摆放定死的固定量,不再由首帧手腕朝向反推;
# q_home 退回自然静止姿、只当 IK 参考(种子按数据 bootstrap),不再进目标公式、不再一身二职。
NERO_INSPIRE_RGBD = RobotSpec(
    name="nero_inspire_rgbd",
    retarget_cfg=RETARGET_CONFIG,
    urdf_dir=RETARGET_URDF_DIR,
    hand_actuated=HAND_ACTUATED,
    arm_urdf=NERO_URDF,
    ee_frame="link7",
    R_hand_ee=_RGBD_R_HAND_EE,
    frame_mode="metric",
    R_base_world=np.array([
        [0.0, 1.0,  0.0],
        [1.0, 0.0,  0.0],
        [0.0, 0.0, -1.0],
    ]),
    p_base_anchor=(0.1, -0.1, 0.5),   # 人手质心映射到的 base 锚点(NERO 舒适可达区)
    metric_scale=1.0,
    arm_position_mode="absolute",     # 绝对米制位置;fixed=仅朝向(锁 anchor)
    q_home=_RGBD_NATURAL_HOME,
    arm_joint_names=ARM_JOINTS,
)

# ---------- kinect RGB-D(fallback):旧 anchored + 重摆 home,可达 555/557 ----------
# 保留作对照/回退。缺点:home 一身二职(既是种子又是映射锚),跨数据集需重摆,故非主路径。
NERO_INSPIRE_RGBD_ANCHORED = RobotSpec(
    name="nero_inspire_rgbd_anchored",
    retarget_cfg=RETARGET_CONFIG,
    urdf_dir=RETARGET_URDF_DIR,
    hand_actuated=HAND_ACTUATED,
    arm_urdf=NERO_URDF,
    ee_frame="link7",
    R_hand_ee=_RGBD_R_HAND_EE,
    frame_mode="anchored",
    arm_position_mode="fixed",        # 位置锁 home;relative 的 5cm 平移在重摆 home 上砸可达
    # 重摆到扫掠中点(配合几何 R_hand_ee 全段可达 555/557)。
    q_home=np.array([0.9341, 0.7620, 2.5581, 1.6231, -0.9934, 0.1864, 0.5745]),
    arm_joint_names=ARM_JOINTS,
)

# ============================================================================
# 平行夹爪本体(NERO 7-DoF + 二指平行夹爪)。臂配置逐字复用 inspire 对应规格(同一 IK),
# 只改:hand_mode=gripper、hand_actuated=1 标量、viz_urdf 指夹爪装配。
# state=8=[7 臂 + 1 夹爪开合];夹爪开合由拇指-食指捏合距离线性映射(见 derive_embodiment)。
# 装配偏移/行程为网格估计值(非标定),够可视化 + 占位,精确尺寸需夹爪 CAD。
# ============================================================================
GRIPPER_URDF = REPO / "assets/assembled/nero_gripper_right.urdf"

NERO_GRIPPER_RGB = RobotSpec(
    name="nero_gripper_rgb",
    retarget_cfg=RETARGET_CONFIG,  # 仍借它读 21 关键点,只用捏合距离
    urdf_dir=RETARGET_URDF_DIR,
    hand_actuated=["gripper_joint"],
    hand_mode="gripper",
    gripper_open_width_m=0.1,       # 开口 0→0.10m(SDK move_gripper_m 直接吃这个值)
    viz_urdf=GRIPPER_URDF,
    arm_urdf=NERO_URDF,
    ee_frame="link7",
    ee_frame_correction_rpy=(np.pi / 2.0, -np.pi / 2.0, 0.0),
    wrist_motion_basis_R=np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
    wrist_position_basis_rpy=(0.0, 0.0, -np.pi / 2.0),
    frame_mode="legacy",
    arm_position_mode="relative",
    q_home=np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]),
    arm_joint_names=ARM_JOINTS,
)

NERO_GRIPPER_RGBD = RobotSpec(
    name="nero_gripper_rgbd",
    retarget_cfg=RETARGET_CONFIG,
    urdf_dir=RETARGET_URDF_DIR,
    hand_actuated=["gripper_joint"],
    hand_mode="gripper",
    gripper_open_width_m=0.1,       # 开口 0→0.10m(SDK move_gripper_m 直接吃这个值)
    viz_urdf=GRIPPER_URDF,
    arm_urdf=NERO_URDF,
    ee_frame="link7",
    R_hand_ee=_RGBD_R_HAND_EE,
    frame_mode="metric",
    R_base_world=np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]),
    p_base_anchor=(0.1, -0.1, 0.5),
    metric_scale=1.0,
    arm_position_mode="absolute",
    q_home=_RGBD_NATURAL_HOME,
    arm_joint_names=ARM_JOINTS,
)

SPECS = {s.name: s for s in [NERO_INSPIRE_RGB, NERO_INSPIRE_RGBD, NERO_INSPIRE_RGBD_ANCHORED,
                             NERO_GRIPPER_RGB, NERO_GRIPPER_RGBD]}
SPECS["nero_inspire"] = NERO_INSPIRE_RGB  # 别名:旧命令默认走普通 RGB(相对,可达)


def get_spec(name: str) -> RobotSpec:
    if name not in SPECS:
        raise SystemExit(f"未知本体 '{name}';可选: {list(SPECS)}")
    return SPECS[name]


_AXIS_VEC = {"X": np.array([1.0, 0.0, 0.0]),
             "Y": np.array([0.0, 1.0, 0.0]),
             "Z": np.array([0.0, 0.0, 1.0])}


def axis_tokens_to_R_hand_ee(tokens) -> np.ndarray:
    """3 个 token(如 +Y -Z -X)= human X/Y/Z 分别落到 ee 的哪根轴 → R_hand_ee(3x3)。

    读法与 IK 打印/Rerun 表一致。内部:这 3 个向量当作 M 的列(human 轴在 ee 系里的方向),
    R_hand_ee = M.T(ee 轴在 hand 系里的方向)。非法(重复轴/左手系)直接报错。
    """
    cols = []
    for t in tokens:
        s = str(t).strip().upper()
        sign = -1.0 if s[0] == "-" else 1.0
        letter = s[-1]
        if letter not in _AXIS_VEC:
            raise SystemExit(f"--r-hand-ee 轴 '{t}' 非法,应为 X/Y/Z 前带可选 +/-")
        cols.append(sign * _AXIS_VEC[letter])
    M = np.stack(cols, axis=1)
    if abs(np.linalg.det(M) - 1.0) > 1e-6 or not np.allclose(M @ M.T, np.eye(3), atol=1e-6):
        raise SystemExit(f"--r-hand-ee {list(tokens)} 不是合法右手旋转(检查重复轴或左手系)")
    return M.T
