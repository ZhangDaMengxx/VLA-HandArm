"""本体层派生:canonical_ds(本体无关)+ 一个 RobotSpec → 这台机器人的 LeRobotDataset。

这是两层架构的「编译」步:规范层是母带,这里按某台机器人的 URDF/重定向配置把它投影成
该机器人的关节空间数据集。换机器人只换 --robot(见 robot_specs.py),规范层不动。

对每帧:
  手:kp(21,3) → ref = kp[task_i]-kp[origin_i] → dex-retarget → 12 关节 → 取 6 驱动。
  臂:wrist_pose → 稳定化(gate+出平面衰减+SavGol,见 wrist_stabilize)→ NeroKin IK(相对首帧,home 锚定;位置可相对跟随)→ SavGol。
  state/action(13)= [7 臂 + 6 手],action = 下一帧目标。ego 从 canonical_ds 取。

用法:
  python sim/derive_embodiment.py                       # 默认 nero_inspire
  python sim/derive_embodiment.py --robot nero_inspire --emit-traj   # 顺带出 robot_traj 供 replay_rerun
"""
import os
import sys
import argparse
import pickle
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from nero_kin import NeroKin
from wrist_stabilize import gate_outliers, attenuate_out_of_plane
from robot_specs import get_spec, axis_tokens_to_R_hand_ee
from schema import STATE_DIM

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from dex_retargeting.retargeting_config import RetargetingConfig

CANON_ROOT = REPO / "sim/out/canonical_ds"
CANON_REPO = "local/handdemo_canonical"
IMG = 256
TASK = "imitate the demonstrated hand motion"


def _axis_mapping_text(B: np.ndarray) -> str:
    names = ["X", "Y", "Z"]
    parts = []
    for src, v in zip(names, B.T):
        idx = int(np.argmax(np.abs(v)))
        sign = "+" if v[idx] >= 0 else "-"
        parts.append(f"human {src}->robot {sign}{names[idx]}")
    return ", ".join(parts)


def vec_to_pose(v: np.ndarray) -> np.ndarray:
    """(7,)[tx,ty,tz,qx,qy,qz,qw] → 4x4。"""
    T = np.eye(4)
    T[:3, 3] = v[:3]
    T[:3, :3] = Rot.from_quat(v[3:7]).as_matrix()
    return T


def load_canonical():
    """读 canonical_ds → (kps(N,21,3), wps(N,4,4), egos(N,H,W,3)uint8, fps)。"""
    ds = LeRobotDataset(CANON_REPO, root=str(CANON_ROOT))
    N = len(ds)
    kps = np.zeros((N, 21, 3), np.float64)
    wps = np.zeros((N, 4, 4), np.float64)
    egos = []
    for i in range(N):
        s = ds[i]
        kps[i] = np.asarray(s["observation.hand_keypoints"], np.float64).reshape(21, 3)
        wps[i] = vec_to_pose(np.asarray(s["observation.wrist_pose"], np.float64))
        ego = np.asarray(s["observation.images.ego"], np.float32)   # (3,H,W) 0..1
        egos.append((ego.transpose(1, 2, 0) * 255.0).clip(0, 255).astype(np.uint8))
    return kps, wps, egos, float(ds.fps)


def retarget_hand(kps, spec):
    """(N,21,3) → (N,12) inspire 关节 + names。关内部低通(平滑交给 SavGol,与 detect_wrist 一致)。"""
    RetargetingConfig.set_default_urdf_dir(str(spec.urdf_dir))
    rt = RetargetingConfig.load_from_file(str(spec.retarget_cfg),
                                          override={"low_pass_alpha": 1.0}).build()
    names = list(rt.optimizer.robot.dof_joint_names)
    idx = np.asarray(rt.optimizer.target_link_human_indices)
    origin_i, task_i = idx[0, :], idx[1, :]
    hand = np.zeros((len(kps), len(names)))
    for f in range(len(kps)):
        ref = kps[f][task_i, :] - kps[f][origin_i, :]
        hand[f] = rt.retarget(ref)
    return hand, names


# 平行夹爪:拇指-食指捏合距离(MANO 米制)线性映射到夹爪单指行程。
# 参考区间为解剖学典型值(估计,非标定):捏合 <=PINCH_CLOSED 记全闭,>=PINCH_OPEN 记全开。
PINCH_CLOSED_M = 0.03   # 拇食指尖接近相触
PINCH_OPEN_M = 0.09     # 手张开的典型捏合跨度


def pinch_to_gripper(kps, spec) -> np.ndarray:
    """(N,21,3) MANO 关键点 → (N,1) 夹爪开口宽度(m)。拇指尖(4)-食指尖(8)距离线性映射。

    0=闭合, spec.gripper_open_width_m=全开。量纲与官方 SDK 一致:pyAgxArm 的
    move_gripper_m(value=...) 吃的就是开口宽度(m),可直接下发,无需换算。
    用固定解剖参考区间(非per-segment),使"全开夹爪"对应"全开人手",跨 demo 语义一致。
    区间为估计值,精确需夹爪+人手标定。
    """
    d = np.linalg.norm(kps[:, 4, :] - kps[:, 8, :], axis=1)          # (N,) 捏合距离 m
    frac = np.clip((d - PINCH_CLOSED_M) / (PINCH_OPEN_M - PINCH_CLOSED_M), 0.0, 1.0)
    return (frac * float(spec.gripper_open_width_m)).reshape(-1, 1)  # (N,1) m 开口宽度


def _smooth_relative_positions(wps, spec) -> np.ndarray:
    """wrist_pose 位置 → 相对首帧的末端位移。默认用于解锁旧的固定腕部位置。"""
    ps = np.asarray(wps[:, :3, 3], dtype=np.float64)
    if len(ps) >= spec.savgol_win:
        ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
    position_basis = Rot.from_euler("xyz", spec.wrist_position_basis_rpy).as_matrix()
    dp = ((ps - ps[0]) @ position_basis.T) * float(spec.arm_position_gain)
    limit = float(spec.arm_position_limit_m)
    if limit > 0:
        norms = np.linalg.norm(dp, axis=1)
        mask = norms > limit
        dp[mask] *= (limit / norms[mask])[:, None]
    return dp


def anchored_base_world(R_world_hand0: np.ndarray, aR: np.ndarray,
                        R_hand_ee: np.ndarray) -> np.ndarray:
    """从首帧算出 R_base_world,使首帧手腕朝向精确映射到 home 末端朝向。

    约束: aR = R_base_world · R_world_hand0 · R_hand_ee  (首帧 D=I)
      =>   R_base_world = aR · R_hand_ee.T · R_world_hand0.T
    这一步把相机↔世界的固定旋转(如 kinect 外参的 ~118°)自动吸收进 R_base_world,
    所以下游不用再靠 motion_basis/compose 去凑坐标轴。
    """
    return aR @ R_hand_ee.T @ R_world_hand0.T


def _solve_arm_anchored(wps, Rs, spec, kin, aR, ap, arm_position_mode, n_clamped):
    """绝对锚定映射:R_base_ee[f] = R_base_world · R_world_hand[f] · R_hand_ee。

    位置也用同一个 R_base_world 把 world 位移投到 base 系,替掉 legacy 的 position_basis。
    """
    N = len(Rs)
    R_hand_ee = np.asarray(spec.R_hand_ee, dtype=np.float64).reshape(3, 3)
    R_base_world = anchored_base_world(Rs[0], aR, R_hand_ee)

    # 位置:world 位移 -> base 系(同一 R_base_world),gain + 限幅。fixed 则锁 home。
    if arm_position_mode == "fixed":
        dp = np.zeros((N, 3), dtype=np.float64)
    else:
        ps = np.asarray(wps[:, :3, 3], dtype=np.float64)
        if len(ps) >= spec.savgol_win:
            ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
        dp = ((ps - ps[0]) @ R_base_world.T) * float(spec.arm_position_gain)
        limit = float(spec.arm_position_limit_m)
        if limit > 0:
            norms = np.linalg.norm(dp, axis=1)
            mask = norms > limit
            dp[mask] *= (limit / norms[mask])[:, None]

    q_raw = np.zeros((N, 7))
    prev = spec.q_home.copy()
    ok = 0
    for f in range(N):
        Rt = R_base_world @ Rs[f] @ R_hand_ee
        Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = ap + dp[f]
        prev, good = kin.ik(Tt, prev, q_rest=spec.q_home, k_null=spec.k_null)
        ok += int(good)
        q_raw[f] = prev
    print(
        f"  臂 IK success {ok}/{N}  gate={spec.gate_deg}° clamp={n_clamped} oop-α={spec.oop_alpha} "
        f"frame=anchored pos={arm_position_mode} max|dp|={np.linalg.norm(dp, axis=1).max():.3f}m "
        f"R_hand_ee={np.array2string(R_hand_ee, precision=1, suppress_small=True)} "
        f"({_axis_mapping_text(R_hand_ee.T)})"
    )
    return savgol_filter(q_raw, spec.savgol_win, spec.savgol_poly, axis=0)


def _bootstrap_seed(kin, T_target, q_fallback, n_seeds: int = 24):
    """确定性多种子 IK:在关节量程上均匀撒 n_seeds 个种子,返回首个收敛解。

    metric 下 home 只是种子(不进目标公式),所以按代表帧找一个"扇面居中"的种子是合法的
    —— 它只决定收敛到哪个 IK 分支,不改目标位姿。全失败则回退 q_fallback。
    """
    best = None
    lo, hi = kin.lo, kin.hi
    for s in range(n_seeds):
        seed = lo + (hi - lo) * ((s + 0.5) / n_seeds)   # 确定性网格,不用随机
        q, ok = kin.ik(T_target, seed, q_rest=None, k_null=0.0)
        if ok:
            return q
        if best is None:
            best = q
    return best if best is not None else q_fallback.copy()


def _solve_arm_metric(wps, Rs, spec, kin, arm_position_mode, n_clamped):
    """度量映射:固定 R_base_world/anchor/scale 的绝对位姿,home 仅作 IK 种子(按数据 bootstrap)。

      R_base_ee[f] = R_base_world · R_world_hand[f] · R_hand_ee     (朝向,固定外参)
      p_base[f]    = p_anchor + scale · R_base_world · (p_world[f] - centroid)   (位置,质心居中)
    与 anchored 的本质区别:R_base_world 是物理摆放定死的常量,不由首帧反推;q_home 不进公式。
    centroid 每段按数据算(务实取舍:保运动形状+米制尺度,不保跨数据集绝对世界位;
    真绝对位需 robot-camera 外参标定)。
    """
    N = len(Rs)
    R_hand_ee = np.asarray(spec.R_hand_ee, dtype=np.float64).reshape(3, 3)
    R_base_world = np.asarray(spec.R_base_world, dtype=np.float64).reshape(3, 3)
    anchor = np.asarray(spec.p_base_anchor, dtype=np.float64).reshape(3)
    scale = float(spec.metric_scale)

    # 位置:世界米制手位 -> base 系,质心居中到 anchor;fixed 则锁 anchor(仅朝向)。
    if arm_position_mode == "fixed":
        dp = np.zeros((N, 3), dtype=np.float64)
    else:
        ps = np.asarray(wps[:, :3, 3], dtype=np.float64)
        if len(ps) >= spec.savgol_win:
            ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
        dp = scale * ((ps - ps.mean(0)) @ R_base_world.T)
    targets = anchor + dp

    # 种子 bootstrap:对代表帧(中点朝向 @ anchor)多种子求"扇面居中"解,再顺序 warm-start。
    mid = N // 2
    T_mid = np.eye(4)
    T_mid[:3, :3] = R_base_world @ Rs[mid] @ R_hand_ee
    T_mid[:3, 3] = anchor
    q_seed = _bootstrap_seed(kin, T_mid, spec.q_home)

    q_raw = np.zeros((N, 7))
    prev = q_seed.copy()
    ok = 0
    for f in range(N):
        Rt = R_base_world @ Rs[f] @ R_hand_ee
        Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = targets[f]
        prev, good = kin.ik(Tt, prev, q_rest=q_seed, k_null=spec.k_null)
        ok += int(good)
        q_raw[f] = prev
    print(
        f"  臂 IK success {ok}/{N}  gate={spec.gate_deg}° clamp={n_clamped} oop-α={spec.oop_alpha} "
        f"frame=metric pos={arm_position_mode} scale={scale} max|dp|={np.linalg.norm(dp, axis=1).max():.3f}m "
        f"R_base_world={np.array2string(R_base_world, precision=0, suppress_small=True)} "
        f"R_hand_ee=({_axis_mapping_text(R_hand_ee.T)})"
    )
    return savgol_filter(q_raw, spec.savgol_win, spec.savgol_poly, axis=0)


def solve_arm(wps, spec, arm_position_mode: str | None = None,
              wrist_rotation_compose: str | None = None):
    """(N,4,4) 手腕位姿 → (N,7) 臂关节。稳定化 + IK(相对首帧,home 锚定)。"""
    N = len(wps)
    arm_position_mode = arm_position_mode or spec.arm_position_mode
    wrist_rotation_compose = wrist_rotation_compose or spec.wrist_rotation_compose
    quats = Rot.from_matrix(wps[:, :3, :3]).as_quat()
    for i in range(1, N):
        if np.dot(quats[i - 1], quats[i]) < 0:
            quats[i] = -quats[i]
    quats = gate_outliers(quats, spec.gate_deg)
    n_clamped = getattr(gate_outliers, "last_clamped", 0)
    quats_s = savgol_filter(quats, spec.savgol_win, spec.savgol_poly, axis=0)
    quats_s /= np.linalg.norm(quats_s, axis=1, keepdims=True)
    Rs = Rot.from_quat(quats_s).as_matrix()
    Rs = attenuate_out_of_plane(Rs, spec.oop_alpha, ref=0)

    kin = NeroKin(spec.arm_urdf, ee_frame=spec.ee_frame)
    anchor = kin.fk(spec.q_home)
    aR, ap = anchor[:3, :3], anchor[:3, 3]

    if spec.frame_mode == "metric":
        return _solve_arm_metric(wps, Rs, spec, kin, arm_position_mode, n_clamped)
    if spec.frame_mode == "anchored":
        return _solve_arm_anchored(wps, Rs, spec, kin, aR, ap, arm_position_mode, n_clamped)

    ee_fix = Rot.from_euler("xyz", spec.ee_frame_correction_rpy).as_matrix()
    motion_basis = np.asarray(spec.wrist_motion_basis_R, dtype=np.float64).reshape(3, 3)
    R0 = Rs[0]
    if arm_position_mode == "fixed":
        dp = np.zeros((N, 3), dtype=np.float64)
    elif arm_position_mode == "relative":
        dp = _smooth_relative_positions(wps, spec)
    else:
        raise SystemExit(f"未知 arm_position_mode '{arm_position_mode}', 可选 fixed/relative")
    q_raw = np.zeros((N, 7))
    prev = spec.q_home.copy()
    ok = 0
    for f in range(N):
        if wrist_rotation_compose == "left":
            dR_human = Rs[f] @ R0.T          # world/camera-axis delta
            dR_robot = motion_basis @ dR_human @ motion_basis.T
            Rt = dR_robot @ aR @ ee_fix
        elif wrist_rotation_compose == "right":
            dR_human = R0.T @ Rs[f]          # body/wrist-local-axis delta
            dR_robot = motion_basis @ dR_human @ motion_basis.T
            Rt = aR @ dR_robot @ ee_fix
        else:
            raise SystemExit(f"未知 wrist_rotation_compose '{wrist_rotation_compose}', 可选 left/right")
        Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = ap + dp[f]
        prev, good = kin.ik(Tt, prev, q_rest=spec.q_home, k_null=spec.k_null)
        ok += int(good)
        q_raw[f] = prev
    print(
        f"  臂 IK success {ok}/{N}  gate={spec.gate_deg}° clamp={n_clamped} oop-α={spec.oop_alpha} "
        f"pos={arm_position_mode} max|dp|={np.linalg.norm(dp, axis=1).max():.3f}m "
        f"rot_compose={wrist_rotation_compose} "
        f"motion_basis_R={np.array2string(motion_basis, precision=1, suppress_small=True)} "
        f"({_axis_mapping_text(motion_basis)}) "
        f"position_basis={tuple(round(float(x), 4) for x in spec.wrist_position_basis_rpy)}"
    )
    return savgol_filter(q_raw, spec.savgol_win, spec.savgol_poly, axis=0)


def main():
    ap = argparse.ArgumentParser(description="canonical_ds + RobotSpec → 本体 LeRobotDataset")
    ap.add_argument("--robot", default="nero_inspire", help="本体名(见 robot_specs.SPECS)")
    ap.add_argument("--emit-traj", action="store_true", help="顺带写 robot_traj_<robot>.pkl 供 replay_rerun")
    ap.add_argument("--arm-position-mode", choices=["relative", "fixed", "absolute"], default=None,
                    help="relative=legacy 相对首帧位移; fixed=锁 home/anchor(仅朝向); absolute=metric 绝对米制位置")
    ap.add_argument("--arm-position-gain", type=float, default=None,
                    help="相对腕部平移增益;默认用 RobotSpec")
    ap.add_argument("--arm-position-limit", type=float, default=None,
                    help="相对 home 的最大末端平移半径(米);默认用 RobotSpec")
    ap.add_argument("--wrist-motion-basis-rpy", type=float, nargs=3, default=None,
                    metavar=("ROLL", "PITCH", "YAW"),
                    help="动态旋转坐标基变换 rpy(弧度);默认用 RobotSpec")
    ap.add_argument("--wrist-position-basis-rpy", type=float, nargs=3, default=None,
                    metavar=("ROLL", "PITCH", "YAW"),
                    help="相对 wrist 平移坐标基变换 rpy(弧度);默认用 RobotSpec")
    ap.add_argument("--wrist-rotation-compose", choices=["left", "right"], default=None,
                    help="left=dR_robot @ home @ ee_fix; right=home @ dR_robot @ ee_fix")
    ap.add_argument("--r-hand-ee", default=None, metavar="HX,HY,HZ",
                    help="anchored 装配轴映射,逗号分隔:human X/Y/Z 分别落到 ee 哪根轴,如 -Y,-Z,+X(用 = 传避免负号被当选项:--r-hand-ee=-Y,-Z,+X)")
    args = ap.parse_args()
    spec = get_spec(args.robot)
    if args.r_hand_ee is not None:
        spec.R_hand_ee = axis_tokens_to_R_hand_ee(args.r_hand_ee.split(","))
    if args.arm_position_gain is not None:
        spec.arm_position_gain = args.arm_position_gain
    if args.arm_position_limit is not None:
        spec.arm_position_limit_m = args.arm_position_limit
    if args.wrist_motion_basis_rpy is not None:
        spec.wrist_motion_basis_R = Rot.from_euler("xyz", args.wrist_motion_basis_rpy).as_matrix()
    if args.wrist_position_basis_rpy is not None:
        spec.wrist_position_basis_rpy = tuple(args.wrist_position_basis_rpy)
    print(f"派生本体: {spec.name}")

    kps, wps, egos, fps = load_canonical()
    N = len(kps)
    print(f"canonical: {N} 帧 @ {fps}fps")

    q_arm = solve_arm(wps, spec, args.arm_position_mode, args.wrist_rotation_compose)

    # 手部:dex 走 retarget→取驱动子集;gripper 走捏合距离→1 标量。
    # hand_full/hand_full_names 存进 traj 供 replay 驱动可视化 URDF(dex=12 关节, gripper=1 行程);
    # hand_state 是进 state/action 的列(dex=6 驱动, gripper=1 行程)。
    if spec.hand_mode == "gripper":
        grip = pinch_to_gripper(kps, spec)                                  # (N,1) m 开口宽度
        grip = savgol_filter(grip, spec.savgol_win, spec.savgol_poly, axis=0)
        grip = np.clip(grip, 0.0, float(spec.gripper_open_width_m))
        hand_full = grip
        hand_full_names = ["gripper_joint"]
        hand_state = grip
        print(f"  夹爪开口宽度(m): min={grip.min():.4f} max={grip.max():.4f} "
              f"full_open={spec.gripper_open_width_m} (可直接喂 SDK move_gripper_m)")
    else:
        hand12, hand_names = retarget_hand(kps, spec)
        hand12 = np.clip(savgol_filter(hand12, spec.savgol_win, spec.savgol_poly, axis=0), 0.0, 1.55)
        act_idx = [hand_names.index(n) for n in spec.hand_actuated]
        hand_full = hand12
        hand_full_names = hand_names
        hand_state = hand12[:, act_idx]

    names_state = spec.arm_joint_names + spec.hand_actuated                 # 动态维度,不写死 13
    dim = len(names_state)
    state = np.concatenate([q_arm, hand_state], axis=1).astype(np.float32)  # (N, dim)
    action = np.concatenate([state[1:], state[-1:]], axis=0).astype(np.float32)

    if args.emit_traj:
        traj = REPO / f"sim/out/robot_traj_{spec.name}.pkl"
        with open(traj, "wb") as f:
            pickle.dump(dict(arm=q_arm, hand=hand_full, hand_joint_names=hand_full_names,
                             arm_joint_names=spec.arm_joint_names), f)
        # 顺带存一份可移植 npz:本环境是 numpy 2.x,ROS2 侧是 numpy 1.x 读不了 pkl,
        # 但 .npy/.npz 格式跨版本稳定。ROS2 的 replay_traj.py 优先读同名 npz。
        np.savez(traj.with_suffix(".npz"),
                 arm=q_arm.astype(np.float64), hand=np.asarray(hand_full, dtype=np.float64),
                 arm_joint_names=np.asarray(spec.arm_joint_names),
                 hand_joint_names=np.asarray(hand_full_names))
        print(f"  emit {traj}  (+ {traj.with_suffix('.npz').name})")

    import shutil
    if spec.out_root.exists():
        shutil.rmtree(spec.out_root)
    features = {
        "observation.state": {"dtype": "float32", "shape": (dim,), "names": names_state},
        "action": {"dtype": "float32", "shape": (dim,), "names": names_state},
        "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3),
                                   "names": ["height", "width", "channel"]},
    }
    ds = LeRobotDataset.create(repo_id=spec.repo_id, fps=int(round(fps)), features=features,
                               root=str(spec.out_root), robot_type=spec.name,
                               use_videos=True, metadata_buffer_size=1)
    for f in range(N):
        ds.add_frame({"observation.state": state[f], "action": action[f],
                      "observation.images.ego": egos[f], "task": TASK})
    ds.save_episode()
    print(f"wrote {N} frames, 1 episode -> {spec.out_root}")


if __name__ == "__main__":
    main()
