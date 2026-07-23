"""本体层派生:canonical_ds(本体无关)+ 一个 RobotSpec → 这台机器人的 LeRobotDataset。

这是两层架构的「编译」步:规范层是母带,这里按某台机器人的 URDF/重定向配置把它投影成
该机器人的关节空间数据集。换机器人只换 --robot(见 robot_specs.py),规范层不动。

对每帧:
  手:kp(21,3) → ref = kp[task_i]-kp[origin_i] → dex-retarget → 12 关节 → 取 6 驱动。
  臂:wrist_pose → 稳定化(gate+出平面衰减+SavGol,见 wrist_stabilize)→ NeroKin IK(相对首帧,home 锚定)→ SavGol。
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
from robot_specs import get_spec
from schema import STATE_DIM

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from dex_retargeting.retargeting_config import RetargetingConfig

CANON_ROOT = REPO / "sim/out/canonical_ds"
CANON_REPO = "local/handdemo_canonical"
IMG = 256
TASK = "imitate the demonstrated hand motion"


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


def solve_arm(wps, spec):
    """(N,4,4) 手腕位姿 → (N,7) 臂关节。稳定化 + IK(相对首帧,home 锚定)。"""
    N = len(wps)
    quats = Rot.from_matrix(wps[:, :3, :3]).as_quat()
    for i in range(1, N):
        if np.dot(quats[i - 1], quats[i]) < 0:
            quats[i] = -quats[i]
    quats = gate_outliers(quats, spec.gate_deg)
    quats_s = savgol_filter(quats, spec.savgol_win, spec.savgol_poly, axis=0)
    quats_s /= np.linalg.norm(quats_s, axis=1, keepdims=True)
    Rs = Rot.from_quat(quats_s).as_matrix()
    Rs = attenuate_out_of_plane(Rs, spec.oop_alpha, ref=0)

    kin = NeroKin(spec.arm_urdf, ee_frame=spec.ee_frame)
    anchor = kin.fk(spec.q_home)
    aR, ap = anchor[:3, :3], anchor[:3, 3]
    ee_fix = Rot.from_euler("xyz", spec.ee_frame_correction_rpy).as_matrix()
    R0 = Rs[0]
    q_raw = np.zeros((N, 7))
    prev = spec.q_home.copy()
    ok = 0
    for f in range(N):
        Rt = (Rs[f] @ R0.T) @ aR @ ee_fix
        Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = ap
        prev, good = kin.ik(Tt, prev, q_rest=spec.q_home, k_null=spec.k_null)
        ok += int(good)
        q_raw[f] = prev
    print(f"  臂 IK success {ok}/{N}  gate={spec.gate_deg}° oop-α={spec.oop_alpha}")
    return savgol_filter(q_raw, spec.savgol_win, spec.savgol_poly, axis=0)


def main():
    ap = argparse.ArgumentParser(description="canonical_ds + RobotSpec → 本体 LeRobotDataset")
    ap.add_argument("--robot", default="nero_inspire", help="本体名(见 robot_specs.SPECS)")
    ap.add_argument("--emit-traj", action="store_true", help="顺带写 robot_traj_<robot>.pkl 供 replay_rerun")
    args = ap.parse_args()
    spec = get_spec(args.robot)
    print(f"派生本体: {spec.name}")

    kps, wps, egos, fps = load_canonical()
    N = len(kps)
    print(f"canonical: {N} 帧 @ {fps}fps")

    hand12, hand_names = retarget_hand(kps, spec)
    hand12 = np.clip(savgol_filter(hand12, spec.savgol_win, spec.savgol_poly, axis=0), 0.0, 1.55)
    q_arm = solve_arm(wps, spec)

    act_idx = [hand_names.index(n) for n in spec.hand_actuated]
    state = np.concatenate([q_arm, hand12[:, act_idx]], axis=1).astype(np.float32)   # (N,13)
    action = np.concatenate([state[1:], state[-1:]], axis=0).astype(np.float32)

    if args.emit_traj:
        traj = REPO / f"sim/out/robot_traj_{spec.name}.pkl"
        with open(traj, "wb") as f:
            pickle.dump(dict(arm=q_arm, hand=hand12, hand_joint_names=hand_names,
                             arm_joint_names=spec.arm_joint_names), f)
        # 顺带存一份可移植 npz:本环境是 numpy 2.x,ROS2 侧是 numpy 1.x 读不了 pkl,
        # 但 .npy/.npz 格式跨版本稳定。ROS2 的 replay_traj.py 优先读同名 npz。
        np.savez(traj.with_suffix(".npz"),
                 arm=q_arm.astype(np.float64), hand=hand12.astype(np.float64),
                 arm_joint_names=np.asarray(spec.arm_joint_names),
                 hand_joint_names=np.asarray(hand_names))
        print(f"  emit {traj}  (+ {traj.with_suffix('.npz').name})")

    import shutil
    if spec.out_root.exists():
        shutil.rmtree(spec.out_root)
    names13 = spec.arm_joint_names + spec.hand_actuated
    features = {
        "observation.state": {"dtype": "float32", "shape": (STATE_DIM,), "names": names13},
        "action": {"dtype": "float32", "shape": (STATE_DIM,), "names": names13},
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
