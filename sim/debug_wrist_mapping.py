"""Print wrist rotation axes and compare dynamic rotation composition.

This script does not run IK. It isolates the coordinate/matrix part of the
pipeline so that a wrong wrist axis is not confused with an IK solution.

Examples:
  python sim/debug_wrist_mapping.py
  python sim/debug_wrist_mapping.py --frame 120
  python sim/debug_wrist_mapping.py --basis 0 -1.57079632679 0 --frame 120
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from robot_specs import get_spec
from nero_kin import NeroKin

REPO = Path(__file__).resolve().parents[1]
CANON_ROOT = REPO / "sim/out/canonical_ds"


def pose_vec_to_mat(v: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = v[:3]
    T[:3, :3] = Rot.from_quat(v[3:7]).as_matrix()
    return T


def load_wrist_poses(root: Path) -> np.ndarray:
    files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
    if not files:
        raise SystemExit(f"找不到 canonical parquet: {root}")
    df = pd.concat([pd.read_parquet(p) for p in files], ignore_index=True)
    if "frame_index" in df.columns:
        df = df.sort_values("frame_index")
    values = np.stack(df["observation.wrist_pose"].to_numpy()).astype(np.float64)
    return np.stack([pose_vec_to_mat(v.reshape(-1)) for v in values])


def axis_angle(R: np.ndarray) -> tuple[float, np.ndarray]:
    rv = Rot.from_matrix(R).as_rotvec()
    angle = float(np.linalg.norm(rv))
    axis = rv / max(angle, 1e-12)
    return np.degrees(angle), axis


def fmt_vec(v: np.ndarray) -> str:
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def describe(name: str, R: np.ndarray) -> None:
    angle, axis = axis_angle(R)
    print(f"{name:<24} angle={angle:8.3f} deg  axis={fmt_vec(axis)}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", default="nero_inspire")
    ap.add_argument("--frame", type=int, default=None, help="只打印指定帧")
    ap.add_argument("--top-k", type=int, default=8,
                    help="未指定 --frame 时,打印相对首帧旋转最大的 K 帧")
    ap.add_argument("--basis", type=float, nargs=3, default=None,
                    metavar=("ROLL", "PITCH", "YAW"),
                    help="临时用 RPY(弧度)覆盖 RobotSpec.wrist_motion_basis_R")
    ap.add_argument("--canonical", default=str(CANON_ROOT))
    args = ap.parse_args()

    spec = get_spec(args.robot)
    wps = load_wrist_poses(Path(args.canonical))
    if len(wps) < 2:
        raise SystemExit("canonical 帧数不足")

    if args.basis is not None:
        basis_label = f"rpy xyz(rad): {tuple(args.basis)}"
        B = Rot.from_euler("xyz", args.basis).as_matrix()
    else:
        basis_label = "RobotSpec.wrist_motion_basis_R"
        B = np.asarray(spec.wrist_motion_basis_R, dtype=np.float64).reshape(3, 3)

    kin = NeroKin(spec.arm_urdf, ee_frame=spec.ee_frame)
    home = kin.fk(spec.q_home)
    aR = home[:3, :3]
    ee_fix = Rot.from_euler("xyz", spec.ee_frame_correction_rpy).as_matrix()

    R0 = wps[0, :3, :3]
    dRs_human_world = np.stack([T[:3, :3] @ R0.T for T in wps])
    dRs_human_body = np.stack([R0.T @ T[:3, :3] for T in wps])
    dRs_robot_world = np.stack([B @ R @ B.T for R in dRs_human_world])
    dRs_robot_body = np.stack([B @ R @ B.T for R in dRs_human_body])

    print(f"canonical frames: {len(wps)}")
    print(f"basis: {basis_label}")
    print("B =")
    print(np.array2string(B, precision=4, suppress_small=True))
    print("axis mapping:")
    for name, axis in zip(("human X", "human Y", "human Z"), np.eye(3)):
        print(f"  {name} -> robot {fmt_vec(B @ axis)}")
    print()

    if args.frame is not None:
        frames = [args.frame]
    else:
        scores = np.asarray([axis_angle(R)[0] for R in dRs_human_world])
        frames = np.argsort(scores)[-args.top_k:][::-1].tolist()

    for f in frames:
        if f < 0 or f >= len(wps):
            print(f"skip frame {f}: out of range")
            continue
        print("=" * 78)
        print(f"frame {f}")
        if f > 0:
            local_human = wps[f, :3, :3] @ wps[f - 1, :3, :3].T
            local_human_body = wps[f - 1, :3, :3].T @ wps[f, :3, :3]
            local_robot = B @ local_human @ B.T
            local_robot_body = B @ local_human_body @ B.T
            describe("human local dR", local_human)
            describe("mapped local dR", local_robot)
            describe("human body local dR", local_human_body)
            describe("mapped body local dR", local_robot_body)
        describe("human world dR", dRs_human_world[f])
        describe("mapped world dR", dRs_robot_world[f])
        describe("human body dR", dRs_human_body[f])
        describe("mapped body dR", dRs_robot_body[f])
        target_left = dRs_robot_world[f] @ aR @ ee_fix
        target_right = aR @ dRs_robot_body[f] @ ee_fix
        describe("target left dR", target_left @ (aR @ ee_fix).T)
        describe("target right dR", (aR @ ee_fix).T @ target_right)
        print("human world dR =")
        print(np.array2string(dRs_human_world[f], precision=4, suppress_small=True))
        print("mapped world dR =")
        print(np.array2string(dRs_robot_world[f], precision=4, suppress_small=True))
        print("human body dR =")
        print(np.array2string(dRs_human_body[f], precision=4, suppress_small=True))
        print("mapped body dR =")
        print(np.array2string(dRs_robot_body[f], precision=4, suppress_small=True))
        print("target left R = dR_robot @ aR @ ee_fix")
        print(np.array2string(target_left, precision=4, suppress_small=True))
        print("target right R = aR @ dR_robot @ ee_fix")
        print(np.array2string(target_right, precision=4, suppress_small=True))


if __name__ == "__main__":
    main()
