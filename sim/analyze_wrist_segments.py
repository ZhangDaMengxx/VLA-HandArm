"""Analyze wrist motion axes for labeled frame segments.

The goal is to replace visual guessing with numbers. Given frame ranges such as
"flip=410:425" and "swing=180:190", this script prints the human wrist motion
axis in each segment and how candidate wrist_motion_basis matrices map those
axes into robot axes.

It reads the already-built canonical dataset. Rebuild canonical first when
switching videos:
  python sim/build_canonical.py --video data/hand3.mp4

Examples:
  python sim/analyze_wrist_segments.py
  python sim/analyze_wrist_segments.py --segment flip=410:425 --segment swing=180:190
  python sim/analyze_wrist_segments.py --basis 3.14159265359 0 0
  python sim/analyze_wrist_segments.py --flip-target +Z --swing-target +X
"""
from __future__ import annotations

import argparse
from itertools import permutations, product
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from robot_specs import get_spec

REPO = Path(__file__).resolve().parents[1]
CANON_ROOT = REPO / "sim/out/canonical_ds"
AXES = {
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]),
    "-Y": np.array([0.0, -1.0, 0.0]),
    "+Z": np.array([0.0, 0.0, 1.0]),
    "-Z": np.array([0.0, 0.0, -1.0]),
}


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


def fmt_vec(v: np.ndarray) -> str:
    return "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"


def axis_label(v: np.ndarray) -> tuple[str, float]:
    dots = {name: float(np.dot(v, axis)) for name, axis in AXES.items()}
    best = max(dots, key=dots.get)
    return best, dots[best]


def parse_segment(text: str) -> tuple[str, int, int]:
    name, sep, span = text.partition("=")
    if not sep:
        raise argparse.ArgumentTypeError("segment 格式应为 name=start:end")
    a, sep, b = span.partition(":")
    if not sep:
        raise argparse.ArgumentTypeError("segment 格式应为 name=start:end")
    start, end = int(a), int(b)
    if end <= start:
        raise argparse.ArgumentTypeError("segment end 必须大于 start")
    return name, start, end


def segment_motion(wps: np.ndarray, start: int, end: int) -> dict:
    if start < 0 or end >= len(wps):
        raise SystemExit(f"segment {start}:{end} 超出帧范围 0:{len(wps)-1}")
    world_rotvecs = []
    body_rotvecs = []
    local_angles = []
    for f in range(start + 1, end + 1):
        Rp = wps[f - 1, :3, :3]
        Rc = wps[f, :3, :3]
        dR_world = Rc @ Rp.T
        dR_body = Rp.T @ Rc
        rv = Rot.from_matrix(dR_world).as_rotvec()
        world_rotvecs.append(rv)
        body_rotvecs.append(Rot.from_matrix(dR_body).as_rotvec())
        local_angles.append(float(np.linalg.norm(rv)))
    world_rotvecs = np.asarray(world_rotvecs)
    body_rotvecs = np.asarray(body_rotvecs)
    local_angles = np.asarray(local_angles)
    world_sum_rv = world_rotvecs.sum(axis=0)
    world_mean_rv = world_rotvecs.mean(axis=0)
    world_net_R = wps[end, :3, :3] @ wps[start, :3, :3].T
    world_net_rv = Rot.from_matrix(world_net_R).as_rotvec()
    body_sum_rv = body_rotvecs.sum(axis=0)
    body_mean_rv = body_rotvecs.mean(axis=0)
    body_net_R = wps[start, :3, :3].T @ wps[end, :3, :3]
    body_net_rv = Rot.from_matrix(body_net_R).as_rotvec()

    def _axis(rv: np.ndarray) -> tuple[float, np.ndarray]:
        angle = float(np.linalg.norm(rv))
        return np.degrees(angle), rv / max(angle, 1e-12)

    return {
        "start": start,
        "end": end,
        "n_steps": end - start,
        "world_sum": _axis(world_sum_rv),
        "world_mean": _axis(world_mean_rv),
        "world_net": _axis(world_net_rv),
        "body_sum": _axis(body_sum_rv),
        "body_mean": _axis(body_mean_rv),
        "body_net": _axis(body_net_rv),
        "total_local_deg": float(np.degrees(local_angles.sum())),
        "max_local_deg": float(np.degrees(local_angles.max())),
        "mean_local_deg": float(np.degrees(local_angles.mean())),
    }


def signed_permutation_rotations() -> list[tuple[str, np.ndarray]]:
    out = []
    base = np.eye(3)
    for perm in permutations(range(3)):
        P = base[:, perm]
        for signs in product([-1.0, 1.0], repeat=3):
            B = P @ np.diag(signs)
            if round(np.linalg.det(B)) == 1:
                cols = []
                for human_axis, col in zip("XYZ", B.T):
                    lab, _ = axis_label(col)
                    cols.append(f"{human_axis}->{lab}")
                out.append((" ".join(cols), B))
    return out


def candidate_basis(spec_basis_R: np.ndarray, override: list[float] | None):
    if override is not None:
        B = Rot.from_euler("xyz", override).as_matrix()
        return [("given", B)]
    bases = [("spec", np.asarray(spec_basis_R, dtype=np.float64).reshape(3, 3))]
    seen = {tuple(np.round(bases[0][1].reshape(-1), 8))}
    for label, B in signed_permutation_rotations():
        key = tuple(np.round(B.reshape(-1), 8))
        if key not in seen:
            bases.append((label, B))
            seen.add(key)
    return bases


def rpy_deg(B: np.ndarray) -> str:
    try:
        rpy = Rot.from_matrix(B).as_euler("xyz", degrees=True)
        return "(" + ", ".join(f"{x:+.0f}" for x in rpy) + ")"
    except Exception:
        return "(gimbal)"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="nero_inspire")
    ap.add_argument("--canonical", default=str(CANON_ROOT))
    ap.add_argument("--segment", action="append", type=parse_segment,
                    default=[("flip", 410, 425), ("swing", 180, 190)],
                    help="动作帧段 name=start:end,可重复。默认 flip=410:425 swing=180:190")
    ap.add_argument("--basis", type=float, nargs=3, default=None,
                    metavar=("ROLL", "PITCH", "YAW"),
                    help="只分析这一组 RPY(弧度)转换出的 basis 矩阵;不传则用 RobotSpec.wrist_motion_basis_R")
    ap.add_argument("--top", type=int, default=12, help="打印候选数")
    ap.add_argument("--flip-target", choices=sorted(AXES), default=None,
                    help="如果给定,按 flip 映射到该 robot 轴打分")
    ap.add_argument("--swing-target", choices=sorted(AXES), default=None,
                    help="如果给定,按 swing 映射到该 robot 轴打分")
    args = ap.parse_args()

    spec = get_spec(args.robot)
    wps = load_wrist_poses(Path(args.canonical))
    print(f"canonical frames: {len(wps)}")

    motions = {}
    for name, start, end in args.segment:
        m = segment_motion(wps, start, end)
        motions[name] = m
        print("=" * 88)
        print(f"segment {name}: frames {start}:{end}  steps={m['n_steps']}")
        for key in ("world_sum", "world_mean", "world_net", "body_sum", "body_mean", "body_net"):
            angle, axis = m[key]
            lab, conf = axis_label(axis)
            print(f"  human {key:<10} angle={angle:8.3f} deg  axis={fmt_vec(axis)}  dominant={lab} ({conf:+.3f})")
        print(f"  local angle total={m['total_local_deg']:.3f} deg  mean/step={m['mean_local_deg']:.3f} deg  max/step={m['max_local_deg']:.3f} deg")

    print("=" * 88)
    print("candidate wrist_motion_basis mappings")
    targets = {}
    if args.flip_target and "flip" in motions:
        targets["flip"] = AXES[args.flip_target]
    if args.swing_target and "swing" in motions:
        targets["swing"] = AXES[args.swing_target]

    rows = []
    for label, B in candidate_basis(spec.wrist_motion_basis_R, args.basis):
        score = 0.0
        mapped = {}
        for name, m in motions.items():
            _, human_axis = m["body_sum"]
            robot_axis = B @ human_axis
            lab, conf = axis_label(robot_axis)
            mapped[name] = (robot_axis, lab, conf)
            if name in targets:
                score += float(np.dot(robot_axis, targets[name]))
        rows.append((score, label, B, mapped))

    if targets:
        rows.sort(key=lambda x: x[0], reverse=True)

    for i, (score, label, B, mapped) in enumerate(rows[:args.top], 1):
        print("-" * 88)
        print(f"#{i:02d} score={score:+.3f}  rpy_xyz_deg={rpy_deg(B)}  {label}")
        print("B =")
        print(np.array2string(B, precision=0, suppress_small=True))
        for name, (axis, lab, conf) in mapped.items():
            print(f"  {name:<10} human_axis -> robot_axis={fmt_vec(axis)}  dominant={lab} ({conf:+.3f})")


if __name__ == "__main__":
    main()
