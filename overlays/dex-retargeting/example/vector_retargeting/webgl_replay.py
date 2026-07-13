"""
CPU-only / GPU-free replay of a retargeting trajectory via MeshCat (WebGL in the browser).

This is an alternative to `render_robot_hand.py`, which uses SAPIEN (Vulkan) and needs a
GPU. Here Pinocchio (already a dependency of dex-retargeting) loads the same URDF and drives
a MeshCat visualizer. All rendering happens client-side in the browser via three.js/WebGL, so
no local Vulkan/OpenGL device is required.

Usage:
    python3 webgl_replay.py --pickle-path data/panda_joints.pkl
    python3 webgl_replay.py --pickle-path data/panda_joints.pkl --fps 30 --loops 0
    python3 webgl_replay.py --pickle-path data/panda_joints.pkl --selftest   # load + 1 frame, then exit

Then open the printed URL in a browser (on Windows/WSL: http://localhost:7000/static/).
"""
import argparse
import os
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

from dex_retargeting.retargeting_config import RetargetingConfig


def build_robot(pickle_path: str):
    pickle_data = np.load(pickle_path, allow_pickle=True)
    meta_data, data = pickle_data["meta_data"], pickle_data["data"]

    # Resolve the URDF exactly like render_robot_hand.py: the config stores a path relative
    # to assets/robots/hands, made absolute once the default urdf dir is set.
    robot_dir = (
        Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
    )
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    config = RetargetingConfig.load_from_file(meta_data["config_path"])

    urdf_path = Path(config.urdf_path)
    urdf_dir = str(urdf_path.parent)
    # Relative mesh paths (e.g. "meshes/visual/hand.glb") resolve against package_dirs / cwd.
    os.chdir(urdf_dir)

    # Floating base so a stored per-frame wrist orientation can rotate/flip the whole
    # hand. Finger-only retargeting pins the base; a free-flyer root re-adds the 6-DOF
    # root (we drive orientation only, translation stays 0).
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    visual_model = pin.buildGeomFromUrdf(
        model, str(urdf_path), pin.GeometryType.VISUAL, package_dirs=[urdf_dir]
    )
    collision_model = pin.buildGeomFromUrdf(
        model, str(urdf_path), pin.GeometryType.COLLISION, package_dirs=[urdf_dir]
    )
    # Optional per-frame base rotation (present in pkls from the updated detect script).
    try:
        wrist_rot = np.asarray(pickle_data["wrist_rot"])
    except (KeyError, IndexError, ValueError):
        wrist_rot = None
    return model, collision_model, visual_model, meta_data, data, wrist_rot


def make_qmap(model, joint_names):
    """Map each retargeting joint name to its (q index, nq) in the Pinocchio config vector."""
    qmap = []
    for name in joint_names:
        if not model.existJointName(name):
            print(f"  [warn] joint '{name}' not in Pinocchio model, skipping")
            qmap.append(None)
            continue
        jid = model.getJointId(name)
        qmap.append((model.idx_qs[jid], model.nqs[jid]))
    return qmap


def set_q(q, qmap, qpos):
    for (slot, val) in zip(qmap, qpos):
        if slot is None:
            continue
        qi, nq = slot
        if nq == 1:  # revolute / prismatic
            q[qi] = val
        elif nq == 2:  # continuous joint -> (cos, sin)
            q[qi] = np.cos(val)
            q[qi + 1] = np.sin(val)


def _mat_to_quat(R):
    q = pin.Quaternion(np.asarray(R, dtype=float))
    q.normalize()
    return np.asarray(q.coeffs(), dtype=float)  # [x, y, z, w]


def _slerp(q0, q1, t):
    """Spherical linear interpolation from q0 toward q1 by fraction t (shortest path)."""
    d = float(q0 @ q1)
    if d < 0.0:
        q1 = -q1
        d = -d
    if d > 0.9995:  # nearly aligned -> plain lerp avoids div-by-zero
        r = q0 + t * (q1 - q0)
        return r / np.linalg.norm(r)
    theta = np.arccos(d) * t
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def smooth_rotations(mats, alpha):
    """Sign-continuous + exponential-SLERP low-pass of a rotation-matrix sequence, so the
    raw (jittery) per-frame wrist orientation matches the smoothing already applied to the
    finger joints. alpha in (0,1]: 1.0 = no smoothing, smaller = smoother (same convention
    as the retargeting LPFilter's low_pass_alpha)."""
    quats = [_mat_to_quat(R) for R in mats]
    for i in range(1, len(quats)):  # sign continuity (shortest-path hemisphere)
        if float(quats[i] @ quats[i - 1]) < 0.0:
            quats[i] = -quats[i]
    if alpha >= 1.0:
        return np.asarray(quats)
    out = [quats[0]]
    for i in range(1, len(quats)):
        out.append(_slerp(out[-1], quats[i], alpha))
    return np.asarray(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pickle-path", required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--loops", type=int, default=0, help="0 = loop forever")
    ap.add_argument("--rot-smooth", type=float, default=0.2,
                    help="wrist-orientation low-pass (0,1]: 1=raw, smaller=smoother. Default 0.2")
    ap.add_argument("--selftest", action="store_true",
                    help="load model, render one frame, print URL, then exit")
    args = ap.parse_args()

    model, collision_model, visual_model, meta_data, data, wrist_rot = build_robot(
        args.pickle_path
    )
    print(f"Loaded model: {model.name} | dof={model.nq} | visual geoms={visual_model.ngeoms}")
    base_quats = smooth_rotations(wrist_rot, args.rot_smooth) if wrist_rot is not None else None
    print(f"Wrist orientation: {f'driving floating base (smooth alpha={args.rot_smooth})' if base_quats is not None else 'not in pkl (base fixed)'}")

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=False)
    viz.loadViewerModel()
    url = viz.viewer.url()
    print("=" * 60)
    print("MeshCat is serving at:")
    print(f"    {url}")
    print("Open it in your browser (WSL/Windows: http://localhost:7000/static/)")
    print("=" * 60)

    qmap = make_qmap(model, list(meta_data["joint_names"]))
    q = pin.neutral(model)

    if args.selftest:
        set_q(q, qmap, np.asarray(data[0]))
        if base_quats is not None:
            q[0:3] = 0.0
            q[3:7] = base_quats[0]
        viz.display(q)
        print(f"SELFTEST_OK: displayed frame 0 ({len(data)} frames total available)")
        return

    dt = 1.0 / args.fps
    print(f"Replaying {len(data)} frames at {args.fps} fps. Ctrl-C to stop.")
    loop = 0
    try:
        while args.loops == 0 or loop < args.loops:
            for i, qpos in enumerate(data):
                set_q(q, qmap, np.asarray(qpos))
                if base_quats is not None:
                    q[0:3] = 0.0
                    q[3:7] = base_quats[i]
                viz.display(q)
                time.sleep(dt)
            loop += 1
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
