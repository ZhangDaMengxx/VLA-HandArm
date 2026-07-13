"""
Inspect a dex-retargeting .pkl WITHOUT rendering:
  - per-joint value range (flags joints that never move)
  - coverage: which movable URDF joints are missing from the pkl (mimic/coupled/excluded)
  - saves a joint-trajectory plot as PNG (headless/CPU, no GUI)

Run from example/vector_retargeting/:
    python3 inspect_pkl.py --pickle-path data/panda_joints.pkl
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import tyro

import matplotlib
matplotlib.use("Agg")            # headless-safe: render to file, no window
import matplotlib.pyplot as plt

from dex_retargeting.retargeting_config import RetargetingConfig

MOVABLE = {"revolute", "prismatic", "continuous", "planar", "floating"}


def urdf_joints(urdf_path):
    root = ET.parse(urdf_path).getroot()
    return [
        (j.get("name"), j.get("type"), j.find("mimic") is not None)
        for j in root.findall("joint")
    ]


def main(pickle_path: str, output_image: str = None):
    robot_dir = Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))

    pickle_data = np.load(pickle_path, allow_pickle=True)
    meta_data, data = pickle_data["meta_data"], pickle_data["data"]

    joint_names = list(meta_data["joint_names"])
    qpos = np.asarray(data, dtype=float)
    if qpos.ndim == 1:
        qpos = qpos.reshape(len(data), -1)
    T, D = qpos.shape

    print("\n=== pkl summary ===")
    print(f"frames        : {T}")
    print(f"dof (columns) : {D}")
    print(f"joint_names   : {len(joint_names)}")
    print(f"config_path   : {meta_data.get('config_path')}")
    if D != len(joint_names):
        print(f"[warn] qpos width {D} != joint_names {len(joint_names)} -- layout mismatch!")

    # per-joint motion; flat joints are candidates for 'not actually driven'
    lo, hi = qpos.min(axis=0), qpos.max(axis=0)
    rng = hi - lo
    print("\n=== per-joint motion (rad or m) ===")
    print(f"{'idx':>3}  {'joint':<28}{'min':>9}{'max':>9}{'range':>9}   flag")
    for i, name in enumerate(joint_names):
        flag = "  <-- flat (never moves)" if rng[i] < 1e-4 else ""
        print(f"{i:>3}  {name:<28}{lo[i]:>9.4f}{hi[i]:>9.4f}{rng[i]:>9.4f}{flag}")

    # URDF vs pkl coverage -> the real "missing joints" answer
    config = RetargetingConfig.load_from_file(meta_data["config_path"])
    urdf_path = Path(config.urdf_path)
    if not urdf_path.is_absolute():
        urdf_path = robot_dir / urdf_path
    urdf_path = urdf_path.resolve()
    if urdf_path.exists():
        movable = [(n, t, m) for (n, t, m) in urdf_joints(urdf_path) if t in MOVABLE]
        pkl_set = set(joint_names)
        not_driven = [(n, t, m) for (n, t, m) in movable if n not in pkl_set]
        print("\n=== URDF vs pkl coverage ===")
        print(f"URDF                   : {urdf_path.name}")
        print(f"movable joints in URDF : {len(movable)}")
        print(f"driven by retargeting  : {len(movable) - len(not_driven)}")
        if not_driven:
            print(f"NOT in pkl ({len(not_driven)}) -- your 'missing' joints:")
            for n, t, m in not_driven:
                tag = "mimic/coupled" if m else "not optimized"
                print(f"    {n:<28} type={t:<11} ({tag})")
        else:
            print("all movable URDF joints are present in the pkl.")
    else:
        print(f"\n[warn] URDF not found, skip coverage: {urdf_path}")

    # trajectory plot
    out_png = output_image or str(Path(pickle_path).with_name(Path(pickle_path).stem + "_joints.png"))
    plt.figure(figsize=(12, 6))
    for i, name in enumerate(joint_names):
        plt.plot(qpos[:, i], label=f"{i}:{name}")
    plt.xlabel("frame"); plt.ylabel("joint value (rad / m)")
    plt.title(f"{Path(pickle_path).name}  ({T} frames, {D} dof)")
    plt.legend(fontsize=7, ncol=2, loc="upper right")
    plt.grid(True, alpha=0.3); plt.tight_layout()
    plt.savefig(out_png, dpi=130)
    print(f"\n[info] saved plot -> {out_png}  (open it in Windows)")


if __name__ == "__main__":
    tyro.cli(main)
