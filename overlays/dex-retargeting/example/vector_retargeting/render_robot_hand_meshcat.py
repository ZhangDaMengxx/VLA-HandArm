"""
CPU-only visualization of dex-retargeting output via pinocchio + meshcat (browser WebGL).
No Vulkan / GPU needed. Also draws colored spheres at the fingertip frames (*_tip links,
which carry no visual mesh -- they are the retargeting target anchors on the robot side).

Run from example/vector_retargeting/:
    python3 render_robot_hand_meshcat.py --pickle-path data/inspire_joints.pkl
Open the printed http://127.0.0.1:7000 URL in your browser (WSL2: Windows browser).
Toggle markers with --no-tips ; resize with --tip-radius 0.008 . Ctrl+C to stop.
"""
from pathlib import Path
import time

import numpy as np
import tyro

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg

from dex_retargeting.retargeting_config import RetargetingConfig

# per-finger marker color, matched by substring of the *_tip frame name
TIP_COLORS = {
    "thumb": 0xff3b30,   # red
    "index": 0x34c759,   # green
    "middle": 0x0a84ff,  # blue
    "ring": 0xffd60a,    # yellow
    "pinky": 0xff2d92,   # magenta
}


def tip_color(name):
    for key, col in TIP_COLORS.items():
        if key in name:
            return col
    return 0xffffff


def main(
    pickle_path: str,
    fps: float = 30.0,
    loop: bool = True,
    tips: bool = True,
    tip_radius: float = 0.006,
):
    robot_dir = Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))

    pickle_data = np.load(pickle_path, allow_pickle=True)
    meta_data, data = pickle_data["meta_data"], pickle_data["data"]

    config = RetargetingConfig.load_from_file(meta_data["config_path"])
    urdf_path = Path(config.urdf_path)
    if not urdf_path.is_absolute():
        urdf_path = robot_dir / urdf_path
    urdf_path = urdf_path.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    print(f"[info] URDF: {urdf_path}")

    package_dirs = [
        str(urdf_path.parent), str(robot_dir),
        str(robot_dir.parent), str(robot_dir.parent.parent),
    ]
    model = pin.buildModelFromUrdf(str(urdf_path))
    visual_model = pin.buildGeomFromUrdf(
        model, str(urdf_path), pin.GeometryType.VISUAL, package_dirs=package_dirs
    )

    viz = MeshcatVisualizer(model, pin.GeometryModel(), visual_model)
    viz.initViewer(open=False)
    viz.loadViewerModel(rootNodeName="hand")
    print(f"\n[info] Meshcat URL: {viz.viewer.url()}")
    print("[info] Open it in your browser (WSL2: use the Windows browser).\n")

    # --- fingertip anchor markers (spheres on every *_tip frame) ------------
    tip_frames = []
    if tips:
        tip_frames = [(f.name, fid) for fid, f in enumerate(model.frames)
                      if f.name.endswith("_tip")]
        for name, _ in tip_frames:
            viz.viewer["fingertips"][name].set_object(
                mg.Sphere(tip_radius), mg.MeshLambertMaterial(color=tip_color(name))
            )
        print(f"[info] tip markers: {[n for n, _ in tip_frames] or 'none found'}")

    def update_tips():
        pin.updateFramePlacements(model, viz.data)
        for name, fid in tip_frames:
            T = np.eye(4)
            T[:3, 3] = viz.data.oMf[fid].translation
            viz.viewer["fingertips"][name].set_transform(T)
    # ------------------------------------------------------------------------

    retargeting_joint_names = list(meta_data["joint_names"])
    name_to_q = {
        model.names[j]: (model.joints[j].idx_q, model.joints[j].nq)
        for j in range(1, model.njoints)
    }
    print(f"[info] model joints: {model.njoints - 1}, pkl joint_names: {len(retargeting_joint_names)}")

    q0 = pin.neutral(model)

    def to_q(qpos):
        qpos = np.asarray(qpos, dtype=float)
        q = q0.copy()
        for i, name in enumerate(retargeting_joint_names):
            if name in name_to_q:
                idx_q, nq = name_to_q[name]
                if nq == 1:
                    q[idx_q] = qpos[i]
                elif nq == 2:                    # continuous joint stored as (cos, sin)
                    q[idx_q] = np.cos(qpos[i])
                    q[idx_q + 1] = np.sin(qpos[i])
        return q

    viz.display(q0)
    if tip_frames:
        update_tips()
    dt = 1.0 / fps if fps > 0 else 0.0
    print(f"[info] Playing {len(data)} frames @ {fps} fps. Ctrl+C to stop.")
    try:
        while True:
            for qpos in data:
                viz.display(to_q(qpos))
                if tip_frames:
                    update_tips()
                if dt:
                    time.sleep(dt)
            if not loop:
                input("[info] Done. Press Enter to exit...")
                break
    except KeyboardInterrupt:
        print("\n[info] stopped.")


if __name__ == "__main__":
    tyro.cli(main)
