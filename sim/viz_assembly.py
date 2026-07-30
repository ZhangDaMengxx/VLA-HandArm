"""静态渲染 NERO+inspire 装配(零位姿)到 .rrd,目视核对适配法兰与手的装配。

法兰几何单独高亮(橙色),手基座坐标系画出来,方便判断 flange_to_hand 该怎么标定。
用桌面版 Rerun 打开输出的 .rrd。
"""
import sys
from pathlib import Path
import numpy as np
import rerun as rr

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_rerun import RobotModel, load_meshes, log_axes

URDF = REPO / "sim/assets/nero_inspire_right.urdf"


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=str(REPO / "assembly.rrd"))
    args = ap.parse_args()

    model = RobotModel(URDF)
    meshes = load_meshes(model)
    q = np.zeros(model.model.nq)
    placements = model.geom_placements(q) if hasattr(model, "geom_placements") else None
    # geom placement 取法:复用 forward 接口
    import pinocchio as pin
    pin.forwardKinematics(model.model, model.data, q)
    pin.updateGeometryPlacements(model.model, model.data, model.visual, model.gdata, q)
    placements = [model.gdata.oMg[i].homogeneous.copy() for i in range(model.visual.ngeoms)]

    rr.init("nero_assembly")
    rr.save(args.save)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    for i, m in enumerate(meshes):
        if m is None:
            continue
        T = placements[i]
        V = (T[:3, :3] @ m["V"].T).T + T[:3, 3]
        name = model.visual.geometryObjects[i].name
        is_flange = "flange" in name.lower() or "adapter" in name.lower()
        col = [255, 150, 40] if is_flange else m.get("color", [180, 180, 190])
        rr.log(f"robot/{name}", rr.Mesh3D(vertex_positions=V, triangle_indices=m["F"],
                                          vertex_normals=m.get("N"),
                                          albedo_factor=col))
    # link8、法兰、手基座的坐标系
    for lname in ("link8", "rh56df_adapter_flange", "base"):
        try:
            fid = model.model.getFrameId(lname)
            pin.updateFramePlacement(model.model, model.data, fid)
            T = model.data.oMf[fid].homogeneous.copy()
            log_axes(f"frames/{lname}", T, length=0.05)
        except Exception as e:
            print(f"frame {lname}: {e}")

    print(f"wrote {args.save}  (visual geoms={model.visual.ngeoms})")


if __name__ == "__main__":
    main()
