"""量各碰撞网格在 q=0 的世界坐标包围盒(center/size),找手腕与手掌基座的重叠。"""
from pathlib import Path
import numpy as np
import pinocchio as pin

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
urdf = str(REPO / "sim/assets/nero_inspire_right.urdf")
model = pin.buildModelFromUrdf(urdf)
geom = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION)
data = model.createData()
gdata = geom.createData()
q = pin.neutral(model)
pin.forwardKinematics(model, data, q)
pin.updateGeometryPlacements(model, data, geom, gdata)

np.set_printoptions(precision=3, suppress=True)
for i, go in enumerate(geom.geometryObjects):
    M = gdata.oMg[i]
    center = M.translation
    size = None
    try:
        g = go.geometry
        g.computeLocalAABB()
        lo = np.array(g.aabb_local.min_)
        hi = np.array(g.aabb_local.max_)
        size = np.round(hi - lo, 3)
        center = M.act(0.5 * (lo + hi))
    except Exception:
        pass
    print(f"{go.name:26s} center={np.round(center, 3)} size={size}")
