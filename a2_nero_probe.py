"""Track2 探针:NERO 臂 MuJoCo 加载 + Pinocchio IK + inspire 手根 link。"""
import sys
from pathlib import Path
import numpy as np
import mujoco

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
PKL_SRC = REPO / "pinocchio-kinematics-lite-main/pinocchio-kinematics-lite-main/src"
sys.path.insert(0, str(PKL_SRC))
nero_urdf = PKL_SRC / "pinocchio_kinematics_lite/assets/nero/nero_description.urdf"
insp_urdf = REPO / ("dex-retargeting-main/dex-retargeting-main/"
                    "assets/robots/hands/inspire_hand/inspire_hand_right.urdf")


def names(m, objtype):
    return [mujoco.mj_id2name(m, objtype, i) for i in range(
        {mujoco.mjtObj.mjOBJ_BODY: m.nbody, mujoco.mjtObj.mjOBJ_JOINT: m.njnt}[objtype])]


print("== NERO arm in MuJoCo ==")
try:
    m = mujoco.MjModel.from_xml_path(str(nero_urdf))
    print(f"OK nq={m.nq} njnt={m.njnt} nbody={m.nbody} nmesh={m.nmesh}")
    print("bodies:", names(m, mujoco.mjtObj.mjOBJ_BODY))
    print("joints:", names(m, mujoco.mjtObj.mjOBJ_JOINT))
except Exception as e:
    print("FAILED:", type(e).__name__, str(e)[:500])

print("== inspire hand in MuJoCo ==")
try:
    mh = mujoco.MjModel.from_xml_path(str(insp_urdf))
    b = names(mh, mujoco.mjtObj.mjOBJ_BODY)
    print(f"OK nq={mh.nq} njnt={mh.njnt} nbody={mh.nbody}")
    print("root body (attach point):", b[1] if len(b) > 1 else "?")
    print("bodies:", b)
except Exception as e:
    print("FAILED:", type(e).__name__, str(e)[:500])

print("== Pinocchio NeroKinematics ==")
try:
    from pinocchio_kinematics_lite import NeroKinematics
    kin = NeroKinematics()
    q = np.zeros(7)
    pose = kin.forward_kinematics(q)
    res = kin.inverse_kinematics(pose, q_init=q)
    print("fk type:", type(pose).__name__)
    print("frames tail:", kin.list_frames()[-6:])
    print("ik_success:", res.success)
except Exception as e:
    import traceback
    traceback.print_exc()
