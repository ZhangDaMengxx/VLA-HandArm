"""Phase A-1 环境探针:检查 WSL 仿真环境 + 试加载 rm75_inspire 装配。"""
import importlib, os, sys

print("PYTHON:", sys.version.split()[0])
print("EXE   :", sys.executable)
print("=" * 60)

pkgs = ["numpy", "mujoco", "dm_control", "mujoco_menagerie", "pinocchio",
        "yourdfpy", "trimesh", "urdf_parser_py", "mediapipe", "dex_retargeting"]
for p in pkgs:
    try:
        m = importlib.import_module(p)
        print(f"  OK   {p:16s} {getattr(m, '__version__', '?')}")
    except Exception as e:
        print(f"  MISS {p:16s} ({type(e).__name__})")
print("=" * 60)

REPO = "/home/zhang123/ros2_ws/lerobotTest"
asm = os.path.join(REPO, "dex-retargeting-main/dex-retargeting-main/"
                   "assets/robots/assembly/rm75_inspire")
print("assembly dir:", os.path.isdir(asm), asm)
if os.path.isdir(asm):
    for f in sorted(os.listdir(asm)):
        print("   -", f)

meshroot = os.path.join(REPO, "dex-retargeting-main/dex-retargeting-main/assets/robots")
for label, sub in [("rm75 visual", "arms/rm75/meshes/visual"),
                   ("rm75 collision", "arms/rm75/meshes/collision"),
                   ("inspire visual", "hands/inspire_hand/meshes/visual"),
                   ("inspire collision", "hands/inspire_hand/meshes/collision")]:
    d = os.path.join(meshroot, sub)
    if os.path.isdir(d):
        exts = sorted({os.path.splitext(x)[1] for x in os.listdir(d)})
        print(f"   {label:18s} exts={exts}")
print("=" * 60)

urdf = os.path.join(asm, "rm75_inspire_right_hand.urdf")
try:
    import mujoco
    m = mujoco.MjModel.from_xml_path(urdf)
    print(f"MuJoCo loaded OK: nq={m.nq} nv={m.nv} njnt={m.njnt} nbody={m.nbody}")
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    print("joints:", names)
except Exception as e:
    print("MuJoCo load FAILED:", type(e).__name__)
    print(str(e)[:400])
