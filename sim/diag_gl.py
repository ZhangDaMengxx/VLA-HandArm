import os, ctypes
os.environ["MUJOCO_GL"] = "osmesa"
import mujoco
print("mujoco", mujoco.__version__, "| MUJOCO_GL", os.environ.get("MUJOCO_GL"))
print("has mujoco.Renderer:", hasattr(mujoco, "Renderer"))

try:
    from mujoco.rendering.classic.renderer import Renderer  # noqa
    print("direct import Renderer: OK")
except Exception as e:
    print("direct import Renderer FAIL:", repr(e)[:200])

try:
    from mujoco.osmesa import GLContext
    ctx = GLContext(640, 480)
    print("osmesa GLContext: OK")
except Exception as e:
    print("osmesa GLContext FAIL:", repr(e)[:200])

for lib in ["libOSMesa.so.8", "libOSMesa.so", "libEGL.so.1", "libGL.so.1"]:
    try:
        ctypes.CDLL(lib)
        print("lib OK  ", lib)
    except Exception:
        print("lib MISS", lib)

for mod in ["pyrender", "meshcat", "trimesh", "PIL", "imageio", "mujoco.viewer"]:
    try:
        __import__(mod)
        print("mod OK  ", mod)
    except Exception as e:
        print("mod MISS", mod, type(e).__name__)

try:
    import OpenGL
    print("PyOpenGL", OpenGL.__version__)
except Exception as e:
    print("PyOpenGL ?", e)
