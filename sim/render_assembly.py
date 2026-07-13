"""渲染装配体到 PNG(离屏 EGL),用于目视检查与安装变换标定。"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
from pathlib import Path

import numpy as np
import mujoco

REPO = Path(__file__).resolve().parents[1]
TARGETS = {
    "rm75_inspire": REPO / ("dex-retargeting-main/dex-retargeting-main/"
                            "assets/robots/assembly/rm75_inspire/rm75_inspire_right_hand.urdf"),
    "nero_inspire": REPO / "sim/assets/nero_inspire_right.urdf",
}
OUT = REPO / "sim/out"
OUT.mkdir(parents=True, exist_ok=True)


def save_png(arr, path):
    try:
        import imageio.v3 as iio
        iio.imwrite(str(path), arr)
        return "imageio"
    except Exception:
        from PIL import Image
        Image.fromarray(arr).save(str(path))
        return "PIL"


for name, urdf in TARGETS.items():
    try:
        m = mujoco.MjModel.from_xml_path(str(urdf))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, height=640, width=880)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(m, cam)
        cam.distance *= 1.5
        cam.elevation = -20
        cam.azimuth = 130
        r.update_scene(d, cam)
        img = r.render()
        writer = save_png(img, OUT / f"{name}.png")
        print(f"OK {name}: {img.shape} via {writer} -> sim/out/{name}.png")
        r.close()
    except Exception as e:
        import traceback
        print(f"FAILED {name}: {type(e).__name__}")
        traceback.print_exc()
