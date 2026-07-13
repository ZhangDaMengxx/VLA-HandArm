"""Sanity-check the new inspire_hand_right_local.yml the same way the visualizer
resolves/loads it: build both configs, confirm joint_names match (so they can
share one renderer), and confirm on a synthetic fully-open hand the local config
opens the pinky while the shipped one leaves it curled."""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "src"))
from dex_retargeting.retargeting_config import RetargetingConfig   # noqa: E402

RetargetingConfig.set_default_urdf_dir(str(REPO / "assets" / "robots" / "hands"))
TELEOP = REPO / "src" / "dex_retargeting" / "configs" / "teleop"
default_cfg = TELEOP / "inspire_hand_right.yml"
local_cfg = TELEOP / "inspire_hand_right_local.yml"

# a synthetic flat/open right hand in MANO frame (meters): fingers extended along +y,
# wrist at origin, MCPs fanned slightly. Enough to check the pinky opens.
def open_hand():
    jp = np.zeros((21, 3), np.float32)
    # MCP x-offsets (index..pinky), y at palm top, then straight fingers
    mcp_x = {5: 0.03, 9: 0.01, 13: -0.01, 17: -0.03}
    lengths = {5: (0.04, 0.03, 0.025), 9: (0.045, 0.03, 0.027),
               13: (0.04, 0.03, 0.025), 17: (0.032, 0.024, 0.02)}
    for mcp, x in mcp_x.items():
        l1, l2, l3 = lengths[mcp]
        jp[mcp] = [x, 0.09, 0.0]
        jp[mcp + 1] = [x, 0.09 + l1, 0.0]
        jp[mcp + 2] = [x, 0.09 + l1 + l2, 0.0]
        jp[mcp + 3] = [x, 0.09 + l1 + l2 + l3, 0.0]
    # thumb splayed out to the side
    jp[1] = [0.03, 0.03, 0.0]; jp[2] = [0.06, 0.05, 0.0]
    jp[3] = [0.08, 0.065, 0.0]; jp[4] = [0.10, 0.075, 0.0]
    return jp


def build(cfg):
    rt = RetargetingConfig.load_from_file(str(cfg)).build()
    hi = np.asarray(rt.optimizer.target_link_human_indices)
    return rt, hi


def pinky_deg(rt, hi, jp, n=30):
    names = rt.joint_names
    pi = names.index("pinky_proximal_joint")
    ref = jp[hi[1], :] - jp[hi[0], :]
    q = None
    for _ in range(n):          # let the low-pass settle
        q = rt.retarget(ref)
    return np.rad2deg(q[pi])


def main():
    rtd, hid = build(default_cfg)
    rtl, hil = build(local_cfg)
    print("default joint_names == local joint_names :", rtd.joint_names == rtl.joint_names)
    print("default human indices:\n", hid)
    print("local   human indices:\n", hil)
    jp = open_hand()
    print(f"\nsynthetic OPEN hand -> pinky proximal (deg), lower = more open:")
    print(f"  shipped base-origin : {pinky_deg(rtd, hid, jp):6.1f}")
    print(f"  local-origin (fix)  : {pinky_deg(rtl, hil, jp):6.1f}")


if __name__ == "__main__":
    main()
