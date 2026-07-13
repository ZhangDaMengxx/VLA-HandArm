"""
Diagnose the open<->close amplitude mismatch between the human hand in the video
and the retargeted inspire hand.

For every frame we measure:
  - human per-finger MCP flexion angle (deg)  = angle(wrist->MCP , MCP->PIP)
        ~0 deg when the finger is flat/extended, grows as it curls.
  - robot per-finger *_proximal_joint angle (deg) from retarget() qpos.
        joint limit is [0, 1.47 rad] = [0, 84.2 deg]  (0 = extended, 84 = curled)
  - human finger SPREAD (fan angle between index and pinky), which the inspire
        4 fingers physically cannot reproduce (no abduction joint).

Then we sweep scaling_factor to see whether the range can be recovered by tuning.

Run:
  MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 --cd '<this dir>' -- \
      /home/zhang123/ros2_ws/enter/envs/lerobot/bin/python diag_openness.py
Env: MAX_FRAMES (default all)
"""
import os
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]                      # .../dex-retargeting
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(HERE.parent))        # for single_hand_detector

from dex_retargeting.retargeting_config import RetargetingConfig       # noqa: E402
from single_hand_detector import SingleHandDetector                    # noqa: E402

URDF_DIR = REPO / "assets" / "robots" / "hands"
CFG = REPO / "src" / "dex_retargeting" / "configs" / "teleop" / "inspire_hand_right.yml"

# locate the video
DATA = HERE.parent / "data"
vids = sorted(DATA.glob("*.mp4"))
VIDEO = DATA / "human_hand_video.mp4"
if not VIDEO.exists():
    if not vids:
        raise SystemExit(f"No .mp4 found in {DATA}")
    VIDEO = vids[0]

MAX_FRAMES = int(os.environ.get("MAX_FRAMES", "100000"))

# MANO / mediapipe 21-keypoint indices
WRIST = 0
FINGERS = {
    "thumb":  dict(mcp=2,  pip=3,  tip=4,  prox="thumb_proximal_pitch_joint"),
    "index":  dict(mcp=5,  pip=6,  tip=8,  prox="index_proximal_joint"),
    "middle": dict(mcp=9,  pip=10, tip=12, prox="middle_proximal_joint"),
    "ring":   dict(mcp=13, pip=14, tip=16, prox="ring_proximal_joint"),
    "pinky":  dict(mcp=17, pip=18, tip=20, prox="pinky_proximal_joint"),
}
LIMIT_DEG = np.rad2deg(1.47)   # proximal joint upper limit for the 4 fingers


def ang(u, v):
    """angle between two vectors, degrees"""
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    c = np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0)
    return float(np.rad2deg(np.arccos(c)))


def human_mcp_flexion(jp, mcp, pip):
    meta = jp[mcp] - jp[WRIST]     # wrist -> MCP  (metacarpal ray)
    prox = jp[pip] - jp[mcp]       # MCP  -> PIP  (proximal phalanx)
    return ang(meta, prox)


def human_spread(jp):
    """fan angle between the index and pinky finger directions (MCP->tip)"""
    vi = jp[FINGERS["index"]["tip"]] - jp[FINGERS["index"]["mcp"]]
    vp = jp[FINGERS["pinky"]["tip"]] - jp[FINGERS["pinky"]["mcp"]]
    return ang(vi, vp)


def span(a, lo=5, hi=95):
    a = np.asarray(a)
    return np.percentile(a, lo), np.percentile(a, hi)


def main():
    RetargetingConfig.set_default_urdf_dir(str(URDF_DIR))
    print(f"video : {VIDEO.name}")
    print(f"config: {CFG.name}\n")

    # ---- pass 1: detect all keypoints once, cache them ----
    detector = SingleHandDetector(hand_type="Right", selfie=False)
    cap = cv2.VideoCapture(str(VIDEO))
    joints = []          # list of (21,3) metric MANO-frame keypoints
    n_read = n_found = 0
    while n_read < MAX_FRAMES:
        ok, bgr = cap.read()
        if not ok:
            break
        n_read += 1
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        num, jp, _, _ = detector.detect(rgb)
        if num > 0:
            joints.append(jp)
            n_found += 1
    cap.release()
    print(f"frames read={n_read}  hand detected={n_found}\n")
    if n_found < 5:
        raise SystemExit("Too few detections.")
    J = np.stack(joints)   # (F,21,3)

    # ---- human metrics ----
    human_flex = {f: np.array([human_mcp_flexion(jp, d["mcp"], d["pip"])
                               for jp in J]) for f, d in FINGERS.items()}
    spread = np.array([human_spread(jp) for jp in J])

    # ---- robot metrics (override-driven; ref built from the built optimizer's
    #      own human indices so base-origin and local-origin variants both work) ----
    def run_retarget(override):
        rt = RetargetingConfig.load_from_file(str(CFG), override=override).build()
        names = rt.joint_names
        idx = {f: names.index(d["prox"]) for f, d in FINGERS.items()}
        yaw_i = names.index("thumb_proximal_yaw_joint")
        hi = np.asarray(rt.optimizer.target_link_human_indices)
        origin_h, task_h = hi[0], hi[1]
        out = {f: [] for f in FINGERS}
        yaw = []
        for jp in J:
            ref = jp[task_h, :] - jp[origin_h, :]
            q = rt.retarget(ref)
            for f in FINGERS:
                out[f].append(np.rad2deg(q[idx[f]]))
            yaw.append(np.rad2deg(q[yaw_i]))
        return {f: np.array(v) for f, v in out.items()}, np.array(yaw)

    robot_flex, robot_yaw = run_retarget({"scaling_factor": 1.15})

    # ================= REPORT =================
    print("=" * 78)
    print("FLEXION (open<->close)  —  per finger, degrees")
    print("  human = MCP flexion angle;  robot = proximal joint (limit 0..%.0f deg)" % LIMIT_DEG)
    print("-" * 78)
    print(f"{'finger':7} | {'HUMAN p5..p95 (span)':26} | {'ROBOT p5..p95 (span)':26} | robot/limit")
    print("-" * 78)
    for f in FINGERS:
        h5, h95 = span(human_flex[f])
        r5, r95 = span(robot_flex[f])
        hspan = h95 - h5
        rspan = r95 - r5
        lim = LIMIT_DEG if f != "thumb" else np.rad2deg(0.6)
        print(f"{f:7} | {h5:6.1f}..{h95:6.1f}  ({hspan:5.1f}) | "
              f"{r5:6.1f}..{r95:6.1f}  ({rspan:5.1f}) | {100*rspan/lim:4.0f}% of range")
    print("-" * 78)
    print("  If ROBOT span << HUMAN span, the open/close amplitude is being compressed.")
    print("  robot p5 near 0  = hand fully opens/flattens;  p95 near limit = fully closes.\n")

    print("=" * 78)
    print("SPREAD (fingers splaying apart)  —  degrees, index-vs-pinky fan angle")
    s5, s95 = span(spread)
    print(f"  human spread: {s5:5.1f} .. {s95:5.1f}   (varies by {s95-s5:.1f} deg)")
    print(f"  robot spread: FIXED — inspire's 4 fingers have NO abduction joint.")
    print(f"                only thumb yaw moves sideways (limit 0..{np.rad2deg(1.308):.0f} deg).\n")

    # ---- scaling sweep (index finger, reusing cached keypoints) ----
    print("=" * 78)
    print("SCALING SWEEP — effect on INDEX proximal (deg), does tuning recover range?")
    print("-" * 78)
    print(f"{'scaling':8} | {'index p5..p95 (span)':26} | opens? (p5~0)  closes? (p95~84)")
    print("-" * 78)
    for s in [1.0, 1.15, 1.3, 1.5, 1.7, 2.0]:
        rf, _ = run_retarget({"scaling_factor": s})
        r5, r95 = span(rf["index"])
        opens = "yes" if r5 < 12 else ("~" if r5 < 25 else "NO")
        closes = "yes" if r95 > 72 else ("~" if r95 > 55 else "NO")
        print(f"{s:8.2f} | {r5:6.1f}..{r95:6.1f}  ({r95-r5:5.1f}) | "
              f"open={opens:3}   close={closes}")
    print("=" * 78)

    # ---- FIX TEST: local per-finger origin (each finger's own MCP) instead of base ----
    # origin link = each finger's proximal (MCP) link; human origin = that finger's MCP.
    # removes the shared far-away palm offset so each finger's curl maps locally.
    local_override = {
        "target_origin_link_names": ["thumb_proximal_base", "index_proximal",
                                     "middle_proximal", "ring_proximal", "pinky_proximal"],
        "target_link_human_indices": np.array([[2, 5, 9, 13, 17],
                                               [4, 8, 12, 16, 20]]),
        "scaling_factor": 1.15,
    }
    robot_local, _ = run_retarget(local_override)

    # HYBRID: keep thumb on base origin (opposition needs it), fingers local.
    hybrid_override = {
        "target_origin_link_names": ["base", "index_proximal",
                                     "middle_proximal", "ring_proximal", "pinky_proximal"],
        "target_link_human_indices": np.array([[0, 5, 9, 13, 17],
                                               [4, 8, 12, 16, 20]]),
        "scaling_factor": 1.15,
    }
    robot_hybrid, _ = run_retarget(hybrid_override)

    print()
    print("=" * 78)
    print("FIX TEST — proximal p5..p95 (deg); p5 near 0 = finger fully opens")
    print("-" * 78)
    print(f"{'finger':7} | {'BASE (shipped)':20} | {'ALL-LOCAL':20} | {'HYBRID (recommend)':20}")
    print("-" * 78)
    for f in FINGERS:
        b5, b95 = span(robot_flex[f])
        l5, l95 = span(robot_local[f])
        h5, h95 = span(robot_hybrid[f])
        print(f"{f:7} | {b5:5.1f}..{b95:5.1f} sp{b95-b5:5.1f} | "
              f"{l5:5.1f}..{l95:5.1f} sp{l95-l5:5.1f} | "
              f"{h5:5.1f}..{h95:5.1f} sp{h95-h5:5.1f}")
    print("=" * 78)
    print("  HYBRID = thumb keeps base origin, 4 fingers use their own MCP as origin.")

    # ---- plot ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        order = ["thumb", "index", "middle", "ring", "pinky"]
        fig, axes = plt.subplots(2, 5, figsize=(20, 8))
        for j, f in enumerate(order):
            lim = LIMIT_DEG if f != "thumb" else np.rad2deg(0.6)
            # time series
            ax = axes[0, j]
            ax.plot(human_flex[f], label="human MCP flex", color="#1f77b4", lw=1.2)
            ax.plot(robot_flex[f], label="robot (base origin)", color="#ff7f0e", lw=1.2)
            ax.plot(robot_hybrid[f], label="robot (hybrid fix)", color="#2ca02c", lw=1.0, alpha=.8)
            ax.axhline(lim, ls="--", c="r", lw=.8)
            ax.axhline(0, ls="--", c="g", lw=.8)
            ax.set_title(f"{f}"); ax.set_xlabel("frame"); ax.set_ylabel("deg")
            if j == 0:
                ax.legend(fontsize=8)
            # scatter human vs robot
            ax2 = axes[1, j]
            ax2.scatter(human_flex[f], robot_flex[f], s=4, alpha=.4, color="#555")
            ax2.axhline(lim, ls="--", c="r", lw=.8)
            ax2.set_xlabel("human MCP flex (deg)")
            ax2.set_ylabel("robot proximal (deg)")
            ax2.set_ylim(-5, max(lim, robot_flex[f].max()) + 5)
        fig.suptitle("Human finger flexion vs retargeted inspire proximal joint "
                     "(green=fully open, red=joint limit)", fontsize=13)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        out = HERE / "diag_openness.png"
        fig.savefig(out, dpi=110)
        print(f"\nsaved plot -> {out}")
    except Exception as e:
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
