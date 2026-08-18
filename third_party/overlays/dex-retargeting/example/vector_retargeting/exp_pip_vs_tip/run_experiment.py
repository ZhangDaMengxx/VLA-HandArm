"""
Compare inspire vector-retargeting target choices on IDENTICAL detected keypoints.

Configs (all share the same 6 actuated joints, scaling=1.15, low_pass_alpha=0.2):
  A  tip-only        : 5 fingertip vectors            (original inspire_hand_right.yml)
  B  tip+PIP (shadow): 5 tip + 5 PIP/middle vectors   (inspire_hand_right_tip_pip.yml)
  B2 tip+fingerPIP   : 5 tip + 4 finger PIP (thumb tip-only, ablation)

Fairness: MediaPipe runs ONCE; every config consumes the same 21x3 sequence.
Accuracy is measured the SAME way the optimizer defines it: for each fingertip,
||robot(base->tip) - 1.15*human(wrist->tip)|| via forward kinematics (meters).

Robustness: add gaussian noise to the 5 FINGERTIPS (indices 4,8,12,16,20) in the
middle third of the clip (PIP indices stay clean -> mimics "tip occluded, knuckle
still visible"). Averaged over several noise seeds. We report how far each method's
robot fingertips drift from the TRUE (clean) target when the tips are corrupted.
"""
import os
import sys
import pickle
from pathlib import Path

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # single_hand_detector

from dex_retargeting.constants import (
    RobotName, RetargetingType, HandType, get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.robot_wrapper import RobotWrapper
from single_hand_detector import SingleHandDetector

ROOT = HERE.parents[2]
ROBOT_DIR = ROOT / "assets" / "robots" / "hands"
URDF = ROBOT_DIR / "inspire_hand" / "inspire_hand_right.urdf"
VIDEO = HERE.parent / "data" / "human_hand_video.mp4"
MAX_FRAMES = int(os.environ.get("MAX_FRAMES", "0"))
SIGMA = float(os.environ.get("SIGMA", "0.015"))      # fingertip noise, meters
NSEED = int(os.environ.get("NSEED", "3"))
SCALING = 1.15

RetargetingConfig.set_default_urdf_dir(str(ROBOT_DIR))

TIP_IDS = [4, 8, 12, 16, 20]                          # human MANO fingertips
TIP_LINKS = ["thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip"]
FINGER_LABEL = ["thumb", "index", "middle", "ring", "pinky"]
ACT_NAMES = ["index_proximal_joint", "middle_proximal_joint", "ring_proximal_joint",
             "pinky_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint"]
ACT_LABEL = ["index", "middle", "ring", "pinky", "thumb_pitch", "thumb_yaw"]

CONFIGS = {
    "A_tip": str(get_default_config_path(RobotName.inspire, RetargetingType.vector, HandType.right)),
    "B_tip+PIP": str(HERE / "inspire_hand_right_tip_pip.yml"),
    "B2_tip+fingerPIP": str(HERE / "inspire_hand_right_tip_fingerpip.yml"),
}

# ----- FK helper (standalone; qpos already carries mimic values) -----
_fk = RobotWrapper(str(URDF))
_base_id = _fk.get_link_index("base")
_tip_ids = [_fk.get_link_index(n) for n in TIP_LINKS]


def robot_tip_vecs(qseq):
    """base->tip vectors for each frame, shape (n,5,3)."""
    V = np.zeros((len(qseq), 5, 3))
    for i, q in enumerate(qseq):
        _fk.compute_forward_kinematics(np.asarray(q, dtype=np.float64))
        b = _fk.get_link_pose(_base_id)[:3, 3]
        for j, tid in enumerate(_tip_ids):
            V[i, j] = _fk.get_link_pose(tid)[:3, 3] - b
    return V


def detect_sequence(video_path, max_frames=0):
    det = SingleHandDetector(hand_type="Right", selfie=False,
                             min_detection_confidence=0.5, min_tracking_confidence=0.5)
    cap = cv2.VideoCapture(str(video_path))
    seq, fresh, last = [], [], None
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        num, joint_pos, _, _ = det.detect(frame[..., ::-1])
        if num == 0:
            if last is not None:
                seq.append(last.copy()); fresh.append(False)
        else:
            last = joint_pos.copy()
            seq.append(joint_pos.copy()); fresh.append(True)
        if max_frames and len(seq) >= max_frames:
            break
    cap.release()
    return np.asarray(seq), np.asarray(fresh)


def run_config(cfg_path, seq):
    rt = RetargetingConfig.load_from_file(str(cfg_path)).build()
    idx = np.asarray(rt.optimizer.target_link_human_indices)
    out = [np.asarray(rt.retarget(jp[idx[1, :], :] - jp[idx[0, :], :])).copy() for jp in seq]
    return np.asarray(out), list(rt.optimizer.robot.dof_joint_names)


def jitter(q):
    return np.zeros(q.shape[1]) if len(q) < 2 else np.abs(np.diff(q, axis=0)).mean(axis=0)


def main():
    print(f"[1/5] detect once (video={VIDEO.name}, max_frames={MAX_FRAMES or 'all'})")
    seq, fresh = detect_sequence(VIDEO, MAX_FRAMES)
    n = len(seq)
    w0, w1 = n // 3, 2 * n // 3
    print(f"      frames={n} fresh={int(fresh.sum())} held={int((~fresh).sum())} | occ window [{w0},{w1}) sigma={SIGMA*100:.1f}cm seeds={NSEED}")

    # ground-truth target from CLEAN human tips
    human_tip_clean = seq[:, TIP_IDS, :] - seq[:, [0], :]        # (n,5,3)
    target_tip = SCALING * human_tip_clean

    # clean runs
    print("[2/5] retargeting clean input for A / B / B2")
    q_clean, names = {}, None
    tip_err_clean, jit_clean = {}, {}
    for name, cfg in CONFIGS.items():
        q, names = run_config(cfg, seq)
        q_clean[name] = q
        rv = robot_tip_vecs(q)
        tip_err_clean[name] = np.linalg.norm(rv - target_tip, axis=2)   # (n,5) meters
        jit_clean[name] = jitter(q)

    act = [names.index(a) for a in ACT_NAMES]

    # occluded runs. Two corruption models applied to the 5 FINGERTIPS in the window
    # (PIP indices 2,6,10,14,18 stay clean -> "tip hidden, knuckle still visible"):
    #   noise   : zero-mean gaussian    -> tests jitter/shake robustness
    #   retract : tip pulled 40% toward wrist -> systematic occlusion bias (realistic)
    print("[3/5] retargeting occluded input: noise (x%d seeds) + retract" % NSEED)

    def corrupt(mode, seed):
        so = seq.copy()
        if mode == "noise":
            rng = np.random.default_rng(seed)
            so[w0:w1][:, TIP_IDS, :] += rng.normal(0, SIGMA, size=(w1 - w0, len(TIP_IDS), 3))
        elif mode == "retract":
            wrist = so[w0:w1][:, [0], :]
            tips = so[w0:w1][:, TIP_IDS, :]
            so[w0:w1][:, TIP_IDS, :] = wrist + 0.60 * (tips - wrist)
        return so

    MODES = {"noise": list(range(NSEED)), "retract": [0]}
    tip_err_occ = {m: {k: [] for k in CONFIGS} for m in MODES}
    jit_occ_w = {m: {k: [] for k in CONFIGS} for m in MODES}
    q_occ_last = {m: {} for m in MODES}
    for mode, seeds in MODES.items():
        for s in seeds:
            seq_occ = corrupt(mode, s)
            for name, cfg in CONFIGS.items():
                q, _ = run_config(cfg, seq_occ)
                rv = robot_tip_vecs(q)
                tip_err_occ[mode][name].append(np.linalg.norm(rv - target_tip, axis=2))
                jit_occ_w[mode][name].append(jitter(q[w0:w1]))
                q_occ_last[mode][name] = q
        for k in CONFIGS:
            tip_err_occ[mode][k] = np.mean(tip_err_occ[mode][k], axis=0)
            jit_occ_w[mode][k] = np.mean(jit_occ_w[mode][k], axis=0)

    # ---------------- numbers ----------------
    def mm(x):
        return x * 1000.0
    fingers = [1, 2, 3, 4]  # index..pinky
    thumb = [0]

    print("\n===== TIP-TRACKING ERROR (mm; lower = closer to TRUE human target) =====")
    hdr = "method".ljust(22) + "".join(f"{l:>8}" for l in FINGER_LABEL) + "  |  fing thumb  all"
    print(hdr); print("-" * len(hdr))

    def err_row(tag, e):
        print(f"{tag:<22}" + "".join(f"{mm(v):8.1f}" for v in e)
              + f"  | {mm(e[fingers].mean()):5.1f}{mm(e[thumb].mean()):6.1f}{mm(e.mean()):6.1f}")

    for name in CONFIGS:
        err_row(name + " clean", tip_err_clean[name].mean(axis=0))
    for mode in MODES:
        print()
        for name in CONFIGS:
            err_row(f"{name} occ:{mode}", tip_err_occ[mode][name][w0:w1].mean(axis=0))

    print("\n===== JITTER (mean |dq| per frame, rad; lower = smoother) =====")
    hdr2 = "method".ljust(22) + "".join(f"{l:>11}" for l in ACT_LABEL) + "     mean"
    print(hdr2); print("-" * len(hdr2))
    for name in CONFIGS:
        jc = jit_clean[name][act]
        print(f"{name + ' clean':<22}" + "".join(f"{v:11.4f}" for v in jc) + f"  {jc.mean():8.4f}")
    for mode in MODES:
        for name in CONFIGS:
            jo = jit_occ_w[mode][name][act]
            print(f"{name + ' occ:' + mode:<22}" + "".join(f"{v:11.4f}" for v in jo) + f"  {jo.mean():8.4f}")

    print("\n===== AGGREGATE (x = ratio vs A_tip; >1 worse, <1 better) =====")
    for name in CONFIGS:
        ec = tip_err_clean[name].mean()
        line = (f"{name:<20} clean tip={mm(ec):5.1f}mm (x{ec / tip_err_clean['A_tip'].mean():.2f}) "
                f"jit={jit_clean[name][act].mean():.4f}")
        for mode in MODES:
            eo = tip_err_occ[mode][name][w0:w1].mean()
            base = tip_err_occ[mode]['A_tip'][w0:w1].mean()
            line += (f"  || {mode}: tip={mm(eo):5.1f}mm (x{eo / base:.2f}) "
                     f"jit={jit_occ_w[mode][name][act].mean():.4f}")
        print(line)

    # ---------------- plots ----------------
    t = np.arange(n)
    colors = {"A_tip": "C0", "B_tip+PIP": "C1", "B2_tip+fingerPIP": "C2"}

    # fig1: joint curves (clean)
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    for k, ax in enumerate(axes.flat):
        for name in CONFIGS:
            ax.plot(t, q_clean[name][:, act[k]], label=name, lw=1.1, color=colors[name], alpha=0.85)
        ax.set_title(ACT_LABEL[k]); ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(fontsize=7)
    fig.suptitle("Actuated joint angles over time (clean input)")
    fig.supxlabel("frame"); fig.supylabel("angle (rad)")
    fig.tight_layout(); fig.savefig(HERE / "fig1_joint_curves.png", dpi=110); plt.close(fig)

    # fig2: tip error over time, one panel per corruption mode
    fig, axs = plt.subplots(1, len(MODES), figsize=(9 * len(MODES), 5.5), squeeze=False)
    for ax, mode in zip(axs[0], MODES):
        ax.axvspan(w0, w1, color="orange", alpha=0.12, label="tips corrupted")
        for name in CONFIGS:
            ax.plot(t, mm(tip_err_clean[name].mean(axis=1)), color=colors[name], lw=1.2, label=f"{name} clean")
            ax.plot(t, mm(tip_err_occ[mode][name].mean(axis=1)), color=colors[name], lw=1.1, ls="--", alpha=0.85, label=f"{name} occ")
        ax.set_title(f"corruption = {mode}"); ax.set_xlabel("frame"); ax.set_ylabel("mean fingertip error (mm)")
        ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=3)
    fig.suptitle("Fingertip error vs TRUE human target (dashed=occluded). Lower is better.")
    fig.tight_layout(); fig.savefig(HERE / "fig2_tip_error.png", dpi=110); plt.close(fig)

    # fig3: summary bars — accuracy (clean + each mode) and jitter
    names_l = list(CONFIGS.keys()); x = np.arange(len(names_l))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 5))
    series = [("clean", [mm(tip_err_clean[k].mean()) for k in names_l])]
    for mode in MODES:
        series.append((f"occ:{mode}", [mm(tip_err_occ[mode][k][w0:w1].mean()) for k in names_l]))
    bw = 0.8 / len(series)
    for i, (lbl, vals) in enumerate(series):
        a1.bar(x + (i - (len(series) - 1) / 2) * bw, vals, bw, label=lbl)
    a1.set_xticks(x); a1.set_xticklabels(names_l, fontsize=8); a1.set_ylabel("tip error, all fingers (mm)")
    a1.set_title("Accuracy: clean vs occluded (lower=better)"); a1.legend(fontsize=8); a1.grid(alpha=0.3, axis="y")

    jseries = [("clean", [jit_clean[k][act].mean() for k in names_l])]
    for mode in MODES:
        jseries.append((f"occ:{mode}", [jit_occ_w[mode][k][act].mean() for k in names_l]))
    for i, (lbl, vals) in enumerate(jseries):
        a2.bar(x + (i - (len(jseries) - 1) / 2) * bw, vals, bw, label=lbl)
    a2.set_xticks(x); a2.set_xticklabels(names_l, fontsize=8); a2.set_ylabel("jitter, mean |dq|/frame (rad)")
    a2.set_title("Smoothness (lower=smoother)"); a2.legend(fontsize=8); a2.grid(alpha=0.3, axis="y")
    fig.tight_layout(); fig.savefig(HERE / "fig3_summary.png", dpi=110); plt.close(fig)

    with (HERE / "results.pkl").open("wb") as f:
        pickle.dump(dict(q_clean=q_clean, q_occ_last=q_occ_last, names=names, act=act,
                         tip_err_clean=tip_err_clean, tip_err_occ=tip_err_occ,
                         jit_clean=jit_clean, jit_occ_w=jit_occ_w,
                         window=(w0, w1), sigma=SIGMA, nseed=NSEED, fresh=fresh), f)
    print("\n[5/5] saved results.pkl, fig1_joint_curves.png, fig2_tip_error.png, fig3_summary.png")


if __name__ == "__main__":
    main()
