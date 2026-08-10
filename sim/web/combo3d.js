// combo3d.js — 合体页(实时 Live)的 3D:一条链上同时驱动 7 臂 + 6 手。
//
// 复用 urdf_view.js 的解析/相机/'ZYX' Euler order,和 arm3d.js / hand3d.js 同一个基类。
// 这里只多两件事:
//   1. 关节分两组(臂 7 + 手 6),要能**各自单独**刷新 —— 合体页允许只接一边,
//      只接臂时手的姿态该停在最后设定值,不能被一次全量 setJointArray 冲成 0。
//   2. 手的 mimic 从 hand3d.js **import**,不抄第三份。
//
// ⚠ mesh 是 9.3MB(19 个 glb:臂 10 + 手 9),比单独看臂的 7MB 大。加载慢是正常的,
// 别当成卡死 —— ensureCombo3d() 那边会显示加载提示。

import { UrdfViewer } from "./urdf_view.js";
import { handJointMap, DRIVEN as HAND_DRIVEN } from "./hand3d.js";

// 臂关节名。和 nero_arm.py 的顺序一致(joint1..joint7)。
export const ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4",
                           "joint5", "joint6", "joint7"];

export class ComboViewer extends UrdfViewer {
  constructor(container) {
    // gridSize 1.2m:装配体零位高约 0.98m,手页那个 0.4m 的网格在这里只有巴掌大,
    // 完全失去"地面在哪"的参照作用。
    super(container, { gridSize: 1.2 });
  }

  /** 只刷臂的 7 个关节,手不动。 */
  setArm(rad7) {
    if (!this.ready || !rad7) return;
    const q = {};
    ARM_JOINTS.forEach((n, i) => { if (rad7[i] != null) q[n] = rad7[i]; });
    this.setJoints(q);
  }

  /** 只刷手的 6 个驱动关节(mimic 自动补),臂不动。 */
  setHand(rad6) {
    if (!this.ready || !rad6) return;
    this.setJoints(handJointMap(rad6));
  }

  /** 一次刷 13 个:[0:7] 臂,[7:13] 手。state(13) 的布局,和管线一致。 */
  setAll(rad13) {
    if (!this.ready || !rad13 || rad13.length < 13) return;
    this.setArm(rad13.slice(0, 7));
    this.setHand(rad13.slice(7, 13));
  }

  /** 自检用:URDF 里这 13 个关节是不是都找到了。缺一个就说明装配 URDF 不对。 */
  missingJoints() {
    return [...ARM_JOINTS, ...HAND_DRIVEN].filter(n => !this.joints[n]);
  }
}
