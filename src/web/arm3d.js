// arm3d.js — 浏览器端 NERO 7 轴臂 3D 查看器。通用部分在 urdf_view.js。
//
// 和手的差别就两点:7 个关节、**没有 mimic**(臂的关节彼此独立)。所以这里薄得多。
//
// 用的 URDF 是 assets/viz/arm/nero_arm_viz.urdf —— build_arm_viz.py 生成的:
// 原始 nero_description.urdf 的 visual 是 Collada(GLTFLoader 读不了,且 link2.dae
// 有 24MB),转成 glb 后 40.6MB → 7.0MB。别直接加载 assets/nero_description 那份。
//
// ⚠ 臂的 URDF 里 7 个关节有 5 个 rpy 两轴同时非零(joint2..joint6),Euler order
// 必须是 'ZYX'(在 urdf_view.applyOrigin 里)。用 three.js 默认的 'XYZ' 整条臂都歪。

import { UrdfViewer } from "./urdf_view.js";

const ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4",
                    "joint5", "joint6", "joint7"];

export class ArmViewer extends UrdfViewer {
  constructor(container) {
    // 臂的工作空间比手大一个量级,网格给 1.2m;mesh 是 STL 转的,给个稍冷的灰。
    super(container, { gridSize: 1.2, meshColor: 0xa8adb6 });
  }

  /** 用 7 个关节角(rad,joint1..joint7 顺序)刷新姿态。 */
  setJointArray(rad7) {
    if (!this.ready || !rad7) return;
    const q = {};
    ARM_JOINTS.forEach((n, i) => { if (rad7[i] != null) q[n] = rad7[i]; });
    this.setJoints(q);
  }

  /** URDF 里读到的限位,给前端生成滑块用(和 nero_arm.NERO_ARM_LIMITS 对照)。 */
  limits() {
    return ARM_JOINTS.map(n => {
      const j = this.joints[n];
      return j ? [j.lower, j.upper] : [-3.14159, 3.14159];
    });
  }
}
