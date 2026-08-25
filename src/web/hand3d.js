// hand3d.js — 浏览器端灵巧手 3D 查看器。通用部分在 urdf_view.js,这里只留手特有的:
// 6 个驱动关节 + mimic 耦合。
//
// 关节耦合(mimic)必须自己补:URDF 里 6 个远端关节是 mimic 驱动关节的,浏览器端
// 没有 ros2_control 帮忙算,不补的话手指第二节永远不弯。
// ⚠ 这和 src/hand_rerun.py 的 MIMIC 表是同一份数据,改一处要改两处。

import { UrdfViewer } from "./urdf_view.js";

// 和 src/hand_rerun.py 的 MIMIC 保持一致。
// ⚠ combo3d.js 也要用这两张表(合体页的手是同一只手),所以 export 出去而**不是**
// 让它再抄一份。已经有 hand_rerun.py 和这里两份要同步了,第三份必然漏。
// ⚠ 2026-08-10 更新:key 改为新 URDF 的实际关节名(right_thumb_3_joint 等)。
// 新 URDF 里 right_thumb_4 是**链式** mimic(mimic right_thumb_3 ×0.7508),
// 这张表是扁平的(一律引用驱动关节),所以展平:0.7508 × 1.1425 = 0.857789。
export const MIMIC = {
  right_thumb_3_joint:  ["right_thumb_2_joint", 1.1425, 0.0],
  right_thumb_4_joint:  ["right_thumb_2_joint", 0.857789, 0.0],
  right_index_2_joint:  ["right_index_1_joint",  1.1169, 0.0],
  right_middle_2_joint: ["right_middle_1_joint", 1.1169, 0.0],
  right_ring_2_joint:   ["right_ring_1_joint",   1.1169, 0.0],
  right_little_2_joint: ["right_little_1_joint",  1.1169, 0.0],
};

export const DRIVEN = ["right_thumb_1_joint", "right_thumb_2_joint",
                       "right_index_1_joint", "right_middle_1_joint",
                       "right_ring_1_joint", "right_little_1_joint"];

/** 6 个驱动角(项目顺序)→ {关节名: rad},mimic 已补算。合体页复用这段。 */
export function handJointMap(rad6) {
  const q = {};
  DRIVEN.forEach((n, i) => { if (rad6[i] != null) q[n] = rad6[i]; });
  for (const [name, [driver, mult, off]] of Object.entries(MIMIC)) {
    if (q[driver] != null) q[name] = q[driver] * mult + off;
  }
  return q;
}

export class HandViewer extends UrdfViewer {
  constructor(container) {
    super(container, { gridSize: 0.4, minFrameAspect: 0.60 });
  }

  /** 用 6 个驱动关节角(rad,项目顺序)刷新姿态,mimic 在这里补算。 */
  setDriven(rad6) {
    if (!this.ready || !rad6) return;
    this.setJoints(handJointMap(rad6));
  }
}
