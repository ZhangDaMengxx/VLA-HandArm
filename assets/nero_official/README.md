# nero_official

AgileX 官方 NERO URDF/mesh,取自 <https://github.com/agilexrobotics/agx_arm_urdf> 的 `nero/` 目录
(main 分支,2026-07-28 取)。用途:给 `sim/assets/nero_gripper_right.urdf` 提供 CAD 真值,
替掉原先按网格反推的估计值(装配偏移、行程、质量/惯量)。

## 只归档了差异部分

`nero/` 整目录 57.5MB,其中臂的 mesh 与本仓库 `assets/nero_old/meshes/` 内容相同,不重复存:

| 文件 | 状态 |
|---|---|
| `gripper_link1.stl` / `gripper_link2.stl` | 与 `nero_old/` 逐字节相同(md5 `c9b94bd5…`),未归档 |
| `gripper_base.stl` / `dae/gripper_base.dae` | 与 `nero_old/` 同尺寸,未归档 |
| `gripper_flange.stl` / `dae/gripper_flange.dae` | **有差异,已归档** |

`gripper_flange` 的差异:官方 13310 三角形、尺寸对称(x ±0.02200、y ±0.02750);
`nero_old/` 那份 13302 三角形、x [-0.02225,+0.02188]、y [-0.02741,+0.02793] 歪斜,
是重导出退化过的版本。URDF 里用官方这份。

## 官方结构与本仓库的差异

- 官方把末端拆成 `gripper_flange`(挂 link7)+ `gripper_base`(挂 flange,z 偏移 0.006);
  本仓库 `nero_old` 另有一个合并网格 `gripper_base_with_flange`。现已按官方拆分法重写。
- 官方夹爪用 mimic:主关节 `gripper` 取值 0→0.1 = **开口总量**,两指
  `mimic multiplier=±0.5`。本仓库保留两个独立 prismatic(pinocchio 不支持 mimic),
  由同一标量驱动,该标量是**单指行程** 0→0.05。真机联调注意这 2 倍量纲差。
- 官方无 `link8`;本仓库 `link8` 仅用于 inspire 手装配(`sim/build_nero_inspire.py`),与夹爪无关。
