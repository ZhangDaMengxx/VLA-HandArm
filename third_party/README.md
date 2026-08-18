# 第三方资产

本目录统一保存项目依赖的上游源码、厂商 SDK 和外部数据资产。除
`overlays/` 外，这些目录不进入 Git；迁移或升级时应保留上游目录结构，并记录来源和版本。

| 目录 | 内容 | Git 策略 |
|------|------|----------|
| `dex-retargeting/` | dex-retargeting 上游源码 | 本地资产，不入库 |
| `dex-urdf/` | dex-urdf 上游资产 | 本地资产，不入库 |
| `egozero/` | EgoZero 上游源码 | 本地资产，不入库 |
| `kinect2-middle/` | RGB-D 第三方数据与工具 | 本地资产，不入库 |
| `pinocchio-kinematics-lite/` | 运动学参考实现 | 本地资产，不入库 |
| `pyAgxArm/` | NERO 厂商 Python SDK | 真机依赖，不入库 |
| `unitree-sdk2/` | Unitree SDK 上游源码 | 本地资产，不入库 |
| `overlays/` | 本项目对上游文件的补丁和扩展 | 项目维护，进入 Git |

代码引用第三方资产时应从仓库根目录下的 `third_party/` 定位，不得写死个人主目录。
