# Camera / 标定工具

这里存放相机设备适配和坐标标定代码，不直接控制机械臂。

## 臂上相机手眼标定

`calibrate_handeye.py` 当前使用 OpenCV 相机适配器，求解固定棋盘格、相机安装在
机械臂末端的 eye-in-hand 问题。它只读取 NERO 关节角并用当前 URDF FK 计算
`T_base_ee`，绝不会发送运动指令。

```bash
python3 src/camera/calibrate_handeye.py \
  --intrinsics /path/to/orbbec_intrinsics.json \
  --camera-model orbbec_336 \
  --camera-index 0 \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --arm-urdf assets/arm/urdf/nero_with_hand_flange_description.urdf \
  --ee-frame link8 \
  --no-mock \
  --output datasets/camera_calibration/orbbec336_handeye.json
```

棋盘格参数是“内角点”数量，不是黑白格子数量；`square-size-m` 是实际小格边长。
程序窗口持续显示检测状态，终端输入：

```text
next    采集当前姿态
undo    撤销最后一组样本
finish  样本足够后求解并保存
quit    退出且不保存
```

建议 15--25 个姿态，必须同时改变位置和朝向，不能只平移。默认只读机械臂；真实硬件
需要显式 `--no-mock`，并且仍由操作者使用现有安全控制方式移动机械臂。

`T_gripper_camera` 中的 `gripper` 就是 `--ee-frame` 指定的 URDF frame。默认基础臂模型
使用 `link7`；若相机安装参照实体末端法兰，使用示例中的带法兰 URDF 和 `link8`，输出
即为 `T_link8_camera`。不要把 `link7` 的结果改名冒充 `T_link8_camera`；二者之间还有
URDF 中已知的固定变换。

每次成功 `next` 都会保留原始棋盘格图像；最终 JSON 同时保存全部输入姿态、质量统计、
完整 `T_gripper_camera` 4x4 矩阵和本仓库约定的 `quaternion_xyzw`。所谓“最小误差”在
有限样本中没有可预知的全局终点，工具因此采用可配置的暂定阈值：默认至少 12 组、固定
标定板平移一致性 RMSE 不超过 3 mm、旋转一致性 RMSE 不超过 0.5 度。达到阈值后会提示
输入 `finish`；不会因为某一组偶然低误差而自动结束。

这两个 RMSE 衡量的是“固定棋盘格变换到 `robot_base` 后是否保持不动”的内部一致性，
并不是有外部 Ground Truth 的绝对外参误差。报告会逐样本保存残差并提示综合残差最大的
样本；它能发现坏样本和退化运动，但物理验收还需要保留姿态复投、独立测试点或外部真值。

## 奥比中光 336

336 的 SDK 帧适配尚未写死，因为具体型号可能是 Gemini/Femto 变体。接入时只需实现
一个返回 BGR 帧的 source，并保留厂商内参/畸变 JSON；手眼求解器不使用深度流。深度
流用于后续 RGB-D Source 和三维质量验证，不能替代棋盘格 RGB 角点检测。
