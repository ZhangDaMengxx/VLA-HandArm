# 部署到真机主机

> 本文部署的是本仓完整 Web/ROS2 开发栈。这里的 `nero_arm_bridge.py` 是 ROS2 硬件桥，
> 与独立 `robot-mcp-server/robot-bridge/bridge.py` 的 MCP HTTP Bridge 不是同一进程。

## 为什么整套跑在机械臂那台

`app_web.py` 是**编排器**,不 import rclpy、不发 ROS 消息 —— 它用 `subprocess` 拉起本机的
`ros_joint_writer.py` / `ros_joint_reader.py` / `skills/runner.py`,再由 `nero_arm_bridge.py`
经 CAN/RS485 驱动硬件。这些 `Popen` 走的全是本机绝对路径,所以 **web、runner、bridge 必须同机**,
而 bridge 必须在插着线的那台。开发机只需浏览器。

主进程使用 `lerobot-v3`（Python 3.12）；上述 rclpy 子进程由 Web 自动加载 Humble，并交给
`ros-humble`（Python 3.10）。跨边界的只有小型 JSON/ROS 关节目标和状态，RGB-D、关键点
推理和 IK 都留在 V3 进程，不会因环境拆分增加图像通信压力。

```
机械臂主机                                    开发机
┌────────────────────────────────────┐      ┌──────────────┐
│ app_web.py :7860 ──Popen──┐        │◄─────│ 浏览器        │
│                           ▼        │ HTTP │ VS Code      │
│              runner / writer /reader│◄─────│ Remote-SSH   │
│                           │ ROS2   │ SSH  └──────────────┘
│              nero_arm_bridge        │
│                    │ CAN / RS485    │
│                  臂 + 灵巧手         │
└────────────────────────────────────┘
```

指令与急停都在真机侧闭环,开发机断网不影响正在执行的动作。

## 步骤

```bash
# 1. 两个仓库(nero_inspire_ros2 是独立仓库,主仓不带它)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone git@github.com:ZhangDaMengxx/VLA-HandArm.git lerobotTest
git clone git@github.com:ZhangDaMengxx/VLA-HandArm-Ros.git src/nero_inspire_ros2

# 2. git 给不了的:厂商 SDK(.gitignore 排除第三方)
mkdir -p ~/ros2_ws/lerobotTest/third_party
rsync -av <开发机>:~/ros2_ws/lerobotTest/third_party/pyAgxArm/ ~/ros2_ws/lerobotTest/third_party/pyAgxArm/

# 3. 分步部署(出错好定位)
cd ~/ros2_ws/lerobotTest
bash deploy/deploy_robot_host.sh --check    # 只体检
bash deploy/deploy_robot_host.sh --env      # V3 主环境 + ROS 薄环境 + 重建 URDF
bash deploy/deploy_robot_host.sh --ros      # colcon build
bash deploy/deploy_robot_host.sh --hw       # CAN/udev/dialout(需 sudo)
bash deploy/deploy_robot_host.sh --verify   # mock 链路 + 安全闸
```

`--hw` 之后**必须登出重进**,dialout 组权限才生效。

## 几个必须知道的坑

**只有 `ros-humble` 必须 Python 3.10。** `rclpy` 的 `.so` 是给 cp310 编的；主环境
`lerobot-v3` 固定 Python 3.12，不能直接 import rclpy，也不需要 import。环境分流由
`src/ros_humble_env.py` 完成。

**`install/` 不能从开发机拷**,里面是编译产物加绝对路径,必须本机 `colcon build`。

**装配 URDF 不入库。** `build_nero_inspire.py` 把 mesh 写成绝对路径(MuJoCo 要求),换机必须重建。
`--env` 会自动跑,只用 stdlib,不需要 conda。

**串口固定别名。** 插拔顺序一变 `ttyUSB0/1` 就对调,那时"代码没错但读的是另一个设备"。
`--hw` 会按 VID/PID 生成 udev 规则做成 `/dev/inspire_hand`;若臂的 USB-CAN 同款芯片会撞,
按规则文件里的注释改用 `ATTRS{serial}` 区分。

**技能轨迹 npz** 虽在 `src/out/`(生成物目录)里,但已用 `!src/out/robot_traj_*.npz` 破例入库,
clone 就能跑 trajectory 类技能。其余 `src/out/` 产物仍不入库。

## 上真机顺序

别跳步。mock 能把整条链路和网页验完,`--dry-run` 能验安全闸,这两步不花钱也不会撞。

```bash
source ~/ros2_ws/robot_host_env.sh
conda activate lerobot-v3
python src/ros_humble_env.py --run src/skills/runner.py --dry-run --once '{"skill_id":"go_home","confirmed":true,"assume_enabled":true}'
python src/ros_humble_env.py --run src/nero_arm_bridge.py --mock --enable-control   # 假数据,不碰硬件
candump $CAN_IFACE                                       # 确认臂在 CAN 上有回包
python src/ros_humble_env.py --run src/nero_arm_bridge.py --no-mock                 # 只读真机
python src/ros_humble_env.py --run src/nero_arm_bridge.py --no-mock --enable-control # 确认角度对了再放开控制
```

## 安全

7860 端口**没有鉴权**,同一路由下任何人打开就能下发关节指令。共用网络的话先加一层:
nginx basic auth、只绑内网段、或 ufw 限源 IP。
