# 部署到真机主机

> 本文部署的是完整 Web/ROS2 开发栈。正式 ROS2 Hardware Driver 位于独立
> `VLA-HandArm-Ros/nero_inspire_hardware`；它与
> `robot-mcp-server/robot-bridge/bridge.py` 的 MCP HTTP Bridge 不是同一进程。

## 为什么整套跑在机械臂那台

`app_web.py` 是**编排器**,不 import rclpy、不发 ROS 消息 —— 它用 `subprocess` 拉起本机的
`ros_joint_writer.py` / `ros_joint_reader.py` / `ros_web_hardware.py` / `skills/runner.py`,再由 Hardware Driver
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
│        runner / writer / reader / Web│◄─────│ Remote-SSH   │
│                           │ ROS2   │ SSH  └──────────────┘
│           nero_hardware_driver       │
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

## 日常统一启动

完成一次性部署后，日常不需要逐条输入 source、launch 和 Python 命令：

```bash
cd ~/ros2_ws/lerobotTest
./start_robot.sh
```

菜单选择 `1. ROS2`、`2. Direct` 或 `3. Mock`。ROS2 会检查 USB 串口和 CAN、加载
`deploy/nero_hardware_env.sh`、启动或复用 Hardware Driver，并等待臂手 READY；随后仍会
先显式失能，再单独询问是否使能机械臂。Direct 会拒绝已运行的 Driver 和启动前已被占用的
串口；菜单也允许同时启动 Web 与 Bridge，但 Bridge 启动后会持有硬件，Web 再点击“接入”
可能产生 CAN/串口竞争。Mock 不访问真机。`Ctrl+C` 会统一停止本次创建的进程，日志位于
`.runtime/launcher/<timestamp>/`；如果机械臂由本次启动器使能，会先调用失能 Service。

需要固定本机参数时：

```bash
cp .nero_runtime.env.example .nero_runtime.env
```

修改其中的稳定串口、CAN、端口或 Python 路径即可；该文件不入 Git。也可用
`./start_robot.sh --help` 查看非交互参数。

## 上真机顺序

别跳步。mock 能把整条链路和网页验完,`--dry-run` 能验安全闸,这两步不花钱也不会撞。

```bash
source ~/ros2_ws/lerobotTest/deploy/nero_hardware_env.sh
conda activate lerobot-v3
python src/ros_humble_env.py --run src/skills/runner.py --dry-run --once '{"skill_id":"go_home","confirmed":true,"assume_enabled":true}'
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nero_inspire_hardware hardware.launch.py enable_control:=true          # 假数据,不碰硬件
candump $CAN_IFACE                                       # 确认臂在 CAN 上有回包
ros2 launch nero_inspire_hardware hardware.launch.py \
  arm_mock:=false hand_mock:=false hand_port:=/dev/inspire_hand                    # 只读真机
ros2 launch nero_inspire_hardware hardware.launch.py \
  arm_mock:=false hand_mock:=false enable_control:=true hand_port:=/dev/inspire_hand
# 确认 /nero/driver_state 中 arm/hand 均 READY 后，再显式使能
ros2 service call /nero/arm/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Driver 会在 usbipd/USB/CAN 断开后进入 `FAULT` 并退避重连。重连不会恢复旧运动命令，也不会
自动使能机械臂；检查 `/nero/driver_state` 和 `/diagnostics` 后人工重新使能。Driver 运行期间
不要启动 `arm_console.py`、`hand_console.py` 或其他直接打开同一硬件的进程。
默认 Backend 下，Web 页面选择真机（`mock=false`）时只启动 ROS worker，不再持有设备；
本地 mock 仍使用 Console。CPV 实时跟随和 Web 联合包 keyframe 回放走 Driver 的三段式
Service；联合包会等待 worker 的 prepare ACK，完成 approach 后才启动。正式轨迹 Action
尚未进入 Driver，逐通道手力控和 clear-error 也会明确报错，不会静默回退到直连。

Web 默认使用 `WEB_HARDWARE_BACKEND=ros`。需要临时回退到旧直连链路时，先停止
Hardware Driver，确认没有其他进程占用 CAN/串口，再启动：

```bash
WEB_HARDWARE_BACKEND=direct python src/lerobot_v3/app_web.py
```

不要在 Hardware Driver 仍运行时使用 Direct。切回 ROS 时先断开 Web 的 arm/hand 会话，
停止 Direct Web，再正常启动 Driver 和默认 Web。页面 API、技能包、组合动作、视频跟随与
MCP 契约不因 Web Backend 改变。

## 安全

7860 端口**没有鉴权**,同一路由下任何人打开就能下发关节指令。共用网络的话先加一层:
nginx basic auth、只绑内网段、或 ufw 限源 IP。
