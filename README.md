# ROS 2 无人机键盘控制

基于本仓库的终端键盘节点，适配 ROS 2 Humble。速度通过独立消息转换 Bridge 接入，解锁和模式切换直接调用 MAVROS 兼容服务。运行时不需要仿真器源码。

## 操作

| 按键 | 功能 |
| --- | --- |
| W / S | ENU 东 / 西 |
| A / D | ENU 北 / 南 |
| Q / E | 上升 / 下降 |
| Z / C | 正 / 负偏航 |
| T | 请求解锁 |
| O | 请求 OFFBOARD |
| X | 请求上锁 |
| 空格 | 目标速度归零，按减速度限制停车 |
| + / - | 增大 / 减小线速度和偏航速度 |
| Ctrl+C | 发送零速度、恢复终端并退出，保持原有解锁状态 |

大小写均可，方向采用固定世界 ENU，不随飞机或镜头旋转。终端通过按键重复和超时判断是否仍按住，不支持真实的按键释放事件或任意多键同时按下。

节点一启动就持续发布零速度。先等待服务就绪，按 T 解锁，再按 O 切入 OFFBOARD，然后按 Q 升高。解锁和切模式不会自动起飞；状态行持续显示实际模式、解锁状态、当前速度和服务响应。

## 接口和参数

所有名称均为相对名称，通过节点命名空间选择车辆。

| 接口 | 默认名称 | 类型 |
| --- | --- | --- |
| 速度输出 | `control/manual` | `trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` |
| 飞机状态订阅 | `mavros/state` | `mavros_msgs/msg/State` |
| 解锁/上锁服务 | `mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |
| 模式服务 | `mavros/set_mode` | `mavros_msgs/srv/SetMode` |

输出只有一个 `velocities` 元素，包含 `linear.x/y/z` 和 `angular.z`。不填写位置和加速度，不执行轨迹跟踪。Bridge 将其转换为 MAVROS 速度设定点。

启动参数见 `config/keyboard.yaml`：

- `speed`：初始线速度，默认 0.5 m/s；`turn`：初始偏航角速度，默认 1 rad/s。
- `publish_rate`：默认 20 Hz，在无按键和服务请求期间也持续发布。
- `key_timeout`：默认 0.6 秒，运动按键停止重复后目标速度归零。
- `linear_acceleration`：各线速度轴的加/减速度限制，默认 1 m/s²。
- `angular_acceleration`：偏航加/减速度限制，默认 2 rad/s²。
- `output_topic`、`state_topic`、`arming_service`、`mode_service`：接口名称。

参数在启动时读取，修改配置后重新启动节点。发布定时器和按键超时都使用单调时钟，不依赖 `/clock`；仿真暂停或变慢时，键盘仍按墙钟 20 Hz 发布并刷新终端状态。输出没有 Header，转换后的时间戳由 Bridge 生成。

## 容器部署

工作区中的 `docker/Dockerfile.keyboard` 当前编译本地 `src/ros2_teleop_keyboard` 源码。该源码来自 `https://github.com/LHiou/ros2_teleop_keyboard.git` 的 `master` 分支。

在工作区根目录运行：

```bash
docker compose build --no-cache keyboard
docker compose up -d --build simulator bridge
docker compose run --rm keyboard
```

键盘使用独立终端，仿真日志不会混入。普通 `docker compose up -d` 不会后台启动键盘服务。Ctrl+C 只退出键盘容器，仿真器和 Bridge 继续运行。

多机时，Bridge 与键盘需选择同一架飞机：

```bash
UAV_NAMESPACE=uav2 docker compose -p uav2 up -d bridge
UAV_NAMESPACE=uav2 docker compose run --rm keyboard
```

仿真场景需要已经包含 `uav2`。所有容器的 `ROS_DOMAIN_ID` 保持一致。

## 在 ROS 2 工作区运行

```bash
colcon build --packages-select teleop_twist_keyboard
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  --params-file install/teleop_twist_keyboard/share/teleop_twist_keyboard/config/keyboard.yaml \
  -r __ns:=/uav1
```

## 文件作用

| 文件 | 作用 |
| --- | --- |
| `teleop_twist_keyboard.py` | 按键读取、速度平滑、固定频率发布、状态显示及异步服务调用 |
| `config/keyboard.yaml` | 控制参数与话题/服务名称 |
| `package.xml` | ROS 包和运行依赖声明 |
| `setup.py` | Python 模块、可执行入口和配置文件安装 |
| `setup.cfg` | 将脚本安装到 ROS 2 的 `lib/teleop_twist_keyboard` 目录 |
| `resource/teleop_twist_keyboard` | ament 包索引标记，用于 `ros2 run` 查找包 |
