# pr2_controller

PR2 在 MuJoCo 中与 **ROS 2 Jazzy** 桥接：发布关节状态与里程计，通过话题控制执行器。

## 依赖

- ROS 2 Jazzy（`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `tf2_ros`）
- Python 包：`pip install mujoco numpy`（MuJoCo 通常不在 `rosdep` 中）

## 编译

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_controller
source install/setup.bash
```

## 运行

```bash
ros2 run pr2_controller pr2_mujoco_sim
# 或
ros2 launch pr2_controller pr2_mujoco_sim.launch.py
```

默认 **`demo_motion:=true`**：与旧版 `scripts/pr2_sim.py` 相同，会自动做夹爪开合、左臂轮换、底盘前后/侧移演示。若**只**想通过话题控制、不要内置动作，请加：

`--ros-args -p demo_motion:=false`

无头仿真（无窗口）：

```bash
ros2 run pr2_controller pr2_mujoco_sim --ros-args -p use_viewer:=false
```

### 故障排除：`Failed to open display` / `could not initialize GLFW`

说明当前环境没有可用的 X11 图形（常见于 Docker、SSH 无 `-X`、或 `DISPLAY` 指向无权限的显示）。

1. **直接无头运行**（推荐）：`--ros-args -p use_viewer:=false`
2. **未设置 `DISPLAY` 时**：节点已默认 `use_viewer:=false`，一般不会再去连 GLFW。
3. **若 `DISPLAY=:1` 但仍报错**：代码会尝试开窗口失败后**自动切换到无头**；若进程仍被 MuJoCo 直接 `exit`，请显式加上 `-p use_viewer:=false`。
4. **本机有桌面、要在容器里看窗口**：需正确挂载 X11 socket、设置 `DISPLAY`，并在宿主机执行 `xhost +local:`（或按需配置 `xauth`）。

## 话题说明

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `joint_states` | `sensor_msgs/JointState` | 非 floating 关节的位姿/速度 |
| 发布 | `odom` | `nav_msgs/Odometry` | `base_link` 在 `odom` 下的位姿（速度由数值差分近似） |
| 发布 | TF | `tf2` | `odom` → `base_link` |
| 订阅 | `joint_commands` | `sensor_msgs/JointState` | 按**关节名**写控制：`*_pos` 用 `position`，`*_vel` 用 `velocity`，`*_tau` 用 `effort`；`NaN` 跳过 |
| 订阅 | `actuator_command` | `std_msgs/Float64MultiArray` | 长度须为 `nu`（32），直接对应 MuJoCo `ctrl`；会**覆盖**分项命令直至调用下方话题解除 |
| 订阅 | `disable_actuator_override` | `std_msgs/Bool` | 发 `data: true` 恢复 `joint_commands` / `cmd_vel` 与默认躯干力 |
| 订阅 | `cmd_vel` | `geometry_msgs/Twist` | 近似全向底盘：线速度在地面平面 + `angular.z`；覆盖执行器索引 5–16 |

默认对 `torso_lift_tau` 施加 `torso_hold_effort`（默认 500），防止躯干下落（在**未**使用全向量 `actuator_command` 时）。

## 与旧脚本的关系

原先 `scripts/pr2_sim.py` 在进程内直接写 `data.ctrl`，**不经过 ROS**。新节点在仿真循环中调用 `rclpy.spin_once`，用话题驱动同一 MuJoCo 模型。

## 示例：左臂肩抬关节力矩

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
  "{name: ['l_shoulder_lift_joint'], position: [], velocity: [], effort: [20.0]}"
```

（该关节对应执行器 `l_shoulder_lift_tau`，使用 `effort` 字段。）
