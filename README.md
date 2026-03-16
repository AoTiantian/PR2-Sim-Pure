# PR2-Sim-Pure

PR2-Sim-Pure 是抽取出的 **PR2 专用最小仿真栈**，仅保留 MuJoCo + ROS 2 控制闭环所需组件。

目标：
- 保留完整 PR2 仿真能力（模型、控制、状态发布、底盘控制）
- 删除与 PR2 仿真无关的多平台耦合代码（Moying/Unitree 其他机型/EtherCAT 实机链路）
- 修改代码错误，便于二次开发与维护

## 仓库说明

- **项目类型**：PR2 移动操作机器人仿真工程
- **技术栈**：MuJoCo、ROS 2 Jazzy、ros2_control、DDS(Unitree SDK2 通道层)
- **定位**：研究/教学导向的轻量仿真基线
- **适用场景**：PR2 运动控制验证、算法集成、仿真实验复现

建议 GitHub 仓库简介（About）可设置为：

`Minimal PR2 simulation stack (MuJoCo + ROS2) extracted from Human-Robot-Cooperation.`

建议 Topics：

`pr2, mujoco, ros2, ros2-control, simulation, robotics`

## 提取后保留的核心模块

```text
PR2-Sim-Pure/
├── pr2_ws/
│   └── src/pr2_controller/
│       ├── pr2_bringup            # 启动与控制器配置（最小）
│       ├── pr2_description        # URDF/Xacro + meshes
│       ├── pr2_mobile_controller  # 全向底盘控制器（/pr2/mecanum_controller/cmd_vel）
│       ├── pr2_mujoco_hardware    # ros2_control hardware plugin（pr2/cmd <-> pr2/state）
│       └── pr2_mujoco_publisher   # TF/位姿/力传感话题发布
└── unitree_mujoco/
    ├── simulate                    # PR2-only MuJoCo 仿真程序
    └── unitree_robots/pr2          # PR2 MJCF 场景与模型资源
```

## 已移除内容（精简说明）

- Moying/MCR 工作区与通信链路
- Unitree go2/h1/g1 等非 PR2 桥接逻辑
- wholebody、ethercat、实机驱动相关包与依赖
- PR2 仿真闭环不需要的冗余控制器配置

## 系统依赖

> 下面是最小依赖建议，按需调整。

### 1) 基础依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libyaml-cpp-dev libglfw3-dev libboost-all-dev libspdlog-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-rclcpp ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### 2) unitree_sdk2

建议安装到 `/opt/unitree_robotics`：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### 3) MuJoCo

下载官方 release 并解压后，在 `unitree_mujoco/simulate` 下创建软链接：

```bash
cd unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

## 编译与运行

### 步骤 A：启动 MuJoCo 仿真器

```bash
cd unitree_mujoco/simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./unitree_mujoco
```

默认读取 `../config.yaml`，当前配置为 `robot: pr2`、`robot_scene: scene.xml`。

### 步骤 B：编译 ROS 2 工作区

```bash
cd pr2_ws
colcon build
source install/setup.bash
```

### 步骤 C：启动 ros2_control

```bash
ros2 launch pr2_bringup pr2_control.launch.py
```

### 步骤 D（可选）：发布 TF 与外部感知话题

```bash
source pr2_ws/install/setup.bash
ros2 run pr2_mujoco_publisher pr2_mujoco_tf
```

## 关键话题与接口

- 仿真指令输入：`pr2/cmd`
- 仿真状态输出：`pr2/state`
- 底盘速度指令：`/pr2/mecanum_controller/cmd_vel`
- PR2 位姿（Publisher）：`/vrpn/pr2/pose`
- 桌面位姿（Publisher）：`/vrpn/desk/pose`
- 力传感（Publisher）：`/left/ft_sensor/wrench`、`/right/ft_sensor/wrench`

## 设计原则与精简策略

本仓库遵循：
- **KISS**：保留最短控制链路，避免多平台兼容逻辑污染
- **YAGNI**：不保留当前 PR2 仿真闭环不需要的模块
- **DRY**：统一到单一 PR2 通信桥（避免并行冗余桥接）
- **SOLID（SRP）**：`simulate` 仅负责仿真桥接，`pr2_ws` 仅负责 ROS 控制

## 常见问题

1. `cmake: command not found`
   - 安装 `cmake` 后重新编译。

2. MuJoCo 头文件/库找不到
   - 检查 `unitree_mujoco/simulate/mujoco` 软链接是否正确。

3. `unitree_sdk2` 找不到
   - 确认安装路径与 `CMAKE_PREFIX_PATH` 一致（默认 `/opt/unitree_robotics`）。

4. ros2_control 能启动但无状态回传
   - 检查仿真程序是否已运行；确认 `domain_id` 和 `interface` 配置一致（默认 `1` + `lo`）。

## License

沿用原仓库许可证，见 `LICENSE`。

## 仓库页面附加说明

可参考 `docs/REPOSITORY_DESCRIPTION.md` 直接填写 GitHub 仓库 About 信息。
