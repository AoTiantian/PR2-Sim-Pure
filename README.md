# PR2-Sim-Pure-Plus

基于 **MuJoCo 3.x** 与 **ROS 2 Jazzy** 的 PR2 机器人仿真开发环境。可选 **Docker / Dev Container** 与 NVIDIA GPU 加速。

## 核心特性

- **GPU 直通**：支持 NVIDIA 显卡硬件加速（如 RTX 系列）。
- **一键环境**：使用 VS Code Dev Container，简化驱动与依赖配置。
- **ROS 2 桥接**：`pr2_controller` 包发布 `joint_states`、`odom`、TF，并订阅 `cmd_vel`、`joint_commands` 等。
- **内置演示**：节点默认 `demo_motion:=true`，效果接近旧版纯 Python 脚本（夹爪、左臂、全向底盘周期动作）。
- **全向底盘**：近似侧移、自旋与手臂力矩/位置控制（详见包内文档）。

## 环境要求（宿主机）

在 Ubuntu 24.04 等系统上建议完成：

1. **显卡驱动**：安装 NVIDIA 驱动；若需 GPU 进容器，BIOS 中按需处理 **Secure Boot**。
2. **Docker**：已安装并配置好免 `sudo`（如需要）。
3. **NVIDIA Container Toolkit**：将 GPU 映射进容器时使用。
4. **图形 / MuJoCo 窗口**：在宿主机允许 X11 访问，例如 `xhost +local:root`（按需）；无桌面可用**无头模式**（见下文）。

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/你的用户名/你的仓库名.git
cd 你的仓库名
```

### 2. 启动开发容器（可选）

- 用 VS Code 打开本仓库根目录。
- 出现 **Reopen in Container** 时选择进入容器。
- 首次构建可能需要几分钟。

### 3. 运行仿真（推荐：ROS 2）

在容器或本机已安装 ROS 2 Jazzy 的前提下：

```bash
cd pr2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pr2_controller
source install/setup.bash

# 带 MuJoCo 窗口（需可用 DISPLAY）
ros2 run pr2_controller pr2_mujoco_sim

# 或使用 launch
ros2 launch pr2_controller pr2_mujoco_sim.launch.py
```

**无窗口 / SSH / 无 X11**：

```bash
ros2 run pr2_controller pr2_mujoco_sim --ros-args -p use_viewer:=false
```

**只要 ROS 控制、不要内置演示动作**：

```bash
ros2 run pr2_controller pr2_mujoco_sim --ros-args -p demo_motion:=false
```

更完整的话题说明、故障排除与示例命令见：

**[pr2_ws/src/pr2_controller/README.md](pr2_ws/src/pr2_controller/README.md)**

### 4. 纯 Python 演示（无 ROS）

不启动 ROS、仅直连 MuJoCo 时仍可使用：

```bash
python3 pr2_ws/src/pr2_controller/scripts/pr2_sim.py
```

## 项目结构

| 路径 | 说明 |
|------|------|
| `.devcontainer/` | Dev Container / Docker 相关配置 |
| `pr2_ws/src/pr2_controller/` | ROS 2 功能包：`pr2_sim_ros.py` 节点、`launch/`、`package.xml` |
| `pr2_ws/src/pr2_controller/scripts/` | 无 ROS 的 `pr2_sim.py` 等脚本 |
| `unitree_mujoco/unitree_robots/pr2/` | MuJoCo MJCF（如 `scene.xml`） |
| `pr2_description/` | PR2 模型描述与网格等资源（若存在） |

MuJoCo 场景默认路径（可在节点参数中修改）：

`unitree_mujoco/unitree_robots/pr2/scene.xml`

## 常见问题排查

| 现象 | 处理思路 |
|------|----------|
| 无法弹出仿真窗口 | 宿主机执行 `xhost +local:root`（或等价配置）；确认 `DISPLAY` 与 X11 转发 |
| `Failed to open display` / `could not initialize GLFW` | 使用 `-p use_viewer:=false` 无头运行；或修复容器内图形环境 |
| MESA / `iris` / DRI 报错 | 尝试 `LIBGL_ALWAYS_SOFTWARE=1`，或优先无头仿真 |
| 容器内无 GPU | 检查 `nvidia-smi`；确认 NVIDIA Container Toolkit 与运行时配置 |
| 构建 / 网络慢 | Dockerfile 中可配置镜像源；检查宿主机代理 |

## License

本项目遵循 **MIT** 开源协议。
