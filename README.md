# PR2-Sim-Pure-Plus

基于 **MuJoCo 3.x** 与 **ROS 2 Jazzy** 的 PR2 机器人仿真开发环境。可选 **Docker / Dev Container** 与 NVIDIA GPU 加速。

## 核心特性

- **GPU 直通**：支持 NVIDIA 显卡硬件加速（如 RTX 系列）。
- **一键环境**：使用 VS Code Dev Container，简化驱动与依赖配置。
- **ROS 2 桥接**：`pr2_mujoco_bridge` 包发布 `joint_states`、`odom`、TF，并订阅 `cmd_vel`、`joint_commands` 等。
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
colcon build --packages-select pr2_mujoco_bridge
source install/setup.bash

# 带 MuJoCo 窗口（需可用 DISPLAY）
ros2 run pr2_mujoco_bridge pr2_mujoco_sim

# 或使用 launch
ros2 launch pr2_mujoco_bridge pr2_mujoco_sim.launch.py
```

**无窗口 / SSH / 无 X11**：

```bash
ros2 run pr2_mujoco_bridge pr2_mujoco_sim --ros-args -p use_viewer:=false
```

**只要 ROS 控制、不要内置演示动作**：

```bash
ros2 run pr2_mujoco_bridge pr2_mujoco_sim --ros-args -p demo_motion:=false
```

更完整的话题说明、故障排除与示例命令见：

**[pr2_ws/src/pr2_mujoco_bridge/README.md](pr2_ws/src/pr2_mujoco_bridge/README.md)**

### 4. 纯 Python 演示（无 ROS）

不启动 ROS、仅直连 MuJoCo 时仍可使用：

```bash
python3 pr2_ws/src/pr2_mujoco_bridge/scripts/pr2_sim.py
```

## 项目结构

| 路径 | 说明 |
|------|------|
| `.devcontainer/` | Dev Container / Docker 相关配置 |
| `pr2_ws/src/pr2_mujoco_bridge/` | ROS 2 功能包 **pr2_mujoco_bridge**（MuJoCo↔ROS 桥、IK、末端位姿等） |
| `pr2_ws/src/pr2_mujoco_bridge/pr2_mujoco_bridge/` | 同名 Python 子包（`ament_python` 惯例，import 名 `pr2_mujoco_bridge`） |
| `pr2_ws/src/pr2_mujoco_bridge/scripts/` | 无 ROS 的 `pr2_sim.py` 等脚本 |
| `src/pr2_ros2_stack/` | 另一套 ROS 2 工程目录（含 `pr2_description`、`pr2_bringup`、`pr2_mujoco_hardware` 等多个子包） |
| `unitree_mujoco/unitree_robots/pr2/` | MuJoCo MJCF（如 `scene.xml`） |
| `pr2_description/` | PR2 模型描述与网格等资源（若存在） |

MuJoCo 场景默认路径（可在节点参数中修改）：

`unitree_mujoco/unitree_robots/pr2/scene.xml`

## AI 编码助手规范

本仓库已经按机器人开发项目配置了项目级 AI guardrails，供 Codex CLI、Claude Code 等工具读取：

| 文件 | 作用 |
|------|------|
| `AGENTS.md` | 项目级主规范；Codex CLI 和其他 agent 的首选入口 |
| `CLAUDE.md` | Claude Code 项目入口，引用 `AGENTS.md` 与 `.claude/rules/*.md` |
| `.claude/rules/` | Claude Code 分层规则：项目布局、ROS 2 Jazzy、PR2 安全、验收流程 |
| `scripts/verify_ai_project.sh` | 检查本仓库机器人项目布局和 AI 规范文件是否完整 |
| `scripts/ai-codex.sh` / `scripts/ai-claude.sh` | 从仓库根目录启动对应 AI 工具的轻量 wrapper |

建议在让 AI 修改代码前先确认当前分支状态：

```bash
git status --short --branch
bash scripts/verify_ai_project.sh
```

涉及控制、力控、顺应、WBC、launch 参数或验证阈值的改动，应同步阅读：

```text
pr2_ws/src/pr2_mujoco_bridge/README_ACCEPTANCE_FEAT.md
```

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
