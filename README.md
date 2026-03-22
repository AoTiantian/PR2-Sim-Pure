# PR2-Sim-Pure-Plus: 基于 MuJoCo 与 ROS 2 的机器人仿真开发环境 
本项目是一个针对 **PR2 机器人** 的精简版仿真环境，集成了 **MuJoCo 3.x** 物理引擎与 **ROS 2 Jazzy** 通信框架。通过 Docker 开发容器技术，实现了 NVIDIA GPU 硬件加速的丝滑仿真体验。 
## 核心特性 
- **GPU 直通**：完美支持 NVIDIA RTX 系列显卡硬件加速（如 RTX 3070）。
- **一键环境**：使用 VS Code Dev Container，免去复杂的驱动与依赖配置。 
- **全向控制**：实现了 PR2 底盘的侧移、自旋以及手臂各自由度的力矩控制。

## 环境要求（宿主机） 在开始之前，请确保你的 Ubuntu 24.04 宿主机已完成以下准备：
1. **显卡驱动**：安装 NVIDIA 驱动并关闭 BIOS 中的 **Secure Boot**。 
2. **Docker**：已安装并配置好免 sudo 权限。 
3. **NVIDIA Container Toolkit**：用于将 GPU “借”给容器使用。 
4. **图形权限**：宿主机终端需运行 `xhost +local:root` 以允许容器弹窗。 
## 📦 快速开始 
1. **克隆仓库**   
```bash   
git clone https://github.com/你的用户名/你的仓库名.git
cd 你的仓库名   
```
2. **启动容器**   
   - 使用 VS Code 打开该文件夹。   
   - 当右下角弹出 `Reopen in Container` 时，点击它。   
   - 等待 Docker 构建完成（第一次可能需要几分钟下载镜像）。 
3. **运行演示脚本**   进入容器内部终端后，运行以下命令：   

```bash   
# 启动 PR2 综合演示脚本   python3 pr2_ws/src/pr2_controller/scripts/pr2_sim.py   
```
## 📂 项目结构说明 
- `.devcontainer/`: Docker 环境配置文件（包含显卡与网络代理注入逻辑）。 
- `pr2_ws/src/`: ROS 2 功能包源代码。 
- `pr2_description/`: PR2 模型描述与 3D 网格。  
- `pr2_controller/scripts/`: 核心 Python 仿真逻辑（如 `pr2_sim.py`）。 
- `unitree_mujoco/unitree_robots/pr2/`: MuJoCo 原生 MJCF 模型文件（`scene.xml`）。 
## 常见问题排查 
- **无法弹出窗口**：请确保在宿主机（不是容器内）执行了 `xhost +local:root`。 
- **显卡报错**：在容器内输入 `nvidia-smi` 检查驱动是否映射成功。 
- **网络超时**：本项目 `Dockerfile` 已内置清华大学镜像源，若构建缓慢请检查宿主机代理设置。 
## License 本项目遵循 MIT 开源协议。