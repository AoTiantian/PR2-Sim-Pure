# third_party：外部仓库本地副本

本目录用于存放与 **PR2 + MuJoCo + ROS 2** 主工程并列参考、实验用的第三方代码，**默认被仓库根目录 `.gitignore` 忽略**，不会随主项目提交。若需纳入版本管理，请调整根目录 `.gitignore` 或使用 `git submodule`。

---

## 目录一览

| 路径 | 上游仓库 | 说明 |
|------|----------|------|
| `cartesian_controllers/` | [fzi-forschungszentrum-informatik/cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)（`ros2` 分支） | ROS 2 + `ros2_control` 笛卡尔运动 / 力 / **柔顺（含类导纳思路）** 控制器套件 |
| `ridgeback_ur5_controller/` | [epfl-lasa/ridgeback_ur5_controller](https://github.com/epfl-lasa/ridgeback_ur5_controller)（`devel` 分支） | Ridgeback 移动底盘 + UR5 的 **导纳控制** 等（历史 **ROS 1 / catkin** 栈） |

克隆示例（已在本地时可跳过）：

```bash
# FZI — ROS 2
git clone -b ros2 --depth 1 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git third_party/cartesian_controllers

# EPFL — ROS 1
git clone -b devel --depth 1 https://github.com/epfl-lasa/ridgeback_ur5_controller.git third_party/ridgeback_ur5_controller
```

---

## 与主工程（`pr2_ws` / `pr2_mujoco_sim`）的关系

主仓库里的 **`pr2_mujoco_bridge`** 通过话题（如 `joint_commands`、`cmd_vel`）桥接 MuJoCo，**不**使用 `ros2_control` 硬件接口。因此：

- 下面两套第三方代码 **都不能** 与 `pr2_mujoco_sim`「开箱即用」直连；
- 复现时要么在**独立 ROS 工作空间**里按各自 README 编译运行，要么只**阅读算法与话题设计**，再自行在 Python 节点中实现类似逻辑并接入现有话题。

---

## 1. `cartesian_controllers`（FZI）— 复现注意点

- **ROS 版本**：面向 **ROS 2** 与 **`ros2_control`**。请使用与系统一致的发行版（例如 **Jazzy**），并阅读上游 [README](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/README.md)。
- **工作空间**：应放在某 **`colcon` 工作空间的 `src/`** 下再编译，而不是直接在 `third_party` 根目录执行 `colcon build`（除非你把整个 `third_party` 当作 `ws/src` 使用，一般不推荐与 PR2 混在同一个 `src` 里不加区分）。
- **依赖**：按上游说明执行 `rosdep install --from-paths ...`；缺依赖时对照报错补装。
- **构建选项**：上游常见做法是先跳过仿真与测试包以加快首次编译，例如：
  - `colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests ...`
- **仿真入门**：若编译了 `cartesian_controller_simulation`，可用 `ros2 launch cartesian_controller_simulation simulation.launch.py` 体验（需显示/GPU 等按环境配置）。
- **对接 PR2**：若将来要用于真实机器人或统一仿真，需要 **URDF、`ros2_control` 标签、硬件/仿真接口** 与控制器参数一致；与当前 Python MuJoCo 桥是**不同架构**，迁移成本需单独评估。

---

## 2. `ridgeback_ur5_controller`（EPFL）— 复现注意点

- **ROS 世代**：README 针对 **ROS 1（如 Indigo）+ `catkin` + `roslaunch`**，**不是** ROS 2。在仅安装 **ROS 2 Jazzy** 的机器上 **无法按原文档原样编译运行**。
- **平台**：针对 **Clearpath Ridgeback + UR5**（及配套 Gazebo、力传感器话题等），与 **PR2 + MuJoCo** 机型、消息、TF 树均不同；导纳部分可作为**结构与参数**参考，不能机械替换 launch。
- **若要在本机复现**：
  - 需要 **ROS 1 Noetic**（或文档对应旧发行版）的 **独立环境**（Docker 或另一台机/conda 外通常用容器更省事）；
  - 按上游 `dependencies.rosinstall`、`rosdep`、`catkin_make` 流程操作；
  - 注意 Gazebo、Ridgeback/UR 相关依赖版本与文档年代差异，可能需自行修补。
- **实用价值**：阅读 `admittance_control` 包内**控制框图、YAML 参数、力外扰与死区/滤波**等实现细节，便于你在 ROS 2 侧自写导纳节点并对照 `pr2_mujoco_bridge` 的话题设计。

---

## 建议用法

1. **先分清目标**：是要「在本机跑通上游仿真」，还是「只借鉴算法接自己的 PR2 仿真」。
2. **独立工作空间**：为 FZI 单独建 `~/ws_cartesian/src`，把 `cartesian_controllers` 链入或复制进去再 `colcon build`，避免与 `pr2_ws` 依赖搅在一起。
3. **ROS 1 仓库**：用容器或旧发行版环境复现 EPFL 栈；若只做算法迁移，以读代码与论文式对照为主即可。

如有具体发行版（Jazzy / Humble）与是否使用 Docker，可再收紧上述命令与依赖列表。
