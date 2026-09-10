# Virtual Human 搬运项目交接文档

更新时间：2026-09-10

本文用于把当前工作交接给下一位 Agent。当前有两条并行工作线：

1. 对比“没有机器人”和“加入机器人”时的木板搬运效果；
2. 调研能够评价人机协同搬运效果的实验指标。

## 一、项目目标

最终目标是比较：

- 人单独搬运长木板；
- 人在一端、PR2 在另一端共同搬运长木板。

核心问题是：加入机器人后，是否能在不降低位置/姿态跟踪质量的前提下，降低人的力学负担，并且不引入明显的人机对抗、振荡或不自然的力分配。

当前 Demo 是仿真验证，不是真人实验。人的作用由 `Virtual Human` 节点产生，木板和 PR2 运行在 MuJoCo 中。

## 二、当前已经完成的实验框架

### 1. 两个正式实验条件

统一入口：

```bash
ros2 launch pr2_virtual_human transport_comparison.launch.py \
  condition:=human_only use_viewer:=true experiment_id:=trial_01
```

```bash
ros2 launch pr2_virtual_human transport_comparison.launch.py \
  condition:=human_robot robot_mode:=admittance \
  use_viewer:=true experiment_id:=trial_01
```

运行前在容器内：

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果只需要批量实验、不打开 MuJoCo 窗口，将 `use_viewer:=true` 改为 `false`。

两种条件必须使用同一份：

```text
pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml
```

### 2. 当前实验配置

配置中的主要时间参数：

- settle：2 s；
- 轨迹跟踪：12 s；
- 末尾保持：1 s；
- 记录频率：50 Hz。

当前轨迹是 6D 闭合轨迹：

- 位置幅值：`[0.20, 0.12, 0.03] m`；
- 姿态幅值：`[0.08, 0.06, 0.10] rad`。

注意：当前 `z` 方向有小幅目标变化，姿态 roll/pitch/yaw 也有目标变化。这是正式对比配置，不是早期只保持水平高度的简单 Demo。

### 3. 物理约束

当前正式对比遵循以下约束：

- 木板与机器人抓取端使用 MuJoCo 刚性 weld；
- 人端和机器人端作用在各自真实端点；
- 人端力产生的 `r × F` 自然力矩保留；
- 不固定人/机器人各承担 50%；
- 不添加自动水平保持；
- 不添加重力前馈或 residual payload compensation；
- 不主动抵消竖直力造成的自然转动力矩；
- 机器人端采用纯 wrench-driven mass-damper admittance；
- 机器人不读取目标轨迹，也不直接进行目标位姿跟踪；
- `hand_force_cancel_moment=false`；
- `pose_tracking_enable=false`、`orientation_tracking_enable=false`、`fixed_target_mode=false`。

这些约束是实验定义的一部分，后续 Agent 不应为了改善曲线而偷偷加入目标位姿、固定力分配或补偿力矩。

## 三、当前实验结果和结论

正式结果位于：

```text
results/transport_comparison/
```

推荐优先查看：

```text
results/transport_comparison/final_aligned/
results/transport_comparison/final/
results/transport_comparison/repeatability/
```

每个运行目录通常包含：

```text
history.csv
metrics.json
run_manifest.json
trajectory_6d.png
human_applied_wrench_6d.png
robot_measured_wrench_6d.png   # human_robot 条件
```

对比目录通常包含：

```text
comparison_metrics.json
trajectory_6d.png
human_wrench_6d.png
robot_assistance.png
board_attitude.png
human_effort.png
```

已经完成：

- human-only 和 human-robot 两种条件的统一运行；
- 12 s 闭合 6D 轨迹 + 1 s HOLD；
- 自动 CSV、JSON 和 PNG 输出；
- 单元/契约测试 14 项通过；
- ROS 2 Jazzy 容器内构建通过；
- 三次重复性运行和配置哈希检查；
- 未发现 NaN/Inf 或明显异常退出。

目前最重要的实验结论不是“机器人已经有效”，而是当前控制结构下机器人效果仍然较差：

| 指标 | human-only | human-robot |
|---|---:|---:|
| 位置 RMSE XYZ | 0.0014 / 0.0038 / 0.0442 m | 0.1012 / 0.0251 / 0.0503 m |
| 人端 force RMS XYZ | 0.15 / 0.59 / 17.67 N | 10.17 / 2.67 / 20.72 N |
| 人端 wrench 饱和占比 | 0.154% | 0% |

加入机器人后 X 方向位置误差和人端 X 向力明显增加。已经尝试过：

- 提高人端线性/姿态阻抗；
- 调整机器人导纳阻尼；
- 将轨迹周期从 12 s 放慢到 18 s。

这些尝试没有从根本上解决问题。当前判断是：机器人只接受人端 wrench 的纯导纳结构存在相位/幅值误差，刚性 weld 耦合后会把误差和负担反馈给人。不要简单把这个问题归结为 PID 参数未调好。

## 四、实验代码关键位置

```text
pr2_ws/src/pr2_virtual_human/launch/transport_comparison.launch.py
pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/trajectory_6d.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/human_impedance_6d.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/human_only_adapter.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/human_robot_controller.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/comparison_recorder.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/comparison_metrics.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/run_plotting.py
pr2_ws/src/pr2_virtual_human/pr2_virtual_human/compare_transport_runs.py
```

物理设计和验收记录：

```text
docs/virtual_human_comparison_demo_refactor_plan.md
```

该文档第 13、14 节包含已落地架构、物理约束审计、正式结果和参数整定回顾。

## 五、评价指标调研工作

报告文件：

```text
docs/cooperative_transport_evaluation_metrics_review.md
```

报告已经整理 26 篇论文，分为四类：

1. 任务完成质量与物体稳定性；
2. 人的负担与人机协作质量；
3. 机器人代价与安全裕度；
4. 更能体现协作机制的进阶指标。

已有基础指标包括：

- 位置/姿态跟踪 RMSE；
- 线速度/角速度误差；
- 两端高度差；
- 完成时间、成功率、路径长度；
- 人端和机器人端 wrench；
- 人端机械功和做功；
- 内力/对抗力；
- 控制饱和和峰值交互载荷；
- sEMG、RULA/REBA、NASA-TLX 等真人实验指标。

新增的进阶指标包括：

- 等性能人力减负率；
- 协作增益/相对有用性；
- 有效 wrench 比率；
- 对抗功率占比；
- 触觉沟通效率；
- 力—运动同步性；
- 协作响应方向性；
- 人类协作基准距离；
- 表观阻抗/透明度；
- 扰动恢复代价；
- 累积人体工效暴露；
- 交互流畅度。

当前最值得接入仿真日志的 5 项是：

1. 等性能人力减负率；
2. 有效 wrench 比率；
3. 对抗功率占比；
4. 力—运动同步性；
5. 扰动恢复代价。

其中第 1 项用于回答“机器人是否真的减轻了人力”；第 2、3 项用于回答“力是否被用于有效搬运，还是变成了内力/对抗”；第 4 项用于分析周期性振荡和相位不同步；第 5 项用于检验闭环鲁棒性。

当前这些进阶指标只写在调研报告中，还没有接入 `comparison_metrics.py`、CSV 记录器和自动绘图。

## 六、交接后建议工作顺序

### 优先级 1：确认现有结果

先查看 `final_aligned` 的 CSV、metrics 和 PNG，确认当前人机协同确实存在 X 向误差和人端负担增加。不要先改控制器。

### 优先级 2：把指标定义落地

建议先实现不需要额外传感器的指标：

- 人端/机器人端 wrench 的合力和合力矩；
- 人端机械功率正功、负功、绝对功；
- 对抗功率占比；
- 两端有效 wrench 与内力分解；
- 人端力与木板速度的互相关和相位延迟；
- 扰动前后峰值误差与恢复时间。

先扩展 `comparison_metrics.py` 和 `comparison_recorder.py`，再修改 PNG 绘图。指标公式必须在报告中写清楚，避免只生成没有解释的曲线。

### 优先级 3：重新评估控制方案

只有在指标可以区分“有效协作”和“人机对抗”后，再决定是否改变当前实验定义。任何引入以下机制的修改都必须明确记录为新的实验条件：

- 机器人读取目标轨迹；
- 机器人目标位姿跟踪；
- 固定人/机器人力分担比例；
- 机器人端主动重力补偿；
- 额外姿态补偿力矩；
- 自动水平保持。

不能把这些机制悄悄加进原始 pure-wrench baseline，否则无法回答“机器人自然协作是否有效”。

## 七、当前 Git 状态

交接时工作区存在以下改动：

```text
M  docs/virtual_human_comparison_demo_refactor_plan.md
M  pr2_ws/src/pr2_virtual_human/pr2_virtual_human/human_robot_controller.py
?? docs/cooperative_transport_evaluation_metrics_review.md
```

其中：

- `human_robot_controller.py` 的改动是处理 recorder 结束 ROS context 后的正常 `RuntimeError` 关闭竞态；
- `virtual_human_comparison_demo_refactor_plan.md` 增加了参数整定回顾和最终结果；
- 指标报告是新增文件；
- 这些改动目前没有提交 Git，接手 Agent 应先检查 diff，再决定是否提交。

## 八、重要实验边界

不要把当前结果描述为“机器人已经成功减负”。准确描述应是：

> 已完成可重复的 human-only / human-robot 对照仿真和自动记录框架；当前 pure-wrench 人机协同基线能够运行，但在 X 方向跟踪和人端负担方面表现不理想，下一步需要用进阶指标定位对抗来源，再决定是否调整控制结构。

