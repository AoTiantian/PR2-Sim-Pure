# 虚拟人搬运对比 Demo 重构计划

## 1. 文档状态

- 状态：已实施并完成最终验收审计
- 范围：仅重构“虚拟人单独搬运”与“虚拟人 + PR2 协作搬运”的对比 Demo
- 当前阶段：重构完成；验收通过项与已知未通过项见 13.4
- 目标入口：

  ```bash
  # 无机器人
  ros2 launch pr2_virtual_human transport_comparison.launch.py \
    condition:=human_only

  # 有机器人，纯力驱动导纳协作
  ros2 launch pr2_virtual_human transport_comparison.launch.py \
    condition:=human_robot robot_mode:=admittance
  ```

## 2. 已确认的实验定义

### 2.1 核心研究问题

在虚拟人模型、目标轨迹、木板参数、仿真时间和评价方法一致的条件下，只改变“是否加入机器人”，比较：

1. 人手端点的六维轨迹跟踪误差；
2. 虚拟人的施力、力矩、力变化率与机械功；
3. 木板姿态稳定性；
4. 机器人的实际协助力与力矩。

正式比较不得预设“机器人一定改善结果”。验收只检查实验是否正确、可重复、可解释，不以获得某个期望结论为通过条件。

### 2.2 机器人协作方式

正式实验采用纯力驱动：

- 机器人不知道目标轨迹；
- 机器人不得订阅或使用虚拟人的期望手部位姿；
- 机器人只根据 MuJoCo 腕部六维力/力矩测量，经导纳控制和 QP 产生运动；
- 禁用当前 Demo 中直接覆盖导纳速度的 `pose_tracking`；
- 禁用直接跟踪虚拟人目标姿态的 `orientation_tracking`；
- 允许 QP 使用腕部力矩执行角向导纳，但不得使用目标姿态前馈。

### 2.3 抓取模型

- 机器人与木板采用 MuJoCo `weld` 刚性抓取；
- 抓取能够传递力和力矩；
- 人和机器人施加在不同端点的力必须通过真实作用点自然形成 `r×F`；当两端 Z 向力不相等时，由此产生的净转动力矩和木板转动是实验物理现象，不得在控制算法中抹除；
- `weld` 产生的约束反力和反力矩必须由 MuJoCo 约束求解器自然计算，不得用额外的用户外力矩模拟或替代；
- 代码、参数、日志和文档统一称为 `rigid_weld_grasp`，不得再将其描述为 point contact；
- 不在本次重构中引入球铰、点接触或多种抓取模式对比。

### 2.4 轨迹范围

第一版必须支持并实际验证完整六维轨迹：

- 平移：X、Y、Z；
- 姿态：roll、pitch、yaw；
- 两种条件使用同一轨迹生成器和同一配置；
- 比较坐标统一为人手端点，而不是一边使用木板质心、另一边使用人手端点；
- 默认正式实验完成一个闭合周期，起点和终点的位置、姿态、线速度及角速度连续。

完整 6D 轨迹允许虚拟人根据目标姿态与实际姿态误差产生正常的任务姿态力矩，这是执行 roll/pitch/yaw 所必需的控制输入。该力矩只能来自统一的姿态阻抗律，不得包含根据人机 Z 向力差、端点杠杆力矩或木板倾斜状态额外构造的抵消项。

默认轨迹建议保持现有尺度中的温和版本：

```yaml
trajectory:
  start_delay_sec: 2.0
  duration_sec: 12.0
  hold_duration_sec: 1.0
  x_amplitude_m: 0.20
  y_amplitude_m: 0.12
  z_amplitude_m: 0.03
  roll_amplitude_rad: 0.08
  pitch_amplitude_rad: 0.06
  yaw_amplitude_rad: 0.10
  cycles: 1.0
```

轨迹使用归一化相位 `s(t)∈[0,1]` 和解析的一阶、二阶导数生成，不再分别用“周期、跟踪时长、ramp 时长”推导出可能不闭合的实际相位。姿态内部使用旋转向量/SO(3)，不使用欧拉角直接累加。

### 2.5 竖直承重原则

采用自然负载分配：

- 虚拟人控制器不知道机器人是否存在；
- 虚拟人不得读取机器人腕力或机器人承重命令；
- 不显式规定人和机器人各承担 50% 或任意百分比；
- 仿真桥不得根据虚拟人实时命令计算并注入“机器人剩余承重”；
- 不叠加独立的机器人端点 Z 高度保持力；
- 不计算或施加用于抵消两端 Z 向力不相等所产生 `r×F` 的主动补偿力矩；
- 不允许出现 `-r×F`、基于人机力差的前馈力矩、自动水平保持力矩或同等作用的隐藏补偿；
- 人类端使用同一个六维阻抗模型，根据目标端点和实际端点误差自然产生支撑力；
- 机器人端的负载由刚性抓取、机器人自身控制和耦合动力学自然形成，并通过传感器测量。

### 2.6 最小算法原则

- 每个物理目标只允许一个明确的控制机制；
- 能由 MuJoCo 刚体动力学、重力、真实作用点和 `weld` 约束自然产生的力、力矩和运动，不在算法中重复计算或注入；
- 不为了让曲线更平、木板更水平或结果更符合预期而增加补偿器；
- 限幅和滤波只用于数值安全与传感器带宽建模，不得承担隐藏的平衡控制；
- 新增任何前馈、补偿、状态分支或特殊参数前，必须先证明基本物理模型无法表达该效应，并获得用户确认。

为避免把启动先后顺序变成实验变量，两种条件均采用相同状态机：

```text
WAIT_FOR_STATE -> LATCH_REFERENCE -> SETTLE -> TRACK -> HOLD -> DONE
```

`LATCH_REFERENCE` 在收到第一份有效仿真状态时固定目标参考；`SETTLE` 中已启用相同的人类阻抗控制，但不推进轨迹；正式指标默认排除启动和 settle 区间。

## 3. 不可违反的重构边界

### 3.1 禁止修改的公共组件

本次重构不得修改以下现有公共组件的源码、接口、默认参数或行为：

- `pr2_ws/src/pr2_mujoco_bridge/**`
- `pr2_ws/src/pr2_wbc_admittance_control/**`
- `pr2_ws/src/pr2_qp_admittance_bringup/**`
- `pr2_ws/src/pr2_wrench_input/**`
- PR2 通用机器人模型及其 actuator/sensor 定义
- 与本对比实验无关的 launch、场景和 Demo

允许通过本 Demo 的 launch 文件给公共节点传入已有参数，但不得为了本 Demo 向公共节点添加分支、参数或特殊补偿。

实施期间对上述路径建立 diff 守卫；每个阶段结束时执行检查，发现改动即视为边界违反。

### 3.2 允许修改的范围

主要改动限制在：

- `pr2_ws/src/pr2_virtual_human/**`
- 本 Demo 专用的新测试文件
- 本 Demo 专用的配置、日志和对比脚本
- `docs/**`

优先复用现有只读 MuJoCo 场景。如果现有场景无法满足一致性要求，只能新增明确命名为 comparison-demo 专用的场景文件，不得修改原场景来影响其他 Demo。

### 3.3 兼容策略

- 新入口为 `transport_comparison.launch.py`；
- 旧 `human_board_demo.launch.py` 和 `pr2_virtual_human_demo.launch.py` 在新入口验收完成前保留；
- 验收后可将旧入口改为薄兼容包装器，但不得继续保留两套轨迹或人类控制算法；
- 不删除历史结果目录；新结果写入独立的 `results/transport_comparison/`。

## 4. 目标架构

```text
pr2_virtual_human
├── config/
│   └── transport_comparison.yaml       # 两种条件共享的唯一实验配置
├── launch/
│   └── transport_comparison.launch.py  # 统一入口，仅选择 plant 条件
├── pr2_virtual_human/
│   ├── trajectory_6d.py                # 纯函数：时间 -> 6D 目标及导数
│   ├── human_impedance_6d.py           # 仅由 6D 任务误差产生 human wrench
│   ├── experiment_state.py             # 统一状态机和仿真时间逻辑
│   ├── comparison_topics.py            # topic/frame/schema 常量
│   ├── human_only_adapter.py            # 无机器人 MuJoCo plant 适配器
│   ├── human_robot_controller.py        # 有机器人时发布同一 human wrench
│   ├── comparison_recorder.py           # 条件无关的统一记录器
│   └── comparison_metrics.py            # 指标计算，不进入控制回路
├── scripts/
│   └── compare_transport_runs.py        # 两个 run 的对齐、汇总和绘图
└── test/
    ├── test_trajectory_6d.py
    ├── test_human_impedance_6d.py
    ├── test_experiment_state.py
    ├── test_metrics_schema.py
    └── test_launch_contract.py
```

模块职责必须保持单向：

```text
共享 YAML -> 6D 轨迹 -> 人类阻抗 -> human wrench -> plant
                                               |
plant state + wrench + target -----------------> recorder -> metrics/plots
```

记录器和绘图模块不得向控制器发布任何反馈量。

`human_impedance_6d.py` 的角向输出只允许包含姿态误差、角速度误差及受限积分项。它不得接收 robot wrench、human/robot Z 向力差、接触点杠杆力矩或任何“需要抵消的力矩”作为输入。

## 5. 统一数据契约

### 5.1 坐标系和参考点

- 仿真世界坐标统一使用 `odom`；
- 目标、实际位姿和 human wrench 均以 `odom` 表达；
- 唯一评价点为木板局部坐标 `human_hand_offset=[1,0,0]` 对应的人手端点；
- 木板质心状态可作为诊断量记录，但不得与人手端点目标直接计算误差；
- robot wrist wrench 必须保留其原始 frame，并在记录器中转换或明确标注，禁止仅靠符号参数猜测含义。

### 5.2 仿真时间

- 状态机、轨迹、采样和指标全部使用 MuJoCo 仿真时间；
- 不使用 `time.monotonic()` 推进实验阶段；
- ROS wall timer 只能触发处理，不得决定轨迹相位；
- 时间倒退、重复时间戳和大步跳变必须有明确处理和测试。

### 5.3 Topic 契约

两种条件至少提供相同的核心 topic：

```text
/transport_comparison/sim_time
/transport_comparison/actual_hand_pose
/transport_comparison/desired_hand_pose
/transport_comparison/human_wrench
/transport_comparison/board_pose
/transport_comparison/state
```

有机器人条件额外提供：

```text
/transport_comparison/robot_wrist_wrench
/transport_comparison/robot_ee_pose
```

缺失的机器人字段在 human-only CSV 中写为空值/NaN，并通过 `condition` 字段解释，不得伪造为零。

### 5.4 输出目录和文件

```text
results/transport_comparison/<experiment_id>/
├── experiment_config.yaml
├── human_only/<run_id>/
│   ├── history.csv
│   ├── metrics.json
│   └── run_manifest.json
├── human_robot/<run_id>/
│   ├── history.csv
│   ├── metrics.json
│   └── run_manifest.json
└── comparison/
    ├── comparison_metrics.json
    ├── trajectory_6d.png
    ├── human_effort.png
    ├── board_attitude.png
    └── robot_assistance.png
```

`run_manifest.json` 至少记录 commit、condition、模型路径、配置哈希、启动命令、仿真步长和软件版本，避免误比较不同配置的结果。

## 6. 公共节点的 Demo 专用参数策略

不修改公共组件，通过 launch 关闭现有混合控制和显式负载分配：

### 6.1 MuJoCo bridge

有机器人条件应显式配置：

```yaml
hand_force_enable: true
hand_force_cancel_moment: false
ctc_payload_auto_balance: false
ctc_payload_force_z: 0.0
ctc_vertical_hold_force_limit: 0.0
left_wrist_tare_duration_sec: 2.0  # 与 settle 时间一致，仅标定传感器静态预载
```

其中 CTC 只作为现有机器人关节内环使用，不再承担本 Demo 特有的实时负载分配或板端高度闭环。

`hand_force_cancel_moment` 必须保持为 `false`。Bridge 应把 human force 按真实端点转换为质心 wrench，使 `r×F` 原样进入 MuJoCo；来自姿态阻抗的任务力矩可以与其相加，但不得先减去或抵消自然的 `r×F`。

腕部 tare 只定义导纳使用的传感器零点，使机器人响应相对静态抓持平衡的交互增量。它不得修改 MuJoCo 外力、`r×F`、weld 反力或 CTC 力矩，也不得被解释为主动抵消力矩。tare 窗口必须与 SETTLE 阶段一致并写入 manifest。

### 6.2 QP/导纳控制器

有机器人条件应显式配置：

```yaml
pose_tracking_enable: false
orientation_tracking_enable: false
freeze_orientation: false
fixed_target_mode: false
```

线向和角向目标速度只能来自腕部 wrench 的导纳方程。QP 的现有稳定性参数放入 Demo YAML，不散落在 launch 源码中。

### 6.3 启动前契约检查

Demo 节点启动时检查关键参数。如果发现 `pose_tracking`、目标姿态跟踪、自动残余承重或 CTC 高度保持仍启用，应立即失败并给出明确错误，而不是带着错误实验配置继续运行。

## 7. 分阶段实施计划

### 阶段 0：冻结基线与建立保护边界

1. 保存当前两种 Demo 的启动命令、配置和代表性结果清单；
2. 记录当前已知问题，不把历史结果当作重构后的正确基准；
3. 建立公共路径 diff 守卫和允许改动路径清单；
4. 定义统一 CSV schema、topic、frame 和符号约定。

完成条件：无需修改公共组件即可明确新 Demo 所需的所有输入输出。

### 阶段 1：先实现共享纯逻辑和单元测试

1. 实现唯一的 `Trajectory6D`；
2. 实现唯一的 `HumanImpedance6D`；
3. 实现只依赖仿真时间的实验状态机；
4. 将四元数、旋转向量、角速度和误差计算集中到一个模块；
5. 为闭合性、导数连续性、坐标变换、限幅、抗积分饱和和时间异常编写测试；
6. 增加力矩来源测试：验证端点 `r×F` 被完整保留，姿态任务力矩不含任何 `-r×F` 或人机力差补偿项。

完成条件：这些模块不依赖 ROS 和 MuJoCo，也不包含 `has_robot` 分支。

### 阶段 2：重构 human-only 适配器

1. 只负责加载无机器人场景和推进 MuJoCo；
2. 从木板位姿计算统一的人手端点位姿和速度；
3. 调用共享轨迹与人类阻抗模块；
4. 将 endpoint wrench 正确变换成木板质心处的 `xfrc_applied`；
5. 明确保留 `tau_com = r×F + tau_task`，禁止插入 `tau_cancel`；
6. 发布统一 topic，不在节点内部绘图或计算对比结论。

完成条件：human-only 能完整运行一条 6D 闭合轨迹并输出统一 schema。

### 阶段 3：重构 human-robot Demo 控制节点

1. 订阅 bridge 发布的人手端点/木板状态和仿真时间；
2. 调用与 human-only 完全相同的轨迹和人类阻抗模块；
3. 只向 bridge 发布 human wrench；
4. 不订阅机器人腕力，不读取机器人承重，不根据机器人状态修改 human wrench；
5. 人类姿态任务力矩只由姿态误差产生，不根据 Z 向力差或自然转动趋势增加抵消项；
6. 通过 launch 参数将公共 QP 配置成纯六维 wrench-driven admittance；
7. 增加启动契约检查，拒绝混合模式。

完成条件：移除机器人目标轨迹通道后，Demo 仍能完成 6D 运行且控制量有限。

### 阶段 4：统一 launch 和 YAML

1. 新建唯一 `transport_comparison.yaml`；
2. 新建 `transport_comparison.launch.py`；
3. `condition` 只决定启动哪个 plant，不改变轨迹和人类参数；
4. launch 源码中只保留节点编排，所有实验参数进入 YAML；
5. 将最终解析配置复制到每次 run 目录。

完成条件：两条目标轨迹的配置哈希一致，除 condition/robot-only 参数外 manifest 一致。

### 阶段 5：统一记录、指标和对比图

1. 使用独立 recorder 记录相同字段；
2. 按仿真时间重采样和对齐两组数据；
3. 计算位置/姿态 RMSE、峰值误差、human wrench RMS/峰值/变化率、机械功和机器人协助 wrench；
4. 区分 command、sensor measurement 和 derived metric；
5. 比较脚本必须从命令行接收两个 run 目录，禁止硬编码路径和 CSV 列号；
6. 删除或归档未完成的 Demo 专用记录脚本，避免多套结果格式继续流通。

完成条件：一个命令能够验证配置兼容性并生成完整对比报告。

### 阶段 6：无界面集成测试与物理审计

依次执行：

1. human-only 静态六维保持；
2. human-robot 静态六维保持；
3. human-only 完整 6D 轨迹；
4. human-robot 完整 6D 轨迹；
5. 两种条件各重复三次；
6. 公共组件 diff 审计；
7. 配置、frame、wrench 符号和能量方向审计；
8. 力矩来源审计：逐项确认记录中的自然 `r×F`、human task torque、weld reaction torque，不存在主动 cancellation torque。

完成条件：满足第 8 节全部验收标准。

### 阶段 7：切换入口与清理遗留逻辑

1. 将旧 launch 变成薄包装器或标记 deprecated；
2. 在 `pr2_virtual_human` 内删除不再使用的重复轨迹、重复阻抗、节点内绘图和机器人感知的人类负载分配代码；
3. 更新 package 安装清单和 README；
4. 保留一次可恢复提交点，确认新旧入口迁移说明完整。

完成条件：`pr2_virtual_human` 中只有一套正式轨迹生成器和一套正式人类阻抗实现。

## 8. 验收标准

### 8.1 功能正确性

- 两种条件均完成 XYZ + roll/pitch/yaw 的完整闭合轨迹；
- 运行期间无异常退出、NaN、Inf、时间倒退或数据 schema 变化；
- 两种条件的目标 6D 轨迹在相同仿真时间重采样后，位置差小于 `1e-9 m`、旋转向量差小于 `1e-9 rad`；
- human controller 的配置哈希完全一致；
- robot controller 没有接收 desired pose；
- 日志中能够区分 human applied wrench、robot measured wrench 和派生的合力指标。

### 8.2 控制与物理合理性

- 稳态阶段不得存在持续控制饱和；任一 human wrench 分量处于限幅边界的采样占比低于 1%；
- QP/CTC 命令处于限幅边界的采样占比低于 1%，否则实验标记为 invalid，而不是继续调大限幅；
- 静态保持阶段木板位姿有界：位置峰峰值小于 `0.02 m`，姿态峰峰值小于 `0.05 rad`；
- 静态阶段按统一符号换算后的竖直外力平衡残差均值小于板重的 10%；
- 不允许出现独立的 50% 承重目标、实时 residual force 注入或额外 Z hold force；
- 两端 Z 向力不相等时，必须能在动力学和日志中观察到对应的自然 `r×F`；不得施加抵消该项的主动补偿力矩；
- human task torque 必须可追溯到姿态阻抗误差，且与 `r×F` 分项记录；
- 除姿态轨迹所需的 task torque 外，所有主动 cancellation/balance/level-hold torque 必须恒为不存在，而不仅是数值恰好为零；
- 允许 human-only 和 human-robot 的跟踪误差不同，这正是实验输出，不作为架构失败条件。

### 8.3 可重复性

- 无 viewer 条件下每种 condition 连续运行三次；
- 每个主要 RMSE/RMS 指标相对标准差不高于 5%；
- 运行配置哈希相同且随机种子、MuJoCo timestep 一致；
- 如果 viewer 会影响实时调度，指标以无 viewer 运行结果为正式结果。

### 8.4 工程质量

- 共享纯逻辑具备单元测试；
- 两个 launch 条件具备短时 smoke test；
- 禁止公共组件路径出现 diff；
- launch 文件不再包含大段控制增益和相互矛盾的物理注释；
- 不存在硬编码 `/workspace/logs/...` 的正式分析脚本；
- 不存在只读取但不生效的正式 Demo 参数。

## 9. 风险与应对

### 风险 1：关闭显式残余承重后板会出现较大启动下沉

应对：使用统一的 LATCH/SETTLE 状态机和相同人类阻抗；先验证静态平衡，再启用轨迹。不得重新加入机器人感知的人类承重分配来掩盖启动问题。

### 风险 2：纯力驱动下 6D 姿态轨迹难以跟踪

应对：先验证 wrench frame、weld 反力矩和角向导纳符号，再调整 Demo YAML 中的被动参数。不得恢复目标姿态直通机器人。

### 风险 3：公共 QP 现有参数无法形成稳定的纯角向导纳

应对：先穷尽现有参数的合法配置。如果确认公共接口不足，停止实施并单独提交“公共组件能力缺口”报告，由用户决定是否扩大范围；不得在本次重构中偷偷修改公共组件。

### 风险 4：腕部传感器包含偏置或内部约束反力，无法直接解释负载分配

应对：增加独立静态校准运行并在 manifest 中记录 tare 策略；保留原始传感器数据，所有符号转换在离线指标层完成，不进入控制回路。

## 10. 实施过程中的停止条件

出现以下任一情况时，应停止当前实施阶段并报告，不得用更多特殊分支继续堆叠：

1. 必须修改公共 bridge/QP 才能继续；
2. 必须让 virtual human 读取 robot wrench 才能稳定；
3. 必须给机器人传入目标轨迹才能完成正式力驱动实验；
4. 必须显式规定人机承重比例才能维持静态平衡；
5. 目标轨迹或评价参考点无法在两种条件间保持一致；
6. 为获得“机器人更好”的结论而需要改变任一条件的人类参数；
7. 必须施加主动抵消力矩才能阻止 Z 向力差产生的自然转动。

## 11. 最终交付物

1. 统一对比 launch 和 YAML；
2. 共享 6D 轨迹与人类阻抗核心；
3. human-only 与 human-robot 两个薄适配器；
4. 条件无关的数据记录器；
5. 自动指标与对比绘图工具；
6. 单元测试、launch smoke test 和三次重复性报告；
7. 公共组件零改动审计结果；
8. README，包括实验含义、启动命令、输出解释和已知限制。

## 12. 需求追踪矩阵

| 已确认要求 | 计划落实位置 | 状态 |
|---|---|---|
| 对比轨迹误差、人类负担、姿态和机器人协助 | 2.1、5、8 | 已覆盖 |
| 机器人采用纯力驱动 A | 2.2、6.2、7 阶段 3 | 已覆盖 |
| 使用刚性 weld 抓取 A | 2.3 | 已覆盖 |
| 第一版保留完整 6D 轨迹 B | 2.4、7、8.1 | 已覆盖 |
| 自然负载分配 A | 2.5、6、8.2 | 已覆盖 |
| 保留 Z 向力差产生的自然力矩，禁止主动抵消力矩 | 2.3、2.5、2.6、7、8.2、10 | 已覆盖 |
| 不触及公共组件 | 3、8.4、10 | 已覆盖 |
| 统一 launch 入口 | 1、7 阶段 4 | 已覆盖 |

当前不存在需要在实施前继续确认的架构性问题。若实施中触发第 9 节风险或第 10 节停止条件，应先向用户报告并重新确认范围。

## 13. 实施后回看与验收记录

### 13.1 已落地架构

- 已新增统一入口 `transport_comparison.launch.py` 和唯一共享 YAML；
- 两种条件共用 `Trajectory6D`、`HumanImpedance6D` 和仿真时间状态机；
- human-robot 控制器不订阅 robot wrench，也不向 QP 传入 desired pose；
- recorder 只记录诊断量，按仿真时间统一采样为 50 Hz；
- 已生成轨迹、人类负担、木板姿态和机器人测量 wrench 四类图；
- 旧入口保留，未删除历史结果和无关 Demo。

### 13.2 物理约束审计

- `hand_force_cancel_moment=false`；human wrench 在质心处始终使用
  `tau_com = r×F + tau_task`；
- `tau_task` 的函数接口只接收目标/实际 6D 状态，不接收机器人状态、
  两端 Z 力差或杠杆力矩；
- 自动 residual payload、CTC Z hold、pose tracking、orientation tracking
  和 fixed target 均关闭，并由运行时 guard 检查；
- 腕部 tare 仅改变传感器参考零点，不进入 human 控制，也不修改 MuJoCo
  外力、自然力矩或 weld 反力；
- robot 使用六维纯质量–阻尼导纳，所有虚拟刚度为零。原因是公共 QP 的
  角位移定义与 `-K·dx` 符号不一致，非零角刚度会形成负弹簧；本次未修改
  公共 QP，而是使用其已有参数表达最简单的被动导纳。

### 13.3 完整运行结果（2026-08-26）

同一配置哈希下，两种条件各完成一次 12 s 闭合 6D 轨迹及 1 s HOLD，
每次记录 650 个有限的 50 Hz 样本：

| 指标 | human-only | human-robot |
|---|---:|---:|
| 位置 RMSE XYZ (m) | 0.0014 / 0.0038 / 0.0442 | 0.1012 / 0.0251 / 0.0503 |
| 姿态 RMSE XYZ (rad) | 0.0373 / 0.0441 / 0.0049 | 0.0120 / 0.0504 / 0.0513 |
| human force RMS XYZ (N) | 0.15 / 0.59 / 17.67 | 10.17 / 2.67 / 20.72 |
| human wrench 饱和占比 | 0.154% | 0% |

结果没有预设机器人一定更好：当前纯力驱动机器人在 X 跟踪上增加了人类
负担和误差，这被保留为实验结论而没有用目标轨迹或补偿器掩盖。

### 13.4 最终验收结论

通过项：

- 14 个单元/契约测试全部通过，package 在 ROS 2 Jazzy 容器内构建通过；
- 两边记录目标在各自初始位姿归一化后，位置最大差
  `1.11e-16 m`、旋转向量最大差 `6.94e-17 rad`；
- 两种条件均完成 650 样本的 6D TRACK + HOLD，无 NaN/Inf 或异常退出；
- human wrench 限幅占比均低于 1%；
- 三次运行的配置哈希完全一致；human-only 主要指标相对标准差为 0%；
- 禁止修改的四个公共 package 无 git diff；自然 `r×F` 与 task torque
  分列记录，代码审计未发现 cancellation/load-share/level-hold 注入。

未完全通过项：

- human-robot 三次运行中，位置 RMSE 与主要 force RMS 的相对标准差大多
  低于 5%，但数值较小的 roll/yaw 姿态与对应 task torque 分量为
  5.4%–7.2%，没有满足“所有主要 RMSE/RMS 均不高于 5%”的严格门槛；
- 原因是公共 QP 由 ROS wall timer 驱动，而实验轨迹由 MuJoCo sim time
  驱动，进程调度会改变闭环采样相位。在禁止修改公共组件的边界内，
  本次不增加同步补偿器或特殊控制分支，保留为明确的公共能力限制；
- 旧 launch 已保留但尚未改成薄包装器。为避免破坏其他 Demo，本次没有
  删除其历史重复逻辑；正式对比唯一入口已经切换为新 launch。
