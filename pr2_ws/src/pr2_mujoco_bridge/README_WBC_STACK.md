# 移动机械臂控制栈（与手绘框图对应）

本目录实现的是 **可替换占位节点**，用于把「状态估计 → WBC → 底座/臂/夹爪」接到现有 `pr2_mujoco_sim`。**真实 QP 全身控制**需在 `pr2_wbc_coordinator` 内替换为优化求解；当前协调器只负责 **参考汇总与话题对接**。

左臂 IK 节点 `pr2_left_arm_ik` 的算法、参数与调参说明见 **[README_IK.md](README_IK.md)**（与本栈配合时需改 `joint_command_topic`，见下文「IK 接到 WBC」）。

## 框图 ↔ 节点

| 框图模块 | 节点（可执行名） | 话题 / 说明 |
|----------|------------------|-------------|
| 状态估计 | `pr2_state_estimator` | `joint_states`+`odom` → `state/joint_states`、`state/odom` |
| 导纳（可选） | `pr2_admittance_stub` | 占位：订阅 `wbc/external_wrench`，发 `wbc/admittance_accel`（零） |
| 零空间（可选） | `pr2_null_space_stub` | 占位：发 `wbc/q_nominal`（JSON 字符串） |
| WBC 求解器 | `pr2_wbc_coordinator` | **占位**：汇总参考 → `cmd_vel` + `joint_commands` |
| 底座积分器 | `pr2_base_accel_integrator` | `wbc/base_acceleration`（`Accel`）→ `wbc/reference/cmd_vel` |
| 手臂 dynamics 映射 | （占位由 WBC 输出力矩） | 向 `wbc/reference/joint_command` 发 `JointState.effort` |
| 夹爪 | （经 WBC） | 向 `wbc/reference/gripper_position` 发 `Float64` 或 `joint_command.position` |

## WBC 协调器输入 / 输出（占位）

**订阅**

- `wbc/reference/cmd_vel`（`Twist`）— 底盘速度（可由积分器提供）
- `wbc/reference/joint_command`（`JointState`）— 臂等关节：通常 `effort` 为力矩
- `wbc/reference/gripper_position`（`Float64`）— 左夹爪目标（`l_gripper_l_finger_joint`）

**发布**

- `cmd_vel` → 仿真 `pr2_mujoco_sim`
- `joint_commands` → 仿真

## 一键启动栈（仿真 + 状态 + WBC）

```bash
cd /workspace/pr2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pr2_mujoco_bridge pr2_mobile_manipulator_stack.launch.py
```

## 分步运行示例

1. **仿真**（`demo_motion:=false`）
2. **WBC 协调器**
3. **发给底盘**（二选一）  
   - 直接：`ros2 topic pub /wbc/reference/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`  
   - 或先加速度再积分：启动 `pr2_base_accel_integrator`，向 `wbc/base_acceleration` 发 `Accel`
4. **IK 接到 WBC**（不要直接发 `/joint_commands`，避免与协调器冲突）：

```bash
ros2 run pr2_mujoco_bridge pr2_left_arm_ik --ros-args -p joint_command_topic:=wbc/reference/joint_command
```

5. **夹爪**（可选）

```bash
ros2 topic pub /wbc/reference/gripper_position std_msgs/msg/Float64 "{data: 0.3}"
```

6. **可选桩**

```bash
ros2 run pr2_mujoco_bridge pr2_admittance_stub
ros2 run pr2_mujoco_bridge pr2_null_space_stub
```

## 注意

- 同一时间应只有 **一个** 节点发布 `joint_commands` / `cmd_vel` 到仿真（本栈由 `pr2_wbc_coordinator` 汇总）。
- 不使用本栈时，可照常直接向 `pr2_mujoco_sim` 发 `cmd_vel` 与 `joint_commands`（或仅用 IK）。
