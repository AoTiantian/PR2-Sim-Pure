# 移动机械臂控制栈（与手绘框图对应）

本目录实现的是一套 **可运行的移动机械臂控制基线**（状态估计 + 参考汇总 + 导纳 + 零空间姿态保持），用于把「状态估计 → WBC 协调 → 底座/臂/夹爪」接到现有 `pr2_mujoco_sim`。若后续做真实 QP-WBC，可在保持话题接口不变的前提下替换协调器内部求解。

左臂 IK 节点 `pr2_left_arm_ik` 的算法、参数与调参说明见 **[README_IK.md](README_IK.md)**（与本栈配合时需改 `joint_command_topic`，见下文「IK 接到 WBC」）。

## 框图 ↔ 节点


| 框图模块           | 节点（可执行名）                                               | 话题 / 说明                                                                                                    |
| -------------- | ------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------- |
| 状态估计           | `pr2_state_estimator`                                  | `joint_states`+`odom` → `state/joint_states`、`state/odom`，并发布 `state/base_twist`、`state/base_acceleration` |
| 导纳（可选）         | `pr2_admittance_controller`（兼容名：`pr2_admittance_stub`） | 订阅 `wbc/external_wrench`，按 6 轴导纳动力学输出 `wbc/base_acceleration`（`Accel`）                                     |
| 零空间（可选）        | `pr2_null_space_stub`                                  | 读取 `state/joint_states` 自动捕获/发布 `wbc/q_nominal`（含 `kp/kd/max_effort`）                                      |
| WBC 协调器        | `pr2_wbc_coordinator`                                  | 汇总 `wbc/reference/`* + `wbc/q_nominal`，叠加次级姿态保持力矩，输出 `cmd_vel` + `joint_commands`（带超时保护）                   |
| 底座积分器          | `pr2_base_accel_integrator`                            | `wbc/base_acceleration`（`Accel`）→ `wbc/reference/cmd_vel`                                                  |
| 手臂 dynamics 映射 | （经协调器）                                                 | 向 `wbc/reference/joint_command` 发 `JointState.effort`（可与 null-space 力矩叠加）                                  |
| 夹爪             | （经 WBC）                                                | 向 `wbc/reference/gripper_position` 发 `Float64` 或 `joint_command.position`                                  |


## WBC 协调器输入 / 输出（实装）

**订阅**

- `wbc/reference/cmd_vel`（`Twist`）— 底盘速度（可由积分器提供）
- `wbc/reference/joint_command`（`JointState`）— 臂等关节：通常 `effort` 为力矩
- `wbc/reference/gripper_position`（`Float64`）— 左夹爪目标（`l_gripper_l_finger_joint`）
- `wbc/q_nominal`（`String` JSON）— 零空间名义姿态（`joints`、`q_nominal`、`kp`、`kd`、`max_effort`）
- `state/joint_states`（`JointState`）— 当前关节状态，用于 null-space 力矩计算

**发布**

- `cmd_vel` → 仿真 `pr2_mujoco_sim`
- `joint_commands` → 仿真

协调器会对 `wbc/reference/cmd_vel`、`wbc/reference/joint_command`、`wbc/reference/gripper_position` 做输入超时保护（默认 0.3s）；超时后自动回零/清空，避免陈旧控制量持续作用。

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

1. **夹爪**（可选）

```bash
ros2 topic pub /wbc/reference/gripper_position std_msgs/msg/Float64 "{data: 0.3}"
```

1. **导纳 + 底座积分器（可选）**

```bash
ros2 run pr2_mujoco_bridge pr2_admittance_controller
ros2 run pr2_mujoco_bridge pr2_base_accel_integrator
```

可调参数示例（更“软”的平面顺应）：

```bash
ros2 run pr2_mujoco_bridge pr2_admittance_controller --ros-args \
  -p mass_linear:="[10.0, 10.0, 20.0]" \
  -p damping_linear:="[60.0, 60.0, 140.0]" \
  -p force_deadzone:="[1.0, 1.0, 1.0]"
```

仍保留原兼容可执行名：

```bash
ros2 run pr2_mujoco_bridge pr2_admittance_stub
ros2 run pr2_mujoco_bridge pr2_null_space_stub
```

## `wbc/q_nominal` JSON 格式（由 `pr2_null_space_stub` 发布）

```json
{
  "mode": "posture_hold",
  "joints": ["l_shoulder_pan_joint", "..."],
  "q_nominal": [0.0, "..."],
  "kp": 15.0,
  "kd": 2.0,
  "max_effort": 10.0
}
```

## 注意

- 同一时间应只有 **一个** 节点发布 `joint_commands` / `cmd_vel` 到仿真（本栈由 `pr2_wbc_coordinator` 汇总）。
- 不使用本栈时，可照常直接向 `pr2_mujoco_sim` 发 `cmd_vel` 与 `joint_commands`（或仅用 IK）。

## 一键导纳验证（不加零空间）

可直接用以下单命令自动完成验证：

```bash
ros2 launch pr2_mujoco_bridge pr2_admittance_validation.launch.py
```

如果你不想每次手动 `build + source`，可直接运行仓库内一键脚本：

```bash
/workspace/pr2_ws/src/pr2_mujoco_bridge/scripts/run_admittance_validation_omni.sh
```

上面脚本默认调用 `pr2_admittance_validation_omni.launch.py`（预设 +X/+Y/-X/-Y 分段施力）。

该 launch 会自动启动：

- `pr2_mujoco_sim`（`demo_motion=false`）
- `pr2_state_estimator`
- `pr2_wbc_coordinator`（`nullspace_enable=false`）
- `pr2_admittance_controller`
- `pr2_base_accel_integrator`
- `pr2_admittance_validator`（注入虚拟外力 step，并打印 PASS/FAIL）

可调参数示例：

```bash
ros2 launch pr2_mujoco_bridge pr2_admittance_validation.launch.py \
  force_x:=40.0 duration_sec:=6.0 force_start_sec:=2.0 settle_after_sec:=3.0 use_viewer:=false

# 多方向分段施力（验证全向底盘导纳）
ros2 launch pr2_mujoco_bridge pr2_admittance_validation.launch.py \
  use_viewer:=false \
  force_schedule_json:='[
    {"start":2.0,"end":7.0,"fx":80.0},
    {"start":7.0,"end":12.0,"fy":80.0},
    {"start":12.0,"end":17.0,"fx":-80.0},
    {"start":17.0,"end":22.0,"fy":-80.0}
  ]' \
  settle_after_sec:=3.0
```

参数含义：

- `duration_sec`：虚拟外力的作用时长（秒）
- `force_start_sec`：开始施力时刻（秒）
- `settle_after_sec`：卸载后继续观察衰减的时长（秒）

### 验证脚本节点说明

`pr2_admittance_validation.launch.py` 内部会启动 `pr2_admittance_validator`，自动执行：

1. 先等待系统稳定（默认 `force_start_sec=2s`）
2. 在 `wbc/external_wrench` 注入虚拟力 step（默认 `force_x=30N`，持续 `duration_sec`）
3. 统计峰值响应：
  - `peak|wbc/base_acceleration.linear.x|`
  - `peak|cmd_vel.linear.x|`
4. 到达总时长 `force_start_sec + duration_sec + settle_after_sec` 后打印 `PASS/FAIL` 并自动退出

默认通过阈值（可在节点参数中修改）：

- `min_peak_accel_x = 0.12`
- `min_peak_cmd_vx = 0.06`
- `min_peak_accel_planar = 0.12`（分段多方向模式下使用）
- `min_peak_cmd_v_planar = 0.06`（分段多方向模式下使用）
- `min_joint_state_count = 200`（防止仿真节点提前退出仍被误判 PASS）

### 期望现象（当前验证的是底座导纳链路）

- 单方向模式：注入力期间沿施力方向加速，卸载后加速度回落并衰减
- 全向分段模式：底座按 `+X -> +Y -> -X -> -Y` 依次响应，方向应随分段切换

> 说明：上面这套自动验证只覆盖 **底座导纳链路**（`external_wrench -> base_acceleration -> cmd_vel`）。  
> 左臂导纳验证请使用下方“左臂笛卡尔导纳（一键验证）”章节。

## 左臂笛卡尔导纳（一键验证）

已新增左臂笛卡尔导纳链路：

- `pr2_arm_admittance_controller`：`wbc/arm_external_wrench + ee_pose -> ik_target_pose`
- `pr2_left_arm_ik`：`ik_target_pose -> wbc/reference/joint_command`（由协调器汇总后输出）
- `pr2_arm_admittance_validator`：默认一维 `+X` 施力并统计末端位移/左臂力矩峰值（位移默认与控制同为 `odom` 坐标）

左臂导纳控制器已支持 `WrenchStamped.frame_id` 的坐标系变换（默认开启），会把外力从输入坐标系转换到控制坐标系后再计算导纳。当前验证 launch 已把施力、控制、位移判据统一到 `odom`，并在启动日志中打印三元组 `frames(force, disp, control)`。若 `ee_pose` 坐标系与导纳目标坐标系不一致，控制器会抑制输出并告警（`strict_frame_consistency=true`）。
另外，验证 launch 默认开启 `pr2_mujoco_sim.lock_base_motion=true` 与 `use_cmd_vel=false`，用于 arm-only 测试时固定底座并关闭底盘速度通道，减少耦合干扰；并新增 `lock_base_settle_sec`，会先让底座自然落稳再锁定，避免“悬空锁死”。

推荐一键脚本（自动 `build + source + launch`）：

```bash
/workspace/pr2_ws/src/pr2_mujoco_bridge/scripts/run_arm_admittance_validation.sh
```

等价 launch：

```bash
ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py
```

默认是最简单的一维验证（`force_x` step）：

```bash
ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py \
  force_x:=30.0 duration_sec:=6.0 force_start_sec:=2.0 \
  settle_after_sec:=3.0 use_viewer:=false
```

如需后续扩展到多方向，才传 `force_schedule_json`（非空时会覆盖单步模式）。

两组回归命令建议固定如下：

```bash
# 组 A：标准 x 向验证
ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py \
  force_x:=30.0 force_start_sec:=2.0 duration_sec:=6.0 settle_after_sec:=3.0 \
  use_viewer:=false

# 组 B：提高激励幅值（观察耦合上界）
ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py \
  force_x:=45.0 force_start_sec:=2.0 duration_sec:=6.0 settle_after_sec:=3.0 \
  use_viewer:=false
```

建议先跑一条零力稳定性基线（应无明显自激振荡）：

```bash
ros2 launch pr2_mujoco_bridge pr2_arm_admittance_validation.launch.py \
  force_x:=0.0 duration_sec:=3.0 force_start_sec:=2.0 settle_after_sec:=1.0 use_viewer:=false
```

### 左臂导纳验收指标（默认）

- `peak|disp_xy| >= min_peak_disp_xy`（默认 `0.0 m`，禁用；可手动开启）
- `peak|disp_xyz| >= min_peak_disp_xyz`（默认 `0.012 m`）
- `peak|disp_xy| <= max_peak_disp_xy`（默认 `0.25 m`，抑制大幅过冲）
- `peak|disp_xyz| <= max_peak_disp_xyz`（默认 `0.30 m`，抑制大幅过冲）
- `peak|disp_z| <= max_peak_disp_z`（默认 `0.015 m`，约束 x 向测试中的 z 串扰）
- `peak|left_effort| <= max_allowed_left_effort`（默认 `120 Nm`）
- `settle_time <= max_settle_time_sec`（默认 `2.0 s`）
- `rms_disp_after_settle <= max_rms_disp_after_settle`（默认 `0.02 m`）
- `effort_rms_after_settle <= max_effort_rms_after_settle`（默认 `20 Nm`）
- `joint_states_count >= min_joint_state_count`（默认 `200`，防止仿真提前退出误判）

默认基线锁定策略：`baseline_latch_mode=at_force_start`（在施力开始时锁定位姿基线，排除启动瞬态）。

### 调参顺序（建议）

1. **先稳**：提高 `damping_linear`、减小 `max_linear_displacement` / `max_linear_velocity`
2. **再灵敏**：逐步降低 `damping_linear` 或 `mass_linear`
3. **最后边界**：增大一维施力（`force_x`），确认位移限幅和力矩上限都能生效

> 当前 `pr2_arm_admittance_validation.launch.py` 默认已使用“稳态优先”参数（导纳与 IK 都更保守），用于减少剧烈抖动与过冲。

