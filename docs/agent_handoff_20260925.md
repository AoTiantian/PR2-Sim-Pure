# 交接文档 v2：人机协同搬运仿真项目（2026-09-25）

> 本文档取代 `docs/agent_handoff_virtual_human_transport.md`（9.10 版）。该版的"已完成/结论"部分仍然有效，但**路径、频率默认值、场景定义均已变化**，以本文为准。指标计算的大白话规范见 `docs/advanced_metrics_spec.md`；周报见 `docs/weekly_report_0910.md`。

---

## 一、项目是什么

MuJoCo + ROS 2 Jazzy 仿真：**虚拟人（弹簧阻尼阻抗程序）与 PR2 机器人协同搬运一块 2 m 长板**。对比两种条件：纯人（human_only）vs 人机（human_robot，PR2 纯力随动导纳控制）。核心研究问题：加入机器人后，人是否在跟踪质量不降的前提下减负。

## 二、运行环境与入口

- **容器**：`relaxed_villani`（运行中）；项目挂载在容器 `/workspace` = 宿主 `/home/chlorine/ros2_mujoco_project`。
- **进入**：`docker exec -it relaxed_villani bash`，然后：
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /workspace/pr2_ws/install/setup.bash
  cd /workspace/pr2_ws
  ```
- **跑一次实验**（自动退出、自动出图、自动落盘）：
  ```bash
  ros2 launch pr2_virtual_human transport_comparison.launch.py condition:=human_robot robot_mode:=admittance use_viewer:=false experiment_id:=<名字>
  ```
  condition 取 `human_only` / `human_robot`。结果落在 `results/runs/run_<时间戳>/`（history.csv + metrics.json + run_manifest.json + experiment_config.yaml + 两张 6D 图）。哪个场景由 yaml 的 `trajectory.active` 决定（见下）。
- **代码是符号链接安装**：改源码/yaml 即时生效，无需重建。

## 三、频率全景（当前默认）

| 环节 | 频率 | 备注 |
|---|---|---|
| MuJoCo 物理步长 | 500 Hz（0.002 s） | 主循环实时配速，仿真:真实=1:1 |
| 虚拟人阻抗（出力施加到板） | 500 Hz | 每物理步一次 |
| 关节 CTC（PD，kp=1000/kd=200） | 500 Hz | 每物理步 |
| 轮子/舵向执行器伺服（MuJoCo 内建） | 500 Hz | 物理步内 |
| 末端导纳 + QP（测力→末端速度→关节速度指令） | 100 Hz | `rate_hz` 参数 |
| 指令转发 coordinator → 仿真 | **10 Hz（默认已改）** | launch `robot_cmd_rate_hz`；QP 内环仍 100 Hz |
| 力测量采样（QP 读 wrist wrench） | 100 Hz | `robot_wrench_rate_hz` 可限速 |
| 数据记录 | 50 Hz | `publish_rate_hz` |

⚠️ **指令转发默认 10 Hz 是 9.25 新改的实验条件**（用户要求）。历史基线（final_aligned、demo_20260910、六个 9.17 运行）全是 100 Hz 指令下跑的，**与新默认的运行不可比**。要复现旧条件：`robot_cmd_rate_hz:=100`。

## 四、已确认的结论（有实验证据，可直接引用）

1. **六维误差的机理分类**（纯人条件）：
   - 位置 X/Y ≈ 0、姿态 X/Z ≈ 0：自由板无阻力，阻抗准完美跟踪（亚毫米~毫米级）；
   - 位置 Z 塌陷 ≈ 0.044 m：**板重 ÷ K_z(400)**，纯阻抗无积分/无重力前馈的设计性稳态误差（实测与公式分毫不差）；
   - 姿态 Y 扭转 ≈ 0.044 rad：**倾覆力矩 r×F ÷ K_θ(400)**，同一机理的转动版；人端任务力矩 +17.59 N·m 恰好抵消自然力矩 -17.59（实测和为 0.00）。
2. **纯导纳（K=0）机器人端稳态垂直出力结构性为零**：有力即漂移、漂移至力消失。这是"机器人没减负"的机理根源，不是调参问题。
3. **仿真静力学自洽**：稳态下合力/合力矩满足平衡方程（人端力+板重+焊点反力=0；力矩同理），数据可信。
4. **X 横向执行赤字与电机强度无关**（已证伪：舵向力矩 ±6.5→±30 无效、轮伺服 kv10→30 对位置无效、门控参数无效、位姿辅助无效）。QP 求解层无罪（指令可达率 99%）。赤字在底盘-接触动力学环节，**最强剩余假设：轮-地接触摩擦**，待裸底盘阶跃辨识。
5. **轮伺服强化（kv10→30、±7→±20 N·m）的真实效果**：显著抑制机动中的姿态抖动（S22 roll/pitch RMSE 0.030/0.013→0.009/0.010）。但它**不改善位置跟踪**，且**会消灭"指令 10 Hz 涌现扛重"现象**（见第 7 条）。
6. **指令转发 10 Hz 在软轮伺服下曾涌现 ~9.5-12.9 N 平均支撑**——但经 6 次（旧代码）+3 次（新代码）重复实验证明**不可复现、非代码属性**，是欠受控回路对消息时序的高方差响应（机制：指令保持把连续"泄力"切成"锁住+下潜"的粘滑过程）。**任何引用该现象作为"机器人会扛重"的结论都不成立。**
7. **S12 旧定义（直线往返+180°yaw）运动学自相矛盾**（手端位置路径与转向需求冲突），已改为手端走半径 1 m 半圆弧（5 waypoint），修复后板子真实转体 163°(人机)/180°(纯人)。
8. **滚转（姿态 X）抖振**：绕板长轴惯量极小（倾覆轴的 1/350），阻尼受离散稳定裕度锁死（含传输延迟，上限 ~0.57，**不可加大阻尼**，5.0/0.8 均自激发散）。有效修法：角刚度 X 100→30（当前值，阻尼比 0.19→0.35，尖峰 0.244→0.060 rad）。
9. **滚转刚度 ↔ 垂直分担耦合**（10 Hz 粘滑状态下）：高滚转刚度（100）时机器人扛 12.9 N；低滚转刚度（30）时板子用倾斜释放两端矛盾，机器人只扛 2.4 N。**注意：该效应量在运行间噪声带内，是趋势性发现而非确定性因果。**
10. **仿真静力学自洽**：人端任务力矩精确抵消 r×F（human_only 和=0.00），合力/合力矩平衡成立，数据可信。

## 五、当前工作区状态（重要！有未提交改动）

HEAD = `d08946a 暂存`（用户自己提交，含 waypoint 引擎、runs/ 布局、周报等）。**工作区有 4 个未提交修改 + 1 个未跟踪文件**，全部是有意保留的状态：

| 文件 | 改动内容 | 为什么 |
|---|---|---|
| `pr2_ws/src/pr2_virtual_human/launch/transport_comparison.launch.py` | ① `robot_cmd_rate_hz` 默认 100→**10**；② 新增 `robot_wrench_rate_hz` 参数；③ 场景 robot_overrides 合并到 QP 参数 | 用户要求默认 10 Hz |
| `pr2_ws/src/pr2_wbc_admittance_control/.../pr2_qp_whole_body_admittance.py` | 新增 `wrench_update_rate_hz` 力测量节流参数（默认关） | 支持力传感器限速实验 |
| `pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml` | ① `output_root` → `results/runs`；② trajectory 改 presets+active 结构，**S12 重定义为半圆弧**；③ 角刚度 X=**100**（复刻态，注释有说明） | 见上文 |
| `unitree_mujoco/.../robot_pr2_board_grasp.xml` | 轮伺服**保持软版 kv10/±7**（相对 HEAD 是"回退"） | 用户要求复刻 cmd10hz 条件 |
| `docs/advanced_metrics_spec.md`（未跟踪） | 12 项进阶指标大白话规范 | 新文档 |

另有 `results/oldcode_repro/`（旧代码 6 次重复运行，是"12.9 N 不可复现"的证据数据）和 `results/archive/`（两批历史数据归档），确认无用可删。

**提交策略建议**：轮伺服回退与指令默认 10 Hz 都是为复刻服务的；如果之后要回到"正式对比"状态，需要决定这两个保留还是恢复，并记录为实验条件变更。

## 六、遗留问题与下一步（按优先级）

1. **X 横向执行赤字根因**（连带人机条件偏航误差）：电机强度已证伪，最强假设轮-地接触摩擦。**下一步：裸底盘阶跃响应辨识**（去掉板和人，直接发速度阶跃，量增益/滞后/滑移），可顺带回答"10 Hz 指令扛重"的精确机制。
2. **垂直耦联振荡**（2.4 Hz，人机条件）：人端加阻尼已证伪（恶化 10 倍）。**下一步：用 damp_fix 数据做人-板-臂耦联模态辨识**。
3. **设计性稳态误差的正规化**（Z 塌陷/Y 扭转）：若需要"机器人真扛重"的结论，用显式机制（`stiffness_linear` Z>0、`ctc_vertical_hold_force_limit`、重力前馈）作为**新条件**实现——不要靠调接口频率碰。
4. **剩余 6 个场景**（S13/S21/S23/S31/S32/S33）：纯配置工作，S12 半圆弧的实现模式可直接复用。
5. **提交策略**：见第五节。
6. **进阶指标接入**：9 项可从现有数据离线计算（清单见 advanced_metrics_spec.md 文末），接入点在 `comparison_metrics.py`。

## 七、陷阱清单（前人踩过的坑，务必读）

1. **机器人端 Z 支撑力是涌现量**：随指令接口频率、轮伺服刚度、场景动态大幅变化（实测 0~19 N），**不可引用单次运行的支撑值作为结论**。
2. **角阻尼不可加大**：滚转轴阻尼上限 ~0.57（离散+传输延迟），0.8/5.0 均自激发散。
3. **容器里 `grep` 是个 shell 函数且行为异常**：一律用 `/bin/grep`；文件含二进制字节时加 `-a`。
4. **宿主与容器的 cwd 每条命令后重置**：复合命令里自己带 `cd`；宿主上部分 `build/` 目录视图与容器不一致，以容器内为准。
5. **launch 传参格式是 `:=`**（`experiment_id:=xxx`），冒号写单个会报 malformed。
6. **对比指标要用同窗口同口径**：力/支撑均值对时间窗口敏感（同一运行不同窗口可差 3 N+），跨运行比较必须统一窗口。
7. **单次运行不作数**：支撑力等涌现量逐次漂移 2~13 N，任何结论至少 3 次重复。
8. **运行粘住/物理发散的处置**：`docker exec` 里 `pkill -f pr2_mujoco_sim` 等清理进程；参数组合发散的例子见"门控放宽+强执行器"实验。
9. **分析数据用成对差值**（人机−纯人）而非绝对值；支撑力等涌现量逐次漂移大。
10. **git**：分支 `feat_python`；用户会自行提交（HEAD 已两次变动），操作前先看 `git log/status`。

## 八、数据索引

| 位置 | 内容 |
|---|---|
| `results/runs/run_*` | **最新**：六次正式运行（S11/S22/S12 × 纯人/人机，9.17）+ 本周验证运行 |
| `results/oldcode_repro/` | 旧代码 6 次重复（"12.9 N 不可复现"的证据） |
| `results/archive/transport_comparison_20260826_0916/` | 全部历史：正式基线（final/final_aligned/repeatability）、cmd10hz、damp_fix、opt_* 等 |
| `results/archive/runs_dev_20260916/` | 场景开发期的 13 个过程运行 |
| `docs/weekly_report_0910.md` | 周报（跟踪问题总结 + 10 Hz 前后对比 + 场景总结） |
| `docs/advanced_metrics_spec.md` | 12 项进阶指标大白话规范 |
| `docs/cooperative_transport_evaluation_metrics_review.md` | 26 篇文献指标调研（公式原版） |
| `scripts/diag_vdes_vs_actual.py`、`scripts/diag_base_gain.py` | 诊断脚本（QP 指令 vs 实际、底盘执行增益），用法见文件头注释 |

## 九、关键数字速查（human_robot，sinusoid，100 Hz 指令，旧基线）

- 位置 RMSE：X 0.117 / Y 0.022 / Z 0.048 m；纯人：0.0014 / 0.0038 / 0.044 m
- 人端力 RMS：X 11.8 / Y 2.3 / Z 19.4 N；纯人：0.15 / 0.59 / 17.67 N
- 机器人端平均垂直支撑：~2.5 N（纯导纳结构性近零）
- QP 指令可达率 99%，底盘实际执行 ~43%（滞后 0.65 s）
- 板重 17.66 N；人端 K=100 N/m、K_θ=100/400/100 N·m/rad；导纳 B=80/80/120、K=0
