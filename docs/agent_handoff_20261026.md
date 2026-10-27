# 交接补充：纯人 S12 已按干预与重复验证修复（2026-10-26）

本文件补充 9.26 交接，覆盖“纯人转体失败”部分。机器人只能由腕部 wrench 驱动的实验约束仍然有效；本轮没有修改机器人底盘/WBC、木板 XML、数值增益或物理步长。

用户要求：判断原因后必须实际修改并验证改善；无改善则回退并重新判断，不能只报告猜测。

## 保留结果

实际代码四组对照，每组独立进程重置重跑三次，组内物理状态序列逐元素一致：

| 条件 | 最大总姿态误差 | 长轴角速度峰值 | 力矩饱和 | 触地步数 |
|---|---:|---:|---:|---:|
| 原实现 | 102.154° | 185.624 rad/s | 75.82% | 119 |
| 仅角阻抗在板体系 | 2.732° | 0.013013 rad/s | 0% | 0 |
| 仅修反馈时序 | 70.483° | 83.322 rad/s | 69.92% | 80 |
| 两者均改，已保留 | 2.691° | 0.013007 rad/s | 0% | 0 |

据此确认：固定世界系各向异性角阻抗是当前 S12 的主要问题；一拍派生状态滞后加重不稳定，但单独修时序不足以解决。

## 实际修改

- `human_impedance_6d.py` 支持 `angular_frame: body/world`。板体系模式通过当前姿态旋转角误差与角速度误差，在板体系按原 K、D 求力矩，再转回世界系并使用原限幅。
- `comparison_config.py` 读取该字段；旧 YAML 缺字段仍为 world，保留复现能力。
- 主 `transport_comparison.yaml` 显式使用 body；数值 K=[30,400,100]、D=[0.5,10,5] 保留。默认场景仍是 sinusoid。
- `human_only_adapter.py` 改用 `mj_step1 → 反馈 → mj_step2`，避免使用积分前一拍的派生位姿/速度。
- 新增/扩展了物理反馈新鲜度、转向后物理轴增益一致性、旧配置兼容等测试，整个包 29 项通过。

sinusoid、S11 纯人回归通过。正常 launch + viewer 完成 S12，记录 1151 个样本，最大姿态误差 2.690°，wrench 饱和为零。viewer 数据保存后的关闭阶段仍有既存 GLX/清理异常，这与物理运行完成分开记录。

最终 S12 偏航 180.008°，手端位置 RMSE 从 0.133920 m 降到 0.044164 m，后者主要是无重力前馈纯阻抗的竖直稳态偏差。

## 证据位置与边界

详细报告、原始数据、四组配置、源码校验值和修改前/后快照：
`results/validated_humanonly_20261026/REPORT.md`。
对照图：`results/validated_humanonly_20261026/factorial_validation.png`。
正常 viewer 数据：`results/validated_humanonly_20261026/formal_viewer/run_20261026_035952/`。
S12 复现配置：`results/validated_humanonly_20261026/S12_retained.yaml`，作为 launch 的 `config_file:=` 传入。

本轮只验收纯人。共享人端模型的人机入口也会读取新 frame 选择，以后做人机比较必须使用同一新配置重建基线，不可把旧 world-frame 人机结果与新纯人结果直接比较。机器人 S12 跟随问题仍未修复。

原有 `.devcontainer/devcontainer.json`、`pr2_sim_ros.py`、`transport_comparison.launch.py` 未提交差异与本轮开始前逐字一致；未覆盖用户既有工作。未提交代码。
