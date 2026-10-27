# 底盘轨迹前馈实验版本

加入前馈前的被动基线提交为 `9a23694`（修复几个轨迹中的控制问题）。本次提交保存隔离的主动控制原型和证据，生产QP、coordinator、人端阻抗、正式launch及摩擦模型仍与该基线一致。

## 实现与结果

- `results/base_feedforward_20261027/`：首轮S22折线实验、原被动/零前馈消融/主动各3次。`ACTIVE_DESIGN.md`是当前方法；`DESIGN.md`保留早期尚未引入主动平移的方案及其适用边界。
- `results/base_feedforward_paths_20261027/`：扩展到S11、sinusoid和S12，每项主动/被动各3次。`PROTOCOL.md`记录独立底盘计划、转圈几何和保持的速度上限。
- 各目录的 `REPORT.md`、`runtime_summary.json`、`group_statistics.json`和PNG/PDF保存对照结果。

已知路径提供底盘名义运动，原线性导纳作为平移柔顺修正，末端角速度仍来自原角导纳。QP优先保留角速度任务，再优化平移和底盘参考。位置误差反馈增益为0，当前是速度前馈实验。原有底盘滤波和10Hz coordinator不变。

S11、S22及正弦的水平跟踪和水平交互力有所改善；S12虽然水平误差下降，但滚转峰值从被动的2.68–5.55°增加至8.67–14.11°。当前原型没有被推广为所有任务的默认控制器。实际角导纳频响、人体偏离计划时的协调和全局稳定性尚未验证。

## 运行

使用原ROS2 Jazzy/MuJoCo环境和已构建的pr2_ws。工作区在容器中挂载为 `/workspace`，环境依赖与被动基线相同。

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/pr2_ws/install/setup.bash
cd /workspace
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 results/base_feedforward_paths_20261027/run_case.py demo_S11_01 --task S11 --mode active --viewer
```

其他任务用 `--task sinusoid`、`--task S12`或`--task S22`。`--mode passive`直接使用原被动QP。每次换新tag避免覆盖已有记录。`--mode zero_feedforward`是新分配器下的消融条件，不等同于原被动控制。

首轮S22原型也可通过其自身 `run_case.py`运行，默认使用一并提交的 `results/friction_paths_20261026/S22.yaml`。两轮均使用一并提交的只读 `logger.py`。

## 证据保存范围

27次原始 `history.csv`以无损 `evidence/<tag>.csv.gz`保存。`evidence_manifest.json`记录原路径、SHA256及压缩文件路径；`run_provenance.json`汇总每次控制条件、源码哈希、参数、退出状态、通用manifest和原始metrics。

逐步调试日志、ROS日志、`raw.npz`及旧源码副本约900MB，保留在原机器，未纳入Git。报告中的原运行目录路径在该机器仍有效；在新检出中请使用压缩CSV和摘要复核主要结果，或重新运行以生成完整话题及接触日志。旧的接触统计和源码完整性检查结果以提交的JSON保留。

校验压缩证据并重算水平RMSE与水平力RMS：

```bash
python3 results/base_feedforward_paths_20261027/verify_evidence.py
```

完整原始运行数据存在时，按各目录说明运行 `summarize.py`、`validate_results.py`和绘图脚本。仅有压缩CSV的新检出不足以重算依赖逐步接触日志的统计或依赖raw.npz的角速度跟踪诊断。

性质测试（source上述ROS工作区后运行）：

```bash
python3 -m pytest results/base_feedforward_20261027/test_angular_priority.py results/base_feedforward_paths_20261027/test_planned_base_path.py -q
```
