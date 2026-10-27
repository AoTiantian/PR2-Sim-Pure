from pathlib import Path
import json
import numpy as np

HERE=Path(__file__).resolve().parent
s=json.loads((HERE/"runtime_summary.json").read_text())
metrics=["hand_xy_rmse_cm","human_xy_force_rms_N","attitude_peak_deg","attitude_rms_deg",
         "peak_roll_deg","peak_pitch_deg","peak_board_tilt_deg","final_yaw_error_deg",
         "human_task_torque_rms_Nm","human_mean_vertical_force_N","both_contact_fraction",
         "human_force_saturation_fraction","human_torque_saturation_fraction"]
groups={}
for task in ["S11","sinusoid","S12"]:
    groups[task]={}
    for mode in ["passive","active"]:
        runs=[v for v in s.values() if v["task"]==task and v["mode"]==mode]
        g={"attempts":len(runs),"with_history":sum("history" in x for x in runs),
           "completed":sum(bool(x.get("full_task_window")) for x in runs),
           "ended_grasping":sum(bool(x.get("final_grasped")) for x in runs)}
        for metric in metrics:
            values=[v[metric] for v in runs if metric in v]
            if values:
                g[metric]={"mean":float(np.mean(values)),"min":float(min(values)),"max":float(max(values))}
        groups[task][mode]=g
(HERE/"group_statistics.json").write_text(json.dumps(groups,indent=2))


def span(task,mode,key,places=2):
    d=groups[task][mode].get(key)
    return "缺失" if d is None else f'{d["min"]:.{places}f}–{d["max"]:.{places}f}'


names={"S11":"直线S11","sinusoid":"正弦","S12":"转圈S12"}
rows="".join(f'|{names[t]}|{span(t,"passive","hand_xy_rmse_cm")}|{span(t,"active","hand_xy_rmse_cm")}|{span(t,"passive","human_xy_force_rms_N")}|{span(t,"active","human_xy_force_rms_N")}|\n' for t in groups)
posture="".join(f'|{names[t]}|{span(t,"passive","attitude_peak_deg")}|{span(t,"active","attitude_peak_deg")}|{span(t,"passive","peak_roll_deg")}|{span(t,"active","peak_roll_deg")}|{span(t,"passive","peak_board_tilt_deg")}|{span(t,"active","peak_board_tilt_deg")}|\n' for t in groups)
status="".join(f'|{names[t]}|{mode}|{g["attempts"]}|{g["completed"]}|{g["ended_grasping"]}|\n' for t,modes in groups.items() for mode,g in modes.items())
individual=""
for tag,m in s.items():
    def fmt(k,digits=3):return f'{m[k]:.{digits}f}' if k in m else '缺失'
    individual+=f'|{tag}|{fmt("hand_xy_rmse_cm")}|{fmt("human_xy_force_rms_N")}|{fmt("attitude_peak_deg")}|{fmt("peak_roll_deg")}|{fmt("final_yaw_error_deg")}|{fmt("both_contact_fraction",5)}|{fmt("longest_both_contact_interruption_ms",1)}|\n'
active=[v for v in s.values() if v["mode"]=="active" and "angular_allocation_error_max_rad_s" in v]
err=max([v["angular_allocation_error_max_rad_s"] for v in active],default=float('nan'))
interruption=max([v.get("longest_both_contact_interruption_ms",0) for v in s.values()],default=0)
report=f'''# 底盘已知轨迹前馈：直线、正弦与转圈验证

2026-10-27。这是对前一S22原型的跨任务测试，不是新调参结果。每项原被动与主动各3次，交错运行；完整顺序见schedule.json。

本轮结果：直线的位置和水平交互力改善最明显，正弦也有改善但仍有轨迹形状偏差。转圈的位置误差和水平交互力降低，但实际滚转和板面倾角显著增大；终点偏航误差也由被动的约0.23–1.62°落后，变为主动的约1.94–3.82°超前。因此当前不能称该方法在转圈上全面优于被动控制，也不应仅因角导纳计算保留就宣称姿态表现不受影响。转圈倾斜代价的根因尚未通过单因素干预确定。

## 位置与水平交互力

统计各任务原TRACK+HOLD窗口，RMSE为 `sqrt(mean(ex²+ey²))`，合力RMS为 `sqrt(mean(Fx²+Fy²))`。表中为三次的最小–最大值，不是置信区间。人体模型和计划与两种控制条件一致。固定人体阻抗会使位置误差与交互力相关，两项不构成两份独立的机制证明；水平力降低也不等于整体承重负担降低。

|任务|被动XY RMSE cm|主动XY RMSE cm|被动人端水平力RMS N|主动人端水平力RMS N|
|---|---:|---:|---:|---:|
{rows}
## 姿态代价

|任务|被动总姿态误差峰值 °|主动总姿态误差峰值 °|被动滚转峰值 °|主动滚转峰值 °|被动板面倾角峰值 °|主动板面倾角峰值 °|
|---|---:|---:|---:|---:|---:|---:|
{posture}
总姿态误差是期望与实际的SO(3)旋转差；滚转为实际板姿态Euler-X；板面倾角为板面法向相对世界竖直方向的夹角。这些指标不可混称。转圈任务必须同时关注水平跟踪和竖直平面倾斜，不能用较小的总旋转误差遮盖更大的滚转。

## 完成情况与角导纳

|任务|条件|尝试次数|完整任务窗口次数|末端仍夹持次数|
|---|---|---:|---:|---:|
{status}
主动运行中记录的角任务分配误差最大绝对分量为{err:.3g}rad/s。是否发生角任务不可行、接触率和求解/子进程错误，均逐次保存在runtime_summary.json。指令层保留角导纳不代表实物/仿真姿态与被动控制相同；主动运动改变交互wrench，真实执行也存在误差。不能仅据QP残差小宣称实际角导纳频响已验证。

所有运行中记录到的最长双侧接触力中断为{interruption:.1f}ms；逐次接触率和末端状态见下表。接触率为两侧均有正法向力的比例，不能当作硬锁定的抓取状态。

## 路径与公平性

算法与前一S22原型相同，angular_priority.py逐字一致；新增PlannedBasePath只支持独立正弦及转圈几何。S22路径等价、解析速度与数值微分、平面约束及原有分配器性质共7项测试通过。生产源码完整性见production_integrity.json。

S12底盘计划根据已知任务路径与初始参考点到底盘偏移[-2.395,.65,0]m生成，而不是直接复制人的半圆。正弦只给底盘XY前馈，没有竖直或姿态正弦前馈。机器人参考源均为独立配置，不订阅人的实时目标来生成计划。双方已知同一任务计划、时钟同步是实验前提。

延续前一原型的0.25m/s、0.10rad/s计划速度上限，没有为转圈另行提高上限。离线审计确认正弦有约6.35%的计划平移速度被裁剪；S12约48.68%的计划平移和62.17%的计划偏航速度被裁剪。运行时裁剪比例见各次日志。此处仅报告裁剪事实，没有通过单因素对照确定它是否是任何姿态代价的根因，也没有把改上限后的结果混入本表。

原S12水平平移与偏航导纳阻尼仍为20；其他任务保留原参数。摩擦系数、抓取深度、质量、人体阻抗、机器人姿态跟踪关闭状态和10Hz转发均保持原配置。所有新文件均在实验目录，未替换生产控制器或前一S22结果。

## 全部运行

|运行|XY RMSE cm|水平力RMS N|姿态误差峰值 °|滚转峰值 °|终点偏航误差 °|双侧接触率|最长中断 ms|
|---|---:|---:|---:|---:|---:|---:|---:|
{individual}
完整CSV和原始话题数据在各运行目录；source_hashes、launch/params、control_condition、exit.json及allocation日志均保留。全部原始运行都纳入报告；如有缺失字段或任务失败，不把它们当作零误差或悄悄丢弃。

## 图与复现

- task_comparison.png/.pdf：三项任务全部轨迹、姿态误差和人端水平力的均值及min/max包络。
- metric_comparison.png/.pdf：三项任务各次结果，特别显示滚转代价。
- PROTOCOL.md、planned_velocity_audit.json：任务计划、几何和限幅说明。

容器中先source ROS环境和/workspace/pr2_ws/install/setup.bash，再执行：

```bash
cd /workspace
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 python3 results/base_feedforward_paths_20261027/run_case.py demo_S12_01 --task S12 --mode active --viewer
```

使用新tag避免覆盖记录。`--task S11`、`--task sinusoid`选择其他任务；`--mode passive`为原被动对照。分析顺序为summarize.py → validate_results.py → build_report.py → plot_results.py。

当前仍未验证人体偏离已知计划、未知速度、障碍物或硬件，也没有对实际末端施加独立测试力矩进行完整导纳辨识。不同任务的实测姿态优劣应按上述数据分别判断。
'''
(HERE/"REPORT.md").write_text(report)
print(rows)
print(posture)
