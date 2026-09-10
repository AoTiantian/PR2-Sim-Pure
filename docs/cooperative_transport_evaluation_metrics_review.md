# 人机协同搬运效果评价指标简要调研

## 1. 调研目标

本文面向“人单独搬运”与“人机协同搬运”的对照实验，评价两个问题：

1. 搬运任务是否完成得准确、稳定、快速且平滑；
2. 加入机器人后，人的物理负担是否下降，同时没有引入明显的人机对抗或机器人控制代价。

文献中与本项目最接近的是 COLA 的长物体协同搬运实验。该文直接采用线速度误差、角速度误差、物体两端高度差和平均外力评价协调性、物体稳定性及人的负担 [1]。扩展物体协同操作研究还总结了交互力、轨迹 RMSE、jerk、力矩变化、能量、努力程度及完成时间等常用指标 [2]。

## 2. 建议指标

### 2.1 任务完成质量与物体稳定性

| 指标 | 直接测量量 | 建议计算方式 | 物理意义 | 出处 |
|---|---|---|---|---|
| 位置跟踪误差 | 木板质心或人端测量点的期望位置 $p_d(t)$、实际位置 $p(t)$ | 三轴 RMSE，并报告合成 RMSE、最大误差 | 衡量木板是否准确沿目标轨迹搬运，是最直接的任务精度指标 | [2], [3] |
| 姿态跟踪误差 | 期望四元数 $q_d(t)$、实际四元数 $q(t)$ | 四元数相对旋转的旋转向量/测地角 RMSE；同时报告 roll、pitch、yaw 分量 | 衡量木板朝向控制质量；对长板而言，小角度误差也会被杠杆长度放大成较大的端点位移 | [2], [9] |
| 线速度跟踪误差 | 木板或双方抓取点的期望线速度 $v_d(t)$、实际线速度 $v(t)$ | $\lVert v_d-v\rVert$ 的均值或 RMSE，另报 XYZ 分量 | 衡量机器人能否及时跟随人的平移意图以及双方移动协调性 | [1] |
| 角速度跟踪误差 | 期望角速度 $\omega_d(t)$、实际角速度 $\omega(t)$ | $\lVert\omega_d-\omega\rVert$ 的均值或 RMSE，另报 XYZ 分量 | 衡量转弯和姿态变化时的动态协调能力 | [1] |
| 木板两端高度差 | 人端高度 $z_h(t)$、机器人端高度 $z_r(t)$ | $|z_h-z_r|$ 的均值、RMSE 和峰值 | 直接反映长板是否保持水平以及垂直方向负载协调是否稳定 | [1] |
| 搬运平滑性 | 木板位置时间序列，或速度峰值时刻 | 归一化 jerk、平方 jerk 积分，或单位时间速度峰值数 | jerk 或速度峰值越少，表示振荡、顿挫和频繁纠正越少，协作更自然 | [2], [5], [6] |
| 完成时间 | 任务开始、到达目标并满足稳定条件的时刻 | $T=t_{end}-t_{start}$ | 反映整体搬运效率；必须与精度、安全指标共同分析，避免用牺牲稳定性换取速度 | [5], [7] |
| 成功率 | 每次实验是否到达目标、是否碰撞、是否掉落或失稳 | 成功次数/总次数 | 衡量系统在重复实验中的可靠性和任务可完成性 | [7] |
| 轨迹长度/路径效率 | 木板质心或移动底座的连续位置 | 实际路径长度；也可除以起终点最短路径长度 | 衡量是否存在无效绕行、往返运动或控制振荡 | [7] |

### 2.2 人的负担与人机协作质量

| 指标 | 直接测量量 | 建议计算方式 | 物理意义 | 出处 |
|---|---|---|---|---|
| 人端输入 wrench | 人端三轴力 $F_h(t)$ 和三轴力矩 $\tau_h(t)$ | 各轴均值、RMS、峰值及合力/合力矩 RMS | 直接反映人为了支撑、加速、转向和稳定木板所付出的力学负担；是两种实验条件的核心对比量 | [1], [2], [4] |
| 最小启动力 | 人端水平力与木板/机器人开始持续运动的时刻 | 运动超过速度阈值时对应的最小外力 | 反映机器人对人的顺应性；启动力越小，通常表示人越容易带动协同系统 | [1] |
| 人端输入变化/控制平滑性 | $F_h(t)$、$\tau_h(t)$ | $\dot F_h$、$\dot\tau_h$ 的 RMS/峰值，或 torque-change 指标 | 反映人是否需要快速、频繁地纠正机器人；数值大通常意味着操作紧张、控制振荡或意图理解不佳 | [2] |
| 人端机械功率与做功 | 人端 wrench，以及人端抓取点线速度 $v_h(t)$、角速度 $\omega_h(t)$ | $P_h=F_h^Tv_h+\tau_h^T\omega_h$；积分得到正功、负功和净功 | 区分“真正推动木板的有效输出”和“抵抗系统运动的负功”；比单独看力更能反映人的能量付出和人机对抗 | [3], [8] |
| 垂直负载分担 | 人端与机器人端竖直力 $F_{h,z}(t)$、$F_{r,z}(t)$ | $\eta_h=F_{h,z}/(F_{h,z}+F_{r,z})$，并报告均值和变化范围 | 量化木板重量实际由谁承担。它用于描述自然形成的分配结果，不应被作为固定 50% 的控制参考 | [9], [10] |
| 内力/对抗力 | 两端作用 wrench、抓取矩阵及木板净 wrench | 将两端 wrench 分解为推动木板运动的有效分量与在抓取零空间内相互抵消的内力分量 | 内力不改变木板合运动，却增加双方负担；内力过大表示人机“互相较劲”或力分配不协调 | [11], [12] |
| 协作响应延迟 | 人端动作/力变化时刻、机器人或木板速度响应时刻 | 互相关峰值延迟，或检测到动作到机器人响应的时间差 | 衡量机器人理解并响应人的意图是否及时；延迟过大会造成拖拽感和额外纠正 | [5] |

### 2.3 机器人代价与安全裕度

| 指标 | 直接测量量 | 建议计算方式 | 物理意义 | 出处 |
|---|---|---|---|---|
| 机器人端 wrench | 机器人抓取点三轴力和三轴力矩 | 各轴 RMS、峰值与冲量 | 判断机器人承担了多少外载荷，并监测抓取点是否出现冲击或异常耦合载荷 | [4], [10] |
| 机器人控制努力 | 关节力矩 $\tau_r$、关节速度 $\dot q$、底座速度命令和限幅状态 | $\int\lVert\tau_r\rVert^2dt$、机械能 $\int\tau_r^T\dot qdt$、命令 RMS、饱和时间占比 | 判断性能提升是否依赖过大的执行器输出；饱和率高也说明结果缺少控制余量和鲁棒性 | [2], [13] |
| 峰值交互载荷 | 人端及机器人端 wrench | 合力/合力矩最大值及超过安全阈值的持续时间 | 反映碰撞、突然拉扯或控制发散风险；平均值较低并不能替代峰值安全检查 | [4] |

### 2.4 未来真人实验可增加的指标

| 指标 | 直接测量量 | 建议计算方式 | 物理意义 | 出处 |
|---|---|---|---|---|
| 肌肉活动与协同收缩 | 上肢、肩背或躯干肌肉的表面肌电 sEMG | 归一化 RMS/积分肌电、肌群协同收缩指数 | 直接评价机器人是否降低肌肉负担及潜在肌肉骨骼风险；可补充 wrench 无法体现的姿势代偿 | [14], [15] |
| 人体运动学负担 | 人体 COM、手部位置、关节角、步长、步频和关节活动范围 | 与纯人搬运条件比较均值及变化率 | 判断机器人是否迫使人采用更慢、不自然或代偿性的步态与姿势 | [16] |
| 主观工作负荷与可用性 | NASA-TLX、SUS 或协作平滑性评分 | 量表总分及分项统计 | 客观误差较小不等于人的体验更好；用于评价感知负担、舒适度、信任和易用性 | [1], [7] |

### 2.5 更能体现“协作机制”的进阶指标

下表中的部分公式是依据文献思想为本项目构造的归一化指标，并非声称原论文使用了完全相同的公式。它们比单独比较 RMSE 或力的大小更适合回答“机器人是否真正提供了有效帮助”。

| 指标 | 直接测量量 | 建议计算方式 | 物理意义 | 文献依据 |
|---|---|---|---|---|
| 等性能人力减负率 | 人端 wrench/做功、位置与姿态误差 | 只在两种条件的跟踪误差处于同一置信区间时，计算 $G_h=(J_{h,solo}-J_{h,HRC})/J_{h,solo}$；或绘制“任务误差—人端负担”Pareto 前沿 | 防止机器人通过放宽精度、降低速度来制造“人更省力”的假象；正值才表示在相同搬运质量下真实减负 | [10], [17] |
| 协作增益/相对有用性 | 单人条件和协作条件下统一定义的任务代价 $J$ | $H=(J_{solo}-J_{team})/J_{solo}$；同时分别用平均单人表现和最佳单人表现作基线 | 回答团队是否真的优于单人，以及机器人是否只是把表现拉到人本来就能达到的水平 | [18], [19], [20] |
| 有效 wrench 比率 | 两端六维 wrench、木板位姿/加速度、质量惯量与抓取矩阵 | 分解为产生木板净运动的 $w_{task}$ 与相互抵消的 $w_{int}$，计算 $\eta_w=\lVert w_{task}\rVert/(\lVert w_{task}\rVert+\lVert w_{int}\rVert)$ | 相同的总施力下，该值越高，说明更多力用于搬运而不是人机“较劲”；比只看交互力 RMS 更有解释力 | [11], [12], [21] |
| 对抗功率占比 | 人端和机器人端 wrench、对应抓取点 twist | 计算端口功率 $P_i=w_i^TV_i$；统计负功或双方功率符号相反部分占总绝对功的比例 | 区分“施力大但在传达意图”和“施力大且真正阻碍运动”；可直接定位控制器自激或反向用力阶段 | [8], [21] |
| 触觉沟通效率 | 人端 wrench、机器人响应、轨迹阶段/转向意图标签 | 用人端信号预测未来 $0.1$–$0.5$ s 的运动意图，报告 AUC/F1、预测提前量；再除以人端 wrench RMS 得到单位用力的信息效率 | 评价机器人能否从较小的人力输入中读懂方向和转向，而不是要求人用大力“推醒”机器人 | [8], [22] |
| 力—运动同步性 | 人端力/力矩、机器人端 wrench、木板速度/角速度 | 计算幅值平方相干度、互谱相位和主频；重点报告步态频带及转向频带的相干度 | 能发现时域 RMSE 看不出的周期性不同步、同相自激和反相对抗；也可检验机器人是否放大人的步态扰动 | [22], [23] |
| 协作响应方向性 | 人端信号与机器人/木板运动信号 | 滑窗互相关求时延；分别统计“人领先机器人”和“机器人领先人”的时间占比、平均领先量及角色切换次数 | 不只测响应有多慢，还能识别谁在发起动作、领导权是否随任务阶段自然切换 | [5], [8], [10] |
| 人类协作基准距离 | 当前人机实验的 wrench、速度、角速度、完成时间分布；人—人基准分布 | 对标准化特征计算 Wasserstein 距离或 Mahalanobis 距离，并逐项报告偏离来源 | 不预设“力越小越自然”；直接评价人机协作在动力学统计上与熟练人—人搬运相差多远 | [22] |
| 表观阻抗/透明度 | 人端小扰动力、木板速度与加速度 | 由 $F_h\rightarrow v$ 的频率响应或局部模型估计等效惯量、阻尼和刚度；与木板本体理论值比较 | 机器人虽承担重量，却不应让人感觉木板变得黏滞或难以启动；残余表观惯量/阻尼越小，透明度越好 | [1], [2] |
| 扰动恢复代价 | 施加到木板的标准化脉冲/阶跃扰动、位姿误差、两端 wrench | 报告峰值偏差、恢复至误差带所需时间、恢复期间新增人端做功和振荡衰减率 | 比无扰动轨迹更能评价闭环鲁棒性，并区分“平时误差小”与“受扰后仍稳定且不把负担甩给人” | [2], [26] |
| 累积人体工效暴露 | 人体关节角、关节力矩、持重时间；真人实验可加肌电 | 计算时间加权 RULA/REBA、高风险姿态持续占比，或关节力矩平方积分 | 平均人端力相同也可能对应完全不同的肩、腰和手腕风险；该指标能捕捉姿势代偿和累积暴露 | [24] |
| 交互流畅度 | 人与机器人开始/停止有效动作的时刻 | 报告功能性等待时间、双方同时有效运动占比、无效静止占比；在轨迹转段和启动/停止阶段单独统计 | 评价机器人是否能预判并并行配合，而不是每次等人纠正后才动作；适合分析轨迹拐点和姿态切换 | [25] |

### 2.6 建议优先加入当前仿真的进阶指标

现有日志已经包含木板状态和两端 wrench，因此无需增加人体传感器即可优先实现：

1. **等性能人力减负率**：作为最终对比结论的主指标；
2. **有效 wrench 比率与对抗功率占比**：解释人的力究竟用到哪里；
3. **力—运动同步性**：定位当前可能存在的周期振荡和方向不同步；
4. **协作响应方向性**：判断人机领导权和响应延迟；
5. **扰动恢复代价**：检验闭环控制是否真正鲁棒；
6. **协作增益**：证明加入机器人后的效果超过单人基线，而不是只展示某一条更好看的曲线。

## 3. 本项目建议的最小指标集

为了避免指标过多，当前仿真阶段建议至少固定以下 12 项，并对“纯人搬运”和“人机协同搬运”使用完全相同的轨迹、木板参数和统计时间窗：

1. 位置 RMSE；
2. 姿态 RMSE；
3. 线速度误差；
4. 角速度误差；
5. 两端高度差；
6. 完成时间；
7. jerk 或速度峰值数；
8. 人端 wrench RMS 与峰值；
9. 人端 wrench 变化率；
10. 人端正机械功/总绝对功；
11. 垂直负载分担比例；
12. 内力 RMS 与机器人控制饱和时间占比。

其中，位置/姿态/速度误差和两端高度差评价“搬得好不好”；人端 wrench、功率、做功和变化率评价“人是否更轻松”；内力、响应延迟与饱和率用于判断这种改善是否来自真正有效的协同，而不是人机对抗或过度控制。

## 4. 参考文献

[1] Y. Du et al., “Learning Human-Humanoid Coordination for Collaborative Object Carrying,” 2025. [PDF](https://yutang-lin.github.io/assets/pdf/cola.pdf)（预印本；其评价指标见 Sec. IV-C、Table III 和 Fig. 5。）

[2] E. Mielke, E. Townsend, D. Wingate, and M. D. Killpack, “Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads,” *Frontiers in Neurorobotics*, 2024. [DOI/全文](https://doi.org/10.3389/fnbot.2024.1291694)

[3] D. Feth, R. Groten, A. Peer, S. Hirche, and M. Buss, “Performance Related Energy Exchange in Haptic Human-Human Interaction in a Shared Virtual Object Manipulation Task,” *World Haptics*, 2009. [DOI](https://doi.org/10.1109/WHC.2009.4810854)

[4] Z. Bai et al., “Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction,” *Advanced Intelligent Systems*, 2025. [DOI/全文](https://doi.org/10.1002/aisy.202400616)

[5] Y. Liu et al., “The Role of Haptic Communication in Dyadic Collaborative Object Manipulation Tasks,” 2022. [arXiv](https://arxiv.org/abs/2203.01287)

[6] S. Bazzi and D. Sternad, “Human control of complex objects: Towards more dexterous robots,” *Advanced Robotics*, vol. 34, no. 17, pp. 1137–1155, 2020. [DOI/全文](https://doi.org/10.1080/01691864.2020.1777198)

[7] D. Sirintuna, T. Kastritsi, I. Ozdamar, J. M. Gandarias, and A. Ajoudani, “Enhancing human–robot collaborative transportation through obstacle-aware vibrotactile warning and virtual fixtures,” *Robotics and Autonomous Systems*, vol. 178, 104725, 2024. [DOI](https://doi.org/10.1016/j.robot.2024.104725)

[8] Z. Rysbek et al., “Robots Taking Initiative in Collaborative Object Manipulation: Lessons from Physical Human-Human Interaction,” 2023. [arXiv](https://arxiv.org/abs/2304.12288)

[9] M. Lawitzky, A. Mörtl, and S. Hirche, “Load Sharing in Human-Robot Cooperative Manipulation,” *RO-MAN*, 2010. [PDF](https://mediatum.ub.tum.de/doc/1082025/1082025.pdf)

[10] A. Mörtl et al., “The Role of Roles: Physical Cooperation between Humans and Robots,” *The International Journal of Robotics Research*, 2012. [DOI](https://doi.org/10.1177/0278364912455366)

[11] F. Gao, M. L. Latash, and V. M. Zatsiorsky, “Internal Forces during Object Manipulation,” *Experimental Brain Research*, 2005. [全文](https://pmc.ncbi.nlm.nih.gov/articles/PMC2847586/)

[12] S. Erhart and S. Hirche, “Internal Force Analysis and Load Distribution for Cooperative Multi-Robot Manipulation,” *IEEE Transactions on Robotics*, 2015. [DOI](https://doi.org/10.1109/TRO.2015.2459412)

[13] T. Stouraitis, I. Chatzinikolaidis, M. Gienger, and S. Vijayakumar, “Dyadic Collaborative Manipulation through Hybrid Trajectory Optimization,” *CoRL/PMLR*, 2018. [全文](https://proceedings.mlr.press/v87/stouraitis18a.html)

[14] L. F. C. Figueredo et al., “Planning to Minimize the Human Muscular Effort during Forceful Human-Robot Collaboration,” *ACM Transactions on Human-Robot Interaction*, 2022. [DOI](https://doi.org/10.1145/3481587)

[15] J. DelPreto and D. Rus, “Sharing the Load: Human-Robot Team Lifting Using Muscle Activity,” *ICRA*, 2019. [DOI](https://doi.org/10.1109/ICRA.2019.8794414)

[16] F. Goell et al., “Integrative biomechanics of a human–robot carrying task: implications for future collaborative work,” *Autonomous Robots*, 2024. [DOI/全文](https://doi.org/10.1007/s10514-024-10184-2)

[17] R. G. Freedman, S. J. Levine, B. C. Williams, and S. Zilberstein, “Helpfulness as a Key Metric of Human-Robot Collaboration,” 2020. [arXiv](https://arxiv.org/abs/2010.04914)

[18] G. Ganesh et al., “Two is better than one: Physical interactions improve motor performance in humans,” *Scientific Reports*, 2014. [DOI/全文](https://doi.org/10.1038/srep03824)

[19] A. Sawers et al., “On the Role of Physical Interaction on Performance of Object Manipulation by Dyads,” *Frontiers in Human Neuroscience*, 2017. [DOI/全文](https://doi.org/10.3389/fnhum.2017.00533)

[20] N. Beckers, E. H. F. van Asseldonk, and H. van der Kooij, “Haptic human–human interaction does not improve individual visuomotor adaptation,” *Scientific Reports*, 2020. [DOI/全文](https://doi.org/10.1038/s41598-020-76706-x)

[21] K. B. Shaw, D. L. Cordon, M. D. Killpack, and J. L. Salmon, “A Decomposition of Interaction Force for Multi-Agent Co-Manipulation,” 2024. [arXiv](https://arxiv.org/abs/2408.01543)

[22] S. W. Jensen, J. L. Salmon, and M. D. Killpack, “Trends in Haptic Communication of Human-Human Dyads: Toward Natural Human-Robot Co-manipulation,” *Frontiers in Neurorobotics*, 2021. [DOI/全文](https://doi.org/10.3389/fnbot.2021.626074)

[23] N. Masumoto and N. Inui, “Motor control hierarchy in joint action that involves bimanual force production,” *Journal of Neurophysiology*, 2015. [全文](https://pmc.ncbi.nlm.nih.gov/articles/PMC4468970/)

[24] M. Lorenzini, M. Lagomarsino, L. Fortini, S. Gholami, and A. Ajoudani, “Ergonomic human-robot collaboration in industry: A review,” *Frontiers in Robotics and AI*, 2022. [DOI/全文](https://doi.org/10.3389/frobt.2022.813907)

[25] G. Hoffman, “Evaluating Fluency in Human–Robot Collaboration,” *IEEE Transactions on Human-Machine Systems*, 2019. [DOI](https://doi.org/10.1109/THMS.2019.2904558)

[26] S. Regmi, D. Burns, and Y. S. Song, “A robot for overground physical human-robot interaction experiments,” *PLOS ONE*, 2022. [DOI/全文](https://doi.org/10.1371/journal.pone.0276980)
