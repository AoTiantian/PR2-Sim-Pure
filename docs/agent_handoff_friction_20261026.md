# 用户要求改为接触摩擦抓取（2026-10-26）

> 后续状态更新：S12已通过将初始抓入深度15mm改为60mm消除本轮复现的末段接触丢失。控制器、摩擦系数、夹紧命令和步长均未改变。最新证据见 `docs/agent_handoff_friction_fix_20261026.md` 和 `results/friction_diagnosis_20261026/REPORT.md`。下文为转换阶段的历史状态。

用户明确要求夹爪—木板接触摩擦，指出先前模型的焊接不符合目标。当前已删除板子weld、恢复对称双接触垫并启用碰撞，保留freejoint，靠有限夹紧执行器抓持。不得用weld/connect、硬置板位姿或隐藏支撑来通过转圈测试。此前焊接模型结果只作历史记录，不可当摩擦模型已通过。

主配置修改初始臂姿态以对准木板、手指初始0.45rad、夹紧目标0.30rad；摩擦系数2.5、木板质量/尺寸、夹爪kp/kv及限幅保持原值。场景原2ms物理步长，使用implicitfast、椭圆摩擦锥、impratio10、容差1e-10。地面新增板子专用碰撞位，板不与默认手指网格重复碰撞。

桥接器新增require_friction_grasp，正式launch及contract guard要求为真；拒绝板子weld/connect、固定板或失效pad碰撞。board_grasped要求两侧正法向力，debug有H_BoardFriction。旧weld_sensor兼容路径在本模型eq_id=-1时不生效。

53项相关测试通过。记录器现在读取实际模型路径和步长，替代旧硬编码0.002；部分早期1ms探针的旧manifest仍错误写0.002，报告已披露，原始文件没有被篡改。

模型转换完成，摩擦S12重复稳定性未解决。当前源码最新viewer：偏航179.52°、最终差0.481°、滚转峰值2.729°、无任务力矩饱和，末端仍夹持；但同设置headless在末段翻滚，峰值163.56°并失去接触。不能挑成功一次宣布修好。viewer保存数据后退出阶段有既存图形清理-11，单独记录。

1ms步长及位姿差分时间戳候选均没有稳定实际收益，已回退。人端human_robot_controller、human_impedance和QP与本轮起点逐字一致。原有人端body-frame及时序修复、用户底盘改动保留。候选与失败分支均保存，后续不要误用friction_final_viewer等试验标签判断保留版本：真正最终是friction_current_viewer及retained_source快照。

固定掌部台架已验证有限摩擦夹持、摩擦置零失去抓持、强制张开几何后失去接触；强制张开是测试夹具操作，生产执行器自主完整释放未验收。不要把台架已知半重力外载荷加入生产控制器。

完整证据与所有分支：results/friction_grasp_20261026/REPORT.md。S12复现配置为该目录S12.yaml；使用正常transport_comparison.launch.py、condition:=human_robot、robot_mode:=admittance、use_viewer:=true。主配置默认仍sinusoid。未提交代码。
