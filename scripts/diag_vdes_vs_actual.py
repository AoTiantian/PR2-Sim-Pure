"""One-off: compare QP v_des commands vs actual board velocity from a diag run."""
import csv
import json

import numpy as np

vdes_t, vdes_x, wf_t, wf_x = [], [], [], []
vach_x = []
t0 = None
with open("/tmp/pr2_wbc_debug_trace.log") as f:
    for line in f:
        try:
            e = json.loads(line)
        except Exception:
            continue
        if e.get("hypothesisId") not in ("H_TorqueOnlyDrift", "H12_BaseStopWhileArmMoves"):
            continue
        d = e.get("data", {})
        ts = e.get("timestamp", 0) / 1000.0
        if t0 is None:
            t0 = ts
        vdes_t.append(ts - t0)
        vdes_x.append(d["v_des"][0])
        wf = d.get("w_filt", d.get("w_raw_msg"))
        wf_t.append(ts - t0)
        wf_x.append(wf[0] if wf else np.nan)
        vach = d.get("v_ach")
        vach_x.append(vach[0] if vach else np.nan)

vdes_t = np.array(vdes_t)
vdes_x = np.array(vdes_x)
wf_t = np.array(wf_t)
wf_x = np.array(wf_x)
vach_x = np.array(vach_x, dtype=np.float64)
m = (vdes_t >= 2) & (vdes_t <= 13)
mv = m & ~np.isnan(vach_x)
print(f"QP events: {len(vdes_t)}, track samples: {int(m.sum())}, with v_ach: {int(mv.sum())}")
print(f"v_des_x  std = {vdes_x[m].std():.4f} m/s, peak |v| = {abs(vdes_x[m]).max():.4f}")
if mv.any():
    print(f"v_ach_x  std = {vach_x[mv].std():.4f} m/s  (QP 求解可达)")
    print(f"QP 内部执行率 v_ach/v_des = {vach_x[mv].std() / vdes_x[mv].std():.2f}")
mw = (wf_t >= 2) & (wf_t <= 13)
print(f"w_filt_x std = {wf_x[mw].std():.2f} N")

# 底盘 vs 机械臂分解 (H12 事件)
ub_t, ub_x, ub_y, va_x2, arm_pk = [], [], [], [], []
with open("/tmp/pr2_wbc_debug_trace.log") as f:
    for line in f:
        try:
            e = json.loads(line)
        except Exception:
            continue
        if e.get("hypothesisId") != "H12_BaseStopWhileArmMoves":
            continue
        d = e.get("data", {})
        ts = e.get("timestamp", 0) / 1000.0
        if t0 is None:
            t0 = ts
        tt = ts - t0
        if not (2 <= tt <= 13):
            continue
        ub = d.get("u_base_world")
        va = d.get("v_ach")
        if ub is None or va is None:
            continue
        ub_t.append(tt)
        ub_x.append(ub[0])
        ub_y.append(ub[1])
        va_x2.append(va[0])
        arm_pk.append(d.get("arm_peak_abs_qdot", float("nan")))
ub_x = np.array(ub_x)
va_x2 = np.array(va_x2)
print(f"\nH12 样本: {len(ub_x)}")
print(f"QP分配 底盘vx std = {ub_x.std():.4f}, v_ach_x std = {va_x2.std():.4f}")
print(f"底盘占末端X速度比例 std(u_base_x)/std(v_ach_x) = {ub_x.std()/max(va_x2.std(),1e-9):.2f}")
print(f"臂关节 |qdot| 峰值中位数 = {np.nanmedian(np.array(arm_pk)):.3f} rad/s (限幅10)")

rows = list(csv.DictReader(open(
    "/workspace/results/transport_comparison/diag_trace/human_robot/run_20260910_111854/history.csv")))
t = np.array([float(r["time"]) for r in rows])
t -= t[0]
px = np.array([float(r["board_position_x"]) for r in rows])
mm = (t >= 2) & (t <= 13)
vx = np.gradient(px[mm], t[mm])
print(f"board X actual velocity std = {vx.std():.4f} m/s")
print(f"execution ratio actual/commanded = {vx.std() / vdes_x[m].std():.2f}")
