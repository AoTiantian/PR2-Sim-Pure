"""Compare every completed trial; no automatic exclusion of failed trajectories."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
summary = {}
for case in sorted(HERE.iterdir()):
    if not case.is_dir() or not (case / "control_condition.json").exists():
        continue
    meta = json.loads((case / "control_condition.json").read_text())
    m = {"mode": meta["mode"]}
    if (case / "exit.json").exists():
        m.update(json.loads((case / "exit.json").read_text()))
    csvs = list(case.glob("run_*/history.csv"))
    if not csvs:
        m["history_missing"] = True
        summary[case.name] = m
        continue
    a = np.genfromtxt(csvs[0], delimiter=",", names=True, dtype=None, encoding="utf8")
    lo, hi = a["time"][[0,-1]]
    m.update(samples=len(a), duration_s=float(hi-lo), history=str(csvs[0].relative_to(HERE)))
    force = np.column_stack([a["human_force_"+c] for c in "xyz"])
    error = np.column_stack([a["position_error_"+c] for c in "xyz"])
    actual = Rotation.from_rotvec(np.column_stack([a["actual_rotvec_"+c] for c in "xyz"]))
    target = Rotation.from_rotvec(np.column_stack([a["desired_rotvec_"+c] for c in "xyz"]))
    angle = (target * actual.inv()).magnitude()
    euler = actual.as_euler("xyz")
    m.update(hand_xy_rmse_cm=float(100*np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
             human_xy_force_rms_N=float(np.sqrt(np.mean(np.sum(force[:,:2]**2,axis=1)))),
             human_mean_vertical_force_N=float(np.mean(force[:,2])),
             attitude_peak_deg=float(np.rad2deg(angle.max())),
             attitude_rms_deg=float(np.rad2deg(np.sqrt(np.mean(angle**2)))),
             peak_roll_deg=float(np.rad2deg(np.max(abs(euler[:,0])))),
             peak_pitch_deg=float(np.rad2deg(np.max(abs(euler[:,1])))),
             human_force_saturation_fraction=float(np.mean(np.any(abs(force)>=60-1e-8,axis=1))))
    records = []
    with (case / "bridge_debug.jsonl").open() as f:
        for line in f:
            d = json.loads(line)
            if d["hypothesisId"] == "H_BoardFriction" and lo<=d["data"]["sim_time"]<=hi:
                records.append(d["data"])
    if records:
        m.update(both_contact_fraction=float(np.mean([x["grasped"] for x in records])),
                 final_grasped=bool(records[-1]["grasped"]))
    if (case / "allocation.jsonl").exists():
        rows = [json.loads(x) for x in (case / "allocation.jsonl").read_text().splitlines()]
        rows = [r for r in rows if lo<=r["sim_time"]<=hi]
        if rows:
            cmd = np.array([r["original_admittance_twist"][3:] for r in rows])
            achieved = np.array([r["before_filter_twist"][3:] for r in rows])
            m.update(angular_allocation_error_max_rad_s=float(np.max(abs(cmd-achieved))),
                     angular_task_infeasible_fraction=float(np.mean([r["primary_infeasible"] for r in rows])),
                     base_path_xy_error_rms_cm=float(100*np.sqrt(np.mean([np.sum(np.array(r["path_error"])[:2]**2) for r in rows]))),
                     base_path_xy_final_error_cm=float(100*np.linalg.norm(np.array(rows[-1]["path_error"])[:2])))
    if (case / "raw.npz").exists():
        raw = np.load(case / "raw.npz")
        for key in ["cmd","reference"]:
            b=raw[key]; m[key+"_wall_rate_hz"]=float((len(b)-1)/(b[-1,0]-b[0,0]))
    summary[case.name] = m
(HERE / "runtime_summary.json").write_text(json.dumps(summary,indent=2))
for k,v in summary.items():
    print(k,json.dumps(v))
