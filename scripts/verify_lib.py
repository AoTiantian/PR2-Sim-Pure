import csv, glob, json, os
import numpy as np

ROOT = "/workspace/results/verify_zsupport"

def load_hist(d):
    with open(d + "/history.csv") as f:
        r = list(csv.DictReader(f))
    cols = {k: np.array([x[k] for x in r]) for k in r[0]}
    out = {}
    for k, v in cols.items():
        try:
            out[k] = v.astype(float)
        except ValueError:
            out[k] = v
    return out

def runs_by_id():
    m = {}
    for d in sorted(glob.glob(ROOT + "/run_*")):
        try:
            man = json.load(open(d + "/run_manifest.json"))
        except Exception:
            continue
        m[man["experiment_id"]] = d
    return m

def track_stats(d):
    h = load_hist(d)
    k = h["state"] == "track"
    g = lambda n: h[n][k]
    return dict(fz_robot=g("robot_force_z").mean(), fz_robot_std=g("robot_force_z").std(),
                fz_human=g("human_force_z").mean(), z_err=g("position_error_z").mean(),
                x_rmse=np.sqrt((g("position_error_x")**2).mean()),
                z_rmse=np.sqrt((g("position_error_z")**2).mean()), t0=h["time"][k][0], t1=h["time"][k][-1])
