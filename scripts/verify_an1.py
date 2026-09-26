"""E1 analysis: robot wrist Fz (=history.csv robot_force_z) vs robot_cmd_rate_hz, from raw npz logs."""
import glob
import re

import numpy as np

SETTLE, TRACK = 2.0, 12.0  # from transport_comparison.yaml experiment block


def window(arr_t_sim, t0, t1):
    return (arr_t_sim >= t0) & (arr_t_sim <= t1)


def stats_for(npz_path):
    d = np.load(npz_path)
    st = d["sim_time"]  # (wall, sim)
    t_sim = st[:, 1]
    rtf = (t_sim[-1] - t_sim[0]) / (st[-1, 0] - st[0, 0])
    wrist = d["wrist"]  # wall, fx,fy,fz,tx,ty,tz  (== history.csv robot_force_*)
    hf = d["hand_force"]  # wall, fx,fy,fz,tx,ty,tz (== history.csv human_force_*)
    cmd = d["cmd_out"]

    # map wall time -> sim time via sim_time stream (nearest)
    def to_sim(arr):
        idx = np.searchsorted(st[:, 0], arr[:, 0])
        idx = np.clip(idx, 0, len(st) - 1)
        return t_sim[idx]

    wrist_sim = to_sim(wrist)
    hf_sim = to_sim(hf)
    m_w = window(wrist_sim, SETTLE, SETTLE + TRACK)
    m_h = window(hf_sim, SETTLE, SETTLE + TRACK)
    cmd_sim = to_sim(cmd)
    m_c = window(cmd_sim, SETTLE, SETTLE + TRACK)
    cmd_hz = m_c.sum() / TRACK if m_c.sum() else float("nan")
    return dict(
        robot_fz_mean=wrist[m_w, 3].mean(), robot_fz_std=wrist[m_w, 3].std(),
        human_fz_mean=hf[m_h, 3].mean(), rtf=rtf, cmd_hz=cmd_hz,
        n_wrist=int(m_w.sum()),
    )


rows = {}
for f in sorted(glob.glob("/tmp/v/E1_r*_*.npz")):
    tag = f.split("/")[-1][:-4]
    m = re.match(r"E1_r(\d+)_(\d+)", tag)
    r, rep = int(m.group(1)), int(m.group(2))
    rows.setdefault(r, []).append((rep, stats_for(f)))

print(f"{'rate':>5} {'n':>2} | robot_wrist_Fz per run [N]                | mean±sd      | human_Fz mean | RTF  | cmd_out Hz(sim)")
for r in sorted(rows, reverse=True):
    ss = [s for _, s in sorted(rows[r], key=lambda x: x[0])]
    fz = np.array([s["robot_fz_mean"] for s in ss])
    print(f"{r:5d} {len(ss):2d} | {np.round(fz, 2)} | {fz.mean():6.2f}±{(fz.std(ddof=1) if len(fz) > 1 else 0):.2f} | "
          f"{np.mean([s['human_fz_mean'] for s in ss]):6.2f} | {np.mean([s['rtf'] for s in ss]):.2f} | "
          f"{np.mean([s['cmd_hz'] for s in ss]):.1f}")
