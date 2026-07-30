#!/usr/bin/env python3
"""
Analyse and compare admittance validation logs.

Supports two modes:

  admittance  — constant-force step response: measures displacement vs force
                 to verify admittance closed-loop accuracy.
  virtual_human — PID trajectory tracking: measures position tracking error
                 and force saturation.

Usage:
  python3 analyse_logs.py admittance <log.csv>
  python3 analyse_logs.py virtual_human <log.csv>
  python3 analyse_logs.py compare <admittance_log.csv> <virtual_human_log.csv>
"""

from __future__ import annotations

import csv
import sys
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

CSV_COLS = {
    "wall_time": 0, "ros_time": 1, "t_rel": 2,
    "ee_x": 3, "ee_y": 4, "ee_z": 5,
    "ee_qw": 6, "ee_qx": 7, "ee_qy": 8, "ee_qz": 9,
    "tgt_x": 10, "tgt_y": 11, "tgt_z": 12,
    "fx": 17, "fy": 18, "fz": 19,
    "cmd_vx": 26, "cmd_vy": 27,
}

def load_log(path: str) -> Optional[np.ndarray]:
    """Load motion logger CSV. Returns (n, 10) array:
       [t, ee_x, ee_y, ee_z, fx, fy, fz, fmag, cmd_vx, cmd_vy]
    """
    rows = []
    with open(path) as fh:
        reader = csv.reader(fh)
        _ = next(reader)  # header
        for row in reader:
            try:
                t = float(row[0])
                eex, eey, eez = float(row[3]), float(row[4]), float(row[5])
                fx, fy, fz = float(row[17]), float(row[18]), float(row[19])
                cmd_vx = float(row[26]) if len(row) > 26 and row[26] != "nan" else 0.0
                cmd_vy = float(row[27]) if len(row) > 27 and row[27] != "nan" else 0.0
                if not all(np.isfinite(v) for v in [eex, eey, eez, fx, fy, fz]):
                    continue
                fmag = np.sqrt(fx**2 + fy**2 + fz**2)
                rows.append([t, eex, eey, eez, fx, fy, fz, fmag, cmd_vx, cmd_vy])
            except (ValueError, IndexError):
                continue
    if not rows:
        return None
    d = np.array(rows)
    d[:, 0] -= d[0, 0]  # relative time
    return d


# ---------------------------------------------------------------------------
# admittance analysis
# ---------------------------------------------------------------------------

def analyse_admittance(log_path: str) -> dict:
    """Analyse a constant-force admittance step response."""
    d = load_log(log_path)
    if d is None:
        print("ERROR: no valid data in log")
        return {}

    t = d[:, 0]
    eex, eey, eez = d[:, 1], d[:, 2], d[:, 3]
    fx, fy, fz = d[:, 4], d[:, 5], d[:, 6]
    fmag = d[:, 7]
    cmd_vx, cmd_vy = d[:, 8], d[:, 9]

    # Find force-on segment
    active = fmag > 1.0  # 1 N threshold
    a_idx = np.where(active)[0]
    if len(a_idx) < 10:
        print("ERROR: no active force segment found")
        return {}

    i0, i1 = a_idx[0], a_idx[-1]
    t_active = t[i0:i1+1]
    duration = t_active[-1] - t_active[0]

    # Baseline EE (before force on)
    pre_slice = slice(max(0, i0 - 20), i0)
    base_x = np.mean(eex[pre_slice])
    base_y = np.mean(eey[pre_slice])
    base_z = np.mean(eez[pre_slice])

    # Displacement from baseline
    dx = eex[i0:i1+1] - base_x
    dy = eey[i0:i1+1] - base_y
    dz = eez[i0:i1+1] - base_z
    dxy = np.sqrt(dx**2 + dy**2)

    # Force stats
    fx_avg = np.mean(fx[i0:i1+1])
    fy_avg = np.mean(fy[i0:i1+1])
    fz_avg = np.mean(fz[i0:i1+1])

    # Effective admittance: displacement per Newton
    # In the direction of the dominant force
    if abs(fx_avg) > abs(fy_avg):
        admittance = np.max(np.abs(dx)) / max(abs(fx_avg), 0.1)
        dir_label = "x"
    else:
        admittance = np.max(np.abs(dy)) / max(abs(fy_avg), 0.1)
        dir_label = "y"

    # Steady-state: last 20% of active segment
    ss_start = int(len(dx) * 0.8)
    ss_dx = np.mean(np.abs(dx[ss_start:]))
    ss_dy = np.mean(np.abs(dy[ss_start:]))
    ss_dxy = np.mean(dxy[ss_start:])

    # Command velocity tracking
    cmd_vx_avg = np.mean(np.abs(cmd_vx[i0:i1+1]))
    cmd_vy_avg = np.mean(np.abs(cmd_vy[i0:i1+1]))

    result = {
        "type": "admittance",
        "duration": duration,
        "peak_dx": float(np.max(np.abs(dx))),
        "peak_dy": float(np.max(np.abs(dy))),
        "peak_dz": float(np.max(np.abs(dz))),
        "peak_dxy": float(np.max(dxy)),
        "ss_dxy": float(ss_dxy),
        "fx_avg": float(fx_avg),
        "fy_avg": float(fy_avg),
        "fz_avg": float(fz_avg),
        "admittance": float(admittance),  # m/N
        "dir_label": dir_label,
        "cmd_vx_avg": float(cmd_vx_avg),
        "cmd_vy_avg": float(cmd_vy_avg),
    }

    print("=" * 60)
    print("PURE ADMITTANCE STEP RESPONSE")
    print("=" * 60)
    print(f"  Active duration:        {duration:.2f} s")
    print(f"  Peak displacement XY:   {result['peak_dxy']:.4f} m")
    print(f"  Peak dx: {result['peak_dx']:.4f} m  dy: {result['peak_dy']:.4f} m  dz: {result['peak_dz']:.4f} m")
    print(f"  Steady-state XY disp:   {result['ss_dxy']:.4f} m")
    print(f"  Avg force: fx={result['fx_avg']:.1f}N  fy={result['fy_avg']:.1f}N  fz={result['fz_avg']:.1f}N")
    print(f"  Effective admittance:   {result['admittance']*1000:.2f} mm/N ({dir_label})")
    print(f"  Avg cmd_vel: vx={result['cmd_vx_avg']:.4f} m/s  vy={result['cmd_vy_avg']:.4f} m/s")

    return result


# ---------------------------------------------------------------------------
# virtual human analysis
# ---------------------------------------------------------------------------

def analyse_virtual_human(log_path: str) -> dict:
    """Analyse a PID virtual human tracking log."""
    d = load_log(log_path)
    if d is None:
        print("ERROR: no valid data in log")
        return {}

    t = d[:, 0]
    eex, eey, eez = d[:, 1], d[:, 2], d[:, 3]
    fx, fy, fz = d[:, 4], d[:, 5], d[:, 6]
    fmag = d[:, 7]

    # Find active segment
    active = fmag > 0.5
    a_idx = np.where(active)[0]
    if len(a_idx) < 10:
        print("ERROR: no active force segment found")
        return {}

    i0, i1 = a_idx[0], a_idx[-1]
    t_active = t[i0:i1+1]
    duration = t_active[-1] - t_active[0]

    afx, afy, afz = fx[i0:i1+1], fy[i0:i1+1], fz[i0:i1+1]
    afmag = fmag[i0:i1+1]
    aeex, aeey, aeez = eex[i0:i1+1], eey[i0:i1+1], eez[i0:i1+1]

    # EE total movement
    dx_total = np.max(aeex) - np.min(aeex)
    dy_total = np.max(aeey) - np.min(aeey)
    dz_total = np.max(aeez) - np.min(aeez)

    # Force stats
    fx_sat_pct = np.sum(np.abs(afx) >= 99.9) / len(afx) * 100
    fy_sat_pct = np.sum(np.abs(afy) >= 99.9) / len(afy) * 100
    fz_sat_pct = np.sum(np.abs(afz) >= 149.9) / len(afz) * 100

    # Oscillation (FFT on fx)
    from numpy.fft import rfft, rfftfreq
    dt = np.mean(np.diff(t_active))
    osc_freq = None
    if dt > 0 and len(afx) > 30:
        fx_d = afx - np.mean(afx)
        spec = np.abs(rfft(fx_d))
        freqs = rfftfreq(len(afx), dt)
        mask = (freqs >= 0.3) & (freqs <= 15)
        if np.any(mask):
            top_idx = np.argmax(spec[mask])
            osc_freq = float(freqs[mask][top_idx])

    # Check if EE movement is smooth (no jerky reversals)
    if len(aeex) > 3:
        dx_dt = np.diff(aeex)
        dy_dt = np.diff(aeey)
        # Count direction reversals
        rev_x = np.sum(np.abs(np.diff(np.signbit(dx_dt))))
        rev_y = np.sum(np.abs(np.diff(np.signbit(dy_dt))))
        rev_rate = (rev_x + rev_y) / duration if duration > 0 else 0
    else:
        rev_rate = 0

    result = {
        "type": "virtual_human",
        "duration": duration,
        "dx_total": float(dx_total),
        "dy_total": float(dy_total),
        "dz_total": float(dz_total),
        "fx_mean": float(np.mean(afx)),
        "fy_mean": float(np.mean(afy)),
        "fz_mean": float(np.mean(afz)),
        "fx_max": float(np.max(np.abs(afx))),
        "fy_max": float(np.max(np.abs(afy))),
        "fz_max": float(np.max(np.abs(afz))),
        "fmag_mean": float(np.mean(afmag)),
        "fmag_max": float(np.max(afmag)),
        "fmag_std": float(np.std(afmag)),
        "fx_sat_pct": float(fx_sat_pct),
        "fy_sat_pct": float(fy_sat_pct),
        "fz_sat_pct": float(fz_sat_pct),
        "osc_freq": osc_freq,
        "rev_rate": float(rev_rate),
    }

    print("=" * 60)
    print("PID VIRTUAL HUMAN TRACKING")
    print("=" * 60)
    print(f"  Active duration:        {duration:.2f} s")
    print(f"  EE total move:  dx={result['dx_total']:.3f}m  dy={result['dy_total']:.3f}m  dz={result['dz_total']:.3f}m")
    print()
    print(f"  Force |F|:  mean={result['fmag_mean']:.1f}N  max={result['fmag_max']:.1f}N  std={result['fmag_std']:.1f}N")
    print(f"  fx: mean={result['fx_mean']:.1f}N  max={result['fx_max']:.1f}N  sat={result['fx_sat_pct']:.1f}%")
    print(f"  fy: mean={result['fy_mean']:.1f}N  max={result['fy_max']:.1f}N  sat={result['fy_sat_pct']:.1f}%")
    print(f"  fz: mean={result['fz_mean']:.1f}N  max={result['fz_max']:.1f}N  sat={result['fz_sat_pct']:.1f}%")
    if osc_freq is not None:
        print(f"\n  Dominant oscillation:   {osc_freq:.2f} Hz")
    print(f"  Direction reversals:    {rev_rate:.1f}/s (lower is smoother)")

    return result


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage:")
        print("  analyse_logs.py admittance <log.csv>")
        print("  analyse_logs.py virtual_human <log.csv>")
        print("  analyse_logs.py compare <adm_log.csv> <vh_log.csv>")
        sys.exit(1)

    mode = sys.argv[1]

    if mode == "admittance":
        analyse_admittance(sys.argv[2])
    elif mode == "virtual_human":
        analyse_virtual_human(sys.argv[2])
    elif mode == "compare":
        if len(sys.argv) < 4:
            print("compare requires two CSV paths")
            sys.exit(1)
        print("=" * 60)
        print("COMPARISON: PURE ADMITTANCE vs PID VIRTUAL HUMAN")
        print("=" * 60)
        r_adm = analyse_admittance(sys.argv[2])
        print()
        r_vh = analyse_virtual_human(sys.argv[3])
        if r_adm and r_vh:
            print()
            print("--- Key comparison ---")
            if r_adm["admittance"] > 0:
                # The virtual human force creates displacement. Compare expected vs actual.
                # For virtual human, the "effective admittance" = total_move / mean_force
                vh_adm = r_vh["dx_total"] / max(r_vh["fx_mean"], 0.1)
                print(f"  Admittance (const force):  {r_adm['admittance']*1000:.2f} mm/N")
                print(f"  Admittance (PID virtual):  {vh_adm*1000:.2f} mm/N")
                # If PID admittance < const admittance, PID is fighting the system
                ratio = vh_adm / r_adm["admittance"] if r_adm["admittance"] > 1e-6 else 1.0
                print(f"  Ratio (PID/const):         {ratio:.2f}")
                if ratio < 0.7:
                    print("  → PID is using extra force to overcome tracking error")
                elif ratio > 1.3:
                    print("  → PID may be over-driving the system")
                else:
                    print("  → PID efficiency is comparable to pure admittance")
    else:
        print(f"Unknown mode: {mode}")
        sys.exit(1)
