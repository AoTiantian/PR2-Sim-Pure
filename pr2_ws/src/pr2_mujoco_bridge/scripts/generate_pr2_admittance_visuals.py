#!/usr/bin/env python3
"""Generate polished PR2 admittance-control visual artifacts from a response CSV."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AXES = ("x", "y", "z")
COLORS = {"x": "#ff4d6d", "y": "#2bd576", "z": "#4dabf7"}
DARK_COLORS = {"x": "#ff6b8a", "y": "#63e6be", "z": "#74c0fc"}


def load_csv(path: str) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    if data.shape == ():
        data = data.reshape(1)
    if not data.dtype.names:
        raise SystemExit(f"CSV has no header: {path}")
    required = {"timestamp", "pos_x", "pos_y", "pos_z", "force_x", "force_y", "force_z"}
    missing = required - set(data.dtype.names)
    if missing:
        raise SystemExit(f"CSV missing required columns: {sorted(missing)}")
    if len(data) < 20:
        raise SystemExit(f"CSV too short for plotting: {path} has {len(data)} rows")
    return data


def col(data: np.ndarray, name: str, default: float | None = None) -> np.ndarray | None:
    if name in (data.dtype.names or ()): return np.asarray(data[name], dtype=float)
    if default is None: return None
    return np.full(len(data), default, dtype=float)


def time_axis(data: np.ndarray) -> np.ndarray:
    t = np.asarray(data["timestamp"], dtype=float)
    return t - float(t[0])


def force_window(data: np.ndarray, t: np.ndarray, threshold=0.5):
    mag = np.maximum.reduce([np.abs(data[f"force_{a}"]) for a in AXES])
    active = mag > threshold
    if not np.any(active): return None, None
    idx = np.flatnonzero(active)
    release_idx = min(idx[-1] + 1, len(t) - 1)
    return float(t[idx[0]]), float(t[release_idx])


def ee_rel_mm(data: np.ndarray, baseline_skip_samples: int):
    idx = min(max(baseline_skip_samples, 0), len(data) - 1)
    return {a: (np.asarray(data[f"pos_{a}"], dtype=float) - float(data[f"pos_{a}"][idx])) * 1000.0 for a in AXES}


def adm_disp_mm(data: np.ndarray):
    if all(col(data, f"adm_disp_{a}") is not None for a in AXES):
        return {a: col(data, f"adm_disp_{a}") * 1000.0 for a in AXES}
    return None


def adm_vel_mm_s(data: np.ndarray):
    if all(col(data, f"adm_vel_{a}") is not None for a in AXES):
        return {a: col(data, f"adm_vel_{a}") * 1000.0 for a in AXES}
    return None


def apply_grid(ax, dark=False):
    ax.grid(True, alpha=0.22 if dark else 0.28, linewidth=0.8)
    for spine in ax.spines.values():
        spine.set_alpha(0.35)


def shade(ax, onset, release, dark=False):
    if onset is not None:
        ax.axvline(onset, color="#e9ecef" if dark else "#495057", ls="--", lw=1, alpha=0.8)
    if release is not None:
        ax.axvline(release, color="#e9ecef" if dark else "#495057", ls=":", lw=1, alpha=0.8)
    if onset is not None and release is not None and release > onset:
        ax.axvspan(onset, release, color="#ffd43b" if dark else "#ffe066", alpha=0.12 if dark else 0.22, zorder=0)


def metrics(data, baseline_skip_samples=60, tail_window_s=3.0):
    t = time_axis(data); ee = ee_rel_mm(data, baseline_skip_samples); onset, release = force_window(data, t)
    tail_start = max(0, t[-1] - tail_window_s)
    if release is not None: tail_start = max(tail_start, release)
    m = {"rows": int(len(data)), "duration_s": float(t[-1]), "force_on_s": onset, "force_off_s": release, "tail_start_s": float(tail_start)}
    mask = t >= tail_start
    for a in AXES:
        vals = ee[a]
        tail = vals[mask]
        m[f"peak_abs_{a}_mm"] = float(np.max(np.abs(vals)))
        m[f"final_{a}_mm"] = float(tail[-1])
        m[f"tail_std_{a}_mm"] = float(np.std(tail))
        m[f"tail_range_{a}_mm"] = float(np.ptp(tail))
    for name in ("qdot_cmd_norm", "tau_norm", "tau_max_abs"):
        v = col(data, name)
        if v is not None:
            m[f"max_{name}"] = float(np.nanmax(v))
    return m


def savefig(fig, out):
    fig.savefig(out, dpi=210, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def dashboard(data, out, dark=False, baseline_skip_samples=60, tail_window_s=3.0):
    t = time_axis(data); onset, release = force_window(data, t); ee = ee_rel_mm(data, baseline_skip_samples)
    adm = adm_disp_mm(data); vel = adm_vel_mm_s(data)
    qdot, tau, taumax = col(data,"qdot_cmd_norm"), col(data,"tau_norm"), col(data,"tau_max_abs")
    if dark:
        plt.style.use("dark_background"); colors = DARK_COLORS; face="#10141f"; panel="#151b2d"
    else:
        plt.style.use("seaborn-v0_8-whitegrid"); colors = COLORS; face="white"; panel="white"
    fig, axs = plt.subplots(5, 1, figsize=(16, 13.5), sharex=True, facecolor=face)
    fig.suptitle(("Optimized PR2 Arm Admittance Response — Dark Telemetry" if dark else "Optimized PR2 Arm Admittance Response — Control Dashboard"), fontsize=18, fontweight="bold", y=0.995)
    for ax in axs:
        ax.set_facecolor(panel); apply_grid(ax, dark); shade(ax, onset, release, dark)
    for a in AXES: axs[0].plot(t, data[f"force_{a}"], color=colors[a], lw=1.8, label=f"F{a.upper()}")
    axs[0].set_ylabel("Force [N]"); axs[0].set_title("External wrench window"); axs[0].legend(ncol=3, loc="upper right")
    if adm:
        for a in AXES: axs[1].plot(t, adm[a], color=colors[a], lw=1.9, label=f"desired Δ{a.upper()}")
        if vel:
            axv = axs[1].twinx()
            for a in AXES: axv.plot(t, vel[a], color=colors[a], ls="--", alpha=0.38, lw=1.0)
            axv.set_ylabel("Desired velocity [mm/s]", alpha=.75)
        axs[1].legend(ncol=3, loc="upper right")
    else:
        axs[1].text(.5,.5,"admittance debug columns missing",transform=axs[1].transAxes,ha="center")
    axs[1].set_ylabel("Desired disp. [mm]"); axs[1].set_title("Admittance output: M-D-K filtered motion")
    for a in AXES: axs[2].plot(t, ee[a], color=colors[a], lw=1.9, label=f"actual Δ{a.upper()}")
    axs[2].set_ylabel("Actual EE [mm]"); axs[2].set_title("End-effector response and cross-axis coupling"); axs[2].legend(ncol=3, loc="upper right")
    if qdot is not None: axs[3].plot(t, qdot, color="#b197fc" if dark else "#7048e8", lw=1.8, label="||qdot_cmd||")
    if tau is not None:
        ax2=axs[3].twinx(); ax2.plot(t, tau, color="#ffa94d", lw=1.5, label="||tau||")
        if taumax is not None: ax2.plot(t, taumax, color="#ff8787", ls="--", lw=1.3, label="max |tau_i|")
        ax2.set_ylabel("Torque [Nm]"); ax2.legend(loc="upper right")
    axs[3].set_ylabel("Velocity norm"); axs[3].set_title("Execution-layer command effort"); axs[3].legend(loc="upper left")
    tail_start = max(0, t[-1]-tail_window_s)
    if release is not None: tail_start=max(tail_start, release)
    mask=t>=tail_start
    for a in AXES: axs[4].plot(t[mask], ee[a][mask], color=colors[a], lw=2.0, label=f"tail Δ{a.upper()}")
    axs[4].set_xlabel("Time [s]"); axs[4].set_ylabel("Tail [mm]"); axs[4].set_title("Force-off settling zoom"); axs[4].legend(ncol=3, loc="upper right")
    m=metrics(data, baseline_skip_samples, tail_window_s)
    txt="\n".join([f"{a.upper()}: final {m[f'final_{a}_mm']:+.2f} mm | σtail {m[f'tail_std_{a}_mm']:.2f} mm | range {m[f'tail_range_{a}_mm']:.2f} mm" for a in AXES])
    axs[4].text(0.012, 0.96, txt, transform=axs[4].transAxes, va="top", fontsize=9, bbox=dict(boxstyle="round,pad=.45", facecolor="#212529" if dark else "#ffffff", alpha=.78, edgecolor="#868e96"))
    savefig(fig, out)


def paper(data, out, baseline_skip_samples=60):
    plt.style.use("seaborn-v0_8-paper")
    t=time_axis(data); onset,release=force_window(data,t); ee=ee_rel_mm(data,baseline_skip_samples); adm=adm_disp_mm(data)
    fig,axs=plt.subplots(2,2,figsize=(15,8.5),facecolor="white")
    fig.suptitle("PR2 Admittance Control: Force → Desired Motion → Actual EE Response",fontsize=16,fontweight="bold")
    for ax in axs.ravel(): apply_grid(ax); shade(ax,onset,release)
    ax=axs[0,0]
    for a in AXES: ax.plot(t,data[f"force_{a}"],color=COLORS[a],lw=1.7,label=f"F{a.upper()}")
    ax.set_title("Input wrench"); ax.set_ylabel("N"); ax.legend(ncol=3)
    ax=axs[0,1]
    if adm:
        for a in AXES: ax.plot(t,adm[a],color=COLORS[a],lw=1.8,label=f"desired {a.upper()}")
    for a in AXES: ax.plot(t,ee[a],color=COLORS[a],lw=1.15,ls="--",alpha=.72,label=f"actual {a.upper()}")
    ax.set_title("Desired vs actual displacement"); ax.set_ylabel("mm"); ax.legend(ncol=3,fontsize=8)
    ax=axs[1,0]
    main='x'; cross=['y','z']
    ax.plot(t,ee[main],color=COLORS[main],lw=2,label="main-axis X")
    for a in cross: ax.plot(t,ee[a],color=COLORS[a],lw=1.6,label=f"cross-axis {a.upper()}")
    ax.set_title("Cross-axis coupling visibility"); ax.set_xlabel("s"); ax.set_ylabel("mm"); ax.legend()
    ax=axs[1,1]
    qdot,tau,taumax=col(data,"qdot_cmd_norm"),col(data,"tau_norm"),col(data,"tau_max_abs")
    if qdot is not None: ax.plot(t,qdot,color="#7048e8",lw=1.8,label="||qdot_cmd||")
    if tau is not None: ax.plot(t,tau/10.0,color="#f08c00",lw=1.5,label="||tau|| / 10")
    if taumax is not None: ax.plot(t,taumax/10.0,color="#e03131",lw=1.3,ls="--",label="max |tau_i| / 10")
    ax.set_title("Command smoothness indicators"); ax.set_xlabel("s"); ax.legend()
    savefig(fig,out)


def phase_and_metrics(data, out, baseline_skip_samples=60):
    plt.style.use("seaborn-v0_8-white")
    t=time_axis(data); ee=ee_rel_mm(data,baseline_skip_samples); adm=adm_disp_mm(data); vel=adm_vel_mm_s(data); m=metrics(data,baseline_skip_samples)
    fig=plt.figure(figsize=(15,9),facecolor="white")
    gs=fig.add_gridspec(2,3,height_ratios=[1,1.1],hspace=.35,wspace=.28)
    fig.suptitle("Admittance Response Diagnostics — Tracking, Phase, Settling",fontsize=16,fontweight="bold")
    ax=fig.add_subplot(gs[0,0])
    if adm:
        for a in AXES: ax.scatter(adm[a],ee[a],s=8,alpha=.45,color=COLORS[a],label=a.upper())
        lim=max(1, max(np.max(np.abs(adm[a])) for a in AXES), max(np.max(np.abs(ee[a])) for a in AXES))
        ax.plot([-lim,lim],[-lim,lim],color="#495057",lw=1,ls="--",label="ideal")
    ax.set_title("Desired-vs-actual tracking"); ax.set_xlabel("Desired disp. [mm]"); ax.set_ylabel("Actual EE [mm]"); ax.legend(); apply_grid(ax)
    ax=fig.add_subplot(gs[0,1])
    if vel:
        for a in AXES: ax.plot(ee[a], vel[a], color=COLORS[a], lw=1.2, label=a.upper())
    ax.set_title("Admittance phase portrait"); ax.set_xlabel("Disp. [mm]"); ax.set_ylabel("Desired vel. [mm/s]"); ax.legend(); apply_grid(ax)
    ax=fig.add_subplot(gs[0,2])
    labels=[a.upper() for a in AXES]
    finals=[m[f"final_{a}_mm"] for a in AXES]; stds=[m[f"tail_std_{a}_mm"] for a in AXES]
    x=np.arange(3); ax.bar(x-.18,finals,width=.36,color=[COLORS[a] for a in AXES],label="final [mm]"); ax.bar(x+.18,stds,width=.36,color="#adb5bd",label="tail σ [mm]")
    ax.axhline(0,color="#343a40",lw=.8); ax.set_xticks(x,labels); ax.set_title("Settling residual summary"); ax.legend(); apply_grid(ax)
    ax=fig.add_subplot(gs[1,:])
    rows=[]; names=[]
    for key in ["peak_abs_x_mm","peak_abs_y_mm","peak_abs_z_mm","tail_std_x_mm","tail_std_y_mm","tail_std_z_mm","tail_range_x_mm","tail_range_y_mm","tail_range_z_mm"]:
        rows.append(m[key]); names.append(key.replace("_mm","").replace("peak_abs_","peak ").replace("tail_", "tail ").replace("_", " "))
    vals=np.array(rows,dtype=float); ypos=np.arange(len(vals))
    ax.barh(ypos,vals,color=[COLORS.get(n.split()[1],"#868e96") if len(n.split())>1 else "#868e96" for n in names])
    ax.set_yticks(ypos,names); ax.invert_yaxis(); ax.set_xlabel("mm"); ax.set_title("Quantitative visual scorecard"); apply_grid(ax)
    for y,v in zip(ypos,vals): ax.text(v + max(vals)*0.01, y, f"{v:.2f}", va="center", fontsize=9)
    savefig(fig,out)


def axis_focus(data, outdir, baseline_skip_samples=60):
    t=time_axis(data); onset,release=force_window(data,t); ee=ee_rel_mm(data,baseline_skip_samples); adm=adm_disp_mm(data)
    outs=[]
    for a in AXES:
        plt.style.use("seaborn-v0_8-whitegrid")
        fig,ax=plt.subplots(figsize=(13,5.5),facecolor="white")
        shade(ax,onset,release); apply_grid(ax)
        ax.plot(t,data[f"force_{a}"],color="#495057",lw=1.4,label=f"F{a.upper()} [N]")
        ax2=ax.twinx()
        if adm: ax2.plot(t,adm[a],color=COLORS[a],lw=2.4,label=f"desired Δ{a.upper()} [mm]")
        ax2.plot(t,ee[a],color=COLORS[a],lw=1.6,ls="--",label=f"actual Δ{a.upper()} [mm]")
        for b in AXES:
            if b != a: ax2.plot(t,ee[b],color=COLORS[b],lw=1.0,alpha=.5,label=f"coupled Δ{b.upper()}")
        ax.set_title(f"Axis-focused admittance response: {a.upper()} channel",fontsize=15,fontweight="bold")
        ax.set_xlabel("Time [s]"); ax.set_ylabel("Force [N]"); ax2.set_ylabel("Displacement [mm]")
        lines,labels=ax.get_legend_handles_labels(); lines2,labels2=ax2.get_legend_handles_labels(); ax2.legend(lines+lines2,labels+labels2,ncol=3,loc="upper right")
        out=Path(outdir)/f"pr2_admittance_axis_{a}_focus.png"; savefig(fig,out); outs.append(str(out))
    return outs


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--csv",default="/tmp/arm_1d_response_plot.csv")
    ap.add_argument("--outdir",default="/tmp/pr2_admittance_visuals")
    ap.add_argument("--baseline-skip-samples",type=int,default=60)
    args=ap.parse_args()
    outdir=Path(args.outdir); outdir.mkdir(parents=True,exist_ok=True)
    data=load_csv(args.csv)
    outputs=[]
    outputs.append(str(outdir/"pr2_admittance_dashboard_white.png")); dashboard(data,outputs[-1],False,args.baseline_skip_samples)
    outputs.append(str(outdir/"pr2_admittance_dashboard_dark.png")); dashboard(data,outputs[-1],True,args.baseline_skip_samples)
    outputs.append(str(outdir/"pr2_admittance_paper_summary.png")); paper(data,outputs[-1],args.baseline_skip_samples)
    outputs.append(str(outdir/"pr2_admittance_diagnostics_scorecard.png")); phase_and_metrics(data,outputs[-1],args.baseline_skip_samples)
    outputs.extend(axis_focus(data,outdir,args.baseline_skip_samples))
    m=metrics(data,args.baseline_skip_samples)
    with open(outdir/"pr2_admittance_metrics.json","w") as f: json.dump(m,f,indent=2)
    print(json.dumps({"outputs": outputs, "metrics": m}, indent=2))

if __name__ == "__main__":
    main()
