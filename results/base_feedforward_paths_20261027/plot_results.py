"""All three tasks, all repetitions, mean/min-max time envelopes."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
summary=json.loads((HERE/"runtime_summary.json").read_text())
tasks=["S11","sinusoid","S12"]
titles={"S11":"Straight S11","sinusoid":"Sinusoid","S12":"180-degree turn S12"}
colors={"passive":"#4c78a8","active":"#2a9d67"}
labels={"passive":"Original passive","active":"Known-path feedforward"}
fig,axs=plt.subplots(3,3,figsize=(14,12),layout="constrained")
for i,task in enumerate(tasks):
    groups={mode:[] for mode in colors}
    first=True
    for tag,m in summary.items():
        if m["task"]!=task or "history" not in m:
            continue
        a=np.genfromtxt(HERE/m["history"],delimiter=",",names=True,dtype=None,encoding="utf8")
        t=a["time"]-a["time"][0]
        target=np.column_stack([a["desired_position_"+k] for k in "xy"])
        actual=np.column_stack([a["actual_position_"+k] for k in "xy"])
        anchor=target[0].copy();target-=anchor;actual-=anchor
        mode=m["mode"]
        axs[i,0].plot(actual[:,0],actual[:,1],color=colors[mode],lw=1,alpha=.65)
        if first:
            axs[i,0].plot(target[:,0],target[:,1],"k--",lw=1.6,label="Desired hand path")
            first=False
        r=Rotation.from_rotvec(np.column_stack([a["actual_rotvec_"+k] for k in "xyz"]))
        d=Rotation.from_rotvec(np.column_stack([a["desired_rotvec_"+k] for k in "xyz"]))
        angle=np.rad2deg((d*r.inv()).magnitude())
        roll=np.rad2deg(abs(r.as_euler("xyz")[:,0]))
        f=np.hypot(a["human_force_x"],a["human_force_y"])
        groups[mode].append((t,angle,f,roll))
    end=23 if task=="S12" else 13
    for mode,rows in groups.items():
        if not rows:
            continue
        grid=np.linspace(0,end,650)
        for j,col in [(1,1),(2,2)]:
            values=np.array([np.interp(grid,r[0],r[col]) for r in rows])
            axs[i,j].plot(grid,values.mean(axis=0),color=colors[mode],label=labels[mode])
            axs[i,j].fill_between(grid,values.min(axis=0),values.max(axis=0),color=colors[mode],alpha=.16)
        axs[i,0].plot([],[],color=colors[mode],label=labels[mode])
    axs[i,0].set(title=titles[task]+": hand XY",xlabel="x (m)",ylabel="y (m)")
    axs[i,0].axis("equal")
    axs[i,1].set(title="SO(3) attitude error",xlabel="Task time (s)",ylabel="Error (deg)")
    axs[i,2].set(title="Human planar force",xlabel="Task time (s)",ylabel="Force magnitude (N)")
    for a in axs[i]:
        a.grid(alpha=.2)
        a.legend(fontsize=7)
fig.savefig(HERE/"task_comparison.png",dpi=170)
fig.savefig(HERE/"task_comparison.pdf")
plt.close(fig)

fig,axs=plt.subplots(1,3,figsize=(13,4),layout="constrained")
for ax,(metric,title,unit) in zip(axs,[
    ("hand_xy_rmse_cm","Hand XY RMSE","cm"),
    ("human_xy_force_rms_N","Human planar force RMS","N"),
    ("peak_roll_deg","Peak roll: attitude trade-off","deg")]):
    for i,task in enumerate(tasks):
        for mode,offset in [("passive",-.18),("active",.18)]:
            vals=[m[metric] for m in summary.values() if m["task"]==task and m["mode"]==mode and metric in m]
            if not vals:
                continue
            x=i+offset
            ax.bar(x,np.mean(vals),width=.34,color=colors[mode],alpha=.7,label=labels[mode] if i==0 else None)
            ax.scatter(x+np.linspace(-.07,.07,len(vals)),vals,s=18,color="black",zorder=3)
    ax.set(xticks=range(3),xticklabels=tasks,title=title,ylabel=unit)
    ax.grid(axis="y",alpha=.2)
axs[0].legend(fontsize=8)
fig.savefig(HERE/"metric_comparison.png",dpi=180)
fig.savefig(HERE/"metric_comparison.pdf")
plt.close(fig)
