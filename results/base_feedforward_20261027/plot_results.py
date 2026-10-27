"""Plots all new matched S22 runs, including the zero-feedforward ablation."""
from pathlib import Path
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "runtime_summary.json").read_text())
colors = {"passive":"#4c78a8", "zero_feedforward":"#e59a31", "active":"#2a9d67"}
names = {"passive":"Original passive", "zero_feedforward":"New allocation, no feedforward",
         "active":"Known base path feedforward"}
fig, ax = plt.subplots(2,2,figsize=(11,8),layout="constrained")
groups = {k:[] for k in colors}
first = True
for tag,m in summary.items():
    if "history" not in m:
        continue
    a = np.genfromtxt(HERE / m["history"],delimiter=",",names=True,dtype=None,encoding="utf8")
    mode=m["mode"];c=colors[mode]
    t=a["time"]-a["time"][0]
    target=np.column_stack([a["desired_position_"+k] for k in "xy"])
    actual=np.column_stack([a["actual_position_"+k] for k in "xy"])
    anchor=target[0].copy();target-=anchor;actual-=anchor
    ax[0,0].plot(actual[:,0],actual[:,1],color=c,alpha=.6,lw=1)
    if first:
        ax[0,0].plot(target[:,0],target[:,1],"k--",lw=2,label="Desired hand path")
        first=False
    r=Rotation.from_rotvec(np.column_stack([a["actual_rotvec_"+k] for k in "xyz"]))
    d=Rotation.from_rotvec(np.column_stack([a["desired_rotvec_"+k] for k in "xyz"]))
    angle=np.rad2deg((d*r.inv()).magnitude())
    force=np.hypot(a["human_force_x"],a["human_force_y"])
    groups[mode].append((t,angle,force,m["hand_xy_rmse_cm"]))
for i,(mode,rows) in enumerate(groups.items()):
    if not rows:
        continue
    c=colors[mode];grid=np.linspace(0,25,600)
    ax[0,0].plot([],[],color=c,label=names[mode])
    for panel,column in [(ax[0,1],1),(ax[1,0],2)]:
        vals=np.array([np.interp(grid,r[0],r[column]) for r in rows])
        panel.plot(grid,vals.mean(axis=0),color=c,label=names[mode])
        panel.fill_between(grid,vals.min(axis=0),vals.max(axis=0),color=c,alpha=.15)
    values=[r[3] for r in rows]
    ax[1,1].bar(i,np.mean(values),color=c,alpha=.65)
    ax[1,1].scatter(i+np.linspace(-.08,.08,len(values)),values,color="black",s=24,zorder=3)
ax[0,0].set(xlabel="Hand x relative to desired start (m)",ylabel="Hand y (m)",title="Full S22 hand trajectories")
ax[0,0].axis("equal");ax[0,0].legend(fontsize=8)
ax[0,1].set(xlabel="Task time (s)",ylabel="SO(3) attitude error (deg)",title="Mean and min/max across runs")
ax[1,0].set(xlabel="Task time (s)",ylabel="Human planar force magnitude (N)")
ax[1,1].set(xticks=range(3),xticklabels=["Passive","No feedforward","Active"],ylabel="Hand XY RMSE (cm)",title="All independent runs (dots)")
for a in ax.flat:
    a.grid(alpha=.2)
fig.savefig(HERE / "S22_comparison.png",dpi=180)
fig.savefig(HERE / "S22_comparison.pdf")
plt.close(fig)

fig,ax=plt.subplots(figsize=(7,5),layout="constrained")
first=True
for tag,m in summary.items():
    if m["mode"]!="active" or not (HERE / tag / "allocation.jsonl").exists():
        continue
    rows=[json.loads(x) for x in (HERE / tag / "allocation.jsonl").read_text().splitlines()]
    actual=np.array([r["actual_base_position"][:2] for r in rows])
    desired=np.array([r["robot_path_position"][:2] for r in rows])
    anchor=desired[0].copy();actual-=anchor;desired-=anchor
    ax.plot(actual[:,0],actual[:,1],label=tag,lw=1.2)
    if first:
        ax.plot(desired[:,0],desired[:,1],"k--",lw=2,label="Independent robot path")
        first=False
ax.set(xlabel="Base x relative to planned start (m)",ylabel="Base y (m)",title="Base feedforward: execution error remains")
ax.axis("equal");ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(HERE / "base_path_execution.png",dpi=180)
fig.savefig(HERE / "base_path_execution.pdf")
plt.close(fig)
