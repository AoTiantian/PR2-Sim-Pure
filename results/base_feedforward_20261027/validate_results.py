"""Contact interruptions, approximate executed angular tracking, source audit."""
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
summary = json.loads((HERE / "runtime_summary.json").read_text())
for tag, m in summary.items():
    if "history" not in m:
        continue
    case = HERE / tag
    a = np.genfromtxt(HERE / m["history"], delimiter=",", names=True, dtype=None, encoding="utf8")
    for name in a.dtype.names:
        if a[name].dtype.kind in "fi":
            assert np.isfinite(a[name]).all(), (tag,name)
    lo, hi = a["time"][[0,-1]]
    bad, start, previous = [], None, None
    for line in (case / "bridge_debug.jsonl").open():
        record = json.loads(line)
        if record["hypothesisId"] != "H_BoardFriction":
            continue
        record = record["data"]
        t = record["sim_time"]
        if not lo <= t <= hi:
            continue
        if not record["grasped"] and start is None:
            start = t
        if record["grasped"] and start is not None:
            bad.append(t-start)
            start = None
        previous = t
    if start is not None:
        bad.append(previous-start+.002)
    m["longest_both_contact_interruption_ms"] = 1000*max(bad,default=0)
    if m["mode"] != "passive":
        rows = [json.loads(x) for x in (case / "allocation.jsonl").read_text().splitlines()]
        rows = [x for x in rows if lo <= x["sim_time"] <= hi]
        ts = np.array([x["sim_time"] for x in rows])
        cmd = np.array([x["original_admittance_twist"][3:] for x in rows])
        ee = np.load(case / "raw.npz")["ee"]
        _,idx = np.unique(ee[:,1],return_index=True)
        ee = ee[np.sort(idx)]
        begin = max(lo+.2,ee[0,1]+.06,ts[0])
        end = min(hi-.2,ee[-1,1]-.06,ts[-1])
        grid = np.linspace(begin,end,500)
        curve = Slerp(ee[:,1],Rotation.from_quat(ee[:,[6,7,8,5]]))
        omega = (curve(grid+.05)*curve(grid-.05).inv()).as_rotvec()/.1
        desired = np.column_stack([np.interp(grid,ts,cmd[:,k]) for k in range(3)])
        m["smoothed_ee_angular_tracking_rms_rad_s"] = float(np.sqrt(np.mean(np.sum((omega-desired)**2,axis=1))))
(HERE / "runtime_summary.json").write_text(json.dumps(summary,indent=2))
before = json.loads((HERE / "production_hashes_before.json").read_text())
after = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in before}
per_run = {}
for tag in summary:
    hashes = json.loads((HERE / tag / "source_hashes.json").read_text())
    per_run[tag] = all(hashes[str(ROOT/p)] == digest for p,digest in before.items())
audit = dict(all_unchanged=before==after, before=before, after=after,
             each_run_matches_original_production=per_run)
assert audit["all_unchanged"] and all(per_run.values())
(HERE / "production_integrity.json").write_text(json.dumps(audit,indent=2))
print("All completed CSV values finite; production unchanged across every run.")
