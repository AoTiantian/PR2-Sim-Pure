#!/usr/bin/env python3
"""
Plot the virtual wrench (fx, fy, fz) from a pr2_motion_logger CSV.

Typical usage:
  python3 plot_virtual_wrench.py /workspace/logs/pr2_motion_20260423_113203.csv

Outputs an SVG (default) next to the CSV unless --out is specified.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Segment:
    t0: float
    t1: float


def _f(x: str) -> float:
    try:
        return float(x)
    except Exception:
        return float("nan")


def _nan(x: float) -> bool:
    return x != x


def _load(csv_path: str) -> Tuple[List[float], List[float], List[float], List[float]]:
    with open(csv_path, newline="") as fp:
        reader = csv.DictReader(fp)
        t: List[float] = []
        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []
        for r in reader:
            t.append(_f(r.get("wall_time_sec", "")))
            fx.append(_f(r.get("fx", "")))
            fy.append(_f(r.get("fy", "")))
            fz.append(_f(r.get("fz", "")))
    if not t:
        raise RuntimeError("CSV has no rows")
    # convert to relative time for readability
    t0 = t[0]
    t = [ti - t0 for ti in t]
    return t, fx, fy, fz


def _detect_force_segments(t: List[float], fx: List[float], fy: List[float], fz: List[float], thr: float) -> List[Segment]:
    segs: List[Segment] = []
    in_seg = False
    s0: Optional[float] = None
    for ti, x, y, z in zip(t, fx, fy, fz):
        mag = math.sqrt((0.0 if _nan(x) else x) ** 2 + (0.0 if _nan(y) else y) ** 2 + (0.0 if _nan(z) else z) ** 2)
        on = mag > thr
        if on and not in_seg:
            in_seg = True
            s0 = ti
        if (not on) and in_seg:
            in_seg = False
            if s0 is not None:
                segs.append(Segment(t0=s0, t1=ti))
            s0 = None
    if in_seg and s0 is not None:
        segs.append(Segment(t0=s0, t1=t[-1]))
    return segs


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV path produced by pr2_motion_logger")
    ap.add_argument("--out", default="", help="output image path (default: <csv_basename>_wrench.svg)")
    ap.add_argument("--format", default="svg", choices=("svg", "png"), help="output format")
    ap.add_argument("--thr", type=float, default=0.5, help="force magnitude threshold (N) to detect active segments")
    args = ap.parse_args()

    csv_path = os.path.abspath(args.csv)
    t, fx, fy, fz = _load(csv_path)
    segs = _detect_force_segments(t, fx, fy, fz, thr=float(args.thr))

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required to plot. Install it in your environment."
        ) from exc

    fig, ax = plt.subplots(figsize=(10.5, 4.2), dpi=140)
    ax.plot(t, fx, label="fx (N)", linewidth=1.8)
    ax.plot(t, fy, label="fy (N)", linewidth=1.8)
    ax.plot(t, fz, label="fz (N)", linewidth=1.8)
    ax.axhline(0.0, color="#666", linewidth=0.8, alpha=0.6)

    for i, s in enumerate(segs):
        ax.axvspan(s.t0, s.t1, color="#ffb86c", alpha=0.18, linewidth=0)
        ax.text(
            (s.t0 + s.t1) * 0.5,
            0.98,
            f"force[{i}]",
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=9,
            color="#8a4b00",
        )

    ax.set_title(f"Virtual force from CSV: {os.path.basename(csv_path)}")
    ax.set_xlabel("t (s) [relative to first CSV row]")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", frameon=True)

    out_path = args.out.strip()
    if not out_path:
        base = os.path.splitext(csv_path)[0]
        out_path = f"{base}_wrench.{args.format}"
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    print(out_path)


if __name__ == "__main__":
    main()

