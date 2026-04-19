#!/usr/bin/env python3
"""PR2 末端力响应离线可视化脚本。

读取 pr2_arm_force_injector 生成的 CSV，绘制：
  - 施加力 vs 时间（上行）
  - 末端位移 vs 时间（下行，相对初始位置）

同时计算上升时间、峰值位移、稳态位移等指标。

用法：
  python3 plot_arm_response.py --csv /tmp/arm_response_1d.csv \\
      --title "PR2 Arm X Step Response" --save arm_response.png
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def load_csv(path: str) -> dict:
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        sys.exit(f"错误：CSV 文件为空 {path}")
    t0 = float(data["timestamp"][0])
    return {
        "t":       data["timestamp"].astype(float) - t0,
        "pos_x":   data["pos_x"].astype(float) - float(data["pos_x"][0]),
        "pos_y":   data["pos_y"].astype(float) - float(data["pos_y"][0]),
        "pos_z":   data["pos_z"].astype(float) - float(data["pos_z"][0]),
        "force_x": data["force_x"].astype(float),
        "force_y": data["force_y"].astype(float),
        "force_z": data["force_z"].astype(float),
    }


def step_onset(force: np.ndarray, t: np.ndarray, threshold: float = 0.5) -> float | None:
    idx = np.argmax(np.abs(force) > threshold)
    return float(t[idx]) if idx < len(t) and np.abs(force[idx]) > threshold else None


def compute_metrics(t: np.ndarray, pos: np.ndarray, onset_t: float | None) -> dict:
    if onset_t is None:
        return {}
    mask = t >= onset_t
    t_rel = t[mask] - onset_t
    p_rel = pos[mask]
    if len(p_rel) == 0:
        return {}
    peak = float(np.max(np.abs(p_rel)))
    idx_63 = np.argmax(np.abs(p_rel) >= 0.63 * peak) if peak > 1e-6 else 0
    rise = float(t_rel[idx_63]) if idx_63 < len(t_rel) else None
    ss = float(np.mean(p_rel[-min(20, len(p_rel)):]))
    return {"peak_disp_mm": round(peak * 1000, 3),
            "rise_time_s": round(rise, 4) if rise is not None else None,
            "ss_disp_mm": round(ss * 1000, 3)}


def plot_response(d: dict, title: str, save: str | None) -> None:
    axes_labels = ["X", "Y", "Z"]
    force_keys = ["force_x", "force_y", "force_z"]
    pos_keys = ["pos_x", "pos_y", "pos_z"]
    colors_f = ["#d62728", "#2ca02c", "#1f77b4"]
    colors_p = ["#8c1515", "#155215", "#0b3b6b"]

    fig = plt.figure(figsize=(14, 7))
    fig.suptitle(title, fontsize=13, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    for i, lbl in enumerate(axes_labels):
        fk, pk = force_keys[i], pos_keys[i]
        onset = step_onset(d[fk], d["t"])
        metrics = compute_metrics(d["t"], d[pk], onset)

        ax_f = fig.add_subplot(gs[0, i])
        ax_f.plot(d["t"], d[fk], color=colors_f[i], linewidth=1.5)
        ax_f.set_title(f"F_{lbl} [N]", fontsize=10)
        ax_f.set_xlabel("Time [s]", fontsize=8)
        ax_f.set_ylabel("Force [N]", fontsize=8)
        ax_f.grid(True, alpha=0.4)
        if onset is not None:
            ax_f.axvline(onset, color="gray", linestyle="--", alpha=0.6, label="onset")

        ax_p = fig.add_subplot(gs[1, i])
        ax_p.plot(d["t"], d[pk] * 1000.0, color=colors_p[i], linewidth=1.5)
        ax_p.set_title(f"EE Δpos_{lbl} [mm]", fontsize=10)
        ax_p.set_xlabel("Time [s]", fontsize=8)
        ax_p.set_ylabel("Displacement [mm]", fontsize=8)
        ax_p.grid(True, alpha=0.4)
        if onset is not None:
            ax_p.axvline(onset, color="gray", linestyle="--", alpha=0.6)

        if metrics:
            txt = "\n".join(
                f"{k}={v}" for k, v in metrics.items() if v is not None)
            ax_p.text(0.03, 0.97, txt, transform=ax_p.transAxes,
                      verticalalignment="top", fontsize=7,
                      bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.6))

    if save:
        plt.savefig(save, dpi=150, bbox_inches="tight")
        print(f"图像已保存至 {save}")
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="PR2 末端力响应可视化")
    parser.add_argument("--csv", required=True, help="CSV 日志文件路径")
    parser.add_argument("--title", default="PR2 Arm Force Response",
                        help="图像标题")
    parser.add_argument("--save", default=None,
                        help="保存 PNG 路径（不指定则只显示）")
    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        sys.exit(f"文件不存在: {args.csv}")

    d = load_csv(args.csv)
    plot_response(d, args.title, args.save)


if __name__ == "__main__":
    main()
