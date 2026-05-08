#!/usr/bin/env python3
"""验证 PR2 力响应日志。"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from statistics import pstdev


def _series(
    rows: list[dict[str, str]],
    prefix: str,
    baseline_index: int,
) -> list[float]:
    values = [float(row[prefix]) for row in rows]
    base = values[min(max(baseline_index, 0), len(values) - 1)]
    return [value - base for value in values]


def _count_sign_changes(values: list[float], diff_epsilon: float) -> int:
    signs: list[int] = []
    for diff in values:
        if diff > diff_epsilon:
            signs.append(1)
        elif diff < -diff_epsilon:
            signs.append(-1)
    return sum(signs[i + 1] != signs[i] for i in range(len(signs) - 1))


def _axis_metrics(
    series: list[float],
    tail_samples: int,
    diff_epsilon: float,
) -> dict[str, float]:
    tail = series[-min(tail_samples, len(series)) :]
    tail_diffs = [tail[i + 1] - tail[i] for i in range(len(tail) - 1)]
    return {
        "final_mm": abs(series[-1]) * 1000.0,
        "peak_mm": max(abs(value) for value in series) * 1000.0,
        "tail_std_mm": pstdev(tail) * 1000.0 if len(tail) > 1 else 0.0,
        "tail_drift_mm": abs(tail[-1] - tail[0]) * 1000.0 if len(tail) > 1 else 0.0,
        "tail_range_mm": (max(tail) - min(tail)) * 1000.0 if tail else 0.0,
        "sign_changes": float(_count_sign_changes(tail_diffs, diff_epsilon)),
    }


def _tracking_axes(raw: str) -> list[str]:
    axes = [axis.strip() for axis in raw.split(",") if axis.strip()]
    return axes or ["x", "y", "z"]


def _tracking_metrics(
    rows: list[dict[str, str]],
    reference_prefix: str,
    actual_prefix: str,
    axes: list[str],
    baseline_index: int,
    tail_samples: int,
) -> tuple[dict[str, dict[str, float]], list[str]]:
    metrics: dict[str, dict[str, float]] = {}
    missing: list[str] = []
    for axis in axes:
        ref_key = f"{reference_prefix}_{axis}"
        actual_key = f"{actual_prefix}_{axis}"
        if ref_key not in rows[0]:
            missing.append(ref_key)
            continue
        if actual_key not in rows[0]:
            missing.append(actual_key)
            continue
        reference = _series(rows, ref_key, baseline_index)
        actual = _series(rows, actual_key, baseline_index)
        error = [actual[i] - reference[i] for i in range(len(reference))]
        ref_tail = reference[-min(tail_samples, len(reference)) :]
        metrics[axis] = {
            "reference_peak_mm": max(abs(value) for value in reference) * 1000.0,
            "actual_peak_mm": max(abs(value) for value in actual) * 1000.0,
            "tracking_error_mm": max(abs(value) for value in error) * 1000.0,
            "reference_tail_std_mm": pstdev(ref_tail) * 1000.0 if len(ref_tail) > 1 else 0.0,
            "reference_tail_drift_mm": abs(ref_tail[-1] - ref_tail[0]) * 1000.0
            if len(ref_tail) > 1
            else 0.0,
        }
    return metrics, missing


def main() -> None:
    parser = argparse.ArgumentParser(description="验证 PR2 力响应 CSV")
    parser.add_argument("--csv", required=True, help="CSV 文件路径")
    parser.add_argument("--tail-samples", type=int, default=120, help="尾段样本数")
    parser.add_argument(
        "--baseline-skip-samples",
        type=int,
        default=0,
        help="定义零点前先跳过多少个样本",
    )
    parser.add_argument(
        "--tail-diff-epsilon-mm",
        type=float,
        default=0.0,
        help="统计尾段方向翻转时忽略小于该阈值的微小增量",
    )
    parser.add_argument("--max-ee-peak-mm", type=float, default=None)
    parser.add_argument("--min-ee-peak-mm", type=float, default=None)
    parser.add_argument("--max-ee-final-mm", type=float, default=None)
    parser.add_argument("--max-ee-tail-std-mm", type=float, default=None)
    parser.add_argument("--max-ee-tail-drift-mm", type=float, default=None)
    parser.add_argument("--max-ee-tail-sign-changes", type=float, default=None)
    parser.add_argument("--min-base-linear-peak-mm", type=float, default=None)
    parser.add_argument("--max-base-linear-tail-std-mm", type=float, default=None)
    parser.add_argument("--max-base-linear-tail-drift-mm", type=float, default=None)
    parser.add_argument("--max-base-linear-tail-sign-changes", type=float, default=None)
    parser.add_argument("--max-base-yaw-tail-std-deg", type=float, default=None)
    parser.add_argument("--max-base-yaw-tail-drift-deg", type=float, default=None)
    parser.add_argument("--max-base-yaw-tail-sign-changes", type=float, default=None)
    parser.add_argument(
        "--force-tracking",
        action="store_true",
        help="Enable dynamic-reference force-tracking checks.",
    )
    parser.add_argument(
        "--reference-prefix",
        default="ee_des",
        help="CSV prefix for dynamic reference columns, for example ee_des or base_ref.",
    )
    parser.add_argument(
        "--actual-prefix",
        default="pos",
        help="CSV prefix for actual response columns, for example pos or base.",
    )
    parser.add_argument(
        "--tracking-axes",
        default="x,y,z",
        help="Comma-separated tracking axes, for example x,y,z or x,y,yaw.",
    )
    parser.add_argument("--min-reference-peak-mm", type=float, default=None)
    parser.add_argument("--max-reference-peak-mm", type=float, default=None)
    parser.add_argument("--min-actual-peak-mm", type=float, default=None)
    parser.add_argument("--max-actual-peak-mm", type=float, default=None)
    parser.add_argument("--max-tracking-error-mm", type=float, default=None)
    parser.add_argument("--max-reference-tail-std-mm", type=float, default=None)
    parser.add_argument("--max-reference-tail-drift-mm", type=float, default=None)
    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        sys.exit(f"CSV 不存在: {args.csv}")

    with open(args.csv, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"CSV 为空: {args.csv}")

    diff_epsilon = args.tail_diff_epsilon_mm / 1000.0
    ee_metrics = {
        axis: _axis_metrics(
            _series(rows, f"pos_{axis}", args.baseline_skip_samples),
            args.tail_samples,
            diff_epsilon,
        )
        for axis in ("x", "y", "z")
    }
    ee_peak_mm = max(metric["peak_mm"] for metric in ee_metrics.values())
    ee_final_mm = max(metric["final_mm"] for metric in ee_metrics.values())
    ee_tail_std_mm = max(metric["tail_std_mm"] for metric in ee_metrics.values())
    ee_tail_drift_mm = max(metric["tail_drift_mm"] for metric in ee_metrics.values())
    ee_tail_sign_changes = max(metric["sign_changes"] for metric in ee_metrics.values())

    print("EE metrics:")
    for axis, metric in ee_metrics.items():
        print(
            f"  {axis}: peak={metric['peak_mm']:.3f} mm, "
            f"final={metric['final_mm']:.3f} mm, "
            f"tail_std={metric['tail_std_mm']:.3f} mm, "
            f"tail_drift={metric['tail_drift_mm']:.3f} mm, "
            f"sign_changes={int(metric['sign_changes'])}"
        )

    failures: list[str] = []
    if args.max_ee_peak_mm is not None and ee_peak_mm > args.max_ee_peak_mm:
        failures.append(f"EE peak {ee_peak_mm:.3f} mm > {args.max_ee_peak_mm:.3f} mm")
    if args.min_ee_peak_mm is not None and ee_peak_mm < args.min_ee_peak_mm:
        failures.append(f"EE peak {ee_peak_mm:.3f} mm < {args.min_ee_peak_mm:.3f} mm")
    if args.max_ee_final_mm is not None and ee_final_mm > args.max_ee_final_mm:
        failures.append(f"EE final {ee_final_mm:.3f} mm > {args.max_ee_final_mm:.3f} mm")
    if args.max_ee_tail_std_mm is not None and ee_tail_std_mm > args.max_ee_tail_std_mm:
        failures.append(f"EE tail std {ee_tail_std_mm:.3f} mm > {args.max_ee_tail_std_mm:.3f} mm")
    if args.max_ee_tail_drift_mm is not None and ee_tail_drift_mm > args.max_ee_tail_drift_mm:
        failures.append(f"EE tail drift {ee_tail_drift_mm:.3f} mm > {args.max_ee_tail_drift_mm:.3f} mm")
    if (
        args.max_ee_tail_sign_changes is not None
        and ee_tail_sign_changes > args.max_ee_tail_sign_changes
    ):
        failures.append(
            f"EE tail sign changes {ee_tail_sign_changes:.0f} > {args.max_ee_tail_sign_changes:.0f}"
        )

    if {"base_x", "base_y", "base_yaw"}.issubset(rows[0]):
        base_x = _axis_metrics(
            _series(rows, "base_x", args.baseline_skip_samples),
            args.tail_samples,
            diff_epsilon,
        )
        base_y = _axis_metrics(
            _series(rows, "base_y", args.baseline_skip_samples),
            args.tail_samples,
            diff_epsilon,
        )
        base_yaw = _axis_metrics(
            _series(rows, "base_yaw", args.baseline_skip_samples),
            args.tail_samples,
            diff_epsilon,
        )
        base_linear_peak_mm = max(base_x["peak_mm"], base_y["peak_mm"])
        base_linear_tail_std_mm = max(base_x["tail_std_mm"], base_y["tail_std_mm"])
        base_linear_tail_drift_mm = max(base_x["tail_drift_mm"], base_y["tail_drift_mm"])
        base_linear_tail_sign_changes = max(base_x["sign_changes"], base_y["sign_changes"])
        base_yaw_tail_std_deg = base_yaw["tail_std_mm"] * 0.0572958
        base_yaw_tail_drift_deg = base_yaw["tail_drift_mm"] * 0.0572958
        print("Base metrics:")
        print(
            f"  linear: peak={base_linear_peak_mm:.3f} mm, "
            f"tail_std={base_linear_tail_std_mm:.3f} mm, "
            f"tail_drift={base_linear_tail_drift_mm:.3f} mm, "
            f"sign_changes={int(base_linear_tail_sign_changes)}"
        )
        print(
            f"  yaw: peak={base_yaw['peak_mm'] * 0.0572958:.3f} deg, "
            f"tail_std={base_yaw_tail_std_deg:.4f} deg, "
            f"tail_drift={base_yaw_tail_drift_deg:.4f} deg, "
            f"sign_changes={int(base_yaw['sign_changes'])}"
        )
        if (
            args.min_base_linear_peak_mm is not None
            and base_linear_peak_mm < args.min_base_linear_peak_mm
        ):
            failures.append(
                f"Base linear peak {base_linear_peak_mm:.3f} mm < {args.min_base_linear_peak_mm:.3f} mm"
            )
        if (
            args.max_base_linear_tail_std_mm is not None
            and base_linear_tail_std_mm > args.max_base_linear_tail_std_mm
        ):
            failures.append(
                f"Base linear tail std {base_linear_tail_std_mm:.3f} mm > {args.max_base_linear_tail_std_mm:.3f} mm"
            )
        if (
            args.max_base_linear_tail_drift_mm is not None
            and base_linear_tail_drift_mm > args.max_base_linear_tail_drift_mm
        ):
            failures.append(
                f"Base linear tail drift {base_linear_tail_drift_mm:.3f} mm > {args.max_base_linear_tail_drift_mm:.3f} mm"
            )
        if (
            args.max_base_linear_tail_sign_changes is not None
            and base_linear_tail_sign_changes > args.max_base_linear_tail_sign_changes
        ):
            failures.append(
                f"Base linear tail sign changes {base_linear_tail_sign_changes:.0f} > {args.max_base_linear_tail_sign_changes:.0f}"
            )
        if (
            args.max_base_yaw_tail_std_deg is not None
            and base_yaw_tail_std_deg > args.max_base_yaw_tail_std_deg
        ):
            failures.append(
                f"Base yaw tail std {base_yaw_tail_std_deg:.4f} deg > {args.max_base_yaw_tail_std_deg:.4f} deg"
            )
        if (
            args.max_base_yaw_tail_drift_deg is not None
            and base_yaw_tail_drift_deg > args.max_base_yaw_tail_drift_deg
        ):
            failures.append(
                f"Base yaw tail drift {base_yaw_tail_drift_deg:.4f} deg > {args.max_base_yaw_tail_drift_deg:.4f} deg"
            )
        if (
            args.max_base_yaw_tail_sign_changes is not None
            and base_yaw["sign_changes"] > args.max_base_yaw_tail_sign_changes
        ):
            failures.append(
                f"Base yaw tail sign changes {base_yaw['sign_changes']:.0f} > {args.max_base_yaw_tail_sign_changes:.0f}"
            )

    if args.force_tracking:
        tracking, missing = _tracking_metrics(
            rows=rows,
            reference_prefix=args.reference_prefix,
            actual_prefix=args.actual_prefix,
            axes=_tracking_axes(args.tracking_axes),
            baseline_index=args.baseline_skip_samples,
            tail_samples=args.tail_samples,
        )
        if missing:
            failures.append(f"Missing force-tracking columns: {', '.join(sorted(missing))}")
        if tracking:
            reference_peak_mm = max(metric["reference_peak_mm"] for metric in tracking.values())
            actual_peak_mm = max(metric["actual_peak_mm"] for metric in tracking.values())
            tracking_error_mm = max(metric["tracking_error_mm"] for metric in tracking.values())
            reference_tail_std_mm = max(
                metric["reference_tail_std_mm"] for metric in tracking.values()
            )
            reference_tail_drift_mm = max(
                metric["reference_tail_drift_mm"] for metric in tracking.values()
            )
            print("Force-tracking metrics:")
            for axis, metric in tracking.items():
                print(
                    f"  {axis}: ref_peak={metric['reference_peak_mm']:.3f} mm, "
                    f"actual_peak={metric['actual_peak_mm']:.3f} mm, "
                    f"track_error={metric['tracking_error_mm']:.3f} mm, "
                    f"ref_tail_std={metric['reference_tail_std_mm']:.3f} mm, "
                    f"ref_tail_drift={metric['reference_tail_drift_mm']:.3f} mm"
                )
            if (
                args.min_reference_peak_mm is not None
                and reference_peak_mm < args.min_reference_peak_mm
            ):
                failures.append(
                    f"Reference peak {reference_peak_mm:.3f} mm < {args.min_reference_peak_mm:.3f} mm"
                )
            if (
                args.max_reference_peak_mm is not None
                and reference_peak_mm > args.max_reference_peak_mm
            ):
                failures.append(
                    f"Reference peak {reference_peak_mm:.3f} mm > {args.max_reference_peak_mm:.3f} mm"
                )
            if args.min_actual_peak_mm is not None and actual_peak_mm < args.min_actual_peak_mm:
                failures.append(
                    f"Actual peak {actual_peak_mm:.3f} mm < {args.min_actual_peak_mm:.3f} mm"
                )
            if args.max_actual_peak_mm is not None and actual_peak_mm > args.max_actual_peak_mm:
                failures.append(
                    f"Actual peak {actual_peak_mm:.3f} mm > {args.max_actual_peak_mm:.3f} mm"
                )
            if (
                args.max_tracking_error_mm is not None
                and tracking_error_mm > args.max_tracking_error_mm
            ):
                failures.append(
                    f"Tracking error {tracking_error_mm:.3f} mm > {args.max_tracking_error_mm:.3f} mm"
                )
            if (
                args.max_reference_tail_std_mm is not None
                and reference_tail_std_mm > args.max_reference_tail_std_mm
            ):
                failures.append(
                    f"Reference tail std {reference_tail_std_mm:.3f} mm > {args.max_reference_tail_std_mm:.3f} mm"
                )
            if (
                args.max_reference_tail_drift_mm is not None
                and reference_tail_drift_mm > args.max_reference_tail_drift_mm
            ):
                failures.append(
                    f"Reference tail drift {reference_tail_drift_mm:.3f} mm > {args.max_reference_tail_drift_mm:.3f} mm"
                )

    if failures:
        print("RESULT: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        sys.exit(1)

    print("RESULT: PASS")


if __name__ == "__main__":
    main()
