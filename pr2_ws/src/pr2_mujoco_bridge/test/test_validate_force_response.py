from __future__ import annotations

import csv
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_force_response.py"


def _write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    fieldnames = [
        "timestamp",
        "pos_x",
        "pos_y",
        "pos_z",
        "force_x",
        "force_y",
        "force_z",
        "base_x",
        "base_y",
        "base_yaw",
        "ee_des_x",
        "ee_des_y",
        "ee_des_z",
        "base_ref_x",
        "base_ref_y",
        "base_ref_yaw",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _run_validator(path: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--csv", str(path), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def test_validator_fails_on_large_final_residual(tmp_path: Path) -> None:
    csv_path = tmp_path / "final_residual.csv"
    rows = []
    for i in range(200):
        pos_x = 0.0 if i < 50 else 0.03
        rows.append(
            {
                "timestamp": float(i) * 0.01,
                "pos_x": pos_x,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "force_x": 0.0,
                "force_y": 0.0,
                "force_z": 0.0,
                "base_x": 0.0,
                "base_y": 0.0,
                "base_yaw": 0.0,
            }
        )
    _write_csv(csv_path, rows)

    result = _run_validator(csv_path, "--max-ee-final-mm", "10")

    assert result.returncode == 1
    assert "EE final" in result.stdout


def test_validator_fails_on_base_tail_drift(tmp_path: Path) -> None:
    csv_path = tmp_path / "base_tail_drift.csv"
    rows = []
    for i in range(200):
        if i < 80:
            base_x = 0.0
        else:
            base_x = (i - 80) * 2.0e-6
        rows.append(
            {
                "timestamp": float(i) * 0.01,
                "pos_x": 0.0,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "force_x": 0.0,
                "force_y": 0.0,
                "force_z": 0.0,
                "base_x": base_x,
                "base_y": 0.0,
                "base_yaw": 0.0,
            }
        )
    _write_csv(csv_path, rows)

    result = _run_validator(csv_path, "--max-base-linear-tail-drift-mm", "0.05")

    assert result.returncode == 1
    assert "Base linear tail drift" in result.stdout


def test_validator_passes_when_final_and_tail_are_bounded(tmp_path: Path) -> None:
    csv_path = tmp_path / "bounded.csv"
    rows = []
    for i in range(200):
        pos_x = 0.0 if i < 40 else 0.004
        rows.append(
            {
                "timestamp": float(i) * 0.01,
                "pos_x": pos_x,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "force_x": 0.0,
                "force_y": 0.0,
                "force_z": 0.0,
                "base_x": 0.0,
                "base_y": 0.0,
                "base_yaw": 0.0,
            }
        )
    _write_csv(csv_path, rows)

    result = _run_validator(
        csv_path,
        "--max-ee-final-mm",
        "10",
        "--max-ee-tail-std-mm",
        "1",
        "--max-base-linear-tail-drift-mm",
        "0.05",
    )

    assert result.returncode == 0
    assert "RESULT: PASS" in result.stdout


def test_force_tracking_validator_accepts_held_reference_and_tracking(tmp_path: Path) -> None:
    csv_path = tmp_path / "force_tracking_pass.csv"
    rows = []
    for i in range(260):
        if i < 50:
            ref = 0.0
        elif i < 150:
            ref = min(0.0015 * (i - 50), 0.15)
        else:
            ref = 0.15
        actual = ref - (0.004 if 50 <= i < 180 else 0.001)
        rows.append(
            {
                "timestamp": float(i) * 0.01,
                "pos_x": actual,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "force_x": 8.0 if 50 <= i < 150 else 0.0,
                "force_y": 0.0,
                "force_z": 0.0,
                "base_x": 0.0,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_des_x": ref,
                "ee_des_y": 0.0,
                "ee_des_z": 0.0,
                "base_ref_x": 0.0,
                "base_ref_y": 0.0,
                "base_ref_yaw": 0.0,
            }
        )
    _write_csv(csv_path, rows)

    result = _run_validator(
        csv_path,
        "--force-tracking",
        "--reference-prefix",
        "ee_des",
        "--actual-prefix",
        "pos",
        "--baseline-skip-samples",
        "40",
        "--tail-samples",
        "80",
        "--min-reference-peak-mm",
        "120",
        "--min-actual-peak-mm",
        "110",
        "--max-tracking-error-mm",
        "6",
        "--max-reference-tail-drift-mm",
        "0.5",
    )

    assert result.returncode == 0
    assert "Force-tracking metrics" in result.stdout
    assert "RESULT: PASS" in result.stdout


def test_force_tracking_validator_fails_when_reference_drifts_after_release(tmp_path: Path) -> None:
    csv_path = tmp_path / "force_tracking_drift.csv"
    rows = []
    for i in range(260):
        ref = 0.0 if i < 50 else 0.12 + max(0, i - 150) * 0.00005
        rows.append(
            {
                "timestamp": float(i) * 0.01,
                "pos_x": ref,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "force_x": 8.0 if 50 <= i < 150 else 0.0,
                "force_y": 0.0,
                "force_z": 0.0,
                "base_x": 0.0,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_des_x": ref,
                "ee_des_y": 0.0,
                "ee_des_z": 0.0,
                "base_ref_x": 0.0,
                "base_ref_y": 0.0,
                "base_ref_yaw": 0.0,
            }
        )
    _write_csv(csv_path, rows)

    result = _run_validator(
        csv_path,
        "--force-tracking",
        "--reference-prefix",
        "ee_des",
        "--actual-prefix",
        "pos",
        "--baseline-skip-samples",
        "40",
        "--max-reference-tail-drift-mm",
        "2",
    )

    assert result.returncode == 1
    assert "Reference tail drift" in result.stdout
