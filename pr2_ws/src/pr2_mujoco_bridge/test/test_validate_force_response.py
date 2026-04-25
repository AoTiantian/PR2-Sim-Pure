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
