import numpy as np

from pr2_virtual_human.comparison_metrics import compute_run_metrics


def _row(time: float) -> dict[str, float | str]:
    row: dict[str, float | str] = {"time": time}
    for axis in "xyz":
        row[f"position_error_{axis}"] = 0.1
        row[f"orientation_error_{axis}"] = 0.01
        row[f"human_force_{axis}"] = 1.0
        row[f"human_task_torque_{axis}"] = 0.5
        row[f"actual_position_{axis}"] = time
        row[f"actual_rotvec_{axis}"] = 0.1 * time
    return row


def test_metrics_schema_and_finite_values() -> None:
    metrics = compute_run_metrics(
        [_row(0.0), _row(0.02), _row(0.04)],
        {"force": np.full(3, 60.0), "task_torque": np.full(3, 60.0)},
    )
    assert metrics["samples"] == 3
    assert metrics["all_required_values_finite"] is True
    assert set(metrics["position_rmse_m"]) == set("xyz")
    assert metrics["human_wrench_saturation_fraction"] == 0.0


def test_metrics_reject_non_monotonic_sim_time() -> None:
    try:
        compute_run_metrics(
            [_row(0.0), _row(0.0)],
            {"force": np.full(3, 60.0), "task_torque": np.full(3, 60.0)},
        )
    except ValueError as error:
        assert "strictly increasing" in str(error)
    else:
        raise AssertionError("non-monotonic sim time must be rejected")
