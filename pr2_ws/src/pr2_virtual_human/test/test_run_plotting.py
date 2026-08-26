from pathlib import Path

from pr2_virtual_human.run_plotting import save_run_plots


def _rows(condition: str) -> list[dict[str, float | str]]:
    result = []
    for index in range(3):
        row: dict[str, float | str] = {
            "time": 0.02 * index,
            "condition": condition,
        }
        for axis_index, axis in enumerate("xyz"):
            row[f"desired_position_{axis}"] = 0.1 * index
            row[f"actual_position_{axis}"] = 0.1 * index + 0.01
            row[f"desired_rotvec_{axis}"] = 0.01 * index
            row[f"actual_rotvec_{axis}"] = 0.01 * index + 0.001
            row[f"human_force_{axis}"] = float(axis_index + index)
            row[f"human_task_torque_{axis}"] = float(axis_index - index)
            row[f"robot_force_{axis}"] = float(index) if condition == "human_robot" else float("nan")
            row[f"robot_torque_{axis}"] = float(-index) if condition == "human_robot" else float("nan")
        result.append(row)
    return result


def test_each_run_generates_trajectory_and_human_wrench_plots(tmp_path: Path) -> None:
    paths = save_run_plots(_rows("human_only"), "human_only", tmp_path)
    assert {path.name for path in paths} == {
        "trajectory_6d.png",
        "human_applied_wrench_6d.png",
    }
    assert all(path.stat().st_size > 0 for path in paths)


def test_robot_run_also_generates_measured_wrench_plot(tmp_path: Path) -> None:
    paths = save_run_plots(_rows("human_robot"), "human_robot", tmp_path)
    assert "robot_measured_wrench_6d.png" in {path.name for path in paths}
