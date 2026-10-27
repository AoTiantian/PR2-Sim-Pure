import json
from pathlib import Path

import pytest
import rclpy

from pr2_virtual_human import comparison_recorder


def test_manifest_records_overridden_model_and_its_actual_timestep(tmp_path, monkeypatch):
    model_path = tmp_path / "different_step.xml"
    model_path.write_text('<mujoco><option timestep="0.0005"/><worldbody/></mujoco>')
    config = Path(__file__).parents[1] / "config/transport_comparison.yaml"
    monkeypatch.setattr(comparison_recorder, "save_run_plots", lambda *args: [])
    rclpy.init(args=["--ros-args", "-p", f"config_file:={config}",
                    "-p", f"model_path:={model_path}", "-p", f"output_root:={tmp_path}"])
    node = comparison_recorder.ComparisonRecorder()
    try:
        for time in [0., .02]:
            row = {"time": time}
            for prefix in ["position_error", "orientation_error", "human_force",
                           "human_task_torque", "actual_position", "actual_rotvec"]:
                row.update({f"{prefix}_{axis}": 0. for axis in "xyz"})
            node.rows.append(row)
        run_dir = node.save()
        manifest = json.loads((run_dir / "run_manifest.json").read_text())
        assert manifest["model_path"] == str(model_path)
        assert manifest["simulation_timestep_sec"] == pytest.approx(.0005)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
