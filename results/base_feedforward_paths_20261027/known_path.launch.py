"""Experimental launch reuses original plant/human/recorder without editing them."""
import importlib.util
import os
from pathlib import Path

import yaml
from launch.actions import ExecuteProcess


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("passive_transport_launch", ROOT / "pr2_ws/src/pr2_virtual_human/launch/transport_comparison.launch.py")
passive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(passive)
OriginalNode = passive.Node


def experiment_node(**kwargs):
    executable = kwargs["executable"]
    if executable not in ("pr2_qp_whole_body_admittance", "comparison_contract_guard"):
        return OriginalNode(**kwargs)
    if executable == "pr2_qp_whole_body_admittance" and os.environ.get("BASE_PATH_MODE") == "passive":
        return OriginalNode(**kwargs)
    parameters = {}
    for group in kwargs.get("parameters", []):
        parameters.update(group)
    if executable == "pr2_qp_whole_body_admittance":
        script = HERE / "experimental_qp.py"
        parameters.update(control_regime="base_path_with_angular_admittance",
                          trajectory_assist_gain=float(os.environ.get("BASE_PATH_GAIN", "1")),
                          robot_path_file=os.environ.get("BASE_PATH_FILE", str(HERE / "robot_path.yaml")))
    else:
        script = HERE / "experimental_guard.py"
    param_file = Path(os.environ["BASE_PATH_OUT"]) / f"{executable}_params.yaml"
    param_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    return ExecuteProcess(cmd=["python3", str(script), "--ros-args", "-r",
                               "__node:="+kwargs["name"], "--params-file", str(param_file)],
                          output="both")


passive.Node = experiment_node


def generate_launch_description():
    return passive.generate_launch_description()
