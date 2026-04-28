from pathlib import Path
import importlib.util
import json

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "audit_pr2_mjcf_limits.py"
_SPEC = importlib.util.spec_from_file_location("audit_pr2_mjcf_limits", SCRIPT_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_audit_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_audit_module)

audit_paths = _audit_module.audit_paths
infer_subsystem = _audit_module.infer_subsystem
write_markdown_report = _audit_module.write_markdown_report


def test_infer_subsystem_groups_pr2_names():
    assert infer_subsystem("fl_caster_l_wheel_joint") == "base_casters_wheels"
    assert infer_subsystem("torso_lift_joint") == "torso"
    assert infer_subsystem("l_shoulder_pan_joint") == "left_arm"
    assert infer_subsystem("r_wrist_roll_joint") == "right_arm"
    assert infer_subsystem("l_gripper_l_finger_joint") == "grippers"
    assert infer_subsystem("floor", kind="geom") == "scene_contact"


def test_audit_extracts_limit_like_fields_and_groups_them(tmp_path: Path):
    model = tmp_path / "robot_pr2.xml"
    scene = tmp_path / "scene.xml"
    model.write_text(
        """
        <mujoco>
          <worldbody>
            <body name="base_link">
              <joint name="fl_caster_l_wheel_joint" type="hinge" damping="1" actuatorfrcrange="-7 7" />
              <joint name="l_shoulder_pan_joint" type="hinge" range="-1 1" actuatorfrcrange="-30 30" />
              <joint name="torso_lift_joint" type="slide" damping="20000" actuatorfrcrange="-10000 10000" />
              <geom name="base_footprint" friction="1 0.005 0.0001" condim="3" />
            </body>
          </worldbody>
          <actuator>
            <velocity name="fl_caster_l_wheel_vel" joint="fl_caster_l_wheel_joint" ctrlrange="-20 20" gear="1" />
            <motor name="l_shoulder_pan_tau" joint="l_shoulder_pan_joint" ctrlrange="-200 200" forcerange="-30 30" />
          </actuator>
        </mujoco>
        """,
        encoding="utf-8",
    )
    scene.write_text(
        """
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" condim="3" solref="0.001 0.01" solimp="0.9 0.95 0.001" />
          </worldbody>
        </mujoco>
        """,
        encoding="utf-8",
    )

    audit = audit_paths(model, scene)

    assert audit["counts"]["joints"] == 3
    assert audit["counts"]["actuators"] == 2
    assert audit["counts"]["geoms"] == 2
    assert "base_casters_wheels" in audit["by_subsystem"]
    assert "left_arm" in audit["by_subsystem"]
    assert "scene_contact" in audit["by_subsystem"]

    base_names = {entry["name"] for entry in audit["by_subsystem"]["base_casters_wheels"]}
    assert "fl_caster_l_wheel_joint" in base_names
    assert "fl_caster_l_wheel_vel" in base_names

    shoulder_actuator = next(
        entry for entry in audit["by_type"]["actuators"] if entry["name"] == "l_shoulder_pan_tau"
    )
    assert shoulder_actuator["attributes"]["ctrlrange"] == "-200 200"
    assert shoulder_actuator["attributes"]["forcerange"] == "-30 30"

    floor = next(entry for entry in audit["by_type"]["geoms"] if entry["name"] == "floor")
    assert floor["attributes"]["solref"] == "0.001 0.01"

    assert any("base wheel/caster" in item["reason"] for item in audit["bottleneck_candidates"])
    assert any(item["name"] == "torso_lift_joint" for item in audit["bottleneck_candidates"])


def test_markdown_report_and_json_are_serializable(tmp_path: Path):
    model = tmp_path / "robot_pr2.xml"
    model.write_text(
        """
        <mujoco>
          <worldbody><joint name="l_wrist_roll_joint" actuatorfrcrange="-10 10" /></worldbody>
        </mujoco>
        """,
        encoding="utf-8",
    )
    audit = audit_paths(model)
    json.dumps(audit)

    out = tmp_path / "audit.md"
    write_markdown_report(audit, out)
    text = out.read_text(encoding="utf-8")
    assert "PR2 MJCF Force/Actuator Limit Audit" in text
    assert "l_wrist_roll_joint" in text
