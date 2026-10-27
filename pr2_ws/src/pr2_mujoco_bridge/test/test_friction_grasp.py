from pathlib import Path
import json
from itertools import product

import mujoco
import numpy as np
import pytest
import yaml

from pr2_mujoco_bridge.friction_grasp import validate_friction_grasp


def scene_ids(model):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "grasp_board")
    geoms = [
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)]
        for name in ("board_geom", "l_gripper_l_board_contact", "l_gripper_r_board_contact")
    ]
    return body, *geoms


def test_production_carrying_scene_has_free_board_and_enabled_friction_contacts():
    root = Path(__file__).resolve().parents[4]
    model = mujoco.MjModel.from_xml_path(
        str(root / "unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml")
    )
    data = mujoco.MjData(model)
    validate_friction_grasp(model, data, *scene_ids(model))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                            "robot_board_point_contact") == -1
    board, *geoms = scene_ids(model)
    assert model.body_mass[board] == pytest.approx(1.8)
    for geom in geoms:
        assert model.geom_condim[geom[0]] >= 3
        assert model.geom_friction[geom[0], 0] == pytest.approx(2.5)
    # A dropped board must collide with the ground, independently of the pads.
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    board_geom = geoms[0][0]
    assert ((model.geom_contype[floor] & model.geom_conaffinity[board_geom])
            or (model.geom_contype[board_geom] & model.geom_conaffinity[floor]))


def test_production_pregrasp_is_near_pads_and_open_fingers_have_clearance():
    root = Path(__file__).resolve().parents[4]
    model = mujoco.MjModel.from_xml_path(
        str(root / "unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml")
    )
    data = mujoco.MjData(model)
    config = yaml.safe_load((root / "pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml").read_text())
    for name, value in json.loads(config["robot"]["initial_qpos_json"]).items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    board, board_geoms, left, right = scene_ids(model)
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_gripper_tool_frame")
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.xmat[board], data.xmat[tool], atol=1e-6)
    for pad in left + right:
        distance = mujoco.mj_geomDistance(model, data, board_geoms[0], pad, .1, None)
        assert abs(distance) < .001  # closing starts near contact, without deep overlap
    for name in json.loads(config["robot"]["initial_qpos_json"]):
        if "gripper" in name:
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint]] = model.jnt_range[joint, 1]
    mujoco.mj_forward(model, data)
    for pad in left + right:
        assert mujoco.mj_geomDistance(model, data, board_geoms[0], pad, .1, None) > .005


def test_s12_pregrasp_keeps_fingertip_patch_inside_board_end_during_relative_yaw():
    root = Path(__file__).resolve().parents[4]
    model = mujoco.MjModel.from_xml_path(
        str(root / "unitree_mujoco/unitree_robots/pr2/scene_grasp_board_long.xml")
    )
    data = mujoco.MjData(model)
    config = yaml.safe_load((root / "pr2_ws/src/pr2_virtual_human/config/transport_comparison.yaml").read_text())
    for name, value in json.loads(config["robot"]["initial_qpos_json"]).items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)
    board, geoms, left, right = scene_ids(model)
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_gripper_tool_frame")
    tool_rotation = data.xmat[tool].reshape(3, 3)
    board_center = tool_rotation.T @ (data.xpos[board] - data.xpos[tool])
    half_length = model.geom_size[geoms[0], 0]
    insertion = half_length - board_center[0]
    vertices = []
    for pad in left + right:
        for signs in product([-1., 1.], repeat=3):
            point = data.geom_xpos[pad] + data.geom_xmat[pad].reshape(3, 3) @ (
                np.asarray(signs) * model.geom_size[pad]
            )
            vertices.append(tool_rotation.T @ (point - data.xpos[tool]))
    # The full-system traces showed relative yaw approaching 30 degrees.
    # Check longitudinal patch coverage through +/-35 degrees, rather than
    # merely requiring contact in the initial, aligned pose.
    worst_margin = min(
        insertion + point[0] * np.cos(angle) + point[1] * np.sin(angle)
        for angle in np.deg2rad([-35., 0., 35.]) for point in vertices
    )
    assert worst_margin > .005
    # This geometric requirement distinguishes the former 15 mm insertion.
    assert worst_margin - .045 < 0.


def small_scene(equality="", free=True):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
    <body name="grasp_board">{'<freejoint/>' if free else ''}
      <geom name="board_geom" type="box" size="1 .05 .02"/></body>
    <body name="tool"><geom name="l_gripper_l_board_contact" size=".1"/>
      <geom name="l_gripper_r_board_contact" size=".1" pos="0 .3 0"/>
    </body></worldbody><equality>{equality}</equality></mujoco>''')
    return model, mujoco.MjData(model)


@pytest.mark.parametrize("kind", ["weld", "connect"])
def test_friction_contract_rejects_hidden_board_attachment(kind):
    anchor = 'anchor="0 0 0"' if kind == "connect" else ""
    model, data = small_scene(f'<{kind} body1="grasp_board" body2="tool" {anchor}/>')
    with pytest.raises(ValueError, match="active board weld/connect"):
        validate_friction_grasp(model, data, *scene_ids(model))


def test_friction_contract_rejects_disabled_pad_collisions():
    model, data = small_scene()
    body, board, left, right = scene_ids(model)
    model.geom_contype[left[0]] = model.geom_conaffinity[left[0]] = 0
    with pytest.raises(ValueError, match="enabled board-pad collisions"):
        validate_friction_grasp(model, data, body, board, left, right)


def test_friction_contract_rejects_fixed_board():
    model, data = small_scene(free=False)
    with pytest.raises(ValueError, match="dynamically free board"):
        validate_friction_grasp(model, data, *scene_ids(model))
