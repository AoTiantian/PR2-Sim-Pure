import mujoco
import numpy as np
import pytest

from pr2_mujoco_bridge.weld_sensor import (
    physical_weld_torque_world,
    weld_sensor_torque_correction,
)


def model_at_heading(yaw, reverse=False):
    q = f"{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}"
    bodies = 'body1="cut" body2="load"' if reverse else 'body1="load" body2="cut"'
    xml = f'''<mujoco><option gravity="0 0 0" timestep=".002"/>
    <worldbody><body><body name="cut" pos="0 0 1" quat="{q}">
      <geom type="sphere" size=".1" mass="1" contype="0" conaffinity="0"/>
      <site name="sensor" zaxis="1 0 0"/>
    </body></body><body name="load" pos=".5 0 1">
      <freejoint/><geom type="sphere" size=".1" mass="1" contype="0" conaffinity="0"/>
    </body></worldbody><equality><weld {bodies}/></equality>
    <sensor><force site="sensor"/><torque site="sensor"/></sensor></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    return model, mujoco.MjData(model)


@pytest.mark.parametrize("yaw", [0., np.pi/4, np.pi/2, np.pi])
def test_weld_torque_matches_constraint_virtual_work(yaw):
    model, data = model_at_heading(yaw)
    mujoco.mj_forward(model, data)
    # Only rotational rows are loaded, so no force lever arm contributes.
    multipliers = np.array([.3, -.5, .8])
    jacobian = data.efc_J.reshape(data.nefc, model.nv)
    generalized = jacobian[3:6].T @ multipliers
    load = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "load")
    expected = data.xmat[load].reshape(3, 3) @ generalized[3:]
    np.testing.assert_allclose(physical_weld_torque_world(model, data, 0, multipliers), expected, atol=1e-12)


@pytest.mark.skipif(mujoco.__version__ != "3.10.0", reason="Compatibility path is scoped to the verified affected release")
@pytest.mark.parametrize("yaw", [0., np.pi/4, np.pi/2, np.pi])
@pytest.mark.parametrize("reverse", [False, True])
def test_sensor_recovers_known_external_load_without_removing_its_moment(yaw, reverse):
    model, data = model_at_heading(yaw, reverse)
    load = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "load")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sensor")
    force, torque = np.array([1.2, -.3, .4]), np.array([.3, -.5, .8])
    data.xfrc_applied[load] = np.r_[force, torque]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    before = [x.copy() for x in [data.qpos, data.qvel, data.qacc, data.efc_force, data.qfrc_constraint]]
    rotation = data.site_xmat[site].reshape(3, 3)
    native = -rotation @ data.sensordata[3:]
    corrected = native + weld_sensor_torque_correction(model, data, 0, site, -1)
    expected = torque + np.cross(data.xipos[load] - data.site_xpos[site], force)
    np.testing.assert_allclose(corrected, expected, atol=1e-3)
    np.testing.assert_allclose(-rotation @ data.sensordata[:3], force, atol=1e-3)
    assert np.linalg.norm(native - expected) > .1
    for original, actual in zip(before, [data.qpos, data.qvel, data.qacc, data.efc_force, data.qfrc_constraint]):
        np.testing.assert_array_equal(original, actual)


def test_other_versions_do_not_receive_a_second_correction(monkeypatch):
    model, data = model_at_heading(0.)
    mujoco.mj_forward(model, data)
    monkeypatch.setattr(mujoco, "__version__", "3.11.0")
    np.testing.assert_array_equal(weld_sensor_torque_correction(model, data, 0, 0, -1), np.zeros(3))
