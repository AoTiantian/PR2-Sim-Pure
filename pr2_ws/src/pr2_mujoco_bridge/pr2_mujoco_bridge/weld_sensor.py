"""MuJoCo 3.10.0 weld torque reconstruction compatibility.

The dynamics solver uses the correct weld Jacobian. In this release the
post-constraint RNE sensor path instead treats the rotational multipliers as
world torques. Apply only the missing physical torque conversion at a sensor
cut; do not change dynamics or remove any real endpoint moment.

Upstream fix: google-deepmind/mujoco commit ac329bd.
"""
import mujoco
import numpy as np


def physical_weld_torque_world(model, data, eq_id, multipliers):
    """Adjoint of the weld rotational Jacobian, as in the upstream fix."""
    first, second = int(model.eq_obj1id[eq_id]), int(model.eq_obj2id[eq_id])
    q0, q0_conjugate, product = np.empty(4), np.empty(4), np.empty(4)
    mujoco.mju_mulQuat(q0, data.xquat[first], model.eq_data[eq_id, 6:10])
    mujoco.mju_negQuat(q0_conjugate, q0)
    mujoco.mju_mulQuatAxis(product, data.xquat[second], multipliers)
    mujoco.mju_mulQuat(q0, product, q0_conjugate)
    return 0.5 * float(model.eq_data[eq_id, 10]) * q0[1:]


def weld_sensor_torque_correction(model, data, eq_id, sensor_site_id, published_sign):
    """World torque correction before sensor tare; no human/target input."""
    zero = np.zeros(3)
    if mujoco.__version__ != "3.10.0" or eq_id < 0 or sensor_site_id < 0:
        return zero
    if int(model.eq_type[eq_id]) != int(mujoco.mjtEq.mjEQ_WELD):
        return zero
    if int(model.eq_objtype[eq_id]) != int(mujoco.mjtObj.mjOBJ_BODY):
        return zero
    rows = np.flatnonzero(
        (data.efc_type == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
        & (data.efc_id == eq_id)
    )
    if len(rows) != 6:
        return zero
    cut_body = int(model.site_bodyid[sensor_site_id])

    def inside(body):
        while body > 0:
            if body == cut_body:
                return 1
            body = int(model.body_parentid[body])
        return 0

    side = inside(int(model.eq_obj1id[eq_id])) - inside(int(model.eq_obj2id[eq_id]))
    if side == 0:
        return zero
    raw_torque = data.efc_force[rows[3:]]
    physical_torque = physical_weld_torque_world(model, data, eq_id, raw_torque)
    # Sensor internal force changes oppositely to external subtree force.
    return -float(published_sign) * side * (physical_torque - raw_torque)
