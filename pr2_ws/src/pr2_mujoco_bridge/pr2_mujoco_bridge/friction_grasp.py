"""Validate that the carrying scene actually uses contact friction."""

import mujoco


def validate_friction_grasp(model, data, board_id, board_geoms, left_pads, right_pads):
    if board_id < 0 or not board_geoms or not left_pads or not right_pads:
        raise ValueError("friction grasp requires a board and both fingertip pads")
    for eq_id in range(model.neq):
        if (data.eq_active[eq_id]
                and model.eq_objtype[eq_id] == mujoco.mjtObj.mjOBJ_BODY
                and model.eq_type[eq_id] in (mujoco.mjtEq.mjEQ_WELD, mujoco.mjtEq.mjEQ_CONNECT)
                and board_id in (model.eq_obj1id[eq_id], model.eq_obj2id[eq_id])):
            raise ValueError("friction grasp forbids an active board weld/connect")
    joints = range(int(model.body_jntadr[board_id]),
                   int(model.body_jntadr[board_id] + model.body_jntnum[board_id]))
    if not any(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in joints):
        raise ValueError("friction grasp requires a dynamically free board")
    for pads in (left_pads, right_pads):
        for pad in pads:
            if not any(
                (model.geom_contype[pad] & model.geom_conaffinity[board])
                or (model.geom_contype[board] & model.geom_conaffinity[pad])
                for board in board_geoms
            ):
                raise ValueError("friction grasp requires enabled board-pad collisions")
