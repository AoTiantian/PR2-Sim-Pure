from pathlib import Path

import mujoco
import numpy as np
import rclpy

from pr2_virtual_human.human_only_adapter import HumanOnlyAdapter


def test_feedback_observes_the_current_physical_state_after_integration() -> None:
    config = Path(__file__).parents[1] / "config" / "transport_comparison.yaml"
    rclpy.init(args=["--ros-args", "-p", f"config_file:={config}", "-p", "use_viewer:=false"])
    node = HumanOnlyAdapter()
    try:
        mujoco.mj_resetData(node.model, node.data)
        node.data.qvel[:] = [.1, .2, .3, .01, .02, .03]
        mujoco.mj_forward(node.model, node.data)
        mujoco.mj_step(node.model, node.data)
        current = mujoco.MjData(node.model)
        current.qpos[:] = node.data.qpos
        current.qvel[:] = node.data.qvel
        mujoco.mj_forward(node.model, current)
        bid = node.board_id
        offset = current.xmat[bid].reshape(3, 3) @ node.config.hand_offset
        expected_position = current.xpos[bid] + offset
        body_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(node.model, current, mujoco.mjtObj.mjOBJ_BODY, bid, body_velocity, 0)
        expected_velocity = body_velocity[3:] + np.cross(body_velocity[:3], offset)
        # Confirm this setup distinguishes stale derived fields from the
        # physical state, rather than testing the already-refreshed case.
        assert np.linalg.norm(node._measure()[0] - expected_position) > 1e-5
        seen = {}
        evaluate = node.impedance.evaluate

        def observe(target, position, velocity, quaternion, angular_velocity):
            seen.update(position=position.copy(), velocity=velocity.copy(),
                        quaternion=quaternion.copy(), omega=angular_velocity.copy())
            return evaluate(target, position, velocity, quaternion, angular_velocity)

        node.impedance.evaluate = observe
        before = node.data.time
        node.step()
        np.testing.assert_allclose(seen["position"], expected_position, atol=1e-12)
        np.testing.assert_allclose(seen["velocity"], expected_velocity, atol=1e-12)
        np.testing.assert_allclose(seen["quaternion"], current.xquat[bid], atol=1e-12)
        np.testing.assert_allclose(seen["omega"], body_velocity[:3], atol=1e-12)
        np.testing.assert_allclose(node.data.time, before + node.model.opt.timestep, atol=1e-12)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
