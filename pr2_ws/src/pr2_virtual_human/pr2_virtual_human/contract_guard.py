"""Runtime guard that rejects mixed-control or active load-compensation modes."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
from std_msgs.msg import Bool

from .comparison_ros import CONTRACT_VALID_TOPIC


class ContractGuard(Node):
    SIM_NAMES = (
        "hand_force_enable",
        "hand_force_cancel_moment",
        "left_wrist_wrench_sign",
        "ctc_payload_auto_balance",
        "ctc_payload_force_z",
        "ctc_vertical_hold_force_limit",
        "left_wrist_tare_duration_sec",
    )
    QP_NAMES = (
        "pose_tracking_enable",
        "orientation_tracking_enable",
        "freeze_orientation",
        "fixed_target_mode",
        "input_wrench_topic",
    )

    def __init__(self) -> None:
        super().__init__("transport_comparison_contract_guard")
        self.declare_parameter("sim_node", "pr2_mujoco_sim")
        self.declare_parameter("qp_node", "pr2_qp_whole_body_admittance")
        self.declare_parameter("expected_tare_duration_sec", 2.0)
        self.sim_client = AsyncParameterClient(self, str(self.get_parameter("sim_node").value))
        self.qp_client = AsyncParameterClient(self, str(self.get_parameter("qp_node").value))
        self.publisher = self.create_publisher(Bool, CONTRACT_VALID_TOPIC, 10)
        self.sim_future = None
        self.qp_future = None
        self.validated = False
        self.create_timer(0.01, self._tick)

    def _publish_valid(self) -> None:
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)

    def _tick(self) -> None:
        if self.validated:
            self._publish_valid()
            return
        if not self.sim_client.services_are_ready() or not self.qp_client.services_are_ready():
            return
        if self.sim_future is None:
            self.sim_future = self.sim_client.get_parameters(list(self.SIM_NAMES))
            self.qp_future = self.qp_client.get_parameters(list(self.QP_NAMES))
            return
        if not self.sim_future.done() or not self.qp_future.done():
            return
        sim = {
            name: parameter_value_to_python(value)
            for name, value in zip(self.SIM_NAMES, self.sim_future.result().values)
        }
        qp = {
            name: parameter_value_to_python(value)
            for name, value in zip(self.QP_NAMES, self.qp_future.result().values)
        }
        errors = []
        expected_sim = {
            "hand_force_enable": True,
            "hand_force_cancel_moment": False,
            "left_wrist_wrench_sign": -1.0,
            "ctc_payload_auto_balance": False,
            "ctc_payload_force_z": 0.0,
            "ctc_vertical_hold_force_limit": 0.0,
            "left_wrist_tare_duration_sec": float(
                self.get_parameter("expected_tare_duration_sec").value
            ),
        }
        expected_qp = {
            "pose_tracking_enable": False,
            "orientation_tracking_enable": False,
            "freeze_orientation": False,
            "fixed_target_mode": False,
            "input_wrench_topic": "mujoco/left_wrist_wrench",
        }
        for name, expected in expected_sim.items():
            if sim.get(name) != expected:
                errors.append(f"sim.{name}={sim.get(name)!r}, expected {expected!r}")
        for name, expected in expected_qp.items():
            if qp.get(name) != expected:
                errors.append(f"qp.{name}={qp.get(name)!r}, expected {expected!r}")
        if errors:
            self.get_logger().fatal("comparison contract violation: " + "; ".join(errors))
            raise RuntimeError("comparison contract violation")
        self.validated = True
        self._publish_valid()
        self.get_logger().info("comparison contract validated: pure wrench drive, natural load distribution")


def main() -> None:
    rclpy.init()
    node = ContractGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        rclpy.try_shutdown()
