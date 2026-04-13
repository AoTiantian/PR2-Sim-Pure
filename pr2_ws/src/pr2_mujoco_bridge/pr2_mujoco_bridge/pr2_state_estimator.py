#!/usr/bin/env python3
"""State estimator for WBC stack.

- Joint states: velocity补全（由位置差分）+ 低通滤波
- Odom: twist低通滤波，并输出估计底座加速度
"""

from __future__ import annotations

from typing import Dict

import rclpy
from geometry_msgs.msg import Accel, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _clamp_alpha(alpha: float) -> float:
    return max(0.0, min(1.0, alpha))


class Pr2StateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_state_estimator")

        self.declare_parameter("joint_state_in", "joint_states")
        self.declare_parameter("odom_in", "odom")
        self.declare_parameter("joint_state_out", "state/joint_states")
        self.declare_parameter("odom_out", "state/odom")
        self.declare_parameter("base_twist_out", "state/base_twist")
        self.declare_parameter("base_accel_out", "state/base_acceleration")
        self.declare_parameter("joint_vel_lpf_alpha", 0.35)
        self.declare_parameter("odom_lpf_alpha", 0.3)

        jin = str(self.get_parameter("joint_state_in").value)
        oin = str(self.get_parameter("odom_in").value)
        jout = str(self.get_parameter("joint_state_out").value)
        oout = str(self.get_parameter("odom_out").value)
        twout = str(self.get_parameter("base_twist_out").value)
        aout = str(self.get_parameter("base_accel_out").value)

        self._a_joint = _clamp_alpha(float(self.get_parameter("joint_vel_lpf_alpha").value))
        self._a_odom = _clamp_alpha(float(self.get_parameter("odom_lpf_alpha").value))

        self._pub_js = self.create_publisher(JointState, jout, 10)
        self._pub_odom = self.create_publisher(Odometry, oout, 10)
        self._pub_tw = self.create_publisher(Twist, twout, 10)
        self._pub_acc = self.create_publisher(Accel, aout, 10)

        self.create_subscription(JointState, jin, self._on_js, 20)
        self.create_subscription(Odometry, oin, self._on_odom, 20)

        self._prev_joint_pos: Dict[str, float] = {}
        self._filt_joint_vel: Dict[str, float] = {}
        self._prev_js_t: float | None = None

        self._prev_tw = Twist()
        self._filt_tw = Twist()
        self._prev_odom_t: float | None = None

        self.get_logger().info(
            "state_estimator: "
            f"{jin}->{jout}, {oin}->{oout}, twist->{twout}, accel->{aout}"
        )

    @staticmethod
    def _to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_js(self, msg: JointState) -> None:
        now_t = self._to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else None
        dt = None if now_t is None or self._prev_js_t is None else max(1e-6, now_t - self._prev_js_t)

        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.effort = list(msg.effort)
        out.velocity = [0.0] * len(msg.name)

        for i, name in enumerate(msg.name):
            vel_meas = None
            if i < len(msg.velocity):
                vel_meas = float(msg.velocity[i])
            elif dt is not None and i < len(msg.position):
                prev = self._prev_joint_pos.get(name, float(msg.position[i]))
                vel_meas = (float(msg.position[i]) - prev) / dt

            if vel_meas is None:
                vel_meas = self._filt_joint_vel.get(name, 0.0)

            prev_f = self._filt_joint_vel.get(name, vel_meas)
            filt = self._a_joint * vel_meas + (1.0 - self._a_joint) * prev_f
            self._filt_joint_vel[name] = filt
            out.velocity[i] = filt

            if i < len(msg.position):
                self._prev_joint_pos[name] = float(msg.position[i])

        if now_t is not None:
            self._prev_js_t = now_t

        self._pub_js.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        now_t = self._to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else None
        dt = None if now_t is None or self._prev_odom_t is None else max(1e-6, now_t - self._prev_odom_t)

        tw_in = msg.twist.twist
        tw_out = Twist()

        tw_out.linear.x = self._a_odom * float(tw_in.linear.x) + (1.0 - self._a_odom) * float(self._filt_tw.linear.x)
        tw_out.linear.y = self._a_odom * float(tw_in.linear.y) + (1.0 - self._a_odom) * float(self._filt_tw.linear.y)
        tw_out.linear.z = self._a_odom * float(tw_in.linear.z) + (1.0 - self._a_odom) * float(self._filt_tw.linear.z)
        tw_out.angular.x = self._a_odom * float(tw_in.angular.x) + (1.0 - self._a_odom) * float(self._filt_tw.angular.x)
        tw_out.angular.y = self._a_odom * float(tw_in.angular.y) + (1.0 - self._a_odom) * float(self._filt_tw.angular.y)
        tw_out.angular.z = self._a_odom * float(tw_in.angular.z) + (1.0 - self._a_odom) * float(self._filt_tw.angular.z)

        acc = Accel()
        if dt is not None:
            acc.linear.x = (tw_out.linear.x - self._prev_tw.linear.x) / dt
            acc.linear.y = (tw_out.linear.y - self._prev_tw.linear.y) / dt
            acc.linear.z = (tw_out.linear.z - self._prev_tw.linear.z) / dt
            acc.angular.x = (tw_out.angular.x - self._prev_tw.angular.x) / dt
            acc.angular.y = (tw_out.angular.y - self._prev_tw.angular.y) / dt
            acc.angular.z = (tw_out.angular.z - self._prev_tw.angular.z) / dt

        out_odom = Odometry()
        out_odom.header = msg.header
        out_odom.child_frame_id = msg.child_frame_id
        out_odom.pose = msg.pose
        out_odom.twist.twist = tw_out
        out_odom.twist.covariance = msg.twist.covariance

        self._pub_odom.publish(out_odom)
        self._pub_tw.publish(tw_out)
        self._pub_acc.publish(acc)

        self._prev_tw = tw_out
        self._filt_tw = tw_out
        if now_t is not None:
            self._prev_odom_t = now_t


def main() -> None:
    rclpy.init()
    node = Pr2StateEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Launch global-shutdown can race with spin and trigger transient conversion errors.
        if rclpy.ok():
            raise exc
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
