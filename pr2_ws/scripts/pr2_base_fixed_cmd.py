#!/usr/bin/env python3
"""
Minimal ROS 2 motion smoke test for PR2 base.

Publish a fixed Twist command to /pr2/mecanum_controller/cmd_vel for a fixed
duration, then publish zero command to stop.
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def build_twist(vx: float, vy: float, wz: float) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = wz
    return msg


def wait_for_subscriber(node: Node, publisher, timeout_sec: float) -> bool:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    return publisher.get_subscription_count() > 0


def publish_for_duration(
    node: Node,
    publisher,
    msg: Twist,
    duration_sec: float,
    rate_hz: float,
) -> None:
    period = 1.0 / rate_hz
    end = time.monotonic() + duration_sec
    while time.monotonic() < end and rclpy.ok():
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish fixed base velocity command for PR2 smoke test."
    )
    parser.add_argument(
        "--topic",
        default="/pr2/mecanum_controller/cmd_vel",
        help="Twist topic consumed by pr2_omni_controller.",
    )
    parser.add_argument("--vx", type=float, default=0.20, help="Linear x (m/s).")
    parser.add_argument("--vy", type=float, default=0.00, help="Linear y (m/s).")
    parser.add_argument("--wz", type=float, default=0.00, help="Angular z (rad/s).")
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Duration of movement command (seconds).",
    )
    parser.add_argument(
        "--stop-duration",
        type=float,
        default=1.0,
        help="Duration of zero-speed stop command after movement (seconds).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz.",
    )
    parser.add_argument(
        "--wait-subscriber",
        type=float,
        default=3.0,
        help="Wait time for cmd_vel subscriber before publishing (seconds).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Node("pr2_base_fixed_cmd")
    publisher = node.create_publisher(Twist, args.topic, 10)

    node.get_logger().info(
        f"Waiting up to {args.wait_subscriber:.1f}s for subscriber on {args.topic}"
    )
    if not wait_for_subscriber(node, publisher, args.wait_subscriber):
        node.get_logger().warning(
            "No subscriber detected. Command will still be published; "
            "please confirm pr2_omni_controller is active."
        )

    move_cmd = build_twist(args.vx, args.vy, args.wz)
    stop_cmd = build_twist(0.0, 0.0, 0.0)

    try:
        node.get_logger().info(
            f"Publishing fixed command for {args.duration:.2f}s: "
            f"vx={args.vx:.3f}, vy={args.vy:.3f}, wz={args.wz:.3f}"
        )
        publish_for_duration(node, publisher, move_cmd, args.duration, args.rate)

        node.get_logger().info(
            f"Publishing stop command for {args.stop_duration:.2f}s"
        )
        publish_for_duration(
            node, publisher, stop_cmd, args.stop_duration, max(args.rate, 10.0)
        )
    finally:
        # Final safeguard: send zero command a few extra times before exit.
        for _ in range(5):
            publisher.publish(stop_cmd)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
