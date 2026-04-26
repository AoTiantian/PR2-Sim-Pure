#!/usr/bin/env python3
"""WBC coordinator (practical baseline implementation).

功能：
- 汇总 wbc/reference/cmd_vel 与 wbc/reference/joint_command
- 支持 null-space 姿态保持（从 wbc/q_nominal JSON 读取）
- 用 state/joint_states 计算次级姿态保持力矩并叠加到关节 effort
- 带输入超时保护，避免旧指令持续输出
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


def _clamp(x: float, low: float, high: float) -> float:
    return max(low, min(high, x))


def _is_nan(x: float) -> bool:
    return x != x


class Pr2WbcCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("pr2_wbc_coordinator")

        # #region agent log
        self._dbg_log_path = "/workspace/.cursor/debug-33df0d.log"
        self._dbg_last_mono = 0.0

        def _dbg_write(hypothesis_id: str, message: str, data: dict) -> None:
            try:
                os.makedirs(os.path.dirname(self._dbg_log_path) or ".", exist_ok=True)
                payload = {
                    "sessionId": "33df0d",
                    "runId": os.environ.get("DEBUG_RUN_ID", "vel_mismatch"),
                    "hypothesisId": hypothesis_id,
                    "location": "pr2_wbc_coordinator.py",
                    "message": message,
                    "data": data,
                    "timestamp": int(time.time() * 1000),
                }
                with open(self._dbg_log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(payload, ensure_ascii=False) + "\n")
            except Exception:
                pass

        self._dbg_write = _dbg_write
        self._dbg_write("H0_DebugInit", "wbc init", {"pid": int(os.getpid())})
        # #endregion agent log

        self.declare_parameter("output_cmd_vel", "cmd_vel")
        self.declare_parameter("output_joint_command", "joint_commands")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("state_joint_topic", "state/joint_states")
        self.declare_parameter("nullspace_topic", "wbc/q_nominal")
        self.declare_parameter("cmd_timeout_sec", 0.3)
        self.declare_parameter("nullspace_enable", True)
        self.declare_parameter("default_nullspace_kp", 15.0)
        self.declare_parameter("default_nullspace_kd", 2.0)
        self.declare_parameter("default_nullspace_max_effort", 10.0)

        self._out_twist = self.get_parameter("output_cmd_vel").value
        self._out_joint = self.get_parameter("output_joint_command").value
        hz = float(self.get_parameter("publish_rate_hz").value)
        self._state_joint_topic = str(self.get_parameter("state_joint_topic").value)
        self._null_topic = str(self.get_parameter("nullspace_topic").value)
        self._timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self._null_en = bool(self.get_parameter("nullspace_enable").value)
        self._kp_def = float(self.get_parameter("default_nullspace_kp").value)
        self._kd_def = float(self.get_parameter("default_nullspace_kd").value)
        self._eff_lim_def = float(self.get_parameter("default_nullspace_max_effort").value)

        self._lock = threading.Lock()
        self._twist: Optional[Twist] = None
        self._joint_merge: Dict[str, Dict[str, float]] = {}
        # name -> {"position": v or nan, "velocity": v or nan, "effort": v or nan}
        self._gripper_pos: Optional[float] = None
        self._latest_state: Dict[str, Tuple[float, float]] = {}
        self._t_twist = self.get_clock().now()
        self._t_joint = self.get_clock().now()
        self._t_gripper = self.get_clock().now()
        self._q_nominal: Dict[str, float] = {}
        self._null_kp = self._kp_def
        self._null_kd = self._kd_def
        self._null_eff_lim = self._eff_lim_def

        self._pub_cmd = self.create_publisher(Twist, self._out_twist, 10)
        self._pub_joint = self.create_publisher(JointState, self._out_joint, 10)

        self.create_subscription(
            Twist, "wbc/reference/cmd_vel", self._cb_twist, 10
        )
        self.create_subscription(
            JointState,
            "wbc/reference/joint_command",
            self._cb_joint,
            10,
        )
        self.create_subscription(
            Float64,
            "wbc/reference/gripper_position",
            self._cb_gripper,
            10,
        )
        self.create_subscription(
            JointState,
            self._state_joint_topic,
            self._cb_state_joint,
            20,
        )
        self.create_subscription(
            String,
            self._null_topic,
            self._cb_q_nominal,
            10,
        )

        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self.get_logger().info(
            "WBC coordinator: refs + nullspace -> "
            f"{self._out_twist}, {self._out_joint}, null={self._null_en}"
        )

    def _cb_twist(self, msg: Twist) -> None:
        with self._lock:
            self._twist = msg
            self._t_twist = self.get_clock().now()

    def _cb_joint(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if name not in self._joint_merge:
                    self._joint_merge[name] = {
                        "position": float("nan"),
                        "velocity": float("nan"),
                        "effort": float("nan"),
                    }
                d = self._joint_merge[name]
                if i < len(msg.position):
                    d["position"] = float(msg.position[i])
                if i < len(msg.velocity):
                    d["velocity"] = float(msg.velocity[i])
                if i < len(msg.effort):
                    d["effort"] = float(msg.effort[i])
            self._t_joint = self.get_clock().now()

    def _cb_gripper(self, msg: Float64) -> None:
        with self._lock:
            self._gripper_pos = float(msg.data)
            self._t_gripper = self.get_clock().now()

    def _cb_state_joint(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                q = float(msg.position[i]) if i < len(msg.position) else 0.0
                dq = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
                self._latest_state[name] = (q, dq)

    def _cb_q_nominal(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
            joints = list(obj.get("joints", []))
            q_list = list(obj.get("q_nominal", []))
            if len(joints) == 0 or len(joints) != len(q_list):
                return
            qn = {str(joints[i]): float(q_list[i]) for i in range(len(joints))}
            with self._lock:
                self._q_nominal = qn
                self._null_kp = float(obj.get("kp", self._kp_def))
                self._null_kd = float(obj.get("kd", self._kd_def))
                self._null_eff_lim = float(obj.get("max_effort", self._eff_lim_def))
        except Exception as exc:
            self.get_logger().warn(f"invalid q_nominal JSON ignored: {exc}")

    def _is_fresh(self, t_msg) -> bool:
        age = (self.get_clock().now() - t_msg).nanoseconds * 1e-9
        return age <= self._timeout

    def _tick(self) -> None:
        with self._lock:
            t = self._twist
            merge = {k: dict(v) for k, v in self._joint_merge.items()}
            gp = self._gripper_pos
            state = dict(self._latest_state)
            q_nom = dict(self._q_nominal)
            kp = self._null_kp
            kd = self._null_kd
            eff_lim = self._null_eff_lim
            fresh_twist = self._is_fresh(self._t_twist)
            fresh_joint = self._is_fresh(self._t_joint)
            fresh_grip = self._is_fresh(self._t_gripper)

        if t is not None and fresh_twist:
            self._pub_cmd.publish(t)
        else:
            self._pub_cmd.publish(Twist())

        if not fresh_joint:
            merge = {}
        if not fresh_grip:
            gp = None

        names = sorted(merge.keys())
        if gp is not None and "l_gripper_l_finger_joint" not in names:
            names.append("l_gripper_l_finger_joint")
        if self._null_en:
            for n in q_nom.keys():
                if n not in names:
                    names.append(n)

        if not names:
            return

        pos: List[float] = []
        vel: List[float] = []
        eff: List[float] = []
        for n in names:
            if n == "l_gripper_l_finger_joint" and gp is not None:
                if n in merge:
                    d = merge[n]
                    pos.append(d.get("position", gp))
                    vel.append(d.get("velocity", float("nan")))
                    eff.append(d.get("effort", float("nan")))
                else:
                    pos.append(gp)
                    vel.append(float("nan"))
                    eff.append(float("nan"))
                continue
            d = merge.get(
                n,
                {
                    "position": float("nan"),
                    "velocity": float("nan"),
                    "effort": float("nan"),
                },
            )
            p = d.get("position", float("nan"))
            v = d.get("velocity", float("nan"))
            e = d.get("effort", float("nan"))

            # Null-space posture hold torque (secondary objective).
            if self._null_en and n in q_nom and n in state:
                q, dq = state[n]
                tau_null = kp * (q_nom[n] - q) - kd * dq
                tau_null = _clamp(tau_null, -eff_lim, eff_lim)
                if _is_nan(e):
                    e = tau_null
                else:
                    e = e + tau_null

            pos.append(p)
            vel.append(v)
            eff.append(e)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = pos
        out.velocity = vel
        out.effort = eff
        self._pub_joint.publish(out)

        # #region agent log
        now_m = time.monotonic()
        if now_m - self._dbg_last_mono > 1.0:
            self._dbg_last_mono = now_m
            # Summarize commanded velocities (if any) for left arm.
            arm_names = [n for n in names if n.startswith("l_") and n.endswith("_joint")]
            vmap = {names[i]: vel[i] if i < len(vel) else float("nan") for i in range(len(names))}
            arm_vel = [float(vmap.get(n, float("nan"))) for n in arm_names]
            finite = [abs(v) for v in arm_vel if v == v]
            self._dbg_write(
                "H2_WBCNotPassingVelocity",
                "wbc publish summary",
                {
                    "fresh_joint_ref": bool(fresh_joint),
                    "n_total": int(len(names)),
                    "n_arm": int(len(arm_names)),
                    "arm_peak_abs_vel": float(max(finite) if finite else 0.0),
                    "nullspace_enable": bool(self._null_en),
                },
            )
        # #endregion agent log


def main() -> None:
    rclpy.init()
    node = Pr2WbcCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Launch global-shutdown can race with spin and trigger take_message conversion errors.
        if rclpy.ok():
            raise exc
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
