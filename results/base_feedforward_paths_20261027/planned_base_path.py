"""Independent odom-relative base plans; no ROS or human feedback dependency."""
from dataclasses import dataclass
import numpy as np

from pr2_virtual_human.trajectory_6d import TrajectoryConfig, build_trajectory
from pr2_virtual_human.spatial import quaternion_to_matrix


@dataclass(frozen=True)
class BasePathSample:
    position: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


class PlannedBasePath:
    def __init__(self, config, base_anchor):
        mode = str(config.get("mode", "waypoints"))
        points = np.asarray(config["waypoints"], dtype=float).copy() if mode == "waypoints" else None
        if points is not None:
            points[:,2] = np.deg2rad(points[:,2])
        durations = config.get("segment_durations_sec")
        c = TrajectoryConfig(
            duration_sec=float(config["duration_sec"]), cycles=float(config.get("cycles", 1.)),
            mode=mode, waypoints=points,
            segment_durations_sec=None if durations is None else np.asarray(durations),
            position_amplitude=np.asarray(config.get("position_amplitude_m", [0,0,0]), dtype=float),
            orientation_amplitude=np.zeros(3))
        self.path = build_trajectory(c, np.asarray(base_anchor, dtype=float), np.array([1.,0,0,0]))
        # Known initial geometry from the planned reference point to the base.
        # For turns: p_base = base_anchor + delta_p_ref + (R_ref-I) r_ref_base.
        self.offset = np.asarray(config.get("reference_to_base_offset_world_m", [0,0,0]), dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("reference-to-base offset must be a finite three-vector")
        if abs(self.offset[2]) > 1e-12:
            raise ValueError("only planar reference-to-base geometry is supported")

    def sample(self, elapsed):
        s = self.path.sample(elapsed)
        R = quaternion_to_matrix(s.quaternion)
        r = R @ self.offset
        return BasePathSample(position=s.position+r-self.offset,
                              linear_velocity=s.linear_velocity+np.cross(s.angular_velocity, r),
                              angular_velocity=s.angular_velocity.copy())
