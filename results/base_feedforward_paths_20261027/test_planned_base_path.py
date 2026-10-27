from pathlib import Path
import numpy as np
import yaml

from planned_base_path import PlannedBasePath
from pr2_virtual_human.trajectory_6d import TrajectoryConfig, build_trajectory

HERE = Path(__file__).resolve().parent


def config(task):
    return yaml.safe_load((HERE / f"robot_path_{task}.yaml").read_text())


def test_s22_matches_previous_path_exactly():
    c = config("S22")
    anchor = np.array([-3.,2.,.1])
    new = PlannedBasePath(c, anchor)
    old = build_trajectory(TrajectoryConfig(duration_sec=24., cycles=1.,
        position_amplitude=np.zeros(3), orientation_amplitude=np.zeros(3),
        mode="waypoints", waypoints=np.array(c["waypoints"])), anchor, np.array([1.,0,0,0]))
    for t in np.linspace(0,24,51):
        a,b=new.sample(t),old.sample(t)
        np.testing.assert_allclose(a.position,b.position,atol=1e-12)
        np.testing.assert_allclose(a.linear_velocity,b.linear_velocity,atol=1e-12)


def test_analytic_velocity_matches_position_derivative_for_every_path():
    for task in ["S11","S12","sinusoid"]:
        c=config(task);p=PlannedBasePath(c,np.array([-3.,2.,.1]))
        for t in np.linspace(.2,c["duration_sec"]-.2,19):
            h=1e-5
            numerical=(p.sample(t+h).position-p.sample(t-h).position)/(2*h)
            np.testing.assert_allclose(p.sample(t).linear_velocity,numerical,atol=1e-8)


def test_turn_uses_opposite_end_geometry_and_no_vertical_motion():
    anchor=np.array([-3.,2.,.1]);p=PlannedBasePath(config("S12"),anchor)
    np.testing.assert_allclose(p.sample(0).position,anchor,atol=1e-12)
    np.testing.assert_allclose(p.sample(22).position-anchor,[2.79,-1.3,0],atol=1e-12)
    for t in np.linspace(0,22,31):
        assert abs(p.sample(t).linear_velocity[2]) < 1e-12


def test_sinusoid_plan_has_no_vertical_or_angular_feedforward():
    p=PlannedBasePath(config("sinusoid"),np.zeros(3))
    for t in np.linspace(0,12,31):
        s=p.sample(t)
        assert s.position[2] == s.linear_velocity[2] == 0.
        np.testing.assert_array_equal(s.angular_velocity,np.zeros(3))
