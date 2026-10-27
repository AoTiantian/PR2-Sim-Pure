"""Nontrivial properties of the experimental hierarchy, independent of ROS."""
import numpy as np

from angular_priority import allocate


def jacobian():
    # Arm has full Cartesian authority; base translation and yaw also move EE.
    J = np.zeros((6,10))
    J[:,:6] = np.eye(6)
    J[0,7],J[1,8] = 1.,1.
    J[0,9],J[1,9],J[5,9] = -.6,.4,1.
    return J


def test_planned_translation_moves_endpoint_without_force():
    J = jacobian()
    u,d = allocate(J,np.zeros(6),np.array([.05,0,0]),-np.ones(10),np.ones(10))
    assert (J @ u)[0] > .045
    np.testing.assert_allclose((J @ u)[3:],0.,atol=1e-6)


def test_base_yaw_cannot_replace_angular_admittance():
    J = jacobian()
    adm = np.array([0.,0,0,.02,-.01,.03])
    for ref in [np.array([0.,0,.1]),np.array([.2,-.2,-.3])]:
        u,d = allocate(J,adm,ref,-np.ones(10),np.ones(10))
        np.testing.assert_allclose((J @ u)[3:],adm[3:],atol=1e-6)
        assert not d["primary_infeasible"]


def test_unachievable_angular_task_reports_residual_and_respects_limits():
    J = jacobian()
    adm = np.array([0.,0,0,.5,-.5,.5])
    bounds = np.full(10,.01)
    u,d = allocate(J,adm,np.array([1.,1.,1.]),-bounds,bounds)
    assert d["primary_infeasible"]
    assert np.max(abs(u)) <= .010001
    np.testing.assert_allclose((J @ u)[3:],[.01,-.01,.02],atol=2e-6)
