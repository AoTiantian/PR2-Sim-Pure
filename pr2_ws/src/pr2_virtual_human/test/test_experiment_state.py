import pytest

from pr2_virtual_human.experiment_state import (
    ExperimentState,
    ExperimentStateMachine,
    ExperimentTiming,
)


def test_state_sequence_uses_simulation_time() -> None:
    machine = ExperimentStateMachine(ExperimentTiming(2.0, 12.0, 1.0))
    assert machine.update(0.0, False) == ExperimentState.WAIT_FOR_STATE
    assert machine.update(0.1, True) == ExperimentState.LATCH_REFERENCE
    assert machine.reference_latch_requested
    assert machine.update(0.2, True) == ExperimentState.SETTLE
    assert machine.update(2.19, True) == ExperimentState.SETTLE
    assert machine.update(2.2, True) == ExperimentState.TRACK
    assert machine.trajectory_time(5.2) == pytest.approx(3.0)
    assert machine.update(14.2, True) == ExperimentState.HOLD
    assert machine.trajectory_time(14.2) == pytest.approx(12.0)
    assert machine.update(15.2, True) == ExperimentState.DONE


def test_duplicate_time_is_allowed_but_backward_time_is_rejected() -> None:
    machine = ExperimentStateMachine(ExperimentTiming(0.0, 1.0, 0.0))
    machine.update(1.0, True)
    machine.update(1.0, True)
    with pytest.raises(ValueError, match="backwards"):
        machine.update(0.9, True)

