"""Simulation-time-only state machine shared by both experiment adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ExperimentState(str, Enum):
    WAIT_FOR_STATE = "wait_for_state"
    LATCH_REFERENCE = "latch_reference"
    SETTLE = "settle"
    TRACK = "track"
    HOLD = "hold"
    DONE = "done"


@dataclass(frozen=True)
class ExperimentTiming:
    settle_sec: float
    tracking_sec: float
    hold_sec: float

    def __post_init__(self) -> None:
        if self.settle_sec < 0.0 or self.tracking_sec <= 0.0 or self.hold_sec < 0.0:
            raise ValueError("invalid experiment timing")


class ExperimentStateMachine:
    def __init__(self, timing: ExperimentTiming) -> None:
        self.timing = timing
        self.state = ExperimentState.WAIT_FOR_STATE
        self.state_enter_time: float | None = None
        self.last_time: float | None = None
        self.reference_latch_requested = False

    def update(self, sim_time: float, state_valid: bool) -> ExperimentState:
        now = float(sim_time)
        if self.last_time is not None and now < self.last_time - 1.0e-12:
            raise ValueError("simulation time moved backwards")
        self.last_time = now
        self.reference_latch_requested = False
        if self.state == ExperimentState.WAIT_FOR_STATE and state_valid:
            self.state = ExperimentState.LATCH_REFERENCE
            self.state_enter_time = now
            self.reference_latch_requested = True
            return self.state
        if self.state == ExperimentState.LATCH_REFERENCE:
            self.state = ExperimentState.SETTLE
            self.state_enter_time = now
        elif self.state == ExperimentState.SETTLE and self.elapsed(now) >= self.timing.settle_sec:
            self.state = ExperimentState.TRACK
            self.state_enter_time = now
        elif self.state == ExperimentState.TRACK and self.elapsed(now) >= self.timing.tracking_sec:
            self.state = ExperimentState.HOLD
            self.state_enter_time = now
        elif self.state == ExperimentState.HOLD and self.elapsed(now) >= self.timing.hold_sec:
            self.state = ExperimentState.DONE
            self.state_enter_time = now
        return self.state

    def elapsed(self, sim_time: float) -> float:
        if self.state_enter_time is None:
            return 0.0
        return max(0.0, float(sim_time) - self.state_enter_time)

    def trajectory_time(self, sim_time: float) -> float:
        if self.state in (ExperimentState.WAIT_FOR_STATE, ExperimentState.LATCH_REFERENCE, ExperimentState.SETTLE):
            return 0.0
        if self.state == ExperimentState.TRACK:
            return min(self.elapsed(sim_time), self.timing.tracking_sec)
        return self.timing.tracking_sec

