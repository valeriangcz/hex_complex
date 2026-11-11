"""Deterministic expert/physics state machine independent of ROS."""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from .adapters import ComplexExpertAdapter
from .types import ExpertCommand


class RunPhase(str, Enum):
    INITIALIZING = "INITIALIZING"
    RESET_PENDING = "RESET_PENDING"
    SETTLING = "SETTLING"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class CoordinatorStatus:
    phase: RunPhase
    reset_count: int
    settling_remaining: int
    failure_reason: str
    completion_reason: str


class ExpertCoordinator:
    """Coordinate reset, settling, expert feedback and command application.

    Expert output computed from feedback at tick ``k`` is first applied at tick
    ``k+1``. Reset requests are only consumed by :meth:`tick`, keeping ROS
    service callbacks away from simulator tensors.
    """

    def __init__(
        self,
        backend,
        expert,
        path=None,
        settling_steps=100,
        completion_translation=0.02,
        completion_rotation=0.05,
        failure_path_distance=1.0,
        contact_force_threshold=10.0,
        contact_loss_steps=50,
        adapter=None,
    ):
        self.adapter = adapter if adapter is not None else ComplexExpertAdapter(expert, path)
        if self.adapter.expert is not expert:
            raise ValueError("adapter and coordinator must share the same expert")
        if abs(float(expert.dt) - float(backend.config.control_dt)) > 1e-12:
            raise ValueError(
                "expert dt {} != backend control dt {}".format(
                    expert.dt, backend.config.control_dt
                )
            )
        self.backend = backend
        self.expert = expert
        self.settling_steps = int(settling_steps)
        if self.settling_steps < 0:
            raise ValueError("settling_steps cannot be negative")
        self.completion_translation = float(completion_translation)
        self.completion_rotation = float(completion_rotation)
        self.failure_path_distance = float(failure_path_distance)
        self.contact_force_threshold = float(contact_force_threshold)
        self.contact_loss_steps = int(contact_loss_steps)
        self.phase = RunPhase.INITIALIZING
        self.reset_count = 0
        self.settling_remaining = 0
        self.failure_reason = ""
        self.completion_reason = ""
        self.last_state = None
        self.pending_command = None
        self._initial_command = None
        self._reset_requested = True
        self._reset_reason = "initial startup"
        self._contact_loss_count = 0
        self.events = []
        if self.path and hasattr(self.backend, "set_reference_path"):
            self.backend.set_reference_path(
                np.asarray([pose.t for pose in self.path], dtype=np.float32)
            )

    @property
    def path(self):
        return self.adapter.path

    @path.setter
    def path(self, value):
        self.adapter.path = tuple(pose.copy() for pose in value)

    def set_velocity_command(self, command):
        """Store a command only; safe for ROS callbacks without stepping physics."""
        if self.phase == RunPhase.CLOSED:
            raise RuntimeError("coordinator is closed")
        self.adapter.set_velocity_command(command)

    @property
    def status(self):
        return CoordinatorStatus(
            self.phase,
            self.reset_count,
            self.settling_remaining,
            self.failure_reason,
            self.completion_reason,
        )

    def request_reset(self, reason="manual reset"):
        if self.phase == RunPhase.CLOSED:
            return False
        self._reset_requested = True
        self._reset_reason = str(reason)
        self.phase = RunPhase.RESET_PENDING
        return True

    def inject_failure(self, reason="injected test failure"):
        """Explicit test hook; the reason is recorded as an injected failure."""
        if self.phase == RunPhase.CLOSED:
            return False
        self._fail("injected: {}".format(reason))
        return True

    def tick(self):
        if self.phase == RunPhase.CLOSED:
            raise RuntimeError("coordinator is closed")
        try:
            if self._reset_requested:
                return self._perform_reset()
            if self.phase == RunPhase.SETTLING:
                self.last_state = self.backend.step(self._initial_command)
                self.settling_remaining -= 1
                if self.settling_remaining <= 0:
                    self._finish_settling("final pose reached after settling")
                return self.last_state

            if self.phase == RunPhase.RUNNING:
                self.last_state = self.backend.step(self.pending_command)
                failure = self._failure_reason(self.last_state)
                if failure:
                    self._fail(failure)
                elif self._is_complete(self.last_state):
                    self._complete("final translation, rotation, and gait criteria met")
                else:
                    self.pending_command = self._expert_command(self.last_state)
                return self.last_state

            if self.phase in (RunPhase.COMPLETED, RunPhase.FAILED):
                # Hold the current pose while diagnostics remain available.
                self.last_state = self.backend.step(self.pending_command)
                return self.last_state
            raise RuntimeError("invalid coordinator phase {}".format(self.phase))
        except Exception as exc:
            self._fail("{}: {}".format(type(exc).__name__, exc))
            return self.last_state

    def _perform_reset(self):
        reason = self._reset_reason
        self._reset_requested = False
        self.failure_reason = ""
        self.completion_reason = ""
        self._contact_loss_count = 0
        initial_state, self._initial_command = self.adapter.reset()
        self.last_state = self.backend.reset(initial_state)
        self.pending_command = self._initial_command
        self.reset_count += 1
        self.settling_remaining = self.settling_steps
        self.phase = RunPhase.SETTLING
        self._event("reset {}: {}".format(self.reset_count, reason))
        if self.settling_remaining == 0:
            self._finish_settling("final pose reached at reset")
        return self.last_state

    def _finish_settling(self, completion_reason):
        self.adapter.on_settled(self.last_state)
        if self._is_complete(self.last_state):
            self._complete(completion_reason)
        else:
            self.pending_command = self._expert_command(self.last_state)
            self.phase = RunPhase.RUNNING
            self._event("settling complete")

    def _expert_command(self, state):
        return self.adapter.compute(state)

    def _is_complete(self, state):
        return self.adapter.is_complete(
            state, self.completion_translation, self.completion_rotation
        )

    def _failure_reason(self, state):
        task_failure = self.adapter.failure_reason(state, self.failure_path_distance)
        if task_failure:
            return task_failure
        contacts = np.linalg.norm(state.contact_forces, axis=1) >= self.contact_force_threshold
        self._contact_loss_count = 0 if contacts.any() else self._contact_loss_count + 1
        if self.contact_loss_steps > 0 and self._contact_loss_count >= self.contact_loss_steps:
            return "no foot contact above {:.3f} N for {} control steps".format(
                self.contact_force_threshold, self._contact_loss_count
            )
        return ""

    def _hold_command(self):
        q = self.last_state.q if self.last_state is not None else self._initial_command.q_des
        adhesions = (
            self.pending_command.adhesions
            if self.pending_command is not None else self._initial_command.adhesions
        )
        return ExpertCommand(q.copy(), np.zeros((6, 3)), adhesions.copy())

    def _complete(self, reason):
        self.completion_reason = str(reason)
        self.phase = RunPhase.COMPLETED
        self.pending_command = self._hold_command()
        self._event("completed: {}".format(reason))

    def _fail(self, reason):
        self.failure_reason = str(reason)
        self.phase = RunPhase.FAILED
        if self._initial_command is not None:
            self.pending_command = self._hold_command()
        self._event("failed: {}".format(reason))

    def _event(self, message):
        self.events.append({
            "sim_time": None if self.last_state is None else self.last_state.sim_time,
            "run_elapsed": None if self.last_state is None else self.last_state.run_elapsed,
            "phase": self.phase.value,
            "message": message,
        })

    def close(self):
        if self.phase != RunPhase.CLOSED:
            self.backend.close()
            self.phase = RunPhase.CLOSED
            self._event("closed")

