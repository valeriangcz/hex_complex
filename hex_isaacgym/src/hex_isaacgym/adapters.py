"""Expert-specific lifecycle and feedback binding, independent of ROS/Gym."""

from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3

from .types import ExpertCommand, InitialState


class ComplexExpertAdapter:
    expert_type = "complex"

    def __init__(self, expert, path):
        if not path:
            raise ValueError("path must be nonempty")
        self.expert = expert
        self.path = tuple(pose.copy() for pose in path)

    def reset(self):
        self.expert.Reset(keep_path=False)
        self.expert.LoadSE3(list(self.path))
        command = ExpertCommand(*self.expert.SetInitBySE3())
        self.initial_state = InitialState(self.path[0].A, command.q_des)
        return self.initial_state, command

    def on_settled(self, state):
        pass

    def compute(self, state):
        return ExpertCommand(*self.expert.RequestSingleStep(
            SE3(state.W_T_R, check=False), state.q.copy(),
            state.q_torque.copy(), state.adhesion_force.copy(),
        ))

    def is_complete(self, state, translation, rotation):
        delta = self.path[-1].inv() * SE3(state.W_T_R, check=False)
        path_reached = len(self.path) == 1 or self.expert._GetCloestIndex(
            SE3(state.W_T_R, check=False)
        ) == len(self.path) - 1
        no_swing = not self.expert.B_e_traj or np.all(
            self.expert.q_traj_index >= np.maximum(self.expert.B_e_traj_len - 1, 0)
        )
        return (path_reached and no_swing and np.linalg.norm(delta.t) <= translation
                and Rotation.from_matrix(delta.R).magnitude() <= rotation)

    def failure_reason(self, state, path_distance):
        positions = np.asarray([pose.t for pose in self.path])
        distance = np.min(np.linalg.norm(positions - state.W_T_R[:3, 3], axis=1))
        if distance > path_distance:
            return "base is {:.3f} m from the reference path".format(distance)
        return ""

    def diagnostics(self):
        return {"expert_type": self.expert_type,
                "follow_path_index": self.expert.follow_path_index}


class ClimbExpertAdapter:
    expert_type = "climb"
    path = ()

    def __init__(self, expert, initial_state, command=(0.02, 0.0, 0.0)):
        self.expert = expert
        self.initial_state = InitialState(initial_state.W_T_R, initial_state.q, initial_state.q_dot)
        self._lock = Lock()
        self._initial_velocity = self._velocity(command)
        self._command = self._initial_velocity.copy()
        self.switch_count = 0

    @staticmethod
    def _velocity(command):
        command = np.asarray(command, dtype=np.float64)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("command must be a finite array with shape (3,)")
        return command.copy()

    def set_velocity_command(self, command):
        command = self._velocity(command)
        with self._lock:
            self._command = command

    @property
    def command(self):
        with self._lock:
            return self._command.copy()

    def reset(self):
        self.set_velocity_command(self._initial_velocity)
        self.switch_count = 0
        command = ExpertCommand(*self.expert.SetInit(self.initial_state.q))
        return self.initial_state, command

    def on_settled(self, state):
        self.expert.SetInit(state.q)

    def compute(self, state):
        group = self.expert.stance_group_index
        result = self.expert.RequestSingleStep(
            SE3(state.W_T_R, check=False), state.q.copy(), state.q_torque.copy(),
            state.adhesion_force.copy(), self.command,
        )
        self.switch_count += int(group != self.expert.stance_group_index)
        return ExpertCommand(*result)

    def is_complete(self, state, translation, rotation):
        return False

    def failure_reason(self, state, path_distance):
        return ""

    def diagnostics(self):
        return {
            "expert_type": self.expert_type, "command": self.command.tolist(),
            "gaits": self.expert.gaits.tolist(),
            "adhesions": self.expert.adhesions.tolist(),
            "estimated_contact_force": self.expert.estimated_contact_force.tolist(),
            "gait_switch_count": self.switch_count,
        }
