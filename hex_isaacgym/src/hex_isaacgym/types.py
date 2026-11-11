"""Lightweight public data contracts for the simulator backend."""

from dataclasses import dataclass

import numpy as np


def _finite_array(value, shape, name, dtype=np.float64):
    array = np.asarray(value, dtype=dtype)
    if array.shape != tuple(shape) or not np.isfinite(array).all():
        raise ValueError(
            "{} must be a finite array with shape {}, got {}".format(
                name, tuple(shape), array.shape
            )
        )
    return array.copy()


def _transform_matrix(value, name="W_T_R"):
    if hasattr(value, "A"):
        value = value.A
    matrix = _finite_array(value, (4, 4), name)
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ValueError("{} has an invalid homogeneous last row".format(name))
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError("{} rotation is not orthonormal".format(name))
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7):
        raise ValueError("{} rotation determinant is not +1".format(name))
    return matrix


@dataclass(frozen=True)
class ExpertCommand:
    """One expert control-period command in expert leg order."""

    q_des: np.ndarray
    tau_ff: np.ndarray
    adhesions: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "q_des", _finite_array(self.q_des, (6, 4), "q_des"))
        object.__setattr__(self, "tau_ff", _finite_array(self.tau_ff, (6, 3), "tau_ff"))
        adhesions = _finite_array(self.adhesions, (6,), "adhesions")
        object.__setattr__(self, "adhesions", adhesions.astype(bool))


@dataclass(frozen=True)
class InitialState:
    """Root pose and commanded-joint state used by :meth:`reset`."""

    W_T_R: np.ndarray
    q: np.ndarray
    q_dot: np.ndarray = None

    def __post_init__(self):
        object.__setattr__(self, "W_T_R", _transform_matrix(self.W_T_R))
        object.__setattr__(self, "q", _finite_array(self.q, (6, 4), "q"))
        q_dot = np.zeros((6, 4), dtype=np.float64) if self.q_dot is None else self.q_dot
        object.__setattr__(self, "q_dot", _finite_array(q_dot, (6, 4), "q_dot"))


@dataclass(frozen=True)
class RobotState:
    """Feedback returned after reset or one complete expert control period."""

    sim_time: float
    run_elapsed: float
    W_T_R: np.ndarray
    q: np.ndarray
    q_dot: np.ndarray
    q_torque: np.ndarray
    adhesion_force: np.ndarray
    contact_forces: np.ndarray

    def __post_init__(self):
        if not np.isfinite(self.sim_time) or not np.isfinite(self.run_elapsed):
            raise ValueError("sim_time and run_elapsed must be finite")
        object.__setattr__(self, "W_T_R", _transform_matrix(self.W_T_R))
        object.__setattr__(self, "q", _finite_array(self.q, (6, 4), "q"))
        object.__setattr__(self, "q_dot", _finite_array(self.q_dot, (6, 4), "q_dot"))
        object.__setattr__(self, "q_torque", _finite_array(self.q_torque, (6, 3), "q_torque"))
        object.__setattr__(self, "adhesion_force", _finite_array(self.adhesion_force, (6,), "adhesion_force"))
        object.__setattr__(self, "contact_forces", _finite_array(self.contact_forces, (6, 3), "contact_forces"))
