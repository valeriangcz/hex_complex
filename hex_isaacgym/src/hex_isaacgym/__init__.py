"""Minimal simulator backend and ROS adapters.

The public contracts and pure actuation helpers remain importable without
loading Isaac Gym or Torch. Import :class:`IsaacGymBackend` only in the process
that owns the simulator.
"""

from .actuation import pd_torques, update_adhesion_forces
from .config import BackendConfig
from .types import ActuatorCommand, InitialState, RobotState

__all__ = (
    "BackendConfig",
    "ActuatorCommand",
    "InitialState",
    "RobotState",
    "pd_torques",
    "update_adhesion_forces",
)
