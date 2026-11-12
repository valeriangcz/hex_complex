"""Shared robot ordering and frame conventions."""

from pathlib import Path

import numpy as np


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
MOTOR_JOINT_NAMES = ("thigh", "knee", "ankle")
COMMAND_JOINT_NAMES = MOTOR_JOINT_NAMES + ("foot",)


def joint_names(parts=COMMAND_JOINT_NAMES):
    """Return URDF joint names in leg-major order."""
    return tuple(
        "j_{}_{}".format(leg, part)
        for leg in LEG_NAMES
        for part in parts
    )


def require_finite_shape(value, shape, name):
    """Convert to an array and validate the public shape contract."""
    array = np.asarray(value)
    if array.shape != tuple(shape) or not np.isfinite(array).all():
        raise ValueError(
            "{} must be a finite array with shape {}, got {}".format(
                name, tuple(shape), array.shape
            )
        )
    return array


def validate_robot_resource_dir(resource_dir):
    """Validate the migrated hex_magnetic resource directory."""
    resource_dir = Path(resource_dir).expanduser().resolve()
    required = (
        resource_dir / "urdf/hex_all.urdf",
        resource_dir / "body_leg_collision.STL",
        resource_dir / "body_env_collision.STL",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing robot resources: {}".format(missing))
    return resource_dir
