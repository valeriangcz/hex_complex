"""Shared robot ordering and frame conventions."""

from pathlib import Path

import numpy as np


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
QUAD_LEG_NAMES = ("lf", "lb", "rf", "rb")


def robot_leg_names(urdf_path):
    """Recognize supported robot layouts independently of XML joint order."""
    import xml.etree.ElementTree as ET
    names = {j.get("name")[2:-6] for j in ET.parse(urdf_path).getroot().findall("joint")
             if j.get("name", "").startswith("j_") and j.get("name", "").endswith("_thigh")}
    for layout in (LEG_NAMES, QUAD_LEG_NAMES):
        if names == set(layout):
            return layout
    raise ValueError("Unsupported URDF leg layout: {}".format(sorted(names)))


MOTOR_JOINT_NAMES = ("thigh", "knee", "ankle")
COMMAND_JOINT_NAMES = MOTOR_JOINT_NAMES + ("foot",)


def joint_names(parts=COMMAND_JOINT_NAMES, leg_names=LEG_NAMES):
    """Return URDF joint names in leg-major order."""
    return tuple(
        "j_{}_{}".format(leg, part)
        for leg in leg_names
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
