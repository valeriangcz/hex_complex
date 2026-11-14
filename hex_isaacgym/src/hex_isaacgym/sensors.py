"""Simulator sensor calculations, independent of ROS and Isaac Gym imports."""
import numpy as np


def attachment_ratio(force, maximum):
    """Normalize measured adhesion force; experts own attachment thresholds."""
    return np.clip(np.asarray(force, dtype=float) / maximum, 0.0, 1.0)


def gravity_direction_body(rotation, gravity):
    """Unit physical down direction in body axes, using actual world gravity."""
    gravity = np.asarray(gravity, dtype=float)
    norm = np.linalg.norm(gravity)
    if gravity.shape != (3,) or not np.isfinite(gravity).all() or not np.isfinite(norm) or norm == 0:
        raise ValueError("simulation gravity must be finite and nonzero")
    return np.asarray(rotation).T @ (gravity / norm)
