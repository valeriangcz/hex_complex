"""Small SE(3) boundary helpers used by ROS and Isaac Gym adapters."""

import numpy as np
from scipy.spatial.transform import Rotation


def transform_from_xyzw(position, quaternion_xyzw):
    position = np.asarray(position, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if position.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("position and quaternion must have shapes (3,) and (4,)")
    if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
        raise ValueError("pose values must be finite")
    norm = np.linalg.norm(quaternion)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    transform[:3, 3] = position
    return transform


def transform_to_xyzw(transform):
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("transform must be a finite 4x4 array")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
        raise ValueError("invalid homogeneous transform bottom row")
    return transform[:3, 3].copy(), Rotation.from_matrix(transform[:3, :3]).as_quat()
