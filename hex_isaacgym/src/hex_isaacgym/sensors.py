"""Simulator sensor calculations, independent of ROS and Isaac Gym imports."""
import numpy as np


def attachment_state(previous, force, maximum, attach_ratio=.85, release_ratio=.05):
    force = np.asarray(force)
    if not 0 <= release_ratio < attach_ratio <= 1 or maximum <= 0:
        raise ValueError("invalid attachment hysteresis")
    previous = np.asarray(previous, dtype=bool)
    return np.where(force >= attach_ratio * maximum, True,
                    np.where(force <= release_ratio * maximum, False, previous))


def imu_specific_force(rotation, velocity, previous_velocity, dt, gravity):
    if previous_velocity is None:
        return np.zeros(3), False
    acceleration = (np.asarray(velocity) - previous_velocity) / dt
    return np.asarray(rotation).T @ (acceleration - np.asarray(gravity)), True
