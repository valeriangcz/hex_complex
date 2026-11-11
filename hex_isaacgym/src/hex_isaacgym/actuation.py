"""Pure NumPy reference implementation of migrated actuation rules."""

import numpy as np


def pd_torques(q_des, q, q_dot, kp, kd, torque_limits, tau_ff=None, motor_indices=None):
    """Compute the source PD law and clip it to URDF effort limits."""
    q_des = np.asarray(q_des, dtype=np.float64)
    q = np.asarray(q, dtype=np.float64)
    q_dot = np.asarray(q_dot, dtype=np.float64)
    kp = np.asarray(kp, dtype=np.float64)
    kd = np.asarray(kd, dtype=np.float64)
    limits = np.asarray(torque_limits, dtype=np.float64)
    if not (q_des.shape == q.shape == q_dot.shape == kp.shape == kd.shape == limits.shape):
        raise ValueError("PD inputs must have one common shape")
    torque = kp * (q_des - q) - kd * q_dot
    if tau_ff is not None:
        if motor_indices is None:
            raise ValueError("motor_indices are required with tau_ff")
        indices = np.asarray(motor_indices, dtype=np.int64)
        feedforward = np.asarray(tau_ff, dtype=np.float64).reshape(-1)
        if feedforward.shape != indices.shape:
            raise ValueError("tau_ff and motor_indices must have the same flat shape")
        torque[indices] += feedforward
    return np.clip(torque, -limits, limits)


def update_adhesion_forces(
    previous_z,
    contact_forces,
    previous_contacts,
    enabled,
    in_initial_buffer,
    force_delta=30.0,
    force_max=300.0,
    contact_threshold=1.0,
):
    """Update the three local-z magnetic forces for each leg.

    Negative values are the source ``rb_forces`` convention. The backend
    applies their negation in Isaac Gym local coordinates.
    """
    previous_z = np.asarray(previous_z, dtype=np.float64)
    contact_forces = np.asarray(contact_forces, dtype=np.float64)
    previous_contacts = np.asarray(previous_contacts, dtype=bool)
    enabled = np.asarray(enabled, dtype=bool)
    if previous_z.shape != (6, 3):
        raise ValueError("previous_z must have shape (6, 3)")
    if contact_forces.shape != (6, 3):
        raise ValueError("contact_forces must have shape (6, 3)")
    if previous_contacts.shape != (6,) or enabled.shape != (6,):
        raise ValueError("previous_contacts and enabled must have shape (6,)")

    contacts = np.linalg.norm(contact_forces, axis=1) > float(contact_threshold)
    filtered = contacts | previous_contacts
    rate = np.where(enabled, -5.0, 1.0)
    if in_initial_buffer:
        filtered[:] = True
        rate[enabled] = -1000.0
    updated = previous_z + rate[:, None] * (float(force_delta) / 3.0)
    updated *= filtered[:, None]
    updated = np.clip(updated, -float(force_max) / 3.0, 0.0)
    return updated, contacts
