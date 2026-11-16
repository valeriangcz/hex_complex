"""Kinematic initial-joint solving and foot-normal alignment used by experts."""
import warnings
import numpy as np
from hex_kinematic.conventions import LEG_NAMES


def complex_foot_angles(kin, q, positions, normals):
    """The ExpertComplex normal-alignment rule, in each leg's local frame."""
    plane = np.column_stack((-positions[:, 1], positions[:, 0], np.zeros(len(positions))))
    plane /= np.maximum(np.linalg.norm(plane, axis=1, keepdims=True), 1e-5)
    parallel = plane * np.sum(normals * plane, axis=1, keepdims=True) - normals
    parallel /= np.maximum(np.linalg.norm(parallel, axis=1, keepdims=True), 1e-5)
    x = kin.Get_X_B3(q)
    cosine = np.clip(np.sum(parallel * x, axis=1), -1, 1)
    if (cosine < 0).any():
        warnings.warn('Norm of landing points may exceed the range of suction cup')
    direction = np.sign(np.sum(np.cross(x, parallel) * kin.Get_Z_B3(q), axis=1))
    return np.clip(np.arccos(cosine) * direction, -np.pi / 2, np.pi / 2)


def climb_initial_joints(kin, *, motor_positions=None, foot_positions=None, ik_branch):
    branches = np.asarray(ik_branch,dtype=int)
    if motor_positions is not None:
        motors = np.asarray(motor_positions,dtype=float)
        if ((motors[:,2] > 1e-8) & (branches != 0) | (motors[:,2] < -1e-8) & (branches != 1)).any():
            raise ValueError('ik_branch must match the supplied ankle angle sign')
    else:
        solutions,valid = kin.InverseKin2MultiBatch(np.asarray(foot_positions,dtype=float))
        for i,leg in enumerate(LEG_NAMES):
            if not valid[i,branches[i]]:
                raise ValueError(f'Initial foot position for {leg} is invalid in IK branch {branches[i]}')
        motors = solutions[np.arange(6),branches]
    for i, leg in enumerate(LEG_NAMES):
        bounds = np.vstack((kin.joints_limits_thigh[i], kin.joints_limits_rest))
        if ((motors[i] < bounds[:,0] - 1e-6) | (motors[i] > bounds[:,1] + 1e-6)).any():
            raise ValueError('initial_state.'+leg+': motor angles outside kinematic limits')
    q = np.zeros((6,4))
    q[:, :3] = motors
    q[:, 3] = -motors[:, 1] - motors[:, 2] - np.pi / 2
    return q
