"""One explicit, named initial-state contract shared by control and simulation."""
from dataclasses import dataclass
import warnings
import numpy as np
from .conventions import LEG_NAMES, MOTOR_JOINT_NAMES
from .se3_utils import transform_from_xyzw

STANCE_GROUPS = (('lb', 'lf', 'rm'), ('lm', 'rb', 'rf'))


def named(values, keys, label):
    if not isinstance(values, dict) or set(values) != set(keys):
        raise ValueError(label + ' must explicitly define ' + ', '.join(keys))
    return [values[key] for key in keys]


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


@dataclass
class RobotInitialState:
    W_T_R: np.ndarray
    q: np.ndarray
    magnetic: np.ndarray
    ik_branch: np.ndarray
    stance_group: int
    motor_mode: int
    servo_enabled: bool


def parse_initial_state(data, expert_type, kin):
    data = dict(data)
    mode = data.pop('mode')
    if mode not in ('joints', 'feet'):
        raise ValueError('initial_state.mode must be joints or feet')
    pose = dict(data.pop('pose'))
    position = named(pose.pop('position'), ('x', 'y', 'z'), 'initial_state.pose.position')
    quaternion = named(pose.pop('orientation'), ('x', 'y', 'z', 'w'), 'initial_state.pose.orientation')
    if pose or not np.isfinite(position + quaternion).all() or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-6):
        raise ValueError('initial_state.pose must contain a finite position and unit quaternion')
    matrix = transform_from_xyzw(position, quaternion)
    branches = named(data.pop('ik_branch'), LEG_NAMES, 'initial_state.ik_branch')
    if any(type(value) is not int or value not in (0, 1) for value in branches):
        raise ValueError('ik_branch must be 0 (positive ankle) or 1 (negative ankle)')
    branches = np.asarray(branches, dtype=int)
    magnetic = named(data.pop('magnetic'), LEG_NAMES, 'initial_state.magnetic')
    if not all(type(value) is bool for value in magnetic):
        raise ValueError('initial_state.magnetic values must be booleans')
    group = data.pop('stance_group')
    if type(group) is not int or group not in (0, 1):
        raise ValueError('stance_group must be 0 or 1')
    if mode == 'joints':
        entries = named(data.pop('q'), LEG_NAMES, 'initial_state.q')
        motors = np.asarray([named(value, MOTOR_JOINT_NAMES, 'q.'+leg) for leg, value in zip(LEG_NAMES, entries)], dtype=float)
        if not np.isfinite(motors).all():
            raise ValueError('initial motor angles must be finite radians')
        if ((motors[:, 2] > 1e-8) & (branches != 0) | (motors[:, 2] < -1e-8) & (branches != 1)).any():
            raise ValueError('ik_branch must match the explicitly supplied ankle angle sign')
    else:
        if data.pop('frame') != 'leg':
            raise ValueError('initial_state.frame must explicitly be leg')
        entries = named(data.pop('feet'), LEG_NAMES, 'initial_state.feet')
        positions = np.asarray([named(value, ('x','y','z'), 'feet.'+leg) for leg, value in zip(LEG_NAMES, entries)], dtype=float)
        if not np.isfinite(positions).all():
            raise ValueError('initial foot positions must be finite meters')
        solutions, valid = kin.InverseKin2MultiBatch(positions)
        for i, leg in enumerate(LEG_NAMES):
            if not valid[i, branches[i]]:
                raise ValueError('initial_state.feet.'+leg+': requested IK branch '+str(branches[i])+' is invalid')
        motors = solutions[np.arange(6), branches]
    for i, leg in enumerate(LEG_NAMES):
        bounds = np.vstack((kin.joints_limits_thigh[i], kin.joints_limits_rest))
        if ((motors[i] < bounds[:,0] - 1e-6) | (motors[i] > bounds[:,1] + 1e-6)).any():
            raise ValueError('initial_state.'+leg+': motor angles outside kinematic limits')
    positions = np.empty((6,3))
    kin.ForwardKin(motors, positions)
    q = np.zeros((6,4))
    q[:, :3] = motors
    if expert_type == 'climb':
        q[:, 3] = -motors[:, 1] - motors[:, 2] - np.pi / 2
    elif expert_type == 'complex':
        # Same +z landing normal as ExpertComplex.SetInit; subsequent planning updates normals.
        normals = np.tile([0.,0.,1.], (6,1))
        angles = complex_foot_angles(kin, motors, positions, normals)
        for leg in STANCE_GROUPS[group]:
            q[LEG_NAMES.index(leg),3] = angles[LEG_NAMES.index(leg)]
    else:
        raise ValueError('expert_type must be climb or complex')
    mode_code, enabled = data.pop('motor_mode'), data.pop('servo_enabled')
    if type(mode_code) is not int or mode_code != 1 or enabled is not True:
        raise ValueError('initial_state must explicitly enable MIT motors and servos')
    if data:
        raise ValueError('unknown initial_state keys: '+str(list(data)))
    return RobotInitialState(matrix, q, np.array(magnetic,dtype=bool), branches, group,
                             mode_code, enabled)
