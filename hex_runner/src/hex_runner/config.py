"""Controller configuration, independent of simulator configuration."""
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import yaml

from hex_kinematic import LEG_NAMES, QUAD_LEG_NAMES
from hex_kinematic.conventions import MOTOR_JOINT_NAMES
from hex_kinematic.se3_utils import transform_from_xyzw


def resource_path(value, base):
    if str(value).startswith("package://"):
        import rospkg
        package,relative = str(value)[10:].split("/",1)
        return (Path(rospkg.RosPack().get_path(package))/relative).resolve()
    path = Path(value).expanduser()
    return (base/path).resolve() if not path.is_absolute() else path.resolve()


def named(values, names, label):
    if not isinstance(values,dict) or set(values) != set(names):
        raise ValueError(label+' must explicitly define '+', '.join(names))
    return [values[name] for name in names]


@dataclass
class InitialParameters:
    W_T_R: np.ndarray
    magnetic: np.ndarray
    ik_branch: np.ndarray
    servo_enabled: bool
    groups: object = None
    stance_group: object = None
    sequence: object = None
    motor_positions: object = None
    foot_positions: object = None
    waypoint_index: object = None


def initial_parameters(values, expert_type, initialization_method='set_init'):
    leg_names = QUAD_LEG_NAMES if expert_type == "quad_climb" else LEG_NAMES
    data = dict(values)
    if ('pose' in data) == ('waypoint_index' in data):
        raise ValueError('initial_state must provide exactly one of pose or waypoint_index')
    waypoint_index = None
    W_T_R = None
    if 'waypoint_index' in data:
        waypoint_index = data.pop('waypoint_index')
        if expert_type != 'complex':
            raise ValueError('initial_state.waypoint_index is only supported for complex')
        if type(waypoint_index) is not int or waypoint_index < 0:
            raise ValueError('initial_state.waypoint_index must be a nonnegative integer')
    else:
        pose = dict(data.pop('pose'))
        position = named(pose.pop('position'),('x','y','z'),'initial_state.pose.position')
        quaternion = named(pose.pop('orientation'),('x','y','z','w'),'initial_state.pose.orientation')
        if pose or not np.isfinite(position+quaternion).all() or not np.isclose(np.linalg.norm(quaternion),1.,atol=1e-6):
            raise ValueError('initial_state.pose must contain a finite position and unit quaternion')
        W_T_R = transform_from_xyzw(position,quaternion)
    branches = named(data.pop('ik_branch'),leg_names,'initial_state.ik_branch')
    if any(type(value) is not int or value not in (0,1) for value in branches):
        raise ValueError('ik_branch must contain 0 or 1 for every leg')
    magnets = named(data.pop('magnetic'),leg_names,'initial_state.magnetic')
    if not all(type(value) is bool for value in magnets):
        raise ValueError('initial_state.magnetic values must be booleans')
    if expert_type == 'quad_climb':
        if 'groups' in data or 'stance_group' in data:
            raise ValueError('quad_climb uses initial_state.sequence; remove groups and stance_group')
        sequence = data.pop('sequence')
        if (not isinstance(sequence, list) or len(sequence) != 4
                or any(type(i) is not int for i in sequence) or set(sequence) != set(range(4))):
            raise ValueError('initial_state.sequence must contain integer indices 0..3 exactly once')
        gait = dict(sequence=sequence)
        if not all(magnets):
            raise ValueError('quad_climb initial magnetic must enable all four feet')
    else:
        gait = dict(groups=data.pop('groups'), stance_group=data.pop('stance_group'))
    initial = InitialParameters(W_T_R,np.asarray(magnets),np.asarray(branches),
        data.pop('servo_enabled'),waypoint_index=waypoint_index,**gait)
    if initial.servo_enabled is not True:
        raise ValueError('initial_state must explicitly enable servos')
    if expert_type in ('climb', 'quad_climb') or initialization_method == 'set_init':
        mode = data.pop('mode')
        if mode == 'joints':
            entries = named(data.pop('q'),leg_names,'initial_state.q')
            initial.motor_positions = np.asarray([named(entry,MOTOR_JOINT_NAMES,'initial_state.q') for entry in entries],dtype=float)
            if not np.isfinite(initial.motor_positions).all():
                raise ValueError('initial motor angles must be finite radians')
        elif mode == 'feet':
            if data.pop('frame') != 'leg':
                raise ValueError('initial_state.frame must be leg')
            entries = named(data.pop('feet'),leg_names,'initial_state.feet')
            initial.foot_positions = np.asarray([named(entry,('x','y','z'),'initial_state.feet') for entry in entries],dtype=float)
            if not np.isfinite(initial.foot_positions).all():
                raise ValueError('initial foot positions must be finite meters')
        else:
            raise ValueError('initial_state.mode must be joints or feet')
    if data:
        raise ValueError('Unused initial_state fields; remove: '+', '.join(data))
    return initial


@dataclass
class RunnerConfig:
    expert_type: str = "climb"
    pose_source: str = "gravity"
    control_dt: float = 0.01
    magnetic_hz: float = 20.0
    feedback_timeout: float = 0.5
    velocity_scales: dict = field(default_factory=dict)
    motor_kp: float = 60.0
    motor_kd: float = 1.0
    motor_torque_limit: float = 27.0
    completion_translation: float = 0.02
    completion_rotation: float = 0.05
    failure_path_distance: float = 1.0
    robot_urdf: str = "package://hex_kinematic/resources/hex_magnetic/urdf/hex_all.urdf"
    expert_parameters: dict = field(default_factory=dict)
    resources: dict = field(default_factory=dict)
    initial_state: object = None
    to_sim_info: Path = None
    data_root: Path = None
    initialization: dict = field(default_factory=dict)
    recording: dict = field(default_factory=dict)

    @property
    def recording_enabled(self):
        return self.expert_type == 'complex' and self.recording.get('enabled', True)

    @property
    def initialization_method(self):
        return self.initialization.get('method', 'set_init')

    @property
    def staged_initialization(self):
        return self.initialization.get('staged', False)

    @property
    def leg_names(self):
        return QUAD_LEG_NAMES if self.expert_type == "quad_climb" else LEG_NAMES

    def __post_init__(self):
        if (not isinstance(self.recording, dict) or set(self.recording)-{'enabled'}
                or type(self.recording.get('enabled', True)) is not bool):
            raise ValueError('recording supports only a boolean enabled')
        if not isinstance(self.initialization,dict) or set(self.initialization)-{'method','staged'}:
            raise ValueError('initialization supports only method and staged')
        if type(self.staged_initialization) is not bool:
            raise ValueError('initialization.staged must be a boolean')
        if self.expert_type == 'quad_climb' and self.staged_initialization:
            raise ValueError('staged initialization currently supports six-leg robots only')
        if self.initialization_method not in ('set_init','set_init_by_se3'):
            raise ValueError('initialization.method must be set_init or set_init_by_se3')
        if self.expert_type != 'complex' and self.initialization_method != 'set_init':
            raise ValueError('set_init_by_se3 applies only to complex')
        if self.expert_type not in ("climb", "quad_climb", "complex") or self.pose_source not in ("gravity", "se3"):
            raise ValueError("expert_type must be climb/quad_climb/complex and pose_source gravity/se3")
        if self.expert_type == "complex" and self.pose_source != "se3":
            raise ValueError("complex requires full SE3 localization")
        if self.expert_type in ("climb", "quad_climb") and self.pose_source != "gravity":
            raise ValueError("climb requires body gravity direction")
        for name in ("control_dt", "magnetic_hz", "feedback_timeout",
                     "motor_torque_limit", "completion_translation", "completion_rotation", "failure_path_distance"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + " must be positive and finite")
        for name in ("motor_kp", "motor_kd"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(name + " must be nonnegative and finite")
        if self.velocity_scales and (set(self.velocity_scales) != {"vx", "vy", "wz"} or
                                     not all(np.isfinite(v) for v in self.velocity_scales.values())):
            raise ValueError("velocity_scales must define finite vx/vy/wz values")
        if {"kinematic", "control_dt", "motor_torque_limits"}.intersection(self.expert_parameters):
            raise ValueError("model/timing/limits belong in runner configuration")

    @classmethod
    def load(cls, filename, data_root=None, to_sim_info=None):
        filename = Path(filename).expanduser().resolve()
        data = yaml.safe_load(filename.read_text())
        root = resource_path(data.pop("workspace_root","../../.."),filename.parent)
        data["data_root"] = Path(data_root).expanduser().resolve() if data_root else root/"data"
        data["resources"] = {name: resource_path(str(value).replace("${DATA_ROOT}",str(data["data_root"])),root)
                             for name,value in data.get("resources",{}).items()}
        data["robot_urdf"] = str(resource_path(data.get("robot_urdf",cls.robot_urdf),filename.parent))
        data["to_sim_info"] = resource_path(to_sim_info or data.get("to_sim_info") or
            "package://hex_runner/config/to_sim_info.yaml",filename.parent)
        method = data.get('initialization',{}).get('method', 'set_init')
        data["initial_state"] = initial_parameters(data["initial_state"],data["expert_type"],method)
        config = cls(**data)
        if config.expert_type in ("climb", "quad_climb") and not config.velocity_scales:
            raise ValueError("climb configuration requires velocity_scales.vx/vy/wz")
        return config
