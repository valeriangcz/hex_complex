"""Controller configuration, independent of simulator configuration."""
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import yaml


def resource(value, base):
    if str(value).startswith("package://"):
        import rospkg
        package, relative = str(value)[10:].split("/", 1)
        return (Path(rospkg.RosPack().get_path(package)) / relative).resolve()
    path = Path(value).expanduser()
    return (base / path).resolve() if not path.is_absolute() else path.resolve()


@dataclass
class RunnerConfig:
    expert_type: str = "climb"
    pose_source: str = "imu"
    control_dt: float = 0.01
    servo_hz: float = 20.0
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
    data_root: Path = None

    def __post_init__(self):
        if self.expert_type not in ("climb", "complex") or self.pose_source not in ("imu", "se3"):
            raise ValueError("expert_type must be climb/complex and pose_source imu/se3")
        if self.expert_type == "complex" and self.pose_source != "se3":
            raise ValueError("complex requires full SE3 localization")
        for name in ("control_dt", "servo_hz", "magnetic_hz", "feedback_timeout",
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
    def load(cls, filename, data_root=None):
        filename = Path(filename).expanduser().resolve()
        data = yaml.safe_load(filename.read_text())
        root = resource(data.pop("workspace_root", "../../.."), filename.parent)
        data["data_root"] = Path(data_root).expanduser().resolve() if data_root else root / "data"
        data["robot_urdf"] = str(resource(data.get("robot_urdf", cls.robot_urdf), filename.parent))
        data["resources"] = {key: resource(str(value).replace("${DATA_ROOT}", str(data["data_root"])), root)
                             for key, value in data.get("resources", {}).items()}
        from hex_kinematic import Kinematic
        from hex_kinematic.initial_state import parse_initial_state
        data["initial_state"] = parse_initial_state(data["initial_state"], data["expert_type"], Kinematic(urdf_path=data["robot_urdf"]))
        config = cls(**data)
        if config.expert_type == "climb" and not config.velocity_scales:
            raise ValueError("climb configuration requires velocity_scales.vx/vy/wz")
        return config
