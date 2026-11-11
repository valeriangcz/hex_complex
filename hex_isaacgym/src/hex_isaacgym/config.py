"""Configuration loading and package-resource resolution for Isaac Gym."""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from hex_kinematic.se3_utils import transform_from_xyzw

from .types import InitialState


def resolve_resource(value, base=None):
    """Resolve a filesystem path or ROS ``package://`` URI."""
    value = str(value)
    if value.startswith("package://"):
        package_and_path = value[len("package://"):].split("/", 1)
        if len(package_and_path) != 2:
            raise ValueError("invalid package URI: {}".format(value))
        try:
            import rospkg
        except ImportError as exc:
            raise RuntimeError(
                "rospkg is required to resolve {} (source the ROS workspace)".format(value)
            ) from exc
        root = Path(rospkg.RosPack().get_path(package_and_path[0]))
        path = root / package_and_path[1]
    else:
        path = Path(value).expanduser()
        if not path.is_absolute() and base is not None:
            path = Path(base) / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


@dataclass(frozen=True)
class BackendConfig:
    robot_urdf: Path
    map_mesh: Path = None
    isaacgym_python: Path = None
    device: str = "cpu"
    physics_dt: float = 0.0025
    substeps: int = 2
    decimation: int = 4
    gravity: tuple = (0.0, 0.0, -9.81)
    static_friction: float = 0.35
    dynamic_friction: float = 0.32
    robot_friction: float = 0.3275
    restitution: float = 0.0
    suction_force_delta: float = 30.0
    suction_force_max: float = 300.0
    initial_buffer_time: float = 0.5
    contact_threshold: float = 1.0
    self_collisions: int = 0
    terrain_type: str = "mesh"
    v_max: float = 0.08
    w_max: float = 0.2
    stance_maxmove_t: float = 0.2
    stance_maxmove_w: float = 0.4

    @property
    def control_dt(self):
        return self.physics_dt * self.decimation

    @classmethod
    def from_dict(cls, data, overrides=None, base=None):
        data = dict(data)
        sim = dict(data.pop("sim", {}))
        terrain = dict(data.pop("terrain", {}))
        control = dict(data.pop("control", {}))
        asset = dict(data.pop("asset", {}))
        flat = {
            "robot_urdf": resolve_resource(asset.pop("robot_urdf"), base=base),
            "map_mesh": None,
            "terrain_type": terrain.pop("type", "mesh"),
            "isaacgym_python": data.pop("isaacgym_python", None),
            "device": data.pop("device", "cpu"),
            "physics_dt": sim.pop("dt", 0.0025),
            "substeps": sim.pop("substeps", 2),
            "decimation": control.pop("decimation", 4),
            "gravity": tuple(sim.pop("gravity", (0.0, 0.0, -9.81))),
            "static_friction": terrain.pop("static_friction", 0.35),
            "dynamic_friction": terrain.pop("dynamic_friction", 0.32),
            "robot_friction": asset.pop("friction", 0.3275),
            "restitution": terrain.pop("restitution", 0.0),
            "suction_force_delta": control.pop("suction_force_delta", 30.0),
            "suction_force_max": control.pop("suction_force_max", 300.0),
            "initial_buffer_time": control.pop("initial_buffer_time", 0.5),
            "contact_threshold": control.pop("contact_threshold", 1.0),
            "v_max": control.pop("v_max", 0.08),
            "w_max": control.pop("w_max", 0.2),
            "stance_maxmove_t": control.pop("stance_maxmove_t", 0.2),
            "stance_maxmove_w": control.pop("stance_maxmove_w", 0.4),
            "self_collisions": asset.pop("self_collisions", 0),
        }
        mesh = terrain.pop("map_mesh", None)
        if mesh is not None:
            flat["map_mesh"] = resolve_resource(mesh, base=base)
        unknown = {"root": data, "sim": sim, "terrain": terrain, "control": control, "asset": asset}
        unknown = {name: values for name, values in unknown.items() if values}
        if unknown:
            raise ValueError("unknown backend configuration keys: {}".format(unknown))
        if overrides:
            flat.update({key: value for key, value in overrides.items() if value is not None})
        if flat["isaacgym_python"] is not None:
            flat["isaacgym_python"] = Path(flat["isaacgym_python"]).expanduser().resolve()
        config = cls(**flat)
        config.validate()
        return config

    @classmethod
    def from_yaml(cls, path, overrides=None):
        path = Path(path).expanduser().resolve()
        with path.open("r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream)
        if not isinstance(payload, dict):
            raise ValueError("backend config must contain a YAML mapping")
        return cls.from_dict(payload, overrides=overrides, base=path.parent)

    def validate(self):
        for name in ("v_max", "w_max", "stance_maxmove_t", "stance_maxmove_w"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError("{} must be positive and finite".format(name))
        if self.terrain_type not in ("mesh", "plane"):
            raise ValueError("terrain.type must be mesh or plane")
        if self.terrain_type == "mesh" and self.map_mesh is None:
            raise ValueError("mesh terrain requires map_mesh")
        if self.device != "cpu" and not self.device.startswith("cuda"):
            raise ValueError("device must be cpu or cuda[:index], got {}".format(self.device))
        if self.physics_dt <= 0.0 or self.substeps < 1 or self.decimation < 1:
            raise ValueError("dt, substeps, and decimation must be positive")
        if len(self.gravity) != 3:
            raise ValueError("gravity must have three components")
        if self.suction_force_delta <= 0.0 or self.suction_force_max <= 0.0:
            raise ValueError("suction force parameters must be positive")
        if self.initial_buffer_time < 0.0 or self.contact_threshold < 0.0:
            raise ValueError("buffer time and contact threshold cannot be negative")


def _resolve(value, base):
    value = str(value)
    if value.startswith("package://"):
        return resolve_resource(value)
    path = Path(value).expanduser()
    return (base / path).resolve() if not path.is_absolute() else path.resolve()


@dataclass(frozen=True)
class SceneConfig:
    backend_config: Path
    robot_urdf: Path
    body_leg_mesh: Path = None
    body_env_mesh: Path = None
    point_map_file: Path = None
    env_cache_file: Path = None
    robot_cache_dir: Path = None
    se3_path: Path = None
    settling_steps: int = 100
    completion_translation: float = 0.02
    completion_rotation: float = 0.05
    failure_path_distance: float = 1.0
    contact_force_threshold: float = 10.0
    contact_loss_steps: int = 50
    expert_type: str = "complex"
    initial_state: InitialState = None
    command: tuple = (0.02, 0.0, 0.0)
    expert_parameters: dict = field(default_factory=dict)
    data_root: Path = None

    def __post_init__(self):
        if self.expert_type not in ("complex", "climb"):
            raise ValueError("expert_type must be complex or climb")
        if self.expert_type == "climb":
            if not isinstance(self.initial_state, InitialState):
                raise ValueError("climb scene requires an initial_state")
            command = np.asarray(self.command, dtype=np.float64)
            if command.shape != (3,) or not np.isfinite(command).all():
                raise ValueError("command must be finite with shape (3,)")
            bound = {"kinematic", "control_dt", "motor_torque_limits", "adhesion_force_limit"}
            if bound.intersection(self.expert_parameters):
                raise ValueError("geometry, timing and physical limits are bound by the backend")

    @property
    def coordinator_options(self):
        """State-machine settings, forwarded without another list of defaults."""
        return {name: getattr(self, name) for name in (
            "settling_steps", "completion_translation", "completion_rotation",
            "failure_path_distance", "contact_force_threshold", "contact_loss_steps",
        )}

    @classmethod
    def from_yaml(cls, path, data_root=None):
        path = Path(path).expanduser().resolve()
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError("scene config must contain a YAML mapping")
        base = path.parent
        configured_root = data.pop("workspace_root", None)
        workspace_root = _resolve(configured_root, base) if configured_root else base
        data_root = Path(data_root).expanduser().resolve() if data_root else workspace_root / "data"
        resources = dict(data.pop("resources"))
        coordinator = dict(data.pop("coordinator", {}))
        expert_type = data.pop("expert_type", "complex")
        climb = dict(data.pop("climb", {}))

        def resource(name, default=None):
            value = resources.pop(name, default)
            if value is None:
                raise ValueError("missing scene resource {}".format(name))
            text = str(value).replace("${DATA_ROOT}", str(data_root))
            return _resolve(text, workspace_root)

        config_path = resource("backend_config")
        robot_urdf = BackendConfig.from_yaml(config_path).robot_urdf
        extra = {}
        if expert_type == "complex":
            for name in ("body_leg_mesh", "body_env_mesh", "point_map_file",
                         "env_cache_file", "robot_cache_dir", "se3_path"):
                extra[name] = resource(name)
        elif expert_type == "climb":
            pose = dict(climb.pop("initial_pose"))
            matrix = transform_from_xyzw(pose.pop("position"), pose.pop("quaternion_xyzw"))
            if pose:
                raise ValueError("unknown initial_pose keys: {}".format(pose))
            extra["initial_state"] = InitialState(matrix, climb.pop("initial_q"))
            extra["command"] = tuple(climb.pop("command", (0.02, 0.0, 0.0)))
            extra["expert_parameters"] = dict(climb.pop("parameters", {}))
        result = cls(
            backend_config=config_path,
            robot_urdf=robot_urdf,
            data_root=data_root,
            expert_type=expert_type,
            settling_steps=int(coordinator.pop("settling_steps", 100)),
            completion_translation=float(coordinator.pop("completion_translation", 0.02)),
            completion_rotation=float(coordinator.pop("completion_rotation", 0.05)),
            failure_path_distance=float(coordinator.pop("failure_path_distance", 1.0)),
            contact_force_threshold=float(coordinator.pop("contact_force_threshold", 10.0)),
            contact_loss_steps=int(coordinator.pop("contact_loss_steps", 50)),
            **extra,
        )
        if data or resources or coordinator or climb:
            raise ValueError(
                "unknown scene keys root={}, resources={}, coordinator={}, climb={}".format(
                    data, resources, coordinator, climb
                )
            )
        for field in (
            "backend_config", "robot_urdf", "body_leg_mesh", "body_env_mesh",
            "point_map_file", "env_cache_file", "se3_path",
        ):
            value = getattr(result, field)
            if value is not None and not value.is_file():
                raise FileNotFoundError(value)
        if result.robot_cache_dir is not None and not result.robot_cache_dir.is_dir():
            raise FileNotFoundError(result.robot_cache_dir)
        return result
