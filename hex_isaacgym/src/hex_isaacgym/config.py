"""Configuration loading and package-resource resolution for Isaac Gym."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

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



@dataclass(frozen=True)
class SimulationConfig:
    backend: BackendConfig
    initial_state: InitialState
    motor_mode: int
    servo_enabled: bool
    magnets: np.ndarray

    @classmethod
    def from_yaml(cls, filename, device=None, initial_config=None):
        from hex_kinematic import Kinematic
        from hex_kinematic.initial_state import parse_initial_state
        if initial_config is None:
            raise ValueError("initial_config must explicitly point to the shared runner YAML")
        filename = Path(filename).expanduser().resolve()
        data = yaml.safe_load(filename.read_text())
        backend = BackendConfig.from_dict(data, overrides={"device": device}, base=filename.parent)
        source = Path(initial_config).expanduser().resolve()
        shared = yaml.safe_load(source.read_text())
        model = resolve_resource(shared["robot_urdf"], base=source.parent)
        if model != backend.robot_urdf:
            raise ValueError("runner and simulator must use the same robot_urdf")
        initial = parse_initial_state(shared["initial_state"], shared["expert_type"], Kinematic(urdf_path=model))
        return cls(backend, InitialState(initial.W_T_R, initial.q), initial.motor_mode,
                   initial.servo_enabled, initial.magnetic)
