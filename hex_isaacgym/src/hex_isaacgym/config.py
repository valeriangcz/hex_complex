"""Configuration loading and package-resource resolution for Isaac Gym."""

from dataclasses import dataclass, replace
from functools import cached_property
from pathlib import Path

import numpy as np
import yaml

from .types import InitialState
from hex_kinematic.sim_info import read_sim_info
from hex_kinematic.conventions import LEG_NAMES, QUAD_LEG_NAMES, robot_leg_names


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
    camera_follow_robot: bool = False
    camera_offset: tuple = (0.9, 1.0, 0.6)

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
        viewer = dict(data.pop("viewer", {}))
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
            "camera_follow_robot": viewer.pop("follow_robot", False),
            "camera_offset": tuple(viewer.pop("camera_offset", (0.9, 1.0, 0.6))),
        }
        mesh = terrain.pop("map_mesh", None)
        if mesh is not None:
            flat["map_mesh"] = resolve_resource(mesh, base=base)
        unknown = {"root": data, "sim": sim, "terrain": terrain, "control": control, "asset": asset, "viewer": viewer}
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
        if type(self.camera_follow_robot) is not bool:
            raise ValueError("viewer.follow_robot must be a boolean")
        offset = np.asarray(self.camera_offset,dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all() or np.linalg.norm(offset) <= 1e-8:
            raise ValueError("viewer.camera_offset must contain three finite values with nonzero length")
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
    servo_enabled: bool
    magnets: np.ndarray

    reference_path: np.ndarray
    to_sim_info: Path

    @cached_property
    def leg_names(self):
        return robot_leg_names(self.backend.robot_urdf)

    @staticmethod
    def _initial_values(filename, expected_legs=None, expected_urdf=None):
        document = read_sim_info(filename)
        legs = tuple(document.get("leg_names", LEG_NAMES))
        if legs not in (LEG_NAMES, QUAD_LEG_NAMES) or (expected_legs is not None and legs != tuple(expected_legs)):
            raise ValueError("initial state leg names do not match backend")
        if expected_urdf is not None and resolve_resource(document["robot_urdf"]) != expected_urdf:
            raise ValueError("initial state URDF does not match backend")
        initial = document["initial_state"]
        if np.asarray(initial["q"]).shape != (len(legs), 4) or np.asarray(initial["magnetic"]).shape != (len(legs),):
            raise ValueError("initial state arrays do not match leg names")
        return dict(initial_state=InitialState(initial["W_T_R"],initial["q"]),
                    servo_enabled=initial["servo_enabled"],
                    magnets=np.asarray(initial["magnetic"],dtype=bool),
                    reference_path=np.asarray(document["reference_path"],dtype=float).reshape(-1,3))

    @classmethod
    def from_yaml(cls, filename, device=None, to_sim_info=None):
        if to_sim_info is None:
            raise ValueError("to_sim_info must point to the file generated by runner")
        backend = BackendConfig.from_yaml(filename,overrides={"device": device})
        source = Path(to_sim_info).expanduser().resolve()
        return cls(backend=backend,to_sim_info=source,**cls._initial_values(source, robot_leg_names(backend.robot_urdf), backend.robot_urdf))

    def reload_initial_state(self):
        return replace(self,**self._initial_values(self.to_sim_info, self.leg_names, self.backend.robot_urdf))
