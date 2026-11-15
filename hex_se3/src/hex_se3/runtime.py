"""Explicit resource resolution shared by SE(3) command-line applications."""

from pathlib import Path

from hex_kinematic import Kinematic
from map_robot_voxles import HexState
from map_robot_voxles.cache_tools import load_config, resolve_path


def load_scene_config(path):
    path = Path(path).expanduser().resolve()
    config = load_config(path)
    config["_config_file"] = path
    config["_workspace_root"] = Path(config["_workspace_root"])
    return config


def scene_path(config, key):
    value = Path(config[key]).expanduser()
    if not value.is_absolute():
        value = config["_workspace_root"] / value
    return value.resolve()


def build_hex_state(scene_config):
    map_config = load_config(scene_path(scene_config, "map_config"))
    robot_config = load_config(scene_path(scene_config, "robot_config"))
    model = Kinematic(urdf_path=resolve_path(robot_config, robot_config["urdf_file"]))
    state = HexState(
        model,
        robot_cache_dir=resolve_path(robot_config, robot_config["cache_dir"]),
        point_map_file=resolve_path(map_config, map_config["point_map_file"]),
        env_cache_file=resolve_path(map_config, map_config["env_cache_file"]),
        body_leg_mesh=resolve_path(robot_config, robot_config["body_leg_mesh"]),
        body_env_mesh=resolve_path(robot_config, robot_config["body_env_mesh"]),
    )
    return state, map_config, robot_config
