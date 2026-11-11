"""Assemble one configured expert, backend and control loop.

Configuration lives in config.py; stepping/reset lifecycle lives in coordinator.py.
SceneConfig remains importable here for existing callers.
"""

from pathlib import Path

from hex_expert import ExpertClimb, ExpertComplex
from hex_kinematic import Kinematic

from .adapters import ClimbExpertAdapter, ComplexExpertAdapter
from .backend import IsaacGymBackend
from .config import BackendConfig, SceneConfig
from .coordinator import ExpertCoordinator


def _build_complex(scene, config, kinematic, path, headless):
    from hex_se3 import load_trajectory
    from map_robot_voxles import HexState

    trajectory = scene.se3_path if path is None else Path(path).expanduser().resolve()
    document = load_trajectory(trajectory)
    poses = document.dense.poses if document.dense is not None else document.waypoints
    hex_state = HexState(
        kinematic, robot_cache_dir=scene.robot_cache_dir,
        point_map_file=scene.point_map_file, env_cache_file=scene.env_cache_file,
        body_leg_mesh=scene.body_leg_mesh, body_env_mesh=scene.body_env_mesh,
    )
    expert = ExpertComplex(
        kinematic, hex_state, control_dt=config.control_dt,
        v_max=config.v_max, w_max=config.w_max,
        stance_maxmove_t=config.stance_maxmove_t, stance_maxmove_w=config.stance_maxmove_w,
    )
    adapter = ComplexExpertAdapter(expert, poses)
    initial_state, _ = adapter.reset()
    backend = IsaacGymBackend(config, headless=headless, initial_state=initial_state)
    return backend, adapter, document


def _build_climb(scene, config, kinematic, path, headless):
    if path:
        raise ValueError("climb mode uses velocity commands, not a trajectory path")
    backend = IsaacGymBackend(config, headless=headless, initial_state=scene.initial_state)
    try:
        expert = ExpertClimb(
            kinematic, control_dt=config.control_dt,
            motor_torque_limits=backend.motor_torque_limits,
            adhesion_force_limit=config.suction_force_max,
            **scene.expert_parameters,
        )
        expert.world_gravity[:] = config.gravity
        adapter = ClimbExpertAdapter(expert, scene.initial_state, scene.command)
    except Exception:
        backend.close()
        raise
    return backend, adapter, None


def build_coordinator(scene, path=None, device=None, headless=True):
    """Build a control loop; optional path/device override YAML for this run."""
    if not isinstance(scene, SceneConfig):
        scene = SceneConfig.from_yaml(scene)
    config = BackendConfig.from_yaml(scene.backend_config, overrides={"device": device})
    kinematic = Kinematic(urdf_path=config.robot_urdf)
    if scene.expert_type == "complex":
        backend, adapter, document = _build_complex(scene, config, kinematic, path, headless)
    elif scene.expert_type == "climb":
        backend, adapter, document = _build_climb(scene, config, kinematic, path, headless)
    else:
        raise ValueError("unknown expert_type {}".format(scene.expert_type))
    try:
        coordinator = ExpertCoordinator(
            backend, adapter.expert, adapter=adapter, **scene.coordinator_options
        )
    except Exception:
        backend.close()
        raise
    return coordinator, document
