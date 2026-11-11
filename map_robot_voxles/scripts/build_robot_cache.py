#!/usr/bin/env python3
"""Build robot workspace and collision caches from an explicit config."""

import argparse

from hex_kinematic import Kinematic
from map_robot_voxles import RoboVoxels
from map_robot_voxles.cache_tools import load_config, resolve_path, write_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    urdf = resolve_path(config, config["urdf_file"])
    body_leg = resolve_path(config, config["body_leg_mesh"])
    body_env = resolve_path(config, config["body_env_mesh"])
    cache_dir = resolve_path(config, config["cache_dir"])
    manifest = resolve_path(config, config["manifest_file"])
    outputs = {
        "leg_voxels": cache_dir / "leg_voxels_info.npz",
        "body_voxels": cache_dir / "body_voxels_info.npz",
        "robot_reachable": cache_dir / "robot_reachable_legs.npz",
    }
    existing = [path for path in tuple(outputs.values()) + (manifest,) if path.exists()]
    if existing and not args.force:
        raise FileExistsError(
            "outputs already exist; pass --force to rebuild explicitly: {}".format(existing)
        )
    model = Kinematic(urdf_path=urdf)
    robot = config.get("robot_voxels", {})
    RoboVoxels(
        model,
        leg_xyz_range=robot.get("leg_xyz_range", [[-0.14, 0.38], [-0.38, 0.38], [-0.4, 0.4]]),
        body_xyz_range=robot.get("body_xyz_range", [[-0.4, 0.4], [-0.5, 0.5], [-0.2, 0.2]]),
        voxel_scale=float(robot.get("voxel_scale", 0.01)),
        sphere_scale=float(robot.get("sphere_scale", 0.017453292519943295)),
        ankle_collision_radius=float(robot.get("ankle_collision_radius", 0.05)),
        leg_body_end_clearance=float(robot.get("leg_body_end_clearance", 0.03)),
        cache_dir=cache_dir,
        body_leg_mesh=body_leg,
        body_env_mesh=body_env,
        rebuild=True,
    )
    write_manifest(
        manifest, "robot", config,
        {"urdf": urdf, "body_leg_mesh": body_leg, "body_env_mesh": body_env},
        outputs,
    )
    print(manifest)


if __name__ == "__main__":
    main()
