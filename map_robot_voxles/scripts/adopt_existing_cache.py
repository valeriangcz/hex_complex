#!/usr/bin/env python3
"""Write provenance for copied legacy caches after validating required files."""

import argparse

from map_robot_voxles.cache_tools import load_config, resolve_path, write_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--kind", required=True, choices=("map", "robot"))
    args = parser.parse_args()
    config = load_config(args.config)
    if args.kind == "map":
        sources = {"source_mesh": resolve_path(config, config["source_mesh"])}
        outputs = {
            "point_map": resolve_path(config, config["point_map_file"]),
            "env_esdf": resolve_path(config, config["env_cache_file"]),
        }
    else:
        sources = {
            "urdf": resolve_path(config, config["urdf_file"]),
            "body_leg_mesh": resolve_path(config, config["body_leg_mesh"]),
            "body_env_mesh": resolve_path(config, config["body_env_mesh"]),
        }
        cache_dir = resolve_path(config, config["cache_dir"])
        outputs = {
            "leg_voxels": cache_dir / "leg_voxels_info.npz",
            "body_voxels": cache_dir / "body_voxels_info.npz",
            "robot_reachable": cache_dir / "robot_reachable_legs.npz",
        }
    manifest = resolve_path(config, config["manifest_file"])
    write_manifest(
        manifest, args.kind, config, sources, outputs,
        status="legacy-adopted-unverified-parameters",
    )
    print(manifest)


if __name__ == "__main__":
    main()
