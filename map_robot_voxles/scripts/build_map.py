#!/usr/bin/env python3
"""Build a point map and environment ESDF from an explicit YAML config."""

import argparse
from pathlib import Path

from map_robot_voxles import EnvPointsVoxels, PointMap
from map_robot_voxles.cache_tools import load_config, resolve_path, write_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    source = resolve_path(config, config["source_mesh"])
    point_map = resolve_path(config, config["point_map_file"])
    env_cache = resolve_path(config, config["env_cache_file"])
    manifest = resolve_path(config, config["manifest_file"])
    existing = [path for path in (point_map, env_cache, manifest) if path.exists()]
    if existing and not args.force:
        raise FileExistsError(
            "outputs already exist; pass --force to rebuild explicitly: {}".format(existing)
        )
    point_map.parent.mkdir(parents=True, exist_ok=True)
    map_parameters = config.get("point_map", {})
    PointMap(str(source), saved_file=str(point_map), **map_parameters)
    EnvPointsVoxels(
        str(point_map), voxel_scale=float(config.get("env_voxel_scale", 0.01)),
        env_cache_file=env_cache, rebuild=True,
    )
    write_manifest(
        manifest, "map", config,
        {"source_mesh": source},
        {"point_map": point_map, "env_esdf": env_cache},
    )
    print(manifest)


if __name__ == "__main__":
    main()
