#!/usr/bin/env python3
"""Inspect a migrated map, robot workspace, or whole-body feasibility scene."""

import argparse
from pathlib import Path

import numpy as np
from spatialmath import SE3

from hex_kinematic import Kinematic
from map_robot_voxles import HexState
from map_robot_voxles.cache_tools import load_config, resolve_path


def make_state(map_config_file, robot_config_file):
    map_config = load_config(map_config_file)
    robot_config = load_config(robot_config_file)
    robot_dir = resolve_path(robot_config, robot_config["cache_dir"])
    model = Kinematic(urdf_path=resolve_path(robot_config, robot_config["urdf_file"]))
    return HexState(
        model,
        robot_cache_dir=robot_dir,
        point_map_file=resolve_path(map_config, map_config["point_map_file"]),
        env_cache_file=resolve_path(map_config, map_config["env_cache_file"]),
        body_leg_mesh=resolve_path(robot_config, robot_config["body_leg_mesh"]),
        body_env_mesh=resolve_path(robot_config, robot_config["body_env_mesh"]),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-config", required=True)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument("--mode", choices=("map", "robot", "feasibility"), default="feasibility")
    parser.add_argument("--xyz", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--rpy-deg", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--off-screen", action="store_true")
    parser.add_argument("--screenshot", type=Path)
    args = parser.parse_args()

    import pyvista as pv
    from map_robot_voxles.visualization import (
        FeasibilityRenderer, MapRenderer, RobotVoxelRenderer,
    )

    if args.off_screen or args.screenshot is not None:
        try:
            pv.start_xvfb()
        except OSError as exc:
            raise RuntimeError(
                "off-screen rendering needs a working DISPLAY or Xvfb; "
                "install Xvfb or run from a graphical session"
            ) from exc
    state = make_state(args.map_config, args.robot_config)
    transform = SE3.Trans(args.xyz) * SE3.RPY(np.deg2rad(args.rpy_deg), order="xyz")
    plotter = pv.Plotter(off_screen=args.off_screen or args.screenshot is not None)
    if args.mode == "map":
        MapRenderer(plotter).render(state.env_pointsmap_voxels, stride=args.stride)
    elif args.mode == "robot":
        RobotVoxelRenderer(plotter).render(
            state.robot_voxels, transform, leg_stride=args.stride,
        )
    else:
        result = state.RobotFeasiCheck(transform)
        FeasibilityRenderer(plotter).render(
            state, transform, result,
            env_stride=args.stride, leg_voxel_stride=args.stride,
        )
    plotter.show_axes()
    plotter.show_grid()
    plotter.show(screenshot=str(args.screenshot) if args.screenshot else None)


if __name__ == "__main__":
    main()
