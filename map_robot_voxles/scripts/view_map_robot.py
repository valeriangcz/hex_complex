#!/usr/bin/env python3
"""Inspect maps, robot workspaces, or feasibility with independent resource loading."""
import argparse
from pathlib import Path
import numpy as np
from spatialmath import SE3
from hex_kinematic import Kinematic
from map_robot_voxles import HexState, PointMap, RoboVoxels
from map_robot_voxles.cache_tools import load_config, resolve_path


def load_robot(config_file):
    config = load_config(config_file)
    model = Kinematic(urdf_path=resolve_path(config, config['urdf_file']))
    return RoboVoxels(model, **config.get('robot_voxels', {}),
                      cache_dir=resolve_path(config, config['cache_dir']),
                      body_leg_mesh=resolve_path(config, config['body_leg_mesh']),
                      body_env_mesh=resolve_path(config, config['body_env_mesh']))


def make_state(map_config_file, robot_config_file):
    config = load_config(map_config_file)
    robot = load_robot(robot_config_file)
    return HexState(robot.kin, robot_voxels=robot,
                    point_map_file=resolve_path(config, config['point_map_file']),
                    env_cache_file=resolve_path(config, config['env_cache_file']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-config')
    parser.add_argument('--robot-config')
    parser.add_argument('--mode', choices=('map', 'robot', 'feasibility'), default='feasibility')
    parser.add_argument('--xyz', nargs=3, type=float, default=(0., 0., 0.))
    parser.add_argument('--rpy-deg', nargs=3, type=float, default=(0., 0., 0.))
    parser.add_argument('--stride', type=int)
    parser.add_argument('--off-screen', action='store_true')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args(argv)
    if args.mode in ('map', 'feasibility') and not args.map_config:
        parser.error('--map-config is required for '+args.mode)
    if args.mode in ('robot', 'feasibility') and not args.robot_config:
        parser.error('--robot-config is required for '+args.mode)
    if args.stride is not None and args.stride < 1:
        parser.error('--stride must be positive')
    import pyvista as pv
    from map_robot_voxles.visualization import FeasibilityRenderer, RobotVoxelRenderer
    off_screen = args.off_screen or args.screenshot is not None
    if args.mode == 'map':
        config = load_config(args.map_config)
        point_map = PointMap(str(resolve_path(config, config['point_map_file'])))
        # The saved cache may retain a source-project mesh path.
        point_map.original_file = str(resolve_path(config, config['source_mesh']))
        point_map.file_type = 'mesh'
        plotter = point_map.PlotProcessingStages(show=False, off_screen=off_screen)
    else:
        transform = SE3.Trans(args.xyz) * SE3.RPY(np.deg2rad(args.rpy_deg), order='xyz')
        plotter = pv.Plotter(off_screen=off_screen)
        if args.mode == 'robot':
            RobotVoxelRenderer(plotter).render(load_robot(args.robot_config), transform,
                                              leg_stride=args.stride or 1, style='standalone')
            plotter.show_grid(xtitle='R x (m)', ytitle='R y (m)', ztitle='R z (m)')
        else:
            state = make_state(args.map_config, args.robot_config)
            FeasibilityRenderer(plotter).render(state, transform, state.RobotFeasiCheck(transform),
                                               env_stride=args.stride or 8, leg_voxel_stride=args.stride or 8)
            plotter.show_grid()
        plotter.show_axes()
        plotter.view_isometric()
        plotter.reset_camera()
    try:
        plotter.show(screenshot=str(args.screenshot) if args.screenshot else None,
                     interactive=not off_screen)
    finally:
        plotter.close()


if __name__ == '__main__':
    main()
