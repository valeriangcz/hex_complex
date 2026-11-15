#!/usr/bin/env python3
"""Dispatch trajectory viewing to the original-style static or animation viewer."""
import argparse
from dataclasses import replace
from pathlib import Path
import numpy as np

from hex_se3.runtime import build_hex_state, load_scene_config
from map_robot_voxles.cache_tools import resolve_path
from map_robot_voxles.resources import package_resource


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(package_resource('hex_se3', 'config/complex_surface4.yaml')))
    parser.add_argument('--path', required=True)
    parser.add_argument('--mode', choices=('static', 'animation'), default='animation')
    parser.add_argument('--source', choices=('teleop', 'waypoints', 'dense', 'bezier'), default='dense')
    parser.add_argument('--indices', nargs='*', type=int)
    parser.add_argument('--landing-stride', '--stride', dest='stride', type=int)
    parser.add_argument('--curve-dt', type=float, default=0.05)
    parser.add_argument('--speed', type=float, default=1.0)
    args = parser.parse_args()
    from hex_se3.se3_animation_vis import SE3PathVisualizer, load_se3_path_json, PoseSeries
    document = load_se3_path_json(args.path)
    if args.source in ('teleop', 'waypoints'):
        series = PoseSeries(np.arange(len(document.waypoints), dtype=float), document.waypoints)
    elif args.source == 'bezier':
        series = document.sample_bezier(args.curve_dt)
    else:
        series = document.dense
    state, map_config, _ = build_hex_state(load_scene_config(args.config))
    map_stl = resolve_path(map_config, map_config['source_mesh'])
    if args.mode == 'animation':
        if args.source in ('teleop', 'waypoints'):
            document = replace(document, dense=series, bezier_segments=())
        viewer = SE3PathVisualizer(state, document, map_stl=map_stl,
                                  source='bezier' if args.source == 'bezier' else 'dense',
                                  landing_stride=args.stride if args.stride is not None else 10,
                                  curve_dt=args.curve_dt, speed=args.speed)
        viewer.run()
    else:
        from hex_se3 import se3_static_vis as static
        indices = args.indices if args.indices is not None else sorted({0, len(series.poses)-1})
        if any(index < 0 or index >= len(series.poses) for index in indices):
            parser.error('--indices contains an out-of-range pose index')
        path = static.StaticPath(Path(args.path),
                                 {'waypoints': document.waypoints, 'selected': series.poses},
                                 'waypoints', series.poses,
                                 tuple(('selected', index) for index in dict.fromkeys(indices)),
                                 8.0, 0.20, 'yellow', 20.0)
        static.MAP_STL = map_stl
        if args.stride is not None:
            static.ENV_LANDING_STRIDE = args.stride
        static.run_viewer(static.SE3StaticVisualizer((path,), state))


if __name__ == '__main__':
    main()
