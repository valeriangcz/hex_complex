#!/usr/bin/env python3
"""View a teleop or optimized trajectory in static or animated mode."""

import argparse

from hex_se3 import load_trajectory
from hex_se3.path_viewer import PathViewer
from hex_se3.runtime import build_hex_state, load_scene_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--path", required=True)
    parser.add_argument("--mode", choices=("static", "animation"), default="animation")
    parser.add_argument("--source", choices=("teleop", "waypoints", "dense", "bezier"), default="dense")
    parser.add_argument("--indices", nargs="*", type=int)
    parser.add_argument("--stride", type=int)
    parser.add_argument("--curve-dt", type=float)
    parser.add_argument("--speed", type=float)
    args = parser.parse_args()
    config = load_scene_config(args.config)
    state, _, _ = build_hex_state(config)
    viewer_config = config.get("viewer", {})
    viewer = PathViewer(
        state, load_trajectory(args.path), mode=args.mode, source=args.source,
        selected_indices=args.indices,
        stride=args.stride or viewer_config.get("stride", 16),
        curve_dt=args.curve_dt or viewer_config.get("curve_dt", 0.05),
        speed=args.speed or viewer_config.get("speed", 1.0),
    )
    viewer.run()


if __name__ == "__main__":
    main()
