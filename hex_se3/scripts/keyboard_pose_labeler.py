#!/usr/bin/env python3
"""Launch the migrated PyVista keyboard pose labeler."""

import argparse
import numpy as np
from spatialmath import SE3

from map_robot_voxles.cache_tools import resolve_path
from hex_se3.keyboard_pose_labeler import KeyboardPoseLabeler
from hex_se3.runtime import build_hex_state, load_scene_config, scene_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument("--xyz", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--rpy-deg", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--landing-stride", type=int, default=1)
    args = parser.parse_args()
    config = load_scene_config(args.config)
    state, map_config, _ = build_hex_state(config)
    output_dir = args.output_dir or str(scene_path(config, "output_dir"))
    pose = SE3.Trans(args.xyz) * SE3.RPY(np.deg2rad(args.rpy_deg), order="xyz")
    app = KeyboardPoseLabeler(
        state, init_pose=pose, landing_stride=args.landing_stride,
        output_dir=output_dir,
        environment_stl_path=str(resolve_path(map_config, map_config["source_mesh"])),
    )
    app.run()


if __name__ == "__main__":
    main()
