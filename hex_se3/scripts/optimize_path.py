#!/usr/bin/env python3
"""Run shortcut and NumPy SE(3) Bezier optimization with explicit resources."""

import argparse
from datetime import datetime
from pathlib import Path

from hex_se3.runtime import build_hex_state, load_scene_config, scene_path
from hex_se3.se3_post_process import OptCfg, PostProcess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--path", required=True)
    parser.add_argument("--backend", choices=("numpy", "numpy-cma", "torch"), default="numpy")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--shortcut-only", action="store_true")
    parser.add_argument("--output-dir")
    args = parser.parse_args()
    config = load_scene_config(args.config)
    state, map_config, robot_config = build_hex_state(config)
    options = OptCfg()
    for key, value in config.get("optimizer", {}).items():
        if not hasattr(options, key):
            raise ValueError("unknown optimizer option: {}".format(key))
        setattr(options, key, value)
    output_dir = args.output_dir or str(scene_path(config, "output_dir"))
    if args.backend == "torch":
        import torch
        from map_robot_voxles.cache_tools import resolve_path
        from map_robot_voxles.hex_state_torch import HexStateTorch
        from hex_se3.se3_post_process_torch import OptCfgTorch, PostProcessTorch
        torch_options = OptCfgTorch()
        for key, value in config.get("torch_optimizer", {}).items():
            if not hasattr(torch_options, key):
                raise ValueError("unknown Torch optimizer option: {}".format(key))
            setattr(torch_options, key, value)
        dtype = torch.float64 if torch_options.torch_precision == "float64" else torch.float32
        scorer = HexStateTorch.from_files(
            resolve_path(map_config, map_config["point_map_file"]),
            resolve_path(robot_config, robot_config["cache_dir"]),
            env_esdf_file=resolve_path(map_config, map_config["env_cache_file"]),
            device=args.device, dtype=dtype,
            pose_chunk_size=torch_options.torch_pose_chunk_size,
        )
        post = PostProcessTorch(
            args.path, scorer, torch_options, output_dir=output_dir,
        )
    else:
        post = PostProcess(args.path, state, options, output_dir=output_dir)
    post.ShortCutPath()
    post.GetCtrlPoes()
    if args.shortcut_only:
        output = Path(output_dir).expanduser().resolve() / (
            "shortcut_initial_se3_path_{}.json".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
        )
        print(post.WriteJson(str(output)))
    elif args.backend in ("numpy-cma", "torch"):
        print(post.Optimize_CMA_ES().output_path)
    else:
        print(post.Optimize().output_path)


if __name__ == "__main__":
    main()
