#!/usr/bin/env python3
"""Run the real single-environment backend without ROS or ExpertComplex."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
for package in ("hex_kinematic", "hex_isaacgym"):
    sys.path.insert(0, str(WORKSPACE_ROOT / "src" / package / "src"))

from hex_isaacgym import BackendConfig, ExpertCommand, InitialState  # noqa: E402
from hex_isaacgym.backend import IsaacGymBackend  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--resets", type=int, default=2)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    baseline_file = WORKSPACE_ROOT / "data/migration_baseline/baseline_arrays.npz"
    robot_urdf = WORKSPACE_ROOT / "src/hex_kinematic/resources/hex_magnetic/urdf/hex_all.urdf"
    map_mesh = WORKSPACE_ROOT / "src/map_robot_voxles/resources/maps/complex_surface4.STL"
    config = BackendConfig(
        robot_urdf=robot_urdf,
        map_mesh=map_mesh,
        isaacgym_python=Path(
            "/home/val/Downloads/IsaacGym_Preview_4_Package/isaacgym/python"
        ),
        device=args.device,
    )
    config.validate()
    with np.load(str(baseline_file), allow_pickle=False) as baseline:
        initial = InitialState(baseline["pose_0_W_T_R"], baseline["expert_q_init"])
        command = ExpertCommand(
            baseline["expert_q_init"],
            np.zeros((6, 3), dtype=np.float64),
            np.ones(6, dtype=bool),
        )

    samples = []
    with IsaacGymBackend(
        config, headless=not args.viewer, initial_state=initial
    ) as backend:
        rigid_positions = backend.get_rigid_body_positions()
        unique_body_positions = len(np.unique(np.round(rigid_positions, 5), axis=0))
        if backend.num_bodies != 55 or backend.num_dofs != 42:
            raise AssertionError("unexpected articulated asset counts")
        if unique_body_positions < 30:
            raise AssertionError(
                "robot links appear collapsed: {} unique positions".format(
                    unique_body_positions
                )
            )
        for reset_index in range(args.resets):
            reset_state = backend.reset(initial)
            np.testing.assert_allclose(reset_state.W_T_R, initial.W_T_R, atol=2e-6)
            np.testing.assert_allclose(reset_state.q, initial.q, atol=2e-6)
            for _ in range(args.steps):
                state = backend.step(command)
                if not all(
                    np.isfinite(value).all()
                    for value in (
                        state.W_T_R,
                        state.q,
                        state.q_dot,
                        state.q_torque,
                        state.adhesion_force,
                        state.contact_forces,
                    )
                ):
                    raise RuntimeError("backend returned non-finite state")
            samples.append({
                "reset": reset_index,
                "sim_time": state.sim_time,
                "run_elapsed": state.run_elapsed,
                "root_position": state.W_T_R[:3, 3].tolist(),
                "max_abs_q_dot": float(np.max(np.abs(state.q_dot))),
                "adhesion_force": state.adhesion_force.tolist(),
                "contact_force_norm": np.linalg.norm(state.contact_forces, axis=1).tolist(),
            })
    report = {
        "status": "PASS",
        "device": args.device,
        "headless": not args.viewer,
        "physics_dt": config.physics_dt,
        "decimation": config.decimation,
        "steps_per_reset": args.steps,
        "resets": args.resets,
        "asset_body_count": backend.num_bodies,
        "asset_dof_count": backend.num_dofs,
        "unique_initial_body_positions": unique_body_positions,
        "samples": samples,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
