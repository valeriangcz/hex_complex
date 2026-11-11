#!/usr/bin/env python3
"""Run the real plane climbing loop and save contact-estimation diagnostics."""

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
for package in ("hex_isaacgym", "hex_expert", "hex_kinematic", "map_robot_voxles", "hex_se3"):
    sys.path.insert(0, str(WORKSPACE / "src" / package / "src"))
os.environ["ROS_PACKAGE_PATH"] = str(WORKSPACE / "src") + os.pathsep + os.environ.get("ROS_PACKAGE_PATH", "")

from hex_isaacgym.coordinator import RunPhase
from hex_isaacgym.runtime import SceneConfig, build_coordinator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    scene = SceneConfig.from_yaml(WORKSPACE / "src/hex_isaacgym/config/expert_climb_scene.yaml")
    coordinator, document = build_coordinator(scene, device=args.device, headless=not args.viewer)
    samples = []
    try:
        assert document is None and not coordinator.path
        state = coordinator.tick()
        np.testing.assert_allclose(state.q, scene.initial_state.q, atol=2e-6)
        np.testing.assert_allclose(coordinator.expert.motor_torque_limits,
                                   coordinator.backend.motor_torque_limits)
        initial = state.W_T_R[:3, 3].copy()
        for step in range(args.steps):
            state = coordinator.tick()
            if step % 50 == 0 or coordinator.phase == RunPhase.FAILED:
                samples.append({
                    "time": state.run_elapsed, "position": state.W_T_R[:3, 3].tolist(),
                    "estimated_pressure": coordinator.expert.estimated_contact_force.tolist(),
                    "contact_force_norm": np.linalg.norm(state.contact_forces, axis=1).tolist(),
                    **coordinator.adapter.diagnostics(),
                })
            if coordinator.phase == RunPhase.FAILED:
                break
        switches = coordinator.adapter.switch_count
        displacement = state.W_T_R[:3, 3] - initial
        failure = coordinator.failure_reason
        passed = coordinator.phase == RunPhase.RUNNING and switches >= 2 and displacement[0] > 0.0
        # Verify command persistence/hold and reset independently of gait success.
        coordinator.set_velocity_command([0.0, 0.0, 0.0])
        coordinator.tick()  # pending command has one control-period latency
        held = coordinator.expert.q_des.copy()
        for _ in range(10):
            coordinator.tick()
        np.testing.assert_array_equal(coordinator.expert.q_des, held)
        coordinator.request_reset("climb smoke test")
        reset = coordinator.tick()
        np.testing.assert_allclose(reset.W_T_R, scene.initial_state.W_T_R, atol=2e-6)
        np.testing.assert_allclose(reset.q, scene.initial_state.q, atol=2e-6)
        np.testing.assert_allclose(coordinator.adapter.command, scene.command)
        report = {"status": "PASS" if passed else "FAIL", "device": args.device,
                  "switches": switches, "displacement": displacement.tolist(),
                  "failure_reason": failure, "samples": samples}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({key: value for key, value in report.items() if key != "samples"}, indent=2))
        if not passed:
            raise AssertionError("climb did not complete two support switches and move forward")
    finally:
        coordinator.close()


if __name__ == "__main__":
    main()
