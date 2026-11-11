#!/usr/bin/env python3
"""Exercise the real backend/coordinator reset and closed-loop boundaries."""

import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
existing_ros_path = os.environ.get("ROS_PACKAGE_PATH", "")
os.environ["ROS_PACKAGE_PATH"] = str(WORKSPACE_ROOT / "src") + (
    os.pathsep + existing_ros_path if existing_ros_path else ""
)
for package in ("hex_kinematic", "map_robot_voxles", "hex_se3", "hex_expert", "hex_isaacgym"):
    sys.path.insert(0, str(WORKSPACE_ROOT / "src" / package / "src"))

from hex_isaacgym.coordinator import RunPhase  # noqa: E402
from hex_isaacgym.runtime import SceneConfig, build_coordinator  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    scene = SceneConfig.from_yaml(
        PACKAGE_ROOT / "config/expert_complex_scene.yaml",
        data_root=WORKSPACE_ROOT / "data",
    )
    scene = replace(
        scene,
        settling_steps=2,
        contact_loss_steps=0,
        failure_path_distance=2.0,
    )
    coordinator, document = build_coordinator(scene, device="cpu", headless=True)
    checks = []
    try:
        path_points = coordinator.backend._reference_path_points
        if path_points.shape != (len(coordinator.path), 3):
            raise AssertionError("viewer reference path was not installed")
        np.testing.assert_allclose(
            path_points, np.asarray([pose.t for pose in coordinator.path]), atol=1e-6
        )
        rigid_positions = coordinator.backend.get_rigid_body_positions()
        if len(np.unique(np.round(rigid_positions, 5), axis=0)) < 30:
            raise AssertionError("articulated robot was collapsed before first tick")
        checks.append("articulated startup pose and viewer reference path")

        initial = coordinator.tick()
        coordinator.tick()
        coordinator.tick()
        if coordinator.phase != RunPhase.RUNNING:
            raise AssertionError("coordinator did not enter RUNNING")
        running = coordinator.tick()
        if running.run_elapsed != 0.03:
            raise AssertionError("unexpected closed-loop elapsed time")
        checks.append("real expert command applied after settling")

        coordinator.request_reset("execution reset test")
        reset_state = coordinator.tick()
        np.testing.assert_allclose(reset_state.W_T_R, coordinator.path[0].A, atol=2e-6)
        np.testing.assert_allclose(reset_state.q, coordinator.pending_command.q_des, atol=2e-6)
        checks.append("reset while RUNNING")

        reset_roots = []
        for index in range(10):
            coordinator.request_reset("consecutive reset {}".format(index + 1))
            state = coordinator.tick()
            reset_roots.append(state.W_T_R.copy())
            if state.run_elapsed != 0.0 or np.max(np.abs(state.q_dot)) != 0.0:
                raise AssertionError("reset did not clear run time/velocity")
        for root in reset_roots[1:]:
            np.testing.assert_allclose(root, reset_roots[0], atol=2e-6)
        checks.append("ten consecutive physical/expert resets")

        coordinator.inject_failure("P5 reset coverage")
        if coordinator.phase != RunPhase.FAILED or "injected" not in coordinator.failure_reason:
            raise AssertionError("failure injection was not reported")
        coordinator.request_reset("failure recovery")
        coordinator.tick()
        if coordinator.phase != RunPhase.SETTLING or coordinator.failure_reason:
            raise AssertionError("reset did not recover FAILED state")
        checks.append("injected failure then reset")

        coordinator.path = (coordinator.path[0],)
        coordinator.completion_translation = 0.05
        coordinator.completion_rotation = 0.05
        coordinator.settling_steps = 0
        coordinator.request_reset("single-pose completion")
        coordinator.tick()
        if coordinator.phase != RunPhase.COMPLETED:
            raise AssertionError("single-pose path did not complete")
        coordinator.request_reset("completion recovery")
        coordinator.tick()
        if coordinator.phase != RunPhase.COMPLETED or coordinator.reset_count < 14:
            raise AssertionError("reset after completion failed")
        checks.append("completion then reset")

        report = {
            "status": "PASS",
            "trajectory_format": document.format,
            "trajectory_pose_count": len(document.poses),
            "reset_count": coordinator.reset_count,
            "final_phase": coordinator.phase.value,
            "checks": checks,
            "events": coordinator.events,
        }
    finally:
        coordinator.close()
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

