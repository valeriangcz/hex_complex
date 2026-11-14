"""Expert lifecycle bound to measured ROS feedback, never a physics object."""
import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3
from hex_expert import ExpertClimb, ExpertComplex
from hex_kinematic import Kinematic


class ExpertAdapter:
    def __init__(self, config):
        self.config = config
        self.kin = Kinematic(urdf_path=config.robot_urdf)
        self.path = ()
        self.expert = None
        self.switch_count = 0
        if config.expert_type == "complex":
            from hex_se3 import load_trajectory
            from map_robot_voxles import HexState
            resources = config.resources
            document = load_trajectory(resources["se3_path"])
            self.path = tuple(document.dense.poses if document.dense is not None else document.waypoints)
            self.hex_state = HexState(self.kin, **{name: resources[name] for name in (
                "robot_cache_dir", "point_map_file", "env_cache_file", "body_leg_mesh", "body_env_mesh")})

    def reset(self, feedback):
        cfg = self.config
        if cfg.expert_type == "climb":
            self.expert = ExpertClimb(self.kin, control_dt=cfg.control_dt,
                                      motor_torque_limits=cfg.motor_torque_limit, **cfg.expert_parameters)
            self.expert.SetInit(cfg.initial_state.q.copy())
        else:
            self.expert = ExpertComplex(self.kin, self.hex_state, control_dt=cfg.control_dt, **cfg.expert_parameters)
            self.expert.motor_torque_limits[:] = cfg.motor_torque_limit
            self.expert.LoadSE3(list(self.path))
            self.expert.SetInit(cfg.initial_state.q.copy())
        from hex_kinematic.initial_state import STANCE_GROUPS
        initial = cfg.initial_state
        self.expert.stance_group_index = initial.stance_group
        self.expert.gaits[:] = [leg in STANCE_GROUPS[initial.stance_group] for leg in self.expert.leg_names]
        if hasattr(self.expert, "last_gaits"):
            self.expert.last_gaits[:] = self.expert.gaits
        self.expert.adhesions[:] = initial.magnetic
        if hasattr(self.expert, "q3_branches"):
            self.expert.q3_branches[:] = initial.ik_branch
        self.switch_count = 0

    def compute(self, feedback, velocity):
        old_group = self.expert.stance_group_index
        state = (feedback.gravity_direction_body.copy() if self.config.expert_type == "climb"
                 else SE3(feedback.pose, check=False))
        arguments = (state, feedback.q.copy(), feedback.tau.copy(), feedback.attached.copy())
        result = self.expert.RequestSingleStep(*(arguments + (velocity,) if self.config.expert_type == "climb" else arguments))
        self.switch_count += int(old_group != self.expert.stance_group_index)
        return result

    def failure_reason(self, feedback):
        if self.path:
            distance = np.min(np.linalg.norm(np.array([pose.t for pose in self.path]) - feedback.pose[:3, 3], axis=1))
            if distance > self.config.failure_path_distance:
                return "base is %.3f m from reference path" % distance
        return ""

    def is_complete(self, feedback):
        if not self.path:
            return False
        delta = self.path[-1].inv().A @ feedback.pose
        reached = len(self.path) == 1 or self.expert._GetCloestIndex(SE3(feedback.pose, check=False)) == len(self.path) - 1
        no_swing = not self.expert.B_e_traj or np.all(
            self.expert.q_traj_index >= np.maximum(self.expert.B_e_traj_len - 1, 0))
        return (reached and no_swing and np.linalg.norm(delta[:3, 3]) <= self.config.completion_translation
                and Rotation.from_matrix(delta[:3, :3]).magnitude() <= self.config.completion_rotation)

    def diagnostics(self):
        if self.expert is None:
            return {}
        value = {"gaits": self.expert.gaits.tolist(), "adhesions": self.expert.adhesions.tolist(),
                 "gait_switch_count": self.switch_count}
        if self.path:
            value["follow_path_index"] = self.expert.follow_path_index
        else:
            value.update(
                leg_names=list(self.expert.leg_names),
                estimated_contact_force=self.expert.estimated_contact_force.tolist(),
                normal_foot_velocity=self.expert.normal_foot_velocity.tolist(),
                swing_reached=self.expert._swing_reached.tolist(),
                contact_count=self.expert.contact_count.tolist(),
                foot_target_leg=self.expert._foot_target_leg.tolist(),
                foot_measured_leg=self.expert.B_e_cur.T.tolist(),
                workspace_ok=self.expert._CheckWorkspace(self.expert._foot_target_leg).tolist(),
                clearance_ok=self.expert._CheckLegClearance(self.expert._foot_target_leg).tolist())
        return value
