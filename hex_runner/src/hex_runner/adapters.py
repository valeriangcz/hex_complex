"""Expert lifecycle bound to measured ROS feedback, never a physics object."""
import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3
from hex_expert import ExpertClimb, ExpertComplex
from hex_kinematic.sim_info import write_sim_info


class ExpertAdapter:
    def __init__(self, config):
        self.config = config
        initial = config.initial_state
        common = dict(robot_urdf=config.robot_urdf,groups=initial.groups,
                      stance_group=initial.stance_group,control_dt=config.control_dt,
                      motor_torque_limits=config.motor_torque_limit,
                      ik_branch=initial.ik_branch,magnetic=initial.magnetic,
                      **config.expert_parameters)
        if config.expert_type == "complex":
            self.expert = ExpertComplex(resources=config.resources,
                                       initial_pose=SE3(initial.W_T_R),**common)
        else:
            self.expert = ExpertClimb(motor_positions=initial.motor_positions,
                                     foot_positions=initial.foot_positions,**common)
        self.switch_count = 0
        self.export_sim_info()

    def export_sim_info(self):
        initial = self.config.initial_state
        path = (self.expert.path_se3 if self.config.expert_type == "complex" else [])
        write_sim_info(self.config.to_sim_info,dict(
            expert_type=self.config.expert_type,robot_urdf=str(self.config.robot_urdf),
            initial_state=dict(
                W_T_R=initial.W_T_R.tolist(),q=self.expert.q_des.tolist(),
                magnetic=self.expert.adhesions.tolist(),motor_mode=initial.motor_mode,
                servo_enabled=initial.servo_enabled),
            reference_path=[pose.t.tolist() for pose in path]))

    def reset(self, feedback):
        initial = self.config.initial_state
        if self.config.expert_type == "complex":
            self.expert.SetInitBySE3(ik_branch=initial.ik_branch,initial_pose=SE3(initial.W_T_R))
        else:
            self.expert.SetInit(self.expert.initial_q)
        self.expert.adhesions[:] = initial.magnetic
        self.switch_count = 0
        self.export_sim_info()
        return (self.expert.q_des.copy(),self.expert.tau_ff.copy(),
                self.expert.adhesions.copy(),np.ones(6,dtype=bool))

    def compute(self, feedback, velocity):
        old_group = self.expert.stance_group_index
        state = (feedback.gravity_direction_body.copy() if self.config.expert_type == "climb"
                 else SE3(feedback.pose, check=False))
        arguments = (state, feedback.q.copy(), feedback.tau.copy(), feedback.attached.copy())
        result = self.expert.RequestSingleStep(*(arguments + (velocity,) if self.config.expert_type == "climb" else arguments))
        self.switch_count += int(old_group != self.expert.stance_group_index)
        return result

    def failure_reason(self, feedback):
        if self.config.expert_type == "complex":
            distance = np.min(np.linalg.norm(np.array([pose.t for pose in self.expert.path_se3]) - feedback.pose[:3, 3], axis=1))
            if distance > self.config.failure_path_distance:
                return "base is %.3f m from reference path" % distance
        return ""

    def is_complete(self, feedback):
        if self.config.expert_type != "complex":
            return False
        path = self.expert.path_se3
        delta = path[-1].inv().A @ feedback.pose
        reached = len(path) == 1 or self.expert._GetCloestIndex(SE3(feedback.pose, check=False)) == len(path) - 1
        no_swing = not self.expert.B_e_traj or np.all(
            self.expert.q_traj_index >= np.maximum(self.expert.B_e_traj_len - 1, 0))
        return (reached and no_swing and np.linalg.norm(delta[:3, 3]) <= self.config.completion_translation
                and Rotation.from_matrix(delta[:3, :3]).magnitude() <= self.config.completion_rotation)

    def diagnostics(self):
        if self.expert is None:
            return {}
        value = {"gaits": self.expert.gaits.tolist(), "adhesions": self.expert.adhesions.tolist(),
                 "gait_switch_count": self.switch_count}
        if self.config.expert_type == "complex":
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
