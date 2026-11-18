"""Expert lifecycle bound to measured ROS feedback, never a physics object."""
import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3
from hex_expert import ExpertClimb, ExpertComplex, ExpertQuadClimb
from hex_kinematic.sim_info import write_sim_info


class ExpertAdapter:
    def __init__(self, config):
        self.config = config
        initial = config.initial_state
        common = dict(robot_urdf=config.robot_urdf,control_dt=config.control_dt,
                      motor_torque_limits=config.motor_torque_limit,
                      ik_branch=initial.ik_branch,magnetic=initial.magnetic,
                      **config.expert_parameters)
        common.update(dict(sequence=initial.sequence) if config.expert_type == "quad_climb"
                      else dict(groups=initial.groups, stance_group=initial.stance_group))
        if config.expert_type == "complex":
            self.expert = ExpertComplex(resources=config.resources,
                                       initial_pose=None if initial.W_T_R is None else SE3(initial.W_T_R),
                                       initial_waypoint_index=initial.waypoint_index,**common)
        else:
            controller = ExpertQuadClimb if config.expert_type == "quad_climb" else ExpertClimb
            self.expert = controller(motor_positions=initial.motor_positions,
                                     foot_positions=initial.foot_positions,**common)
        self.switch_count = 0
        self.export_sim_info()

    def export_sim_info(self):
        initial = self.config.initial_state
        path = (self.expert.path_se3 if self.config.expert_type == "complex" else [])
        W_T_R = (self.expert.initial_pose.A if self.config.expert_type == "complex"
                 else initial.W_T_R)
        write_sim_info(self.config.to_sim_info,dict(
            leg_names=list(self.config.leg_names),expert_type=self.config.expert_type,robot_urdf=str(self.config.robot_urdf),
            initial_state=dict(
                W_T_R=W_T_R.tolist(),q=self.expert.q_des.tolist(),
                magnetic=self.expert.adhesions.tolist(),
                servo_enabled=initial.servo_enabled),
            reference_path=[pose.t.tolist() for pose in path]))

    def reset(self, feedback):
        initial = self.config.initial_state
        if self.config.expert_type == "complex":
            self.expert.SetInitBySE3(ik_branch=initial.ik_branch,
                initial_pose=None if initial.W_T_R is None else SE3(initial.W_T_R),
                initial_waypoint_index=initial.waypoint_index)
        else:
            self.expert.SetInit(self.expert.initial_q)
        self.expert.adhesions[:] = initial.magnetic
        self.switch_count = 0
        self.export_sim_info()
        return (self.expert.q_des.copy(),self.expert.tau_ff.copy(),
                self.expert.adhesions.copy(),np.ones(len(self.config.leg_names),dtype=bool))

    def compute(self, feedback, velocity):
        sequential = self.config.expert_type == "quad_climb"
        old_step = self.expert.swing_count if sequential else self.expert.stance_group_index
        state = (feedback.gravity_direction_body.copy() if self.config.expert_type in ("climb", "quad_climb")
                 else SE3(feedback.pose, check=False))
        arguments = (state, feedback.q.copy(), feedback.tau.copy(), feedback.attached.copy())
        result = self.expert.RequestSingleStep(*(arguments + (velocity,) if self.config.expert_type in ("climb", "quad_climb") else arguments))
        self.switch_count += (self.expert.swing_count - old_step if sequential
                              else int(old_step != self.expert.stance_group_index))
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
        if self.config.expert_type == "quad_climb":
            e = self.expert
            value.update(gait_phase=e.gait_phase, swing_leg_index=e.swing_leg_index,
                         sequence=list(e.sequence), sequence_index=e.sequence_index)
            value["support_moment_residual_R"] = getattr(e, "support_moment_residual_R", np.zeros(3)).tolist()
        return value
