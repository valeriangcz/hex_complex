"""Expert lifecycle bound to measured ROS feedback, never a physics object."""
import numpy as np
from copy import deepcopy
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
            manual = config.initialization_method == 'set_init'
            self.expert = ExpertComplex(resources=config.resources,
                                       initial_pose=None if manual or initial.W_T_R is None else SE3(initial.W_T_R),
                                       initial_waypoint_index=None if manual else initial.waypoint_index,**common)
            if manual:
                from hex_expert.initial_state import climb_initial_joints
                self.expert.q3_branches[:] = initial.ik_branch
                self.manual_initial_q = climb_initial_joints(self.expert.kin,motor_positions=initial.motor_positions,
                    foot_positions=initial.foot_positions,ik_branch=initial.ik_branch)
                self.expert.SetInit(self.manual_initial_q,initial_pose=None if initial.W_T_R is None else SE3(initial.W_T_R),
                    initial_waypoint_index=initial.waypoint_index)
                self.expert.adhesions[:] = initial.magnetic
        else:
            controller = ExpertQuadClimb if config.expert_type == "quad_climb" else ExpertClimb
            if config.expert_type == 'climb':
                common['operating_mode'] = config.operating_mode
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

    def prepare_initialization(self):
        """Compute one final target, independently of how runner will reach it."""
        initial = self.config.initial_state
        if self.config.expert_type == "complex":
            if self.config.initialization_method == 'set_init':
                self.expert.q3_branches[:] = initial.ik_branch
                self.expert.SetInit(self.manual_initial_q,
                    initial_pose=None if initial.W_T_R is None else SE3(initial.W_T_R),
                    initial_waypoint_index=initial.waypoint_index)
            else:
                self.expert.SetInitBySE3(ik_branch=initial.ik_branch,
                    initial_pose=None if initial.W_T_R is None else SE3(initial.W_T_R),
                    initial_waypoint_index=initial.waypoint_index)
        else:
            self.expert.SetInit(self.expert.initial_q)
        self.expert.adhesions[:] = initial.magnetic
        self.switch_count = 0
        self.export_sim_info()
        # Only small control-state fields are copied; models, caches and paths stay shared.
        fields = (
            'q_des','B_e_cur','B_landing_n','B_support_n','landing_point','W_landing_points',
            'q3_branches','gaits','last_gaits','adhesions','initial_pose',
            '_foot_target_leg','_previous_contact_feet','static_contact_forces_R',
            '_tau_ff_support_mask','_tau_ff_transition_start','_tau_ff_transition_remaining',
            '_feedforward_support_mask','_feedforward_blend_start','_torque_blend_remaining',
        )
        self._initial_target_state = {name:deepcopy(getattr(self.expert,name))
                                      for name in fields if hasattr(self.expert,name)}
        return (self.expert.q_des.copy(),self.expert.tau_ff.copy(),
                self.expert.adhesions.copy(),np.ones(len(self.config.leg_names),dtype=bool))

    def initialization_lift_target(self):
        from hex_expert.initial_state import climb_initial_joints
        points = np.array([[.18,.08,.04],[.18,-.08,.04],[.19,0.,.04],
                           [.18,-.08,.04],[.18,.08,.04],[.19,0.,.04]])
        return climb_initial_joints(self.expert.kin,foot_positions=points,
                                   ik_branch=self.config.initial_state.ik_branch)

    def initialization_feedforward(self,feedback,adhesions):
        e = self.expert
        if self.config.expert_type == 'climb' and self.config.operating_mode == 'suspended':
            e.tau_ff.fill(0.0)
            return e.tau_ff.copy()
        fk = e.kin.ForwardKinAll(feedback.q)
        e.B_e_cur[:] = fk['B_foot'].T
        e.adhesions[:] = adhesions
        if self.config.expert_type == 'complex':
            tau = e._ComputeQuasiStaticTauFF(SE3(feedback.pose,check=False),feedback.q[:,:3],fk['R_com'],feedback.attached)
        else:
            direction = feedback.gravity_direction_body
            tau = e._ComputeQuasiStaticTauFF(9.81*direction/np.linalg.norm(direction),feedback.q[:,:3],fk['R_com'])
        return tau.copy()

    def finish_initialization(self,feedback,command):
        # Commit exactly the target prepared before interpolation, including SE3 normals.
        # Re-running a solver here could silently select different landing points.
        for name,value in self._initial_target_state.items():
            setattr(self.expert,name,deepcopy(value))
        # Let the expert's existing support-transition blend start at the held torque.
        if self.config.expert_type == 'climb':
            self.expert.SeedFeedforward(command.tau)
        else:
            self.expert.tau_ff[:] = command.tau

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

    def planning_snapshot(self):
        """Read-only native complex planning state; recorder owns copied snapshots."""
        e = self.expert
        return dict(landing_leg=e.landing_point.T, landing_world=e.W_landing_points.T,
                    stance=e.gaits, trajectory_index=e.q_traj_index,
                    feet=e.B_e_traj, body=e.interp_path_T,
                    follow_path_index=e.follow_path_index,
                    swing_failures=getattr(e, '_last_swing_planning_failures', {}))
