"""Single-environment NumPy velocity climbing expert.

Gait equations follow hex_lab.controllers.expert_climb.ExpertClimb. Contact
uses CF_Estimate foot-on-surface forces, so pressure has a negative normal
component. Measured motion and consecutive samples reject swing loads.
No simulator is owned.
"""

import math

import numpy as np

from hex_kinematic import Kinematic, LEG_NAMES


class ExpertClimb:
    """Velocity-command climbing in lb/lf/lm/rb/rf/rm order.

    ``kinematic`` must have a URDF mass model. Joint arrays are (6,4), motor
    torques (6,3), and commands are body-frame (vx, vy, yaw_rate). Geometry and
    mass properties come from the supplied Kinematic instance.
    """

    def __init__(
        self, kinematic=None, *, groups, stance_group, control_dt=0.01, motor_torque_limits=27.0,
        robot_urdf=None, motor_positions=None, foot_positions=None, ik_branch=None, magnetic=None,
        contact_force_threshold=7.0,
        contact_confirm_steps=10, contact_velocity_threshold=0.03,
        torque_blend_time=0.1, stance_height=0.08,
        joint_position_gain=40.0,
        magnetic_attach_above=0.85, magnetic_release_below=0.05,
        operating_mode='suspended',
    ):
        self.groups = groups
        self.initial_stance_group = stance_group
        self._InitController(
            kinematic=kinematic,
            control_dt=control_dt,
            motor_torque_limits=motor_torque_limits,
            robot_urdf=robot_urdf,
            motor_positions=motor_positions,
            foot_positions=foot_positions,
            ik_branch=ik_branch,
            magnetic=magnetic,
            contact_force_threshold=contact_force_threshold,
            contact_confirm_steps=contact_confirm_steps,
            contact_velocity_threshold=contact_velocity_threshold,
            torque_blend_time=torque_blend_time,
            stance_height=stance_height,
            joint_position_gain=joint_position_gain,
            magnetic_attach_above=magnetic_attach_above,
            magnetic_release_below=magnetic_release_below)
        if operating_mode not in ('suspended', 'ground'):
            raise ValueError('operating_mode must be suspended or ground')
        self.operating_mode = operating_mode

    def _InitController(
        self, kinematic=None, *, control_dt=0.01, motor_torque_limits=27.0,
        robot_urdf=None, motor_positions=None, foot_positions=None, ik_branch=None, magnetic=None,
        contact_force_threshold=7.0,
        contact_confirm_steps=10, contact_velocity_threshold=0.03,
        torque_blend_time=0.1, stance_height=0.08,
        joint_position_gain=45.0,
        magnetic_attach_above=0.85, magnetic_release_below=0.05,
    ):
        if kinematic is None:
            kinematic = Kinematic(urdf_path=robot_urdf)
        if not isinstance(kinematic, Kinematic):
            raise TypeError("kinematic must be hex_kinematic.Kinematic")
        if not hasattr(kinematic, "total_mass"):
            raise ValueError("kinematic requires a URDF mass model (urdf_path)")
        for name, value in (
            ("control_dt", control_dt),
            ("contact_force_threshold", contact_force_threshold),
            ("contact_velocity_threshold", contact_velocity_threshold),
            ("torque_blend_time", torque_blend_time), ("stance_height", stance_height),
            ("joint_position_gain", joint_position_gain),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be positive and finite".format(name))
        if (isinstance(contact_confirm_steps, bool)
                or not isinstance(contact_confirm_steps, (int, np.integer))
                or contact_confirm_steps < 1):
            raise ValueError("contact_confirm_steps must be a positive integer")
        limits = np.asarray(motor_torque_limits, dtype=np.float64)
        if limits.ndim == 0:
            limits = np.full((kinematic.num_legs, 3), limits.item())
        if limits.shape != (kinematic.num_legs, 3) or not np.isfinite(limits).all() or (limits <= 0).any():
            raise ValueError("motor_torque_limits must be positive finite scalar or (num_legs,3)")

        self.kin = kinematic
        # 四足仍沿用原行为；六足构造函数明确选择悬空或地面模式。
        self.operating_mode = None
        if not 0 <= magnetic_release_below < magnetic_attach_above <= 1:
            raise ValueError("require 0 <= magnetic_release_below < magnetic_attach_above <= 1")
        self.magnetic_attach_above = magnetic_attach_above
        self.magnetic_release_below = magnetic_release_below
        self.leg_names = list(kinematic.leg_names)
        self.num_legs = kinematic.num_legs
        self.dt = float(control_dt)
        self.motor_torque_limits = limits.copy()
        self.contact_force_threshold = float(contact_force_threshold)
        self.contact_confirm_steps = int(contact_confirm_steps)
        self.contact_velocity_threshold = float(contact_velocity_threshold)
        self.stance_height = float(stance_height)
        self.joint_position_gain = float(joint_position_gain)
        self.torque_blend_steps = max(1, round(torque_blend_time / self.dt))
        self.total_mass = float(kinematic.total_mass)
        self._swing_start_leg = self._MakeSwingStartLeg()
        self.Reset()
        self.initial_q = None
        if motor_positions is not None or foot_positions is not None:
            from .initial_state import climb_initial_joints
            self.initial_q = climb_initial_joints(self.kin,motor_positions=motor_positions,
                foot_positions=foot_positions,ik_branch=ik_branch)
            self.SetInit(self.initial_q)
            if magnetic is not None:
                self.adhesions[:] = magnetic

    def _MakeSwingStartLeg(self):
        points = np.zeros((self.num_legs, 3))
        for i, name in enumerate(self.leg_names):
            points[i] = [0.19 if name.endswith("m") else 0.18,
                         0.08 if name in ("lb", "rf") else (-0.08 if name in ("lf", "rb") else 0.), 0.04]
        return points

    def _SolveSupportForces(self, arms, gravity_body):
        if len(arms) < 3:
            return None
        A = np.vstack((np.tile(np.eye(3), (1, len(arms))),
                       np.hstack([self._Skew(arm) for arm in arms])))
        b = np.r_[-self.total_mass * gravity_body, np.zeros(3)]
        if np.linalg.matrix_rank(A) != 6:
            return None
        forces = A.T @ np.linalg.pinv(A @ A.T, rcond=1e-9) @ b
        if not np.isfinite(forces).all() or np.linalg.norm(A @ forces - b) > 1e-5 * max(1., np.linalg.norm(b)):
            return None
        return forces.reshape(-1, 3)

    @staticmethod
    def _array(value, shape, name):
        value = np.asarray(value, dtype=np.float64)
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError("{} must be a finite array with shape {}".format(name, shape))
        return value

    def Reset(self, q_init=None):
        """Clear controller history; optionally anchor to explicit joint angles.

        Without q_init, the next RequestSingleStep anchors to its q_cur input.
        Robot parameters and physical limits are preserved.
        """
        if q_init is not None:
            self._array(q_init, (self.num_legs, 4), "q_init")
        self.stance_group_index = self.initial_stance_group
        self._ResetState(q_init)

    def _ResetState(self, q_init=None):
        """Reset buffers shared by grouped and sequential controllers."""
        if q_init is not None:
            q_init = self._array(q_init, (self.num_legs, 4), "q_init")
        self.gaits = np.zeros(self.num_legs, dtype=bool)
        self.adhesions = np.zeros(self.num_legs, dtype=bool)
        self._swing_reached = np.zeros(self.num_legs, dtype=bool)
        self.contact_count = np.zeros(self.num_legs, dtype=np.int64)
        self.q_des = np.zeros((self.num_legs, 4))
        self._foot_target_leg = np.zeros((self.num_legs, 3))
        self.B_e_cur = np.zeros((3, self.num_legs))
        # Planar climbing uses local +z as the surface outward normal.
        self.B_landing_n = np.zeros((3, self.num_legs))
        self.B_landing_n[2] = 1.0
        self.B_support_n = self.B_landing_n.copy()
        self.estimated_contact_force = np.zeros(self.num_legs)
        self.normal_foot_velocity = np.zeros(self.num_legs)
        self._previous_contact_feet = None
        self.tau_ff = np.zeros((self.num_legs, 3))
        self.static_contact_forces_R = np.zeros((self.num_legs, 3))
        self._feedforward_support_mask = np.zeros(self.num_legs, dtype=bool)
        self._feedforward_blend_start = np.zeros((self.num_legs, 3))
        self._torque_blend_remaining = 0
        self.compute_count = 0
        self.set_init = False
        if q_init is not None:
            self._initialize(q_init)

    def _AnchorTargets(self, q_init):
        self.q_des[:] = q_init
        self.kin.ForwardKin(q_init[:, :3], self._foot_target_leg)
        self.B_e_cur[:] = self._foot_target_leg.T
        self._previous_contact_feet = self._foot_target_leg.copy()
        self.set_init = True

    def _initialize(self, q_init):
        self._AnchorTargets(q_init)
        self.gaits[self.groups[self.stance_group_index]] = True
        self.adhesions[:] = self.gaits

    def SetInit(self, q_init):
        """Reset and anchor gait/foot targets to q_init, returning snapshots."""
        self.Reset(q_init)
        return self.q_des.copy(), self.tau_ff.copy(), self.adhesions.copy()

    def RequestSingleStep(self, gravity_direction_body, q_cur, q_torque, attached, command):
        """Advance one control period without modifying inputs.

        gravity_direction_body: finite nonzero (3,) downward direction in body.
        The direction is normalized internally; gravity magnitude is 9.81 m/s².
        q_cur: (N,4) drive angles; q_torque: (N,3) measured motor torques.
        attached: (N,) normalized per-foot sensor ratios in [0, 1].
        command: (3,) body vx/vy in m/s and yaw rate in rad/s.
        Returns q_des (N,4), tau_ff (N,3), adhesions (N,), servo_enabled (N,).
        N is the kinematic model leg count.
        Stance servos are passive; swing servos track their target angles.
        """
        direction = self._array(gravity_direction_body, (3,), "gravity_direction_body")
        scale = np.max(np.abs(direction))
        if scale == 0:
            raise ValueError("gravity_direction_body must be nonzero")
        direction = direction / scale
        gravity_body = 9.81 * direction / np.linalg.norm(direction)
        q_cur = self._array(q_cur, (self.num_legs, 4), "q_cur")
        q_torque = self._array(q_torque, (self.num_legs, 3), "q_torque")
        attached = np.asarray(attached, dtype=float)
        if attached.shape != (self.num_legs,) or not np.isfinite(attached).all() or ((attached < 0) | (attached > 1)).any():
            raise ValueError("attached must contain one finite ratio per leg in [0, 1]")
        command = self._array(command, (3,), "command")
        if not self.set_init:
            self._initialize(q_cur)
        self.kin.ForwardKin(q_cur[:, :3], self.B_e_cur.T)
        if np.any(command):
            self._PlanGait(command, q_cur[:, :3], q_torque, attached)
            delta = self._foot_target_leg - self.B_e_cur.T
            dq = np.einsum("nij,nj->ni", self.kin.DampInvJac(q_cur[:, :3]), delta)
            self.q_des[:, :3] = q_cur[:, :3] + self.joint_position_gain * dq * self.dt
            self.q_des[:, 0] = np.clip(self.q_des[:, 0],
                self.kin.joints_limits_thigh[:, 0], self.kin.joints_limits_thigh[:, 1])
            self.q_des[:, 1:3] = np.clip(self.q_des[:, 1:3],
                self.kin.joints_limits_rest[None, :, 0], self.kin.joints_limits_rest[None, :, 1])
            self.q_des[:, 3] = -self.q_des[:, 1] - self.q_des[:, 2] - np.pi / 2.0
        else:
            self.contact_count.fill(0)
        com = (np.zeros(3) if self.operating_mode == 'suspended'
               else self.kin.ForwardKinAll(q_cur)["R_com"])
        self._ComputeQuasiStaticTauFF(gravity_body, q_cur[:, :3], com)
        self.compute_count += 1
        if self.operating_mode is None:  # 保留未迁移的四足调试行为。
            self.tau_ff.fill(0.0)
        return self.q_des.copy(), self.tau_ff.copy(), self.adhesions.copy(), ~self.gaits.copy()

    def _ContactDetection(self, q_cur, q_torque):
        """Confirm sustained pressure on a foot whose normal motion has stopped.

        Motor torque also contains swing acceleration/tracking effort. A pressure
        threshold alone can mark an airborne descending foot as stance, stop its
        descent and disable its servo before the magnet ever touches the surface.
        """
        force = self.kin.CF_Estimate(q_torque, q_cur)
        self.estimated_contact_force[:] = np.sum(force * self.B_landing_n.T, axis=1)
        feet = np.empty((self.num_legs, 3))
        self.kin.ForwardKin(q_cur, feet)
        if self._previous_contact_feet is None:
            self.normal_foot_velocity.fill(np.inf)
        else:
            velocity = (feet - self._previous_contact_feet) / self.dt
            self.normal_foot_velocity[:] = np.sum(velocity * self.B_landing_n.T, axis=1)
        self._previous_contact_feet = feet.copy()
        contact = ((self.estimated_contact_force <= -self.contact_force_threshold)
                   & (np.abs(self.normal_foot_velocity) <= self.contact_velocity_threshold)
                   & ~self.gaits & self._swing_reached)

        # Scattered dynamic-load spikes must not accumulate into a false landing.
        self.contact_count[~contact] = 0
        self.contact_count[contact] += 1
        detected = self.contact_count >= self.contact_confirm_steps
        self.contact_count[detected] = 0
        self.B_support_n[:, detected] = self.B_landing_n[:, detected]
        
        # 悬空测试没有接触压力；地面模式使用上面的连续压力和速度判定。
        if self.operating_mode != 'ground':
            detected = (~self.gaits & self._swing_reached & (feet[:,2] <= -self.stance_height))
        return detected

    def _PlanGait(self, command, q_cur, q_torque, attached):
        # Compare in the float32 precision used by MagneticState.
        attached = np.asarray(attached, dtype=np.float32)
        confirmed_attached = attached > self.magnetic_attach_above
        confirmed_released = attached < self.magnetic_release_below
        landed = self._ContactDetection(q_cur, q_torque)
        # Start stance from the measured touchdown, not the deeper press target.
        self._foot_target_leg[landed] = self.B_e_cur.T[landed]
        # self.gaits[landed|confirmed_attached] = True
        self.gaits[landed] = True
        self.adhesions[:] = self.gaits
        next_targets = self._ComputeFootTargets(command)

        workspace = self._CheckWorkspace(next_targets)
        clearance = self._CheckLegClearance(next_targets)
        self.adhesions[self._swing_reached] = True
        swing = ~self.gaits

        set_all = swing & ((clearance & workspace) | ~self._swing_reached)
        # for i in range(6):
        #     if not self.gaits[i]:
        #         print(f"swing {i} clearance={clearance[i]} workspace={workspace[i]} swing_reached={self._swing_reached[i]} set_all={set_all[i]}")
        # print("\n")
        set_z = swing & (~clearance | ~workspace)
        self._foot_target_leg[set_z, 2] = next_targets[set_z, 2]
        self._foot_target_leg[set_all] = next_targets[set_all]
        # A horizontal workspace/clearance limit may stop body translation, but
        # must not freeze height recovery after a deep touchdown.
        self._foot_target_leg[self.gaits, 2] = next_targets[self.gaits, 2]
        if np.all(swing | (workspace & clearance)):
            self._foot_target_leg[self.gaits, :2] = next_targets[self.gaits, :2]

        #为了便于调试，在没有电磁铁反馈的时候，直接设置吸附和释放为真
        # if self.gaits.all():
        #     confirmed_attached.fill(1)
        #     confirmed_released.fill(1)
        
        new_group = self.groups[1 - self.stance_group_index]
        old_group = self.groups[self.stance_group_index]
        if np.all((confirmed_attached & self.gaits)[new_group]):
            self.adhesions[old_group] = False
            if np.all(confirmed_released[old_group]):
                self.stance_group_index = 1 - self.stance_group_index
                self._swing_reached.fill(False)
                self.contact_count.fill(0)
                self.gaits.fill(False)
                self.gaits[new_group] = True

    @staticmethod
    def _InversePlanarTransform(x, y, omega):
        c, s = np.cos(omega), np.sin(omega)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        transform = np.eye(4)
        transform[:3, :3] = rotation.T
        transform[:3, 3] = -rotation.T @ np.array([x, y, 0.0])
        return transform

    def _ComputeFootTargets(self, command):
        indices = np.arange(self.num_legs)
        feet_R = self.kin._B2R(self._foot_target_leg.T, indices).T
        next_R = np.empty_like(feet_R)
        for stance, sign in ((True, 1.0), (False, -1.0)):
            transform = self._InversePlanarTransform(*(command * self.dt * sign))
            mask = self.gaits == stance
            next_R[mask] = feet_R[mask] @ transform[:3, :3].T + transform[:3, 3]
        next_leg = self.kin._R2B(next_R.T, indices).T
        feet = self._foot_target_leg
        # A reached target is not proof that the measured foot has lifted.
        self._swing_reached |= self._foot_target_leg[:,2] >= self._swing_start_leg[:, 2] - 0.005
        returning = ~self._swing_reached & ~self.gaits
        speed_z = min(1.2, max(np.linalg.norm(command[:2]) * 1.5, abs(command[2]) * 1.2))
        # Keep a nonzero approach rate: the old zero lower bound trapped
        # the target at z=-0.16 even when the measured foot was still airborne.
        speed_z = speed_z * np.clip((feet_R[:, 2] + 0.16) / 0.12, 0.1, 1.0)
        stance_z_error = feet[:, 2] + self.stance_height
        increment = self.dt * speed_z * 0.2
        direct = self.gaits & (np.abs(stance_z_error) < increment)
        gradual = self.gaits & ~direct
        next_leg[direct, 2] = -self.stance_height
        next_leg[gradual, 2] = (feet[gradual, 2]
                                - np.sign(stance_z_error[gradual]) * increment[gradual])
        delta = self._swing_start_leg - feet
        distance = np.linalg.norm(delta, axis=1, keepdims=True)
        direction = np.divide(delta, distance, out=np.zeros_like(delta), where=distance > 0.0)
        return_target = feet + direction * np.minimum(self.dt * speed_z[:, None], distance)
        next_leg[returning] = return_target[returning]
        descending = ~self.gaits & self._swing_reached
        # Return to the lift waypoint, then descend there. Advancing xy for the
        # entire descent overshoots the lateral workspace at large vy.
        # next_leg[descending, :2] = feet[descending, :2]
        next_leg[descending, 2] = np.maximum(
            next_leg[descending, 2] - speed_z[descending] * self.dt, -0.2)
        # for i in range(6):
        #     if not self.gaits[i]:
        #         print(f"{i} next_B{next_leg[i]} cur_B{self.B_e_cur[:,i].T}")
        # print("\n")

        return next_leg

    @staticmethod
    def _CheckWorkspace(feet):
        xy_length = np.linalg.norm(feet[:, :2], axis=1)
        # The original atan(y/x) test accepts negative x; retain its semantics.
        # atan2 plus wrapping avoids dividing by zero at x=0.
        angle = (np.arctan2(feet[:, 1], feet[:, 0]) + np.pi / 2.0) % np.pi - np.pi / 2.0
        return ((xy_length >= 0.14) & (xy_length <= 0.3)
                & (feet[:, 2] >= -0.2) & (feet[:, 2] <= 0.1)
                & (angle >= -np.pi / 4.0) & (angle <= np.pi / 4.0))

    def _CheckLegClearance(self, feet):
        feet_R = self.kin._B2R(feet.T, np.arange(self.num_legs)).T
        free = np.zeros(self.num_legs, dtype=bool)
        for back, front, middle in ((0, 1, 2), (3, 4, 5)):
            free[back] = np.linalg.norm(feet_R[back, :2] - feet_R[middle, :2]) > 0.06
            free[front] = np.linalg.norm(feet_R[front, :2] - feet_R[middle, :2]) > 0.06
            free[middle] = free[back] & free[front]
        return free

    @staticmethod
    def _Skew(vector):
        x, y, z = vector
        return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])

    def _ComputeQuasiStaticTauFF(self, gravity_body, q_cur, R_com):
        """Balance gravity in body R, then map forces into local Jacobians.

        As in the source climbing expert, swing-leg target torque is zero.
        Blending may retain a decaying old torque during a support transition.
        """
        support = self.gaits & self.adhesions
        indices = np.flatnonzero(support)
        target_tau = np.zeros((self.num_legs, 3))
        self.static_contact_forces_R.fill(0.0)
        if indices.size and self.operating_mode != 'suspended':
            feet_R = self.kin._B2R(self.B_e_cur[:, indices], indices).T
            forces = self._SolveSupportForces(feet_R - R_com, gravity_body)
            if forces is not None:
                self.static_contact_forces_R[indices] = forces
                forces_leg = self.kin.RVectorToLeg(forces.T, indices).T
                target_tau[indices] = -np.einsum(
                    "nai,na->ni", self.kin.Jacobian(q_cur[indices]), forces_leg)
        target_tau = np.clip(target_tau, -self.motor_torque_limits, self.motor_torque_limits)
        if not np.array_equal(support, self._feedforward_support_mask):
            self._feedforward_blend_start[:] = self.tau_ff
            self._torque_blend_remaining = self.torque_blend_steps
            self._feedforward_support_mask[:] = support
        if self._torque_blend_remaining:
            completed = self.torque_blend_steps - self._torque_blend_remaining + 1
            blend = completed / self.torque_blend_steps
            self.tau_ff[:] = ((1.0 - blend) * self._feedforward_blend_start + blend * target_tau)
            self._torque_blend_remaining -= 1
        else:
            self.tau_ff[:] = target_tau
        self.tau_ff[:] = np.clip(self.tau_ff, -self.motor_torque_limits, self.motor_torque_limits)
        return self.tau_ff

    def SeedFeedforward(self, tau):
        """初始化交接沿用当前输出作为混合起点，避免直接切换力矩。"""
        self.tau_ff[:] = tau
        self._feedforward_blend_start[:] = tau
        self._torque_blend_remaining = self.torque_blend_steps
