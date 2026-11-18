"""Four-leg sequential magnetic climbing: at most one swing leg."""
import numpy as np
from hex_kinematic import Kinematic, QUAD_LEG_NAMES
from .expert_climb import ExpertClimb


class ExpertQuadClimb(ExpertClimb):
    """Inputs and outputs use lf, lb, rf, rb order."""

    def __init__(self, kinematic=None, *, sequence, **kwargs):
        if "groups" in kwargs or "stance_group" in kwargs:
            raise TypeError("ExpertQuadClimb uses sequence; remove groups and stance_group")
        if (not isinstance(sequence, (list, tuple, np.ndarray))
                or np.asarray(sequence, dtype=object).ndim != 1 or len(sequence) != 4
                or any(isinstance(i, (bool, np.bool_)) or not isinstance(i, (int, np.integer)) for i in sequence)
                or set(sequence) != set(range(4))):
            raise ValueError("sequence must contain each integer leg index 0..3 exactly once")
        if kinematic is None:
            kinematic = Kinematic(urdf_path=kwargs.get("robot_urdf"))
        if not isinstance(kinematic, Kinematic) or tuple(kinematic.leg_names) != QUAD_LEG_NAMES:
            raise ValueError("ExpertQuadClimb requires a four-leg URDF in lf/lb/rf/rb order")
        magnetic = kwargs.get("magnetic")
        if magnetic is not None and (np.asarray(magnetic).shape != (4,) or not np.asarray(magnetic).all()):
            raise ValueError("quad initial magnetic must enable all four feet")
        self.sequence = tuple(int(i) for i in sequence)
        self._InitController(kinematic, **kwargs)

    def Reset(self, q_init=None):
        if q_init is not None:
            self._array(q_init, (4, 4), "q_init")
        self.sequence_index = 0
        self.swing_leg_index = None
        self.gait_phase = "WAIT_ATTACH"
        self.swing_count = 0
        self.support_moment_residual_R = np.zeros(3)
        self._ResetState(q_init)
        self.gaits[:] = True
        self.adhesions[:] = True

    def _initialize(self, q_init):
        self._AnchorTargets(q_init)
        self.gaits[:] = True
        self.adhesions[:] = True

    def _PlanGait(self, command, q_cur, q_torque, attached):
        ratios = np.asarray(attached, dtype=np.float32)
        attached_ok = ratios > self.magnetic_attach_above
        released = ratios < self.magnetic_release_below
        landed = self._ContactDetection(q_cur, q_torque) | attached_ok
        leg = self.swing_leg_index
        if self.gait_phase == "WAIT_ATTACH":
            if attached_ok.all():
                if leg is not None:
                    self.sequence_index = (self.sequence_index + 1) % 4
                leg = self.sequence[self.sequence_index]
                self.swing_leg_index = leg
                self.adhesions[leg] = False
                self.gait_phase = "RELEASE"
        elif self.gait_phase == "RELEASE":
            others = np.arange(4) != leg
            if released[leg] and attached_ok[others].all():
                self.gaits[leg] = False
                self._swing_reached[leg] = False
                self.contact_count[leg] = 0
                self._foot_target_leg[leg] = self.B_e_cur[:, leg]
                self.gait_phase = "LIFT"
                self.swing_count += 1
        elif self.gait_phase == "LIFT":
            if np.linalg.norm(self.B_e_cur[:, leg] - self._swing_start_leg[leg]) <= 0.005:
                self._swing_reached[leg] = True
                self.adhesions[leg] = True
                self.gait_phase = "DESCEND"
        elif self.gait_phase == "DESCEND" and landed[leg]:
            self._foot_target_leg[leg] = self.B_e_cur[:, leg]
            self.gaits[leg] = True
            self.contact_count[leg] = 0
            self.gait_phase = "WAIT_ATTACH"
            # Do not release the next leg until a later control period.

        targets = self._ComputeFootTargets(command)
        workspace = self._CheckWorkspace(targets)
        clearance = self._CheckLegClearance(targets)
        swing = ~self.gaits
        lift = swing & ~self._swing_reached
        full = swing & ((workspace & clearance) | lift)
        self._foot_target_leg[full] = targets[full]
        self._foot_target_leg[swing & ~full, 2] = targets[swing & ~full, 2]
        self._foot_target_leg[self.gaits, 2] = targets[self.gaits, 2]
        if np.all(swing | (workspace & clearance)):
            self._foot_target_leg[self.gaits, :2] = targets[self.gaits, :2]

    def _ComputeFootTargets(self, command):
        feet = self._foot_target_leg
        if self.swing_leg_index is None:
            return feet.copy()  # Startup: await four confirmed attachments.
        indices = np.arange(4)
        body = self.kin._B2R(feet.T, indices).T
        next_body = np.empty_like(body)
        swing_scale = 3.0
        for stance, sign in ((True, 1.0), (False, -1.0)):
            scale = 1.0 if stance else swing_scale
            transform = self._InversePlanarTransform(*(command * self.dt * sign * scale))
            mask = self.gaits == stance
            next_body[mask] = body[mask] @ transform[:3, :3].T + transform[:3, 3]
        targets = self.kin._R2B(next_body.T, indices).T
        speed = min(1.2, max(np.linalg.norm(command[:2]) * 1.5, abs(command[2]) * 1.2))
        speed = speed * np.clip((body[:, 2] + 0.16) / 0.12, 0.1, 1.)
        error = feet[:, 2] + self.stance_height
        targets[self.gaits, 2] = (feet[:, 2] - np.clip(error, -self.dt*speed*.2, self.dt*speed*.2))[self.gaits]
        leg = self.swing_leg_index
        swing_speed = min(1.2, speed[leg] * swing_scale)
        if self.gait_phase == "LIFT":
            delta = self._swing_start_leg[leg] - feet[leg]
            distance = np.linalg.norm(delta)
            targets[leg] = feet[leg] + delta * min(1., self.dt * swing_speed / max(distance, 1e-12))
        elif self.gait_phase == "DESCEND":
            targets[leg, 2] = max(-0.2, feet[leg, 2] - self.dt * swing_speed)
        return targets

    def _CheckLegClearance(self, feet):
        points = self.kin._B2R(feet.T, np.arange(4)).T
        free = np.zeros(4, dtype=bool)
        for front, back in ((0, 1), (2, 3)):
            free[[front, back]] = np.linalg.norm(points[front, :2] - points[back, :2]) > 0.06
        return free

    def _SolveSupportForces(self, arms, gravity_body):
        """Balance net force, minimize net moment, then minimize force norm.

        The projection spans force adjustments whose sum is zero. Its
        pseudoinverse removes only achievable moments, including for two feet.
        """
        self.support_moment_residual_R = np.zeros(3)
        if len(arms) < 2:
            return None
        n = len(arms)
        force = -self.total_mass * gravity_body
        base = np.tile(force / n, n)
        moments = np.hstack([self._Skew(arm) for arm in arms])
        null = np.kron(np.eye(n) - np.ones((n, n)) / n, np.eye(3))
        result = base - null @ np.linalg.pinv(moments @ null, rcond=1e-9) @ (moments @ base)
        self.support_moment_residual_R = moments @ result
        return result.reshape(n, 3)
