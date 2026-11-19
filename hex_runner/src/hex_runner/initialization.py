"""Feedback-driven six-leg initialization, independent of ROS transport."""
from enum import Enum
import numpy as np
from .core import Command, finite
from .console import banner


class InitStage(str, Enum):
    WAIT_FEEDBACK = 'WAIT_FEEDBACK'
    WAIT_RELEASE = 'WAIT_RELEASE'
    LIFT = 'LIFT'
    MOVE_TARGET = 'MOVE_TARGET'
    WAIT_ATTACH = 'WAIT_ATTACH'
    COMPLETE = 'COMPLETE'


class StagedInitialization:
    JOINT_SPEED = 1.0  # rad/s, including the servo angle
    MOTOR_TOLERANCE = 0.03
    SERVO_TOLERANCE = 0.05
    CONFIRM_SAMPLES = 10

    def __init__(self, config, adapter):
        self.config, self.adapter = config, adapter
        self.stage = None
        self._set_stage(InitStage.WAIT_FEEDBACK)
        self.last_sample = None
        self.command = None
        self.confirmed = 0
        self.elapsed = 0.0
        self.motor_error = self.servo_error = None
        self.waiting_legs = []
        self.waiting_reason = 'waiting for fresh motor/servo/magnetic/pose feedback'
        self.zero_sent = False
        self.zero_sample = None

    def _set_stage(self, stage):
        if self.stage != stage:
            messages = {InitStage.WAIT_FEEDBACK: "等待新鲜电机、舵机、吸附及位姿反馈",
                        InitStage.WAIT_RELEASE: "等待全部吸附释放",
                        InitStage.LIFT: "六脚抬起",
                        InitStage.MOVE_TARGET: "移动到配置初始位置",
                        InitStage.WAIT_ATTACH: "等待配置吸附反馈",
                        InitStage.COMPLETE: "初始化完成"}
            banner("INIT", stage.value, messages[stage],
                   "green" if stage == InitStage.COMPLETE else "cyan")
        self.stage = stage

    def _segment(self, start, target, stage):
        self.start = finite(start, (6, 4))
        self.target = finite(target, (6, 4))
        # Quintic smoothstep peak derivative is 1.875.
        self.duration = max(self.config.control_dt,1.875*float(np.max(np.abs(self.target-self.start)))/self.JOINT_SPEED)
        self.elapsed = 0.0
        self.confirmed = 0
        self._set_stage(stage)

    def _arrived(self, q):
        self.motor_error = float(np.max(np.abs(q[:,:3]-self.target[:,:3])))
        self.servo_error = float(np.max(np.abs(q[:,3]-self.target[:,3])))
        ok = self.motor_error <= self.MOTOR_TOLERANCE and self.servo_error <= self.SERVO_TOLERANCE
        self.confirmed = self.confirmed+1 if ok and self.elapsed >= self.duration else 0
        return self.confirmed >= self.CONFIRM_SAMPLES

    def step(self, feedback, unavailable=''):
        if self.stage in (InitStage.WAIT_FEEDBACK,InitStage.WAIT_RELEASE):
            # Even with cached feedback, reset must first issue zero torque.
            if not self.zero_sent:
                self.zero_sent = True
                self.zero_sample = None if feedback is None else feedback.motor_sample
                return None, 'zero', False
            if feedback is None:
                self.waiting_reason = unavailable or 'waiting for complete fresh feedback'
                return self.command, 'zero', False
            if self.zero_sample is not None and feedback.motor_sample == self.zero_sample:
                self.waiting_reason = 'waiting for a new motor sample after zero torque'
                return self.command, 'zero', False
            self._set_stage(InitStage.WAIT_RELEASE)
            threshold = self.config.expert_parameters.get('magnetic_release_below',0.05)
            released = (np.asarray(feedback.attached,dtype=np.float32) < threshold) & ~np.asarray(feedback.enabled,dtype=bool)
            self.waiting_legs = np.flatnonzero(~released).tolist()
            self.waiting_reason = 'waiting for all magnets to release' if self.waiting_legs else ''
            if self.waiting_legs:
                return self.command, 'zero', False
            # No interpolation starts from a configured or invented measured pose.
            self.goal = Command(*self.adapter.prepare_initialization())
            self._segment(feedback.q,self.adapter.initialization_lift_target(),InitStage.LIFT)
            self.command = Command(feedback.q,np.zeros((6,3)),np.zeros(6,dtype=bool),np.ones(6,dtype=bool))
            self.last_sample = feedback.motor_sample
            return self.command, 'initialize', False
        if feedback is None:
            raise ValueError('initialization lost feedback: '+unavailable)
        if feedback.motor_sample == self.last_sample:
            return self.command, None, False
        self.last_sample = feedback.motor_sample
        if self.stage in (InitStage.LIFT,InitStage.MOVE_TARGET):
            self.elapsed = min(self.duration,self.elapsed+self.config.control_dt)
            u = self.elapsed/self.duration
            blend = 10*u**3-15*u**4+6*u**5
            q = self.start+(self.target-self.start)*blend
            # Preserve the expert's configured initial support state for its existing
            # feedforward calculation; physical magnet commands remain off below.
            tau = np.zeros((6,3)) if self.stage == InitStage.LIFT else self.adapter.initialization_feedforward(feedback,self.goal.adhesions)
            self.command = Command(q,tau,np.zeros(6,dtype=bool),np.ones(6,dtype=bool))
            self.waiting_reason = 'waiting for measured joint targets'
            if self._arrived(feedback.q):
                if self.stage == InitStage.LIFT:
                    self._segment(self.command.q,self.goal.q,InitStage.MOVE_TARGET)
                else:
                    self._set_stage(InitStage.WAIT_ATTACH)
                    self.command.adhesions[:] = self.goal.adhesions
                    self.confirmed = 0
            return self.command, 'initialize', False
        if self.stage == InitStage.WAIT_ATTACH:
            mask = self.goal.adhesions
            attach = self.config.expert_parameters.get('magnetic_attach_above',0.85)
            release = self.config.expert_parameters.get('magnetic_release_below',0.05)
            ratios = np.asarray(feedback.attached,dtype=np.float32)
            actual = np.asarray(feedback.enabled,dtype=bool)
            ok = np.where(mask,(ratios>attach)&actual,(ratios<release)&~actual)
            self.waiting_legs = np.flatnonzero(~ok).tolist()
            self.waiting_reason = 'waiting for configured magnet feedback' if self.waiting_legs else ''
            self.command = Command(self.goal.q,self.adapter.initialization_feedforward(feedback,mask),mask,np.ones(6,dtype=bool))
            # Positions must still be attained when attachment is acknowledged.
            arrived = self._arrived(feedback.q)
            if ok.all() and arrived:
                self.adapter.finish_initialization(feedback,self.command)
                self._set_stage(InitStage.COMPLETE)
                self.waiting_reason = ''
                return self.command, 'initialize', True
            return self.command, 'initialize', False
        return self.command, None, True

    def diagnostics(self):
        return dict(initialization_stage=self.stage.value,initialization_waiting_reason=self.waiting_reason,
                    initialization_waiting_legs=self.waiting_legs,initialization_motor_error=self.motor_error,
                    initialization_servo_error=self.servo_error,initialization_confirm_samples=self.confirmed)
