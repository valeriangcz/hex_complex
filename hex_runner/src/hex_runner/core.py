"""Pure control state machine: no ROS, simulator, Torch, or hardware I/O."""
from dataclasses import dataclass
from enum import Enum
from threading import RLock
import numpy as np
from hex_kinematic import LEG_NAMES
from .console import banner


class Phase(str, Enum):
    WAITING = "WAITING"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAULT = "FAULT"


def finite(value, shape):
    value = np.asarray(value, dtype=float)
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError("expected finite array with shape " + str(shape))
    return value.copy()


@dataclass
class Feedback:
    q: np.ndarray
    tau: np.ndarray
    pose: object  # (4,4) for complex, None for climb.
    attached: np.ndarray  # Normalized sensor ratios, not booleans.
    enabled: np.ndarray
    motor_sample: object
    gravity_direction_body: object = None  # (3,) unit down vector for climb.


@dataclass
class Command:
    q: np.ndarray
    tau: np.ndarray
    adhesions: np.ndarray
    servo_enabled: np.ndarray

    def __post_init__(self):
        n = np.asarray(self.q).shape[0]
        if n not in (4, 6):
            raise ValueError("commands require four or six legs")
        self.q = finite(self.q, (n, 4))
        self.tau = finite(self.tau, (n, 3))
        self.adhesions = np.asarray(self.adhesions, dtype=bool).copy()
        enabled = np.asarray(self.servo_enabled)
        if enabled.shape != (n,) or enabled.dtype != np.bool_:
            raise ValueError("servo_enabled must be a boolean array with shape (n,)")
        self.servo_enabled = enabled.copy()
        if self.adhesions.shape != (n,):
            raise ValueError("adhesions must have shape (n,)")


class FeedbackCache:
    def __init__(self):
        self.values = {}
        self.lock = RLock()

    def update(self, name, value, now, valid=True, sample=None):
        with self.lock:
            previous = self.values.get(name)
            # Replayed motor samples must not refresh freshness or advance the expert.
            if name == "motor" and previous and previous[3] == sample and valid:
                return
            self.values[name] = (value, float(now), bool(valid), sample)

    def snapshot(self, now, timeout, pose_source):
        with self.lock:
            # A callback can arrive after tick captured `now` but before this lock.
            ages = {name: max(0.0, now - item[1]) for name, item in self.values.items()}
            for name in ("motor", "servo", "magnetic", pose_source):
                if name not in self.values:
                    return None, "missing " + name, ages
                if not self.values[name][2]:
                    return None, "invalid " + name, ages
                if ages[name] > timeout:
                    return None, "stale " + name, ages
            motor, servo, magnetic, pose = [self.values[name][0] for name in ("motor", "servo", "magnetic", pose_source)]
            q = np.column_stack((motor[0], servo))
            return Feedback(q=q, tau=motor[1], pose=pose if pose_source == "se3" else None,
                            attached=magnetic[0], enabled=magnetic[1],
                            motor_sample=self.values["motor"][3],
                            gravity_direction_body=pose if pose_source == "gravity" else None), "", ages

    def motor_snapshot(self, now, timeout):
        """Read fresh motor feedback independently of servo, magnets and pose."""
        with self.lock:
            item = self.values.get('motor')
            if item is None or not item[2] or max(0.0,now-item[1]) > timeout:
                return None
            return (item[0][0].copy(),item[0][1].copy(),item[3])


class Runner:
    def __init__(self, config, adapter):
        self.config, self.adapter = config, adapter
        self.phase = Phase.WAITING
        self.command = None
        self.failure_reason = ""
        self.velocity = np.zeros(3)
        self.pending = None
        self.pending_disables = set()
        self.motor_servo_disabled = False
        self.magnetic_forced_off = False
        self.last_motor_sample = None
        self.compute_count = 0
        self.reset_count = 0
        self.actuator_output = None  # This tick only: reset, compute, or disable.
        self.events = []
        self.initialization = None
        self.lock = RLock()
        banner("RUNNER", "WAITING", "等待初始化")

    def request(self, action, ready):
        # Service callbacks submit requests; tick owns state changes and expert calls.
        with self.lock:
            if action in ("disable_actuators", "release_magnets"):
                self.pending_disables.add(action)
                return True, action + " queued"
            if self.pending is not None:
                return False, "another operation is pending"
            if action not in ("reset", "start", "pause"):
                return False, "unknown operation"
            if action in ("start", "pause") and self.config.expert_type != "complex":
                return False, action + " applies only to complex"
            if action == "reset" and not ready and not self.config.staged_initialization:
                return False, "device feedback is not ready"
            if action == "start" and (self.motor_servo_disabled or self.magnetic_forced_off):
                return False, "reset required after actuator disable or magnet release"
            if action == "start" and (not ready or self.phase not in (Phase.READY, Phase.PAUSED)):
                return False, "start requires fresh feedback and READY/PAUSED"
            if action == "pause" and self.phase != Phase.RUNNING:
                return False, "pause requires RUNNING"
            self.pending = action
            return True, action + " queued"

    def set_velocity(self, velocity):
        velocity = finite(velocity, (3,))
        with self.lock:
            if self.config.expert_type not in ("climb", "quad_climb"):
                return False
            if self.motor_servo_disabled or self.magnetic_forced_off:
                return False
            if self.phase not in (Phase.READY, Phase.RUNNING):
                return False
            self.velocity = velocity
            return True

    def _phase(self, phase, reason=""):
        if self.phase != phase or reason:
            self.events.append({"phase": phase.value, "reason": reason})
        if self.phase != phase or (reason and reason != getattr(self, "_printed_reason", "")):
            messages = {Phase.WAITING: "等待初始化", Phase.INITIALIZING: "开始初始化",
                        Phase.READY: "准备就绪", Phase.RUNNING: "开始运行",
                        Phase.PAUSED: "已暂停", Phase.COMPLETED: "运行完成", Phase.FAULT: "故障"}
            color = ("red" if phase == Phase.FAULT else "yellow" if phase == Phase.PAUSED
                     else "green" if phase in (Phase.READY, Phase.COMPLETED) else "cyan")
            message = messages[phase] + ("；" + reason if reason else "")
            banner("RUNNER", self.phase.value + " -> " + phase.value, message, color)
            self._printed_reason = reason
        self.phase = phase

    def tick(self, feedback, unavailable=""):
        with self.lock:
            self.actuator_output = None
            try:
                # Disable/release requests merge independently and preempt normal operations.
                if self.pending_disables:
                    if "disable_actuators" in self.pending_disables:
                        self.actuator_output = "disable"
                        banner("RUNNER", "DISABLE", "电机/舵机失能指令已处理", "yellow")
                    if "release_magnets" in self.pending_disables:
                        banner("RUNNER", "RELEASE_MAGNETS", "吸附关闭指令已处理", "yellow")
                    self.motor_servo_disabled |= "disable_actuators" in self.pending_disables
                    self.magnetic_forced_off |= "release_magnets" in self.pending_disables
                    self.pending_disables.clear()
                    self.pending = None
                    self.initialization = None
                    self.velocity[:] = 0
                    if self.phase != Phase.FAULT:
                        self._phase(Phase.WAITING if self.command is None else Phase.PAUSED)
                    return self.command
                action, self.pending = self.pending, None
                if action == "reset":
                    staged = self.config.staged_initialization
                    if self.config.initial_state is None:
                        raise ValueError("initial_state is required for reset")
                    if staged:
                        from .initialization import StagedInitialization
                        self.initialization = StagedInitialization(self.config,self.adapter)
                        self.command = None
                    else:
                        if feedback is None:
                            raise ValueError("reset lost feedback: " + unavailable)
                        self.command = Command(*self.adapter.prepare_initialization())
                        self.initialization = None
                    self.motor_servo_disabled = False
                    self.magnetic_forced_off = False
                    self.velocity[:] = 0
                    self.failure_reason = ""
                    if staged:
                        self._phase(Phase.INITIALIZING)
                    else:
                        self.last_motor_sample = feedback.motor_sample
                        self.reset_count += 1
                        banner("INIT", "COMPLETE", "初始化完成", "green")
                        self._phase(Phase.READY)
                        self.actuator_output = "reset"
                        return self.command
                if self.phase == Phase.INITIALIZING:
                    candidate,output,complete = self.initialization.step(feedback,unavailable)
                    if candidate is not None and (np.abs(candidate.tau)>self.config.motor_torque_limit+1e-6).any():
                        raise ValueError('initialization exceeded configured torque limit')
                    self.command,self.actuator_output = candidate,output
                    if complete:
                        self.reset_count += 1
                        self.last_motor_sample = feedback.motor_sample
                        self._phase(Phase.READY)
                    return self.command
                if self.command is None:
                    return None
                # Only reset can clear these latches; start/velocity cannot resume motion.
                if self.motor_servo_disabled or self.magnetic_forced_off:
                    return self.command
                if feedback is None and self.phase != Phase.FAULT:
                    raise ValueError(unavailable or "device feedback unavailable")
                if action == "pause":
                    self._phase(Phase.PAUSED)
                elif action == "start":
                    self._phase(Phase.RUNNING)
                if self.phase in (Phase.FAULT, Phase.COMPLETED, Phase.PAUSED):
                    return self.command
                if self.config.expert_type in ("climb", "quad_climb"):
                    self._phase(Phase.RUNNING if np.any(self.velocity) else Phase.READY)
                if self.phase != Phase.RUNNING or feedback.motor_sample == self.last_motor_sample:
                    return self.command
                self.last_motor_sample = feedback.motor_sample
                candidate = Command(*self.adapter.compute(feedback, self.velocity.copy()))
                if (np.abs(candidate.tau) > self.config.motor_torque_limit + 1e-6).any():
                    raise ValueError("expert exceeded configured torque limit")
                reason = self.adapter.failure_reason(feedback)
                if reason:
                    raise ValueError(reason)
                complete = self.adapter.is_complete(feedback)
                self.command = candidate
                self.compute_count += 1
                self.actuator_output = "compute"
                if complete:
                    self._phase(Phase.COMPLETED)
            except Exception as exc:
                self.actuator_output = None
                self.failure_reason = str(exc)
                self._phase(Phase.FAULT, self.failure_reason)
            return self.command
