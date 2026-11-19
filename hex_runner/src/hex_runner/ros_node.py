"""ROS transport for the shared runner. Device ownership stays outside this node."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from dm_motor_fusion.msg import MultiLegCommand, MultiLegState
from feetech_servo.msg import servo_cmd, servo_state
from magnetic.msg import MagneticCmd, MagneticState
from hex_kinematic.ros_contract import read_valid, read_motor, read_servo, read_magnetic, write_magnetic, write_motor, write_servo, write_servo_enabled
from hex_kinematic.se3_utils import transform_from_xyzw
from .config import RunnerConfig
from .core import Runner, FeedbackCache, Phase, finite
from .adapters import ExpertAdapter
from .console import banner


class RunnerNode:
    def __init__(self, config, output_dir=None, adapter=None):
        self.config = config
        if config.expert_type in ("climb", "quad_climb") and not config.velocity_scales:
            raise ValueError("climb requires velocity_scales in runner configuration")
        self.cache = FeedbackCache()
        self.runner = Runner(config, adapter if adapter is not None else ExpertAdapter(config))
        self.recorder = None
        if config.recording_enabled:
            try:
                from .recording import H5Recorder
                self.recorder = H5Recorder(config)
            except Exception as exc:
                banner('RECORDING', 'ERROR', '无法启动 H5 记录：' + str(exc), 'red')
        self.output_dir = Path(output_dir).expanduser() if output_dir else None
        self.last_slow = {"magnetic": -float("inf")}
        self.motor_pub = rospy.Publisher("hex/motor/command", MultiLegCommand, queue_size=1)
        self.servo_pub = rospy.Publisher("hex/servo/command", servo_cmd, queue_size=1)
        self.magnetic_pub = rospy.Publisher("hex/magnetic/command", MagneticCmd, queue_size=1)
        self.diag_pub = rospy.Publisher("hex/runner/diagnostics", DiagnosticArray, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("hex/motor/state", MultiLegState, self._motor, queue_size=1),
            rospy.Subscriber("hex/servo/state", servo_state, self._servo, queue_size=1),
            rospy.Subscriber("hex/magnetic/state", MagneticState, self._magnetic, queue_size=1),
            rospy.Subscriber("hex/perception/" + ("pose" if config.pose_source == "se3" else "gravity"),
                             PoseStamped if config.pose_source == "se3" else Float64MultiArray,
                             self._pose if config.pose_source == "se3" else self._gravity, queue_size=1),
            rospy.Subscriber("hex/cmd_vel", Twist, self._velocity, queue_size=1)]
        self.services = [rospy.Service("hex/runner/" + action, Trigger,
                         lambda request, operation=action: self._operation(operation))
                         for action in ("reset", "start", "pause", "disable_actuators", "release_magnets")]
        self.last_diag = -float("inf")
        rospy.on_shutdown(self.shutdown)

    def _update(self, name, decode, message, sample=None):
        try:
            value, valid = decode(message)
        except (ValueError, TypeError, AttributeError) as error:
            rospy.logwarn_throttle(1.0, "invalid %s feedback: %s", name, error)
            value, valid = None, False
        self.cache.update(name, value, time.monotonic(), valid, sample)

    def _motor(self, message):
        self._record_feedback('motor', message)
        def decode(msg):
            q = finite(read_motor(msg, "pos", self.config.leg_names), (len(self.config.leg_names), 3))
            tau = finite(read_motor(msg, "tau", self.config.leg_names), (len(self.config.leg_names), 3))
            finite(read_motor(msg, "vel", self.config.leg_names), (len(self.config.leg_names), 3))
            return (q, tau), all(read_valid(msg, self.config.leg_names, motor=True)) and msg.header.stamp.to_nsec() > 0
        self._update("motor", decode, message, message.header.stamp.to_nsec())

    def _servo(self, message):
        self._record_feedback('servo', message)
        self._update("servo", lambda msg: (finite(read_servo(msg, self.config.leg_names), (len(self.config.leg_names),)), all(read_valid(msg, self.config.leg_names))), message)

    def _magnetic(self, message):
        self._record_feedback('magnetic', message)
        self._update("magnetic", lambda msg: ((finite(read_magnetic(msg, "attached_", self.config.leg_names), (len(self.config.leg_names),)),
                     read_magnetic(msg, "enabled_", self.config.leg_names)), all(read_magnetic(msg, "valid_", self.config.leg_names))), message)

    def _pose(self, message):
        self._record_feedback('pose', message)
        def decode(msg):
            if msg.header.frame_id != "world":
                raise ValueError("SE3 frame must be world")
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orientation = msg.pose.orientation
            quaternion = finite([orientation.x, orientation.y, orientation.z, orientation.w], (4,))
            if abs(np.linalg.norm(quaternion) - 1.0) > 1e-3:
                raise ValueError("orientation quaternion must have unit norm")
            return transform_from_xyzw(finite(position, (3,)), quaternion), True
        self._update("se3", decode, message)

    def _record_feedback(self, name, message):
        recorder = getattr(self, 'recorder', None)
        if recorder is None or not recorder.active:
            return
        try:
            values = dict(stamp_ns=message.header.stamp.to_nsec(), received_ns=time.monotonic_ns())
            legs = self.config.leg_names
            if name == 'pose':
                p, q = message.pose.position, message.pose.orientation
                position = np.array([p.x, p.y, p.z])
                quaternion = np.array([q.x, q.y, q.z, q.w])
                valid = (message.header.frame_id == 'world' and values['stamp_ns'] > 0
                         and np.isfinite(position).all() and np.isfinite(quaternion).all()
                         and abs(np.linalg.norm(quaternion)-1) <= 1e-3)
                values.update(position=position, quaternion_xyzw=quaternion,
                              valid=bool(valid), frame_id=message.header.frame_id)
            elif name == 'motor':
                position = np.asarray(read_motor(message, 'pos', legs))
                velocity = np.asarray(read_motor(message, 'vel', legs))
                torque = np.asarray(read_motor(message, 'tau', legs))
                valid = (np.asarray(read_valid(message, legs, motor=True), bool)
                         & np.isfinite(position).all(axis=1) & np.isfinite(velocity).all(axis=1)
                         & np.isfinite(torque).all(axis=1) & (values['stamp_ns'] > 0))
                values.update(position=position, velocity=velocity, torque=torque, valid=valid)
            elif name == 'servo':
                position = np.asarray(read_servo(message, legs))
                values.update(position=position, valid=(np.asarray(read_valid(message, legs), bool)
                              & np.isfinite(position) & (values['stamp_ns'] > 0)))
            else:
                attached = np.asarray(read_magnetic(message, 'attached_', legs))
                values.update(attached=attached, enabled=read_magnetic(message, 'enabled_', legs),
                    valid=(np.asarray(read_magnetic(message, 'valid_', legs), bool)
                           & np.isfinite(attached) & (values['stamp_ns'] > 0)))
            recorder.feedback(name, values)
        except Exception as exc:
            recorder.fail('sensor capture: ' + str(exc))

    def _gravity(self, message):
        def decode(msg):
            direction = finite(msg.data, (3,))
            scale = np.max(np.abs(direction))
            if scale == 0:
                raise ValueError("body gravity direction must be nonzero")
            direction /= scale
            return direction / np.linalg.norm(direction), True
        self._update("gravity", decode, message)

    def _snapshot(self, now):
        return self.cache.snapshot(now, self.config.feedback_timeout, self.config.pose_source)

    def _operation(self, action):
        feedback, reason, _ = self._snapshot(time.monotonic())
        accepted, message = self.runner.request(action, feedback is not None)
        if not accepted and feedback is None:
            message += ": " + reason
        return TriggerResponse(success=accepted, message=message)

    def _velocity(self, message):
        try:
            if self.config.expert_type not in ("climb", "quad_climb"):
                return
            raw = finite([message.linear.x, message.linear.y, message.angular.z], (3,))
            scales = np.array([self.config.velocity_scales[name] for name in ("vx", "vy", "wz")])
            self.runner.set_velocity(raw * scales)
        except ValueError as error:
            rospy.logwarn_throttle(1.0, "invalid velocity: %s", error)

    def tick(self):
        now = time.monotonic()
        feedback, reason, ages = self._snapshot(now)
        command = self.runner.tick(feedback, reason)
        recorder = getattr(self, 'recorder', None)
        if recorder is not None and recorder.active:
            try:
                recorder.tick(self.runner, self.runner.adapter.planning_snapshot(), rospy.Time.now().to_nsec())
            except Exception as exc:
                recorder.fail('runner capture: ' + str(exc))
        disabled = self.runner.motor_servo_disabled
        off = self.runner.magnetic_forced_off
        output_key = (disabled, off, self.runner.reset_count)
        if getattr(self, "last_output_key", None) != output_key:
            self.last_slow = {"magnetic": -float("inf")}
            self.last_output_key = output_key
        output = self.runner.actuator_output
        if output is not None:
            motor = MultiLegCommand(option={"disable": -1, "zero": 0, "reset": 1, "compute": 1, "initialize": 1}[output])
            if not disabled and output != 'zero':
                write_motor(motor, "pos", command.q[:, :3], self.config.leg_names)
                write_motor(motor, "tau", command.tau, self.config.leg_names)
                write_motor(motor, "vel", np.zeros((len(self.config.leg_names), 3)), self.config.leg_names)
                write_motor(motor, "kp", np.full((len(self.config.leg_names), 3), self.config.motor_kp), self.config.leg_names)
                write_motor(motor, "kd", np.full((len(self.config.leg_names), 3), self.config.motor_kd), self.config.leg_names)
            self.motor_pub.publish(motor)
        if output is not None and output != 'zero':
            message = servo_cmd(enable=not disabled)
            write_servo_enabled(message, np.zeros(len(self.config.leg_names), dtype=bool) if disabled else command.servo_enabled, self.config.leg_names)
            if not disabled:
                write_servo(message, command.q[:, 3], self.config.leg_names)
            self.servo_pub.publish(message)
        initializing = self.runner.phase == Phase.INITIALIZING
        if (command is not None or off or initializing) and now - self.last_slow["magnetic"] >= 1.0 / self.config.magnetic_hz:
            message = MagneticCmd()
            write_magnetic(message, np.zeros(len(self.config.leg_names), dtype=bool) if off or command is None else command.adhesions, leg_names=self.config.leg_names)
            self.magnetic_pub.publish(message)
            self.last_slow["magnetic"] = now
        if now - self.last_diag >= 0.1:
            status = DiagnosticStatus(name="hex_runner", hardware_id="hex/shared-controller",
                level=DiagnosticStatus.ERROR if self.runner.phase == Phase.FAULT else
                      DiagnosticStatus.WARN if feedback is None else DiagnosticStatus.OK,
                message=self.runner.phase.value)
            values = dict(motor_servo_disabled=disabled, magnetic_forced_off=off, expert_type=self.config.expert_type, pose_source=self.config.pose_source,
                          feedback_ready=feedback is not None, feedback_reason=reason, feedback_age=ages,
                          failure_reason=self.runner.failure_reason, reset_count=self.runner.reset_count,
                          compute_count=self.runner.compute_count, command=self.runner.velocity.tolist(),
                          servo_enabled=[False]*len(self.config.leg_names) if disabled else (None if command is None else command.servo_enabled.tolist()))
            values.update(self.runner.adapter.diagnostics())
            if self.runner.initialization is not None:
                values.update(self.runner.initialization.diagnostics())
            if hasattr(self,'cache'):
                values['motor_feedback_ready'] = self.cache.motor_snapshot(now,self.config.feedback_timeout) is not None
            status.values = [KeyValue(str(key), json.dumps(value)) for key, value in values.items()]
            diagnostic = DiagnosticArray(status=[status])
            diagnostic.header.stamp = rospy.Time.now()
            self.diag_pub.publish(diagnostic)
            self.last_diag = now

    def spin(self):
        # Wall pacing avoids deadlocks before /clock is available and during sim reset.
        deadline = time.monotonic()
        while not rospy.is_shutdown():
            self.tick()
            deadline += self.config.control_dt
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()  # No catch-up expert cycles.

    def shutdown(self):
        # Do not issue implicit disable/demagnetize commands on shutdown.
        recorder = getattr(self, 'recorder', None)
        if recorder is not None:
            recorder.close()
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            report = dict(phase=self.runner.phase.value, reset_count=self.runner.reset_count,
                          compute_count=self.runner.compute_count, failure_reason=self.runner.failure_reason,
                          events=self.runner.events, expert=self.runner.adapter.diagnostics())
            (self.output_dir / ("runner_%d.json" % time.time_ns())).write_text(json.dumps(report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--data-root", default="")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--to-sim-info", default="")
    args = parser.parse_args(rospy.myargv()[1:])
    config = RunnerConfig.load(args.config, args.data_root or None, args.to_sim_info or None)
    rospy.init_node("hex_runner")
    node = RunnerNode(config, args.output_dir or None)
    print("HEX_RUNNER_INITIALIZED", flush=True)
    node.spin()
