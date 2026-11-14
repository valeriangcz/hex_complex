"""ROS transport for the shared runner. Device ownership stays outside this node."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from dm_motor_fusion.msg import MultiLegCommand, MultiLegState
from feetech_servo.msg import servo_cmd, servo_state
from magnetic.msg import MagneticCmd, MagneticState
from hex_kinematic.ros_contract import read_motor, read_servo, read_magnetic, write_magnetic, write_motor, write_servo, write_servo_enabled
from hex_kinematic.se3_utils import transform_from_xyzw, transform_to_xyzw
from .config import RunnerConfig
from .core import Runner, FeedbackCache, Phase, finite
from .adapters import ExpertAdapter


class RunnerNode:
    def __init__(self, config, output_dir=None, adapter=None):
        self.config = config
        if config.expert_type == "climb" and not config.velocity_scales:
            raise ValueError("climb requires velocity_scales in runner configuration")
        self.cache = FeedbackCache()
        self.runner = Runner(config, adapter if adapter is not None else ExpertAdapter(config))
        self.output_dir = Path(output_dir).expanduser() if output_dir else None
        self.last_slow = {"servo": -float("inf"), "magnetic": -float("inf")}
        self.motor_pub = rospy.Publisher("hex/motor/command", MultiLegCommand, queue_size=1)
        self.servo_pub = rospy.Publisher("hex/servo/command", servo_cmd, queue_size=1)
        self.magnetic_pub = rospy.Publisher("hex/magnetic/command", MagneticCmd, queue_size=1)
        self.diag_pub = rospy.Publisher("hex/runner/diagnostics", DiagnosticArray, queue_size=1)
        self.path_pub = rospy.Publisher("hex/reference_path", PathMessage, queue_size=1, latch=True)
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
        self._publish_path()
        rospy.on_shutdown(self.shutdown)

    def _update(self, name, decode, message, sample=None):
        try:
            value, valid = decode(message)
        except (ValueError, TypeError, AttributeError) as error:
            rospy.logwarn_throttle(1.0, "invalid %s feedback: %s", name, error)
            value, valid = None, False
        self.cache.update(name, value, time.monotonic(), valid, sample)

    def _motor(self, message):
        def decode(msg):
            q = finite(read_motor(msg, "pos"), (6, 3))
            tau = finite(read_motor(msg, "tau"), (6, 3))
            finite(read_motor(msg, "vel"), (6, 3))
            return (q, tau), all(msg.valid) and msg.header.stamp.to_nsec() > 0
        self._update("motor", decode, message, message.header.stamp.to_nsec())

    def _servo(self, message):
        self._update("servo", lambda msg: (finite(read_servo(msg), (6,)), all(msg.valid)), message)

    def _magnetic(self, message):
        self._update("magnetic", lambda msg: ((finite(read_magnetic(msg, "attached_"), (6,)),
                     read_magnetic(msg, "enabled_")), all(read_magnetic(msg, "valid_"))), message)

    def _pose(self, message):
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
            if self.config.expert_type != "climb":
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
        disabled = self.runner.motor_servo_disabled
        off = self.runner.magnetic_forced_off
        output_key = (disabled, off, self.runner.reset_count)
        if getattr(self, "last_output_key", None) != output_key:
            self.last_slow = {"servo": -float("inf"), "magnetic": -float("inf")}
            self.last_output_key = output_key
        if command is not None or disabled:
            motor = MultiLegCommand(option=-1 if disabled else 1)
            if not disabled:
                write_motor(motor, "pos", command.q[:, :3])
                write_motor(motor, "tau", command.tau)
                write_motor(motor, "vel", np.zeros((6, 3)))
                write_motor(motor, "kp", np.full((6, 3), self.config.motor_kp))
                write_motor(motor, "kd", np.full((6, 3), self.config.motor_kd))
            self.motor_pub.publish(motor)
        if (command is not None or disabled) and now - self.last_slow["servo"] >= 1.0 / self.config.servo_hz:
            message = servo_cmd(enable=not disabled)
            write_servo_enabled(message, np.zeros(6, dtype=bool) if disabled else command.servo_enabled)
            if not disabled:
                write_servo(message, command.q[:, 3])
            self.servo_pub.publish(message)
            self.last_slow["servo"] = now
        if (command is not None or off) and now - self.last_slow["magnetic"] >= 1.0 / self.config.magnetic_hz:
            message = MagneticCmd()
            write_magnetic(message, np.zeros(6, dtype=bool) if off else command.adhesions)
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
                          servo_enabled=[False]*6 if disabled else (None if command is None else command.servo_enabled.tolist()))
            values.update(self.runner.adapter.diagnostics())
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

    def _publish_path(self):
        if not self.runner.adapter.path:
            return
        message = PathMessage()
        message.header.frame_id = "world"
        message.header.stamp = rospy.Time.now()
        for value in self.runner.adapter.path:
            p, q = transform_to_xyzw(value.A)
            pose = PoseStamped(header=message.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            message.poses.append(pose)
        self.path_pub.publish(message)

    def shutdown(self):
        # Do not issue implicit disable/demagnetize commands on shutdown.
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
    args = parser.parse_args(rospy.myargv()[1:])
    config = RunnerConfig.load(args.config, args.data_root or None)
    rospy.init_node("hex_runner")
    RunnerNode(config, args.output_dir or None).spin()
