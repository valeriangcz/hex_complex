"""Independent device simulator. No expert, planner or control coordinator."""
import argparse
from threading import Lock
import time
import json
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger, TriggerResponse
from dm_motor_fusion.msg import MultiLegCommand, MultiLegState
from feetech_servo.msg import servo_cmd, servo_state
from magnetic.msg import MagneticCmd, MagneticState
from hex_kinematic.ros_contract import write_valid, read_motor, read_servo, read_servo_enabled, read_magnetic, write_magnetic, write_motor, write_servo
from hex_kinematic.se3_utils import transform_to_xyzw
from .config import SimulationConfig
from .types import ActuatorCommand
from .backend import IsaacGymBackend
from .sensors import attachment_ratio, gravity_direction_body


class SimulationNode:
    def __init__(self, config, headless=True, publish_clock=False, wall_rate_hz=None):
        self.config = config
        self.backend = IsaacGymBackend(config.backend, headless=headless, initial_state=config.initial_state)
        self.lock = Lock()
        self.command = self._initial_command()
        self.pending_reset = False
        self.initial_state_pending = True
        self.backend.reset(config.initial_state, config.magnets)
        self.backend.set_reference_path(config.reference_path)
        self.reset_count = 0
        self.wall_rate_hz = 1. / config.backend.control_dt if wall_rate_hz is None else wall_rate_hz
        self.motor_pub = rospy.Publisher("hex/motor/state", MultiLegState, queue_size=1)
        self.servo_pub = rospy.Publisher("hex/servo/state", servo_state, queue_size=1)
        self.magnetic_pub = rospy.Publisher("hex/magnetic/state", MagneticState, queue_size=1)
        self.pose_pub = rospy.Publisher("hex/perception/pose", PoseStamped, queue_size=1)
        self.gravity_pub = rospy.Publisher("hex/perception/gravity", Float64MultiArray, queue_size=1)
        # self.joints_pub = rospy.Publisher("hex/joint_states", JointState, queue_size=1)
        self.diag_pub = rospy.Publisher("hex/sim/diagnostics", DiagnosticArray, queue_size=1)
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1) if publish_clock else None
        self.subscribers = [
            rospy.Subscriber("hex/motor/command", MultiLegCommand, self._motor, queue_size=1),
            rospy.Subscriber("hex/servo/command", servo_cmd, self._servo, queue_size=1),
            rospy.Subscriber("hex/magnetic/command", MagneticCmd, self._magnetic, queue_size=1)]
        self.reset_service = rospy.Service("hex/sim/reset", Trigger, self._reset)
        rospy.on_shutdown(self.backend.close)

    def _initial_command(self):
        return ActuatorCommand(self.config.initial_state.q.copy(), np.zeros((len(self.config.leg_names), 3)), self.config.magnets.copy(),
                               servo_enabled=np.full(len(self.config.leg_names), self.config.servo_enabled, dtype=bool))

    def _motor(self, message):
        if message.option not in (-1, 0, 1, 2):
            rospy.logwarn_throttle(1., "sim rejects unsupported motor mode %s", message.option)
            return
        try:
            with self.lock:
                if message.option in (1, 2):
                    pos, tau, velocity, kp, kd = [read_motor(message, name, self.config.leg_names)
                                                for name in ("pos", "tau", "vel", "kp", "kd")]
                    if not all(np.isfinite(value).all() for value in (pos, tau, velocity, kp, kd)):
                        raise ValueError("motor fields must be finite")
                    if (kp < 0).any() or (kd < 0).any():
                        raise ValueError("motor gains must be nonnegative")
                    self.command.q_des[:, :3] = pos
                    self.command.tau_ff = tau
                    self.command.motor_velocity = velocity
                    self.command.motor_kp = kp
                    self.command.motor_kd = kd
                else:
                    self.command.motor_kp.fill(0.)
                    self.command.motor_kd.fill(0.)
                    self.command.motor_velocity.fill(0.)
                    self.command.tau_ff.fill(0.)
        except ValueError as error:
            rospy.logwarn_throttle(1., "invalid motor command: %s", error)

    def _servo(self, message):
        try:
            with self.lock:
                enabled = read_servo_enabled(message, self.config.leg_names)
                angles = read_servo(message, self.config.leg_names)
                if not np.isfinite(angles[enabled]).all():
                    raise ValueError("enabled servo angles must be finite")
                self.command.q_des[enabled, 3] = angles[enabled]
                self.command.servo_enabled = enabled
        except ValueError as error:
            rospy.logwarn_throttle(1., "invalid servo command: %s", error)

    def _magnetic(self, message):
        with self.lock:
            self.command.adhesions = read_magnetic(message, leg_names=self.config.leg_names)

    def _reset(self, _request):
        with self.lock:
            if self.pending_reset:
                return TriggerResponse(success=False, message="physical reset already queued")
            self.pending_reset = True
        return TriggerResponse(success=True, message="physical reset queued; suspend gait and reset runner separately")

    def tick(self):
        with self.lock:
            reset_state = None
            if self.pending_reset:
                self.config = self.config.reload_initial_state()
                self.backend.set_reference_path(self.config.reference_path)
                reset_state = self.backend.reset(self.config.initial_state, self.config.magnets)
                self.command = self._initial_command()
                self.initial_state_pending = False
                self.pending_reset = False
                self.reset_count += 1
            if self.initial_state_pending:
                reset_state = self.backend.get_state()
                self.initial_state_pending = False
            command = self.command
        state = reset_state if reset_state is not None else self.backend.step(command)
        self._publish(state, command)

    def _publish(self, state, command):
        stamp = rospy.Time.from_sec(state.sim_time) if self.clock_pub is not None else rospy.Time.now()
        if self.clock_pub is not None:
            self.clock_pub.publish(Clock(clock=stamp))
        motor = MultiLegState(option=1)  # Simulator always uses torque control.
        motor.header.stamp = stamp
        write_valid(motor, self.config.leg_names, motor=True)
        for suffix, values in (("pos", state.q[:, :3]), ("vel", state.q_dot[:, :3]), ("tau", state.q_torque)):
            write_motor(motor, suffix, values, self.config.leg_names)
        self.motor_pub.publish(motor)
        servo = servo_state()
        write_valid(servo, self.config.leg_names)
        servo.header.stamp = stamp
        write_servo(servo, state.q[:, 3], self.config.leg_names)
        self.servo_pub.publish(servo)
        attached = attachment_ratio(state.adhesion_force, self.config.backend.suction_force_max)
        magnetic = MagneticState()
        write_magnetic(magnetic, command.adhesions, "enabled_", self.config.leg_names)
        write_magnetic(magnetic, attached, "attached_", self.config.leg_names)
        write_magnetic(magnetic, [True] * len(self.config.leg_names), "valid_", self.config.leg_names)
        magnetic.header.stamp = stamp
        self.magnetic_pub.publish(magnetic)
        position, quaternion = transform_to_xyzw(state.W_T_R)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion)
        self.pose_pub.publish(pose)
        direction = gravity_direction_body(state.W_T_R[:3, :3], self.config.backend.gravity)
        self.gravity_pub.publish(Float64MultiArray(data=direction.tolist()))
        status = DiagnosticStatus(name="hex_sim", hardware_id="isaacgym", level=0, message="RUNNING")
        values = dict(sim_time=state.sim_time, run_elapsed=state.run_elapsed, reset_count=self.reset_count,
                      servo_enabled=command.servo_enabled.tolist(),
                      adhesion_force=state.adhesion_force.tolist(), contact_force_norm=np.linalg.norm(state.contact_forces, axis=1).tolist(),
                      motor_torque_max=float(np.abs(state.q_torque).max()),
                      servo_torque_max=float(self.backend.last_torques[self.backend.command_indices.view(len(self.config.leg_names), 4)[:, 3]].abs().max().item()))
        status.values = [KeyValue(key, json.dumps(value)) for key, value in values.items()]
        message = DiagnosticArray(status=[status])
        message.header.stamp = stamp
        self.diag_pub.publish(message)

    def spin(self):
        deadline = time.monotonic()
        while not rospy.is_shutdown():
            self.tick()
            if self.wall_rate_hz > 0:
                deadline += 1. / self.wall_rate_hz
                delay = deadline - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    deadline = time.monotonic()


def bool_arg(value):
    if value.lower() not in ("true", "false"):
        raise argparse.ArgumentTypeError("expected true or false")
    return value.lower() == "true"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--to-sim-info", required=True)
    parser.add_argument("--device", default="")
    parser.add_argument("--headless", type=bool_arg, default=True)
    parser.add_argument("--publish-clock", type=bool_arg, default=False)
    parser.add_argument("--wall-rate-hz", type=float)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.wall_rate_hz is not None and (not np.isfinite(args.wall_rate_hz) or args.wall_rate_hz < 0):
        parser.error("wall-rate-hz must be finite and nonnegative")
    config = SimulationConfig.from_yaml(args.config, args.device or None, args.to_sim_info)
    rospy.init_node("hex_sim")
    SimulationNode(config, args.headless, args.publish_clock, args.wall_rate_hz).spin()
