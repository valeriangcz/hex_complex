#!/usr/bin/env python3
"""ROS1 entry point; all startup options come from command-line args."""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as PathMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

from hex_isaacgym.coordinator import RunPhase
from hex_isaacgym.config import SceneConfig
from hex_isaacgym.runtime import build_coordinator
from hex_kinematic import COMMAND_JOINT_NAMES, LEG_NAMES
from hex_kinematic.se3_utils import transform_to_xyzw


def _bool_arg(value):
    if value.lower() in ("true", "1"):
        return True
    if value.lower() in ("false", "0"):
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--path")
    parser.add_argument("--data-root")
    parser.add_argument("--device")
    parser.add_argument("--output-dir")
    parser.add_argument("--headless", type=_bool_arg, nargs="?", const=True, default=True)
    parser.add_argument("--viewer", dest="headless", action="store_false")
    parser.add_argument("--publish-clock", type=_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--wall-rate-hz", type=float, default=0.0)
    args = parser.parse_args(rospy.myargv()[1:] if argv is None else argv)
    if not np.isfinite(args.wall_rate_hz) or args.wall_rate_hz < 0:
        parser.error("--wall-rate-hz must be finite and nonnegative")
    # roslaunch passes empty optional overrides as quoted empty strings.
    for name in ("path", "data_root", "device", "output_dir"):
        setattr(args, name, getattr(args, name) or None)
    return args


class ExpertSimNode:
    def __init__(self, args):
        self.publish_clock = args.publish_clock
        self.wall_rate_hz = args.wall_rate_hz
        scene = SceneConfig.from_yaml(args.scene_config, data_root=args.data_root)
        self.coordinator, self.document = build_coordinator(
            scene, path=args.path, device=args.device, headless=args.headless
        )
        rigid_positions = self.coordinator.backend.get_rigid_body_positions()
        self.initial_unique_body_positions = len(
            np.unique(np.round(rigid_positions, 5), axis=0)
        )
        self.output_dir = Path(args.output_dir or scene.data_root / "runs").expanduser().resolve()
        self.joint_names = [
            "j_{}_{}".format(leg.lower(), joint)
            for leg in LEG_NAMES for joint in COMMAND_JOINT_NAMES
        ]
        self.joint_pub = rospy.Publisher("/hex/joint_states", JointState, queue_size=2)
        self.pose_pub = rospy.Publisher("/hex/base_pose", PoseStamped, queue_size=2)
        self.path_pub = (
            rospy.Publisher("/hex/reference_path", PathMessage, queue_size=1, latch=True)
            if self.coordinator.path else None
        )
        self.diag_pub = rospy.Publisher("/hex/diagnostics", DiagnosticArray, queue_size=2)
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=2) if self.publish_clock else None
        self.reset_service = rospy.Service("/hex/reset", Trigger, self._reset_callback)
        self.command_sub = (
            rospy.Subscriber("/hex/cmd_vel", Twist, self._command_callback, queue_size=1)
            if scene.expert_type == "climb" else None
        )
        self._publish_path()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "hex expert sim: scene=%s expert=%s path=%s device=%s headless=%s "
            "asset=%d bodies/%d dofs, %d distinct initial body positions",
            args.scene_config, scene.expert_type, args.path or scene.se3_path,
            self.coordinator.backend.config.device, args.headless,
            self.coordinator.backend.num_bodies,
            self.coordinator.backend.num_dofs,
            self.initial_unique_body_positions,
        )

    def _command_callback(self, message):
        try:
            self.coordinator.set_velocity_command(
                (message.linear.x, message.linear.y, message.angular.z)
            )
        except (ValueError, RuntimeError) as exc:
            rospy.logwarn("rejected /hex/cmd_vel: %s", exc)

    def _reset_callback(self, _request):
        accepted = self.coordinator.request_reset("ROS /hex/reset")
        return TriggerResponse(
            success=accepted,
            message="reset queued for control loop" if accepted else "coordinator is closed",
        )

    def spin(self):
        rate = rospy.Rate(self.wall_rate_hz) if self.wall_rate_hz > 0.0 else None
        while not rospy.is_shutdown():
            state = self.coordinator.tick()
            if state is not None:
                self._publish_state(state)
            if rate is not None:
                rate.sleep()

    def _stamp(self, sim_time):
        if self.clock_pub is not None:
            stamp = rospy.Time.from_sec(float(sim_time))
            self.clock_pub.publish(Clock(clock=stamp))
            return stamp
        return rospy.Time.now()

    def _publish_state(self, state):
        stamp = self._stamp(state.sim_time)
        position, quaternion = transform_to_xyzw(state.W_T_R)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion)
        self.pose_pub.publish(pose)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = self.joint_names
        joints.position = state.q.reshape(-1).tolist()
        joints.velocity = state.q_dot.reshape(-1).tolist()
        effort = np.zeros((6, 4), dtype=np.float64)
        effort[:, :3] = state.q_torque
        joints.effort = effort.reshape(-1).tolist()
        self.joint_pub.publish(joints)

        status = self.coordinator.status
        diagnostic = DiagnosticStatus()
        diagnostic.name = "hex_expert_sim"
        diagnostic.hardware_id = "isaacgym/hex_magnetic"
        diagnostic.level = (
            DiagnosticStatus.ERROR if status.phase == RunPhase.FAILED
            else DiagnosticStatus.OK
        )
        diagnostic.message = status.phase.value
        diagnostic.values = [
            KeyValue("sim_time", "{:.9f}".format(state.sim_time)),
            KeyValue("run_elapsed", "{:.9f}".format(state.run_elapsed)),
            KeyValue("reset_count", str(status.reset_count)),
            KeyValue("settling_remaining", str(status.settling_remaining)),
            KeyValue("asset_body_count", str(self.coordinator.backend.num_bodies)),
            KeyValue("asset_dof_count", str(self.coordinator.backend.num_dofs)),
            KeyValue("initial_unique_body_positions", str(self.initial_unique_body_positions)),
            KeyValue("adhesion_force", np.array2string(state.adhesion_force, precision=3)),
            KeyValue("failure_reason", status.failure_reason),
            KeyValue("completion_reason", status.completion_reason),
        ]
        for key, value in self.coordinator.adapter.diagnostics().items():
            diagnostic.values.append(KeyValue(key, json.dumps(value)))
        # PhysX reaction forces are diagnostics only; the climb expert estimates
        # contact from applied motor torques without receiving these forces.
        diagnostic.values.append(KeyValue(
            "contact_force_norm", json.dumps(np.linalg.norm(state.contact_forces, axis=1).tolist())
        ))
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [diagnostic]
        self.diag_pub.publish(array)

    def _publish_path(self):
        if self.path_pub is None:
            return
        message = PathMessage()
        message.header.frame_id = "world"
        message.header.stamp = rospy.Time.now()
        for pose_value in self.coordinator.path:
            position, quaternion = transform_to_xyzw(pose_value.A)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion)
            message.poses.append(pose)
        self.path_pub.publish(message)

    def shutdown(self):
        if not hasattr(self, "coordinator"):
            return
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            report = {
                "phase": self.coordinator.phase.value,
                "reset_count": self.coordinator.reset_count,
                "failure_reason": self.coordinator.failure_reason,
                "completion_reason": self.coordinator.completion_reason,
                "events": self.coordinator.events,
                "expert": self.coordinator.adapter.diagnostics(),
            }
            target = self.output_dir / "expert_sim_{:.0f}.json".format(time.time())
            target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        self.coordinator.close()


def main():
    rospy.init_node("hex_expert_sim", disable_signals=False)
    node = ExpertSimNode(parse_args())
    node.spin()


if __name__ == "__main__":
    main()

