#!/usr/bin/env python3
"""Verify one-shot Twist updates and reset against an isolated ROS master."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def wait_until(predicate, description, timeout=45.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("timeout waiting for " + description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=11431)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(args.port)
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    os.environ["ROS_LOG_DIR"] = "/tmp/hex_climb_ros_logs"
    os.environ["ROS_HOME"] = "/tmp/hex_climb_ros_home"
    os.environ["ROS_PACKAGE_PATH"] = str(workspace / "src") + os.pathsep + os.environ.get("ROS_PACKAGE_PATH", "")
    sources = [str(workspace / "src" / package / "src") for package in (
        "hex_isaacgym", "hex_expert", "hex_kinematic", "map_robot_voxles", "hex_se3"
    )]
    os.environ["PYTHONPATH"] = os.pathsep.join(sources + [os.environ.get("PYTHONPATH", "")])
    sys.path[:0] = sources
    import rosgraph
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Trigger

    latest = {}
    processes = []
    logs = []
    try:
        for name, command in (
            ("master", ["roscore", "-p", str(args.port)]),
            ("node", [sys.executable, str(workspace / "src/hex_isaacgym/scripts/expert_sim_node.py"),
                      "--scene-config", str(workspace / "src/hex_isaacgym/config/expert_climb_scene.yaml"),
                      "--device", "cpu", "--headless", "--output-dir", "/tmp/hex_climb_ros_runs",
                      "--wall-rate-hz", "100.0"]),
        ):
            log = open("/tmp/hex_climb_ros_{}.log".format(name), "w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
            if name == "master":
                wait_until(rosgraph.is_master_online, "isolated ROS master", timeout=15)
                rospy.init_node("hex_climb_ros_test", anonymous=True, disable_signals=True)

        def feedback(message):
            for status in message.status:
                if status.name == "hex_expert_sim":
                    latest.update({entry.key: entry.value for entry in status.values})
                    latest["phase"] = status.message

        subscriber = rospy.Subscriber("/hex/diagnostics", DiagnosticArray, feedback, queue_size=1)
        publisher = rospy.Publisher("/hex/cmd_vel", Twist, queue_size=1)
        wait_until(lambda: latest.get("phase") == "RUNNING", "climb RUNNING")
        wait_until(lambda: publisher.get_num_connections() > 0, "velocity subscriber")
        assert json.loads(latest["command"]) == [.02, 0., 0.]
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = .01, .005, .03
        publisher.publish(command)  # exactly one message; no refresh/timeout
        wait_until(lambda: json.loads(latest.get("command", "null")) == [.01, .005, .03], "updated command")
        start = float(latest["run_elapsed"])
        wait_until(lambda: float(latest["run_elapsed"]) > start + .2, "command persistence")
        assert json.loads(latest["command"]) == [.01, .005, .03]
        command.linear.x = float("nan")
        publisher.publish(command)
        start = float(latest["run_elapsed"])
        wait_until(lambda: float(latest["run_elapsed"]) > start + .1, "invalid command rejection")
        assert json.loads(latest["command"]) == [.01, .005, .03]
        publisher.publish(Twist())
        wait_until(lambda: json.loads(latest["command"]) == [0., 0., 0.], "zero command")
        resets = int(latest["reset_count"])
        rospy.wait_for_service("/hex/reset", timeout=10)
        assert rospy.ServiceProxy("/hex/reset", Trigger)().success
        wait_until(lambda: int(latest["reset_count"]) > resets, "queued reset")
        assert json.loads(latest["command"]) == [.02, 0., 0.]
        assert "follow_path_index" not in latest
        assert "/hex/reference_path" not in dict(rospy.get_published_topics())
        assert latest["phase"] != "FAILED"
        report = {"status": "PASS", "checks": ["startup fixed command", "one-shot Twist update",
                  "last command persists", "nonfinite command rejected", "zero command",
                  "reset restores initial command", "climb-specific diagnostics/no reference path"]}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        subscriber.unregister()
        publisher.unregister()
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
