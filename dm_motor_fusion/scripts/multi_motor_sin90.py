#!/usr/bin/env python3
import math

import rospy
from std_msgs.msg import Float64MultiArray


class MultiMotorSinNode:
    def __init__(self):
        rospy.init_node("multi_motor_sin90")

        self.publish_hz = rospy.get_param("~publish_hz", 200.0)
        self.mode = int(rospy.get_param("~mode", 2))  # 2: Traj_follow
        self.amp_deg = float(rospy.get_param("~amplitude_deg", 90.0))
        self.base_freq_hz = float(rospy.get_param("~base_freq_hz", 0.15))
        self.freq_step_hz = float(rospy.get_param("~freq_step_hz", 0.02))
        self.phase_step_deg = float(rospy.get_param("~phase_step_deg", 20.0))

        # dm_motor_fusion uses radians for position command.
        self.amp_rad = self.amp_deg * math.pi / 180.0
        self.phase_step_rad = self.phase_step_deg * math.pi / 180.0

        self.legs = [
            ("LF", [1, 2, 3]),
            ("LM", [4, 5, 6]),
            ("LB", [7, 8, 9]),
            ("RF", [10, 11, 12]),
            ("RM", [13, 14, 15]),
            ("RB", [16, 17, 18]),
        ]

        self.pubs = {
            name: rospy.Publisher(f"/{name}/sita_des", Float64MultiArray, queue_size=10)
            for name, _ in self.legs
        }

    def _motor_signal(self, motor_id, t):
        # Every motor has different frequency and phase.
        freq = self.base_freq_hz + (motor_id - 1) * self.freq_step_hz
        omega = 2.0 * math.pi * freq
        phase = (motor_id - 1) * self.phase_step_rad
        pos = self.amp_rad * math.sin(omega * t + phase)
        vel = self.amp_rad * omega * math.cos(omega * t + phase)
        return pos, vel

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time

            for leg_name, ids in self.legs:
                p1, v1 = self._motor_signal(ids[0], t)
                p2, v2 = self._motor_signal(ids[1], t)
                p3, v3 = self._motor_signal(ids[2], t)

                msg = Float64MultiArray()
                # Format required by existing C++ node:
                # [mode, pos1, pos2, pos3, vel1, vel2, vel3]
                msg.data = [self.mode, p1, p2, p3, v1, v2, v3]
                self.pubs[leg_name].publish(msg)

            rate.sleep()


if __name__ == "__main__":
    node = MultiMotorSinNode()
    node.run()
