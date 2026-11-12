#!/usr/bin/env python3
import math

import rospy
from dm_motor_fusion.msg import LegCommand, MultiLegCommand


class MultiMotorSinNode:
    def __init__(self):
        rospy.init_node("multi_motor_sin90")

        self.publish_hz = rospy.get_param("~publish_hz", 200.0)
        self.option = int(rospy.get_param("~option", 2))  # 2: pos_vel / 轨迹跟踪
        self.amp_deg = float(rospy.get_param("~amplitude_deg", 90.0))
        self.base_freq_hz = float(rospy.get_param("~base_freq_hz", 0.15))
        self.freq_step_hz = float(rospy.get_param("~freq_step_hz", 0.02))
        self.phase_step_deg = float(rospy.get_param("~phase_step_deg", 20.0))
        self.kp = float(rospy.get_param("~kp", 100.0))
        self.kd = float(rospy.get_param("~kd", 1.0))

        # dm_motor_fusion 使用弧度。
        self.amp_rad = self.amp_deg * math.pi / 180.0
        self.phase_step_rad = self.phase_step_deg * math.pi / 180.0

        # 组内 CAN ID 从小到大依次为：脚踝, 膝关节, 大腿
        self.legs = [
            ("lf", [1, 2, 3]),
            ("lm", [4, 5, 6]),
            ("lb", [7, 8, 9]),
            ("rf", [10, 11, 12]),
            ("rm", [13, 14, 15]),
            ("rb", [16, 17, 18]),
        ]

        self.pub = rospy.Publisher("/hex/motor/command", MultiLegCommand, queue_size=10)

    def _motor_signal(self, motor_id, t):
        # 每个电机频率与相位不同。
        freq = self.base_freq_hz + (motor_id - 1) * self.freq_step_hz
        omega = 2.0 * math.pi * freq
        phase = (motor_id - 1) * self.phase_step_rad
        pos = self.amp_rad * math.sin(omega * t + phase)
        vel = self.amp_rad * omega * math.cos(omega * t + phase)
        return pos, vel

    def _leg_command(self, ids, t):
        # ids[0]=脚踝, ids[1]=膝关节, ids[2]=大腿（符号方向由节点内 pn 系数处理）
        ankle_pos, ankle_vel = self._motor_signal(ids[0], t)
        knee_pos, knee_vel = self._motor_signal(ids[1], t)
        thigh_pos, thigh_vel = self._motor_signal(ids[2], t)

        leg = LegCommand()
        leg.ankle_pos, leg.ankle_vel, leg.ankle_tau = ankle_pos, ankle_vel, 0.0
        leg.knee_pos, leg.knee_vel, leg.knee_tau = knee_pos, knee_vel, 0.0
        leg.thigh_pos, leg.thigh_vel, leg.thigh_tau = thigh_pos, thigh_vel, 0.0
        leg.ankle_kp = leg.knee_kp = leg.thigh_kp = self.kp
        leg.ankle_kd = leg.knee_kd = leg.thigh_kd = self.kd
        return leg

    def run(self):
        rate = rospy.Rate(self.publish_hz)
        start_time = rospy.Time.now().to_sec()
        msg = MultiLegCommand()

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - start_time
            msg.option = self.option
            for leg_name, ids in self.legs:
                setattr(msg, leg_name, self._leg_command(ids, t))
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    node = MultiMotorSinNode()
    node.run()
