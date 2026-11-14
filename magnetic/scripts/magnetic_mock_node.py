#!/usr/bin/env python3
"""Report successful magnetic commands while logging real serial traffic."""
import math
from threading import Lock

import rospy
import serial
from magnetic.msg import MagneticCmd, MagneticState
from magnetic.protocol import LEG_NAMES, encode_command, validate_channel_order


class MagneticMockNode:
    def __init__(self):
        self.channel_order = rospy.get_param("~channel_order")
        validate_channel_order(self.channel_order)
        state_hz = rospy.get_param("~state_hz", 50.0)
        if not math.isfinite(state_hz) or state_hz <= 0:
            raise ValueError("state_hz must be positive and finite")
        self.port = serial.Serial(rospy.get_param("~port"), rospy.get_param("~baud_rate", 115200),
                                  timeout=0.1, write_timeout=0.5)
        self.lock = Lock()
        self.enabled = (False,) * 6
        self.publisher = rospy.Publisher("hex/magnetic/state", MagneticState, queue_size=1)
        self.subscriber = rospy.Subscriber("hex/magnetic/command", MagneticCmd, self.command)
        self.timer = rospy.Timer(rospy.Duration(1.0 / state_hz), self.publish)

    def command(self, message):
        data = encode_command(message, self.channel_order)
        try:
            with self.lock:
                self.port.write(data)
                self.enabled = tuple(getattr(message, leg) for leg in LEG_NAMES)
            rospy.loginfo("TX %r", data)
        except (serial.SerialException, OSError) as error:
            rospy.logfatal("magnetic mock serial write failed: %s", error)
            rospy.signal_shutdown(str(error))

    def publish(self, event):
        with self.lock:
            enabled = self.enabled
        message = MagneticState()
        message.header.stamp = rospy.Time.now()
        for leg, value in zip(LEG_NAMES, enabled):
            setattr(message, "attached_" + leg, float(value))
            setattr(message, "enabled_" + leg, value)
            setattr(message, "valid_" + leg, True)
        self.publisher.publish(message)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                data = self.port.read(max(1, self.port.in_waiting))
                if data:
                    rospy.loginfo("RX %r", data)
        finally:
            self.timer.shutdown()
            self.subscriber.unregister()
            with self.lock:
                self.port.close()


def main():
    rospy.init_node("magnetic")
    try:
        MagneticMockNode().spin()
    except (serial.SerialException, OSError, ValueError, KeyError) as error:
        rospy.logfatal("magnetic mock driver failed: %s", error)
        rospy.signal_shutdown(str(error))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
