#!/usr/bin/env python3
"""Bridge the six-channel electromagnet serial board to ROS."""
from threading import Lock

import rospy
import serial
from magnetic.msg import MagneticCmd, MagneticState
from magnetic.protocol import Protocol


class MagneticNode:
    def __init__(self):
        self.protocol = Protocol(rospy.get_param("~"))
        self.port = serial.Serial(rospy.get_param("~port"), rospy.get_param("~baud_rate", 115200),
                                  timeout=0.1, write_timeout=0.5)
        self.write_lock = Lock()
        self.publisher = rospy.Publisher("hex/magnetic/state", MagneticState, queue_size=1)
        self.subscriber = rospy.Subscriber("hex/magnetic/command", MagneticCmd,
                                           self.command, queue_size=1)

    def command(self, message):
        try:
            with self.write_lock:
                self.port.write(self.protocol.encode(message))
        except (serial.SerialException, OSError) as error:
            rospy.logfatal("magnetic serial write failed: %s", error)
            rospy.signal_shutdown(str(error))

    def spin(self):
        try:
            while not rospy.is_shutdown():
                data = self.port.read(max(1, self.port.in_waiting))
                for frame in self.protocol.feed(data):
                    try:
                        values = self.protocol.decode(frame)
                    except (ValueError, KeyError) as error:
                        rospy.logwarn_throttle(1.0, "magnetic malformed frame: %s", error)
                        continue
                    message = MagneticState()
                    message.header.stamp = rospy.Time.now()
                    for leg, (ratio, enabled, valid, raw, output) in values.items():
                        setattr(message, "attached_" + leg, ratio)
                        setattr(message, "enabled_" + leg, enabled)
                        setattr(message, "valid_" + leg, valid)
                        if not valid:
                            rospy.logwarn_throttle_identical(
                                1.0, "magnetic %s invalid: adc_raw=%s, valid range=[%s, %s], io_state=%s"
                                % (leg, raw, self.protocol.lower, self.protocol.upper, output))
                    self.publisher.publish(message)
        finally:
            self.subscriber.unregister()
            with self.write_lock:
                self.port.close()


def main():
    rospy.init_node("magnetic")
    try:
        MagneticNode().spin()
    except (serial.SerialException, OSError, ValueError, KeyError) as error:
        rospy.logfatal("magnetic driver failed: %s", error)
        rospy.signal_shutdown(str(error))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
