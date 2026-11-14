"""Publish body-frame unit gravity direction from WIT native quaternions."""
import time

import rospy
import serial
from std_msgs.msg import Float64MultiArray

from .wit_imu import ImuConfig, WitDevice, native_quaternion, gravity_direction_body


def run(config):
    publisher = rospy.Publisher('hex/perception/gravity', Float64MultiArray, queue_size=1)
    with serial.Serial(config.port, config.baud, timeout=min(0.02, config.serial_timeout),
                       write_timeout=1.0, exclusive=True) as port:
        device = WitDevice(port, config)
        try:
            device.configure()
            rospy.loginfo('Native IMU gravity: %s at %d baud, target %.1f Hz',
                          config.port, config.baud, config.publish_hz)
            last_valid = time.monotonic()
            next_publish = last_valid
            pending = None
            while not rospy.is_shutdown():
                for frame in device.frames():
                    if frame[1] != 0x59:
                        continue
                    try:
                        q = native_quaternion(frame)
                    except ValueError as error:
                        rospy.logwarn_throttle(1.0, 'Invalid native IMU quaternion: %s', error)
                        continue
                    last_valid = time.monotonic()
                    pending = gravity_direction_body(q, config.imu_to_body_rotation)
                # SIGINT can initiate ROS shutdown while a serial read is pending.
                # Cleanup time must not be interpreted as loss of sensor data.
                if rospy.is_shutdown():
                    break
                now = time.monotonic()
                if now - last_valid > config.serial_timeout:
                    raise RuntimeError('No valid native IMU quaternion within serial_timeout')
                if pending is not None and now >= next_publish:
                    publisher.publish(Float64MultiArray(data=pending.tolist()))
                    pending = None  # Never republish an already consumed sample.
                    next_publish = max(next_publish + 1.0 / config.publish_hz, now)
        finally:
            try:
                device.restore()
            except Exception as error:
                rospy.logerr('Could not restore IMU output configuration: %s', error)
                raise


def main():
    rospy.init_node('imu_gravity')
    try:
        run(ImuConfig.load(rospy.get_param('~config')))
    except (ValueError, TypeError, OSError, RuntimeError, serial.SerialException) as error:
        rospy.logfatal('IMU gravity node stopped: %s', error)
        raise SystemExit(1)
