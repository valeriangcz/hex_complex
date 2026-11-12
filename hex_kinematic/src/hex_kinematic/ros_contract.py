"""Pure named mapping for the device ROS contracts; does not import ROS or Gym."""
import numpy as np
from .conventions import LEG_NAMES, MOTOR_JOINT_NAMES



def read_motor(message, suffix):
    return np.array([[getattr(getattr(message, leg), joint + "_" + suffix)
                      for joint in MOTOR_JOINT_NAMES] for leg in LEG_NAMES], dtype=float)


def write_motor(message, suffix, values):
    for i, leg in enumerate(LEG_NAMES):
        for j, joint in enumerate(MOTOR_JOINT_NAMES):
            setattr(getattr(message, leg), joint + "_" + suffix, float(values[i, j]))


def read_servo(message):
    return np.deg2rad([getattr(message, leg) for leg in LEG_NAMES])


def write_servo(message, radians):
    for leg, value in zip(LEG_NAMES, np.rad2deg(radians)):
        setattr(message, leg, float(value))


def read_servo_enabled(message):
    return np.array([message.enable and getattr(message, "enable_" + leg)
                     for leg in LEG_NAMES], dtype=bool)


def write_servo_enabled(message, enabled):
    for leg, value in zip(LEG_NAMES, enabled):
        setattr(message, "enable_" + leg, bool(value))


def read_magnetic(message, prefix=""):
    return np.array([getattr(message, prefix + leg) for leg in LEG_NAMES], dtype=bool)


def write_magnetic(message, values, prefix=""):
    for leg, value in zip(LEG_NAMES, values):
        setattr(message, prefix + leg, bool(value))
