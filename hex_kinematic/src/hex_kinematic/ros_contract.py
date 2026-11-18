"""Pure named mapping for the device ROS contracts; does not import ROS or Gym."""
import numpy as np
from .conventions import LEG_NAMES, MOTOR_JOINT_NAMES



def read_motor(message, suffix, leg_names=LEG_NAMES):
    return np.array([[getattr(getattr(message, leg), joint + "_" + suffix)
                      for joint in MOTOR_JOINT_NAMES] for leg in leg_names], dtype=float)


def write_motor(message, suffix, values, leg_names=LEG_NAMES):
    for i, leg in enumerate(leg_names):
        for j, joint in enumerate(MOTOR_JOINT_NAMES):
            setattr(getattr(message, leg), joint + "_" + suffix, float(values[i, j]))


def read_servo(message, leg_names=LEG_NAMES):
    return np.deg2rad([getattr(message, leg) for leg in leg_names])


def write_servo(message, radians, leg_names=LEG_NAMES):
    for leg, value in zip(leg_names, np.rad2deg(radians)):
        setattr(message, leg, float(value))


def read_servo_enabled(message, leg_names=LEG_NAMES):
    return np.array([message.enable and getattr(message, "enable_" + leg)
                     for leg in leg_names], dtype=bool)


def write_servo_enabled(message, enabled, leg_names=LEG_NAMES):
    for leg, value in zip(leg_names, enabled):
        setattr(message, "enable_" + leg, bool(value))


def read_magnetic(message, prefix="", leg_names=LEG_NAMES):
    return np.array([getattr(message, prefix + leg) for leg in leg_names],
                    dtype=float if prefix == "attached_" else bool)


def write_magnetic(message, values, prefix="", leg_names=LEG_NAMES):
    for leg, value in zip(leg_names, values):
        setattr(message, prefix + leg, float(value) if prefix == "attached_" else bool(value))


WIRE_LEG_NAMES = ("lf", "lm", "lb", "rf", "rm", "rb")


def read_valid(message, leg_names=LEG_NAMES, motor=False):
    values = np.asarray(message.valid, dtype=bool)
    if motor:
        values = values.reshape(6, 3).all(axis=1)
    return values[[WIRE_LEG_NAMES.index(name) for name in leg_names]]


def write_valid(message, leg_names=LEG_NAMES, motor=False):
    values = [name in leg_names for name in WIRE_LEG_NAMES]
    message.valid = np.repeat(values, 3).tolist() if motor else values
