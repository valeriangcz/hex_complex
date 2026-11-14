"""WIT native quaternion protocol. No ROS or Euler-angle dependency."""
from dataclasses import dataclass, field
from pathlib import Path
import math
import struct
import time

import numpy as np
import yaml


RATE_CODES = {0.2: 1, 0.5: 2, 1: 3, 2: 4, 5: 5, 10: 6, 20: 7, 50: 8, 100: 9, 200: 11}


@dataclass
class ImuConfig:
    port: str = '/dev/ttyUSB0'
    baud: int = 115200
    publish_hz: float = 100.0
    serial_timeout: float = 0.5
    imu_to_body_rotation: object = field(default_factory=lambda: np.eye(3))

    def __post_init__(self):
        if not isinstance(self.port, str) or not self.port:
            raise ValueError('port must be a nonempty string')
        if isinstance(self.baud, bool) or not isinstance(self.baud, int) or self.baud <= 0:
            raise ValueError('baud must be a positive integer')
        if isinstance(self.publish_hz, bool) or self.publish_hz not in RATE_CODES:
            raise ValueError('publish_hz must be one of ' + str(list(RATE_CODES)))
        if not math.isfinite(self.serial_timeout) or self.serial_timeout <= 1 / self.publish_hz:
            raise ValueError('serial_timeout must exceed one sample period')
        # 8N1 uses ten line bits per byte; reserve headroom for register replies.
        if self.publish_hz * 11 * 10 > self.baud * 0.9:
            raise ValueError('baud is insufficient for publish_hz native quaternion packets')
        rotation = np.asarray(self.imu_to_body_rotation, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0)):
            raise ValueError('imu_to_body_rotation must be a proper 3x3 rotation')
        self.imu_to_body_rotation = rotation.copy()

    @classmethod
    def load(cls, filename):
        values = yaml.safe_load(Path(filename).expanduser().read_text())
        if not isinstance(values, dict):
            raise ValueError('IMU YAML must contain a mapping')
        return cls(**values)


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:11])
            if sum(frame[:10]) & 255 != frame[10]:
                self.bad_checksums += 1
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:11]
        return frames


def native_quaternion(frame):
    if len(frame) != 11 or frame[:2] != b'\x55\x59' or sum(frame[:10]) & 255 != frame[10]:
        raise ValueError('expected valid native 0x59 frame')
    q = np.array(struct.unpack('<4h', frame[2:10]), dtype=float) / 32768
    norm = np.linalg.norm(q)
    if not 0.9 <= norm <= 1.1:
        raise ValueError('invalid native quaternion norm')
    return q / norm  # Hamilton w,x,y,z, directly from the module.


def gravity_direction_body(q, imu_to_body_rotation):
    """Project world down into IMU, then body; q is normalized w,x,y,z."""
    w, x, y, z = q
    # R_WI.T @ [0,0,-1] is minus the third row of R_WI.
    gravity_imu = np.array([2*(w*y-x*z), -2*(y*z+w*x), 2*(x*x+y*y)-1])
    direction = imu_to_body_rotation @ gravity_imu
    return direction / np.linalg.norm(direction)


class WitDevice:
    """Temporarily configure native-only output; never change baud or save flash."""
    def __init__(self, port, config):
        self.port, self.config = port, config
        self.parser = FrameParser()
        self.original = None

    def command(self, address, value):
        self.port.write(b'\xff\xaa' + bytes([address]) + struct.pack('<H', value))
        self.port.flush()
        time.sleep(0.08)

    def unlock(self):
        self.command(0x69, 0xB588)

    def frames(self):
        # Bound each read to avoid unbounded latency if a large backlog exists.
        return self.parser.feed(self.port.read(min(4096, max(1, self.port.in_waiting))))

    def read_config(self):
        self.port.reset_input_buffer()
        self.parser.buffer.clear()
        self.unlock()
        self.command(0x27, 2)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            for frame in self.frames():
                if frame[1] == 0x5F:
                    return struct.unpack('<4H', frame[2:10])
        raise RuntimeError('IMU register read timed out; verify configured port and baud')

    def configure(self):
        self.original = self.read_config()
        self.unlock()
        self.command(2, 0x0200)
        self.command(3, RATE_CODES[self.config.publish_hz])
        actual = self.read_config()
        if actual[:3] != (0x0200, RATE_CODES[self.config.publish_hz], self.original[2]):
            raise RuntimeError('IMU configuration readback mismatch: ' + repr(actual))
        self.port.reset_input_buffer()
        self.parser.buffer.clear()

    def restore(self):
        if self.original is None:
            return
        self.unlock()
        self.command(2, self.original[0])
        self.command(3, self.original[1])
        actual = self.read_config()
        if actual[:3] != self.original[:3]:
            raise RuntimeError('IMU configuration restore failed: ' + repr(actual))
