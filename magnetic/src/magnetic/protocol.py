"""Serial framing, named channel mapping and Hall sensor calibration."""
import re

LEG_NAMES = ("lf", "lm", "lb", "rf", "rm", "rb")
FIELDS = re.compile(rb"\[(adc_raw|io_state)\]\s*:\s*([^;]+);")


def validate_channel_order(channel_order):
    if len(channel_order) != 6 or set(channel_order) != set(LEG_NAMES):
        raise ValueError("channel_order must contain each of the six leg names once")


def encode_command(message, channel_order):
    values = ",".join(str(int(getattr(message, leg))) for leg in channel_order)
    return ("[IO_CMD]:" + values + ";").encode("ascii")


class Protocol:
    def __init__(self, config):
        self.channel_order = config["channel_order"]
        self.lower = config["lower_bound"]
        self.upper = config["upper_bound"]
        self.calibration = config["calibration"]
        self.buffer = b""
        validate_channel_order(self.channel_order)
        if not self.lower < self.upper:
            raise ValueError("lower_bound must be less than upper_bound")
        for leg in LEG_NAMES:
            item = self.calibration[leg]
            if not self.lower <= item["raw_min"] < item["raw_max"] <= self.upper:
                raise ValueError(leg + ": require lower_bound <= raw_min < raw_max <= upper_bound")

    def feed(self, data):
        """Return complete frames; retain an unfinished frame across reads."""
        frames = (self.buffer + data).split(b"#")
        self.buffer = frames.pop()
        if len(self.buffer) > 8192:
            self.buffer = b""  # Discard an unterminated/corrupt frame.
        return frames

    def decode(self, frame):
        fields = {key: [int(value) for value in values.split(b",")]
                  for key, values in FIELDS.findall(frame)}
        adc, io = fields[b"adc_raw"], fields[b"io_state"]
        if len(adc) != 6 or len(io) != 6:
            raise ValueError("expected six adc_raw and six io_state values")
        result = {}
        for leg, raw, output in zip(self.channel_order, adc, io):
            item = self.calibration[leg]
            ratio = (raw - item["raw_min"]) / (item["raw_max"] - item["raw_min"])
            valid = self.lower <= raw <= self.upper and output in (0, 1)
            result[leg] = (max(0.0, min(1.0, ratio)), output == 1, valid, raw, output)
        return result

    def encode(self, message):
        return encode_command(message, self.channel_order)
