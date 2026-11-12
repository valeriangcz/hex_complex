"""Configurable Joy decoding with last-command hold."""
from dataclasses import dataclass, field
import math
from threading import RLock


@dataclass
class StickConfig:
    joy_topic: str = '/joy'
    publish_hz: float = 20.0
    axes: dict = field(default_factory=lambda: dict(vx=0, vy=1, wz=2))
    buttons: dict = field(default_factory=lambda: dict(start=0, pause=4, reset=1, sim_reset=3, disable_actuators=6, release_magnets=7))

    def __post_init__(self):
        for name in ('publish_hz',):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not self.joy_topic:
            raise ValueError('joy_topic must not be empty')
        if set(self.axes) != {'vx','vy','wz'}:
            raise ValueError('axes must define vx, vy, wz')
        if set(self.buttons) != {'start','pause','reset','sim_reset','disable_actuators','release_magnets'}:
            raise ValueError('buttons must define start, pause, reset, sim_reset, disable_actuators, release_magnets')
        for index in list(self.axes.values()) + list(self.buttons.values()):
            if isinstance(index, bool) or not isinstance(index, int) or index < 0:
                raise ValueError('axis/button indices must be nonnegative integers')
        if len(set(self.buttons.values())) != len(self.buttons):
            raise ValueError('button indices must be distinct')


class StickState:
    def __init__(self, config):
        self.config = config
        self.previous = {action: False for action in config.buttons}
        self.velocity = (0., 0., 0.)
        self.lock = RLock()

    def receive(self, axes, buttons):
        with self.lock:
            pressed = {action: index < len(buttons) and buttons[index] == 1
                       for action, index in self.config.buttons.items()}
            edges = {action for action in pressed if pressed[action] and not self.previous[action]}
            self.previous = pressed
            # Held disable buttons suppress ordinary operations; both overrides may fire.
            if pressed['disable_actuators'] or pressed['release_magnets']:
                actions = [name for name in ('disable_actuators','release_magnets') if name in edges]
            else:
                # One ordinary operation per Joy sample: sim_reset > reset > pause > start.
                actions = next(([name] for name in ('sim_reset','reset','pause','start') if name in edges), [])
            valid = all(index < len(axes) and math.isfinite(axes[index])
                        for index in self.config.axes.values())
            self.velocity = tuple(axes[self.config.axes[name]]
                                  for name in ('vx','vy','wz')) if valid else (0., 0., 0.)
            if actions:
                self.velocity = (0., 0., 0.)
            return actions, valid

    def command(self):
        with self.lock:
            return self.velocity
