"""Joy -> velocity topic, runner services and simulation reset. Does not publish device commands."""
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from .core import StickConfig, StickState

# Expiry for queued button actions only; velocity commands never expire here.
OPERATION_REQUEST_MAX_AGE = 0.25
OVERRIDE_ACTIONS = ("disable_actuators", "release_magnets")
NORMAL_ACTIONS = ("sim_reset", "reset", "pause", "start")


class ClimbJoyNode:
    def __init__(self):
        self.config = StickConfig(
            joy_topic=rospy.get_param('~joy_topic', '/joy'),
            publish_hz=rospy.get_param('~publish_hz', 20.0),
            axes=rospy.get_param('~axes', dict(vx=0, vy=1, wz=2)),
            buttons=rospy.get_param('~buttons', dict(start=0, pause=4, reset=1, sim_reset=3, disable_actuators=6, release_magnets=7)),
        )
        self.state = StickState(self.config)
        self.publisher = rospy.Publisher('hex/cmd_vel', Twist, queue_size=1)
        self.pending = {}
        self.pending_lock = threading.Lock()
        self.wake = threading.Event()
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self._services, name='stick_services', daemon=True)
        self.worker.start()
        self.subscriber = rospy.Subscriber(self.config.joy_topic, Joy, self._joy, queue_size=1)
        rospy.on_shutdown(self.close)

    def _joy(self, message):
        now = time.monotonic()
        actions, valid = self.state.receive(message.axes, message.buttons)
        if not valid:
            rospy.logwarn_throttle(1., 'Joy axes missing or non-finite; velocity set to zero')
        if actions:
            with self.pending_lock:
                # Keep only the latest ordinary operation; overrides always take precedence.
                for name in NORMAL_ACTIONS:
                    self.pending.pop(name, None)
                if any(name in self.pending for name in OVERRIDE_ACTIONS):
                    actions = [name for name in actions if name in OVERRIDE_ACTIONS]
                for action in actions:
                    self.pending[action] = now
            self.wake.set()

    def _services(self):
        while not self.closed.is_set():
            self.wake.wait(.1)
            self.wake.clear()
            with self.pending_lock:
                pending, self.pending = self.pending, {}
            for action in OVERRIDE_ACTIONS + NORMAL_ACTIONS:
                if action not in pending:
                    continue
                if action in NORMAL_ACTIONS and (len(pending) > 1 or time.monotonic()-pending[action] >= OPERATION_REQUEST_MAX_AGE):
                    rospy.logwarn('Discarding superseded or stale joystick %s', action)
                    continue
                service = 'hex/sim/reset' if action == 'sim_reset' else 'hex/runner/' + action
                try:
                    rospy.wait_for_service(service, timeout=.2)
                    if action in NORMAL_ACTIONS and time.monotonic()-pending[action] >= OPERATION_REQUEST_MAX_AGE:
                        rospy.logwarn('Discarding joystick %s after service discovery delay', action)
                        continue
                    if action in NORMAL_ACTIONS:
                        with self.pending_lock:
                            superseded = bool(self.pending)
                        if superseded:
                            rospy.logwarn('Discarding joystick %s superseded by a newer request', action)
                            continue
                    result = rospy.ServiceProxy(service, Trigger)()
                    if not result.success:
                        rospy.logwarn('Service %s rejected joystick %s: %s', service, action, result.message)
                    else:
                        rospy.loginfo('Service %s accepted joystick %s: %s', service, action, result.message)
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rospy.logwarn('Joystick action %s failed on %s: %s; press again to retry', action, service, error)

    def spin(self):
        deadline = time.monotonic()
        while not rospy.is_shutdown():
            vx, vy, wz = self.state.command()
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
            self.publisher.publish(message)
            deadline += 1. / self.config.publish_hz
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()

    def close(self):
        self.closed.set()
        self.wake.set()


def main():
    rospy.init_node('climb_joy')
    ClimbJoyNode().spin()
