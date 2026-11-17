#!/usr/bin/env python3
"""Six-servo, one-command-in-flight ROS/serial round-trip benchmark."""
import json
import math
import queue
import signal
import threading
import time

import rosgraph
import rospy
from feetech_servo.msg import servo_cmd, servo_state


LEGS = ("lf", "lm", "lb", "rf", "rm", "rb")


class Benchmark:
    def __init__(self):
        self.stop = threading.Event()
        self.states = queue.Queue()
        self.sent = False
        self.ambiguous = False
        self.last_monitor = 0.0
        self.samples = []
        self.valid_cycles = 0
        self.valid_reads = [0] * 6
        self.timeouts = 0
        self.started = None
        self.finished = None
        self.amplitude = self.positive("amplitude_deg", 5.0)
        self.period = self.positive("period_s", 10.0)
        self.duration = self.positive("duration_s", 60.0)
        self.timeout = self.positive("feedback_timeout_s", 2.0)
        self.connect_timeout = self.positive("connect_timeout_s", 10.0)
        self.transition_speed = self.positive("transition_speed_deg_s", 3.0)
        if self.amplitude > 90.0:
            raise ValueError("amplitude_deg must be <= 90")
        self.driver = rospy.resolve_name(rospy.get_param("~interface_node", "interface"))
        self.config = rospy.get_param(self.driver)
        self.validate_config()
        self.pub = rospy.Publisher("servo_cmd", servo_cmd, queue_size=1, tcp_nodelay=True)
        self.sub = rospy.Subscriber("servo_state", servo_state, self.receive,
                                    queue_size=1, tcp_nodelay=True)
        self.master = rosgraph.Master(rospy.get_name())

    @staticmethod
    def positive(name, default):
        value = rospy.get_param("~" + name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be a finite positive number")
        return float(value)

    def validate_config(self):
        c = self.config
        if c.get("state_hz") != 0:
            raise ValueError("Benchmark requires interface state_hz=0")
        offsets, directions = c["offset_deg"], c["direction"]
        span = c["angle_span_deg"]
        if len(offsets) != 6 or len(directions) != 6:
            raise ValueError("Expected six calibrated offsets and directions")
        for name, offset, direction in zip(LEGS, offsets, directions):
            if direction not in (-1, 1) or not math.isfinite(offset):
                raise ValueError("Invalid calibration for " + name)
            if not (0 <= offset - self.amplitude and offset + self.amplitude <= span):
                raise ValueError(name + ": sine exceeds absolute range; calibrate offset_deg first")

    def receive(self, state):
        self.states.put((time.monotonic(), state))

    def check_stop(self):
        if self.stop.is_set() or rospy.is_shutdown():
            raise InterruptedError("Test interrupted")

    def monitor(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_monitor < 1.0:
            return
        publishers, subscribers, _ = self.master.getSystemState()
        pubs, subs = dict(publishers), dict(subscribers)
        if set(pubs.get(self.pub.resolved_name, [])) != {rospy.get_name()}:
            raise RuntimeError("servo_cmd must have exactly this benchmark as publisher")
        if set(pubs.get(self.sub.resolved_name, [])) != {self.driver}:
            raise RuntimeError("servo_state must have exactly the test interface as publisher")
        if set(subs.get(self.pub.resolved_name, [])) != {self.driver}:
            raise RuntimeError("servo_cmd must have exactly the test interface as subscriber")
        if rospy.get_param(self.driver + "/state_hz") != 0:
            raise RuntimeError("Independent state polling was enabled during benchmark")
        self.last_monitor = now

    def connect(self):
        deadline = time.monotonic() + self.connect_timeout
        while self.pub.get_num_connections() != 1 or self.sub.get_num_connections() != 1:
            self.check_stop()
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out connecting to isolated test interface")
            time.sleep(0.02)
        self.monitor(force=True)

    def exchange(self, angles=None, measured=False, cleanup=False):
        if not cleanup:
            self.check_stop()
            self.monitor()
        if not self.states.empty():
            self.ambiguous = True
            raise RuntimeError("Unexpected feedback: cannot associate command with response")
        msg = servo_cmd(enable=angles is not None)
        if angles is not None:
            for name, angle in zip(LEGS, angles):
                if not math.isfinite(angle) or abs(angle) > 90.0:
                    raise ValueError("Target outside relative [-90, 90] degrees")
                setattr(msg, "enable_" + name, True)
                setattr(msg, name, angle)
        start = time.monotonic()
        if measured and self.started is None:
            self.started = start
        self.pub.publish(msg)
        self.sent = True
        deadline = start + self.timeout
        try:
            while True:
                # Do not interrupt an in-flight exchange: its reply must be drained
                # before a cleanup disable can be associated with a new response.
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise queue.Empty
                try:
                    received, state = self.states.get(timeout=min(remaining, 0.05))
                    break
                except queue.Empty:
                    if time.monotonic() >= deadline:
                        raise
        except queue.Empty:
            self.ambiguous = True
            if measured:
                self.timeouts += 1
                self.finished = time.monotonic()
            raise TimeoutError("No complete six-servo feedback within %.3fs" % self.timeout)
        valid = [bool(flag) and math.isfinite(getattr(state, name))
                 for flag, name in zip(state.valid, LEGS)]
        if measured:
            self.finished = received
            self.samples.append(received - start)
            self.valid_reads = [n + int(ok) for n, ok in zip(self.valid_reads, valid)]
            self.valid_cycles += int(all(valid))
        if not all(valid):
            raise RuntimeError("Invalid feedback for: " + ", ".join(n for n, ok in zip(LEGS, valid) if not ok))
        return [getattr(state, name) for name in LEGS]

    def transition(self, start):
        if any(abs(angle) > 90.0 for angle in start):
            raise ValueError("Measured position outside [-90, 90]; refusing active transition")
        # Cosine interpolation has zero endpoint velocity and a bounded peak speed.
        duration = max(0.1, math.pi * max(abs(a) for a in start) / (2 * self.transition_speed))
        begun = time.monotonic()
        while True:
            self.check_stop()
            phase = min(1.0, (time.monotonic() - begun) / duration)
            scale = (1.0 + math.cos(math.pi * phase)) / 2.0
            self.exchange([angle * scale for angle in start])
            if phase >= 1.0:
                return
            time.sleep(0.01)

    def run(self):
        self.connect()
        baud = self.config["baud_rate"]
        rospy.loginfo("Port=%s baud=%s; 166-byte minimum cycle: %.3fms, ceiling %.1fHz (8N1)",
                      self.config["port"], baud, 1660.0 / baud * 1000.0, baud / 1660.0)
        rospy.loginfo("Initialize: disable torque, read positions, then slowly approach calibrated zero")
        self.transition(self.exchange())
        # Smoothly introduce amplitude over a complete period; excluded from results.
        warmup = time.monotonic()
        while True:
            elapsed = time.monotonic() - warmup
            if elapsed >= self.period:
                break
            envelope = 0.5 * (1.0 - math.cos(math.pi * elapsed / self.period))
            angle = self.amplitude * envelope * math.sin(2 * math.pi * elapsed / self.period)
            self.exchange([angle] * 6)
            time.sleep(0.01)
        rospy.loginfo("Measuring maximum complete read/write rate for %.1fs", self.duration)
        begun = time.monotonic()
        next_report = begun + 1.0
        while time.monotonic() - begun < self.duration:
            elapsed = time.monotonic() - begun
            angle = self.amplitude * math.sin(2 * math.pi * elapsed / self.period)
            self.exchange([angle] * 6, measured=True)
            if time.monotonic() >= next_report:
                rospy.loginfo("Measured %.1fHz; six-servo valid %.1fHz", *self.rates())
                next_report = time.monotonic() + 1.0
        rospy.loginfo("Measurement complete; slowly return to zero and disable torque")
        # Start the return from the measured position rather than the last target.
        positions = self.exchange([angle] * 6)
        self.transition(positions)

    def rates(self):
        elapsed = self.finished - self.started if self.started is not None else 0.0
        return ((len(self.samples) / elapsed, self.valid_cycles / elapsed)
                if elapsed > 0 else (0.0, 0.0))

    def report(self, error):
        hz, valid_hz = self.rates()
        ordered = sorted(self.samples)
        result = dict(status="aborted" if error else "completed", error=error,
                      elapsed_s=(self.finished - self.started) if self.started is not None else 0.0,
                      cycles=len(ordered), valid_cycles=self.valid_cycles,
                      cycle_hz=hz, valid_cycle_hz=valid_hz, difference_from_100_hz=valid_hz - 100.0,
                      valid_reads=dict(zip(LEGS, self.valid_reads)),
                      invalid_reads=dict(zip(LEGS, [len(ordered) - n for n in self.valid_reads])),
                      feedback_timeouts=self.timeouts,
                      mean_ms=1000 * sum(ordered) / len(ordered) if ordered else None,
                      p95_ms=1000 * ordered[math.ceil(0.95 * len(ordered)) - 1] if ordered else None,
                      max_ms=1000 * max(ordered) if ordered else None)
        rospy.loginfo("RESULT %s", json.dumps(result, sort_keys=True))
        rospy.loginfo("Actual complete cycle %.1fHz; all-six-valid %.1fHz; difference from 100Hz %+.1fHz",
                      hz, valid_hz, valid_hz - 100.0)

    def disable(self):
        if not self.sent:
            return
        if self.ambiguous:
            # No sequence field exists. A late reply must never confirm this disable.
            self.pub.publish(servo_cmd(enable=False))
            time.sleep(0.1)
            raise RuntimeError("Disable attempted; NOT confirmed because previous feedback was missing/ambiguous")
        self.exchange(cleanup=True)
        rospy.loginfo("Disable sent; six valid positions returned (torque writes have no per-servo ACK)")


def main():
    rospy.init_node("sine_benchmark", disable_signals=True)
    benchmark = None
    error = None
    try:
        benchmark = Benchmark()
        for signum in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signum, lambda *_: benchmark.stop.set())
        benchmark.run()
    except Exception as exc:
        error = str(exc)
        rospy.logerr("Benchmark stopped: %s", error)
    finally:
        if benchmark is not None:
            try:
                benchmark.disable()
            except Exception as exc:
                rospy.logerr("Cleanup: %s", exc)
                error = error or str(exc)
            benchmark.report(error)
        rospy.signal_shutdown("benchmark finished")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
