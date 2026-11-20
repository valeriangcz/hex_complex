#!/usr/bin/env python3
"""Start sim only after runner has written its initial-state YAML."""
import argparse
import signal
import subprocess
import sys
from threading import Event, Thread
import time

import rospy
import yaml


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner-config', required=True)
    parser.add_argument('--implementation', choices=['cpp', 'python'])
    parser.add_argument('--backend-config', required=True)
    parser.add_argument('--to-sim-info', required=True)
    parser.add_argument('--data-root', default='')
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--headless', default='false')
    parser.add_argument('--publish-clock', default='false')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.implementation is None:
        with open(args.runner_config) as stream:
            args.implementation = 'cpp' if yaml.safe_load(stream)['expert_type'] == 'climb' else 'python'
    ready = Event()
    runner = None
    sim = None

    def relay(output):
        for line in output:
            print(line, end='', flush=True)
            if line.strip() == 'HEX_RUNNER_INITIALIZED':
                ready.set()

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        runner = subprocess.Popen([
            'rosrun', 'hex_runner', 'runner_node_cpp' if args.implementation == 'cpp' else 'runner_node.py', '--config', args.runner_config,
            '--data-root', args.data_root, '--output-dir', args.output_dir,
            '--to-sim-info', args.to_sim_info,
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        Thread(target=relay, args=(runner.stdout,), daemon=True).start()
        while not ready.wait(.1):
            if runner.poll() is not None:
                raise RuntimeError('runner initialization failed; simulator was not started')
        sim = subprocess.Popen([
            'rosrun', 'hex_isaacgym', 'sim_node.py', '--config', args.backend_config,
            '--to-sim-info', args.to_sim_info, '--device', args.device,
            '--headless', args.headless, '--publish-clock', args.publish_clock,
        ])
        while runner.poll() is None and sim.poll() is None:
            time.sleep(.1)
        return runner.returncode if runner.poll() is not None else sim.returncode
    except KeyboardInterrupt:
        return 0
    finally:
        stop(sim)
        stop(runner)


if __name__ == '__main__':
    sys.exit(main())
