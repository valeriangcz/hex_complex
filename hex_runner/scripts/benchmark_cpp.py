#!/usr/bin/env python3
"""Offline Python/C++ expert parity and timing; never publishes ROS commands."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
import yaml

from hex_expert import ExpertClimb
from hex_kinematic import Kinematic
from hex_kinematic.ros_contract import read_motor, read_servo, read_magnetic
from hex_runner.config import RunnerConfig


def main():
    root=Path(__file__).resolve().parents[3]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=root/'src/hex_runner/config/climb.yaml')
    parser.add_argument('--probe',type=Path,default=root/'devel/lib/hex_runner/hex_cpp_probe')
    parser.add_argument('--bag',type=Path)
    parser.add_argument('--mode',choices=['suspended','ground'],default='suspended')
    parser.add_argument('--iterations',type=int,default=2000)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    config=RunnerConfig.load(args.config)
    kin=Kinematic(urdf_path=config.robot_urdf); initial=config.initial_state
    expert=ExpertClimb(kin,groups=initial.groups,stance_group=initial.stance_group,
        control_dt=config.control_dt,motor_torque_limits=config.motor_torque_limit,
        foot_positions=initial.foot_positions,motor_positions=initial.motor_positions,
        ik_branch=initial.ik_branch,magnetic=initial.magnetic,operating_mode=args.mode,
        **config.expert_parameters)
    initial_q=expert.q_des.copy(); samples=[]
    if args.bag:
        import rosbag
        latest={}
        with rosbag.Bag(str(args.bag)) as bag:
            for topic,message,_ in bag.read_messages(topics=['/hex/motor/state','/hex/servo/state',
                    '/hex/magnetic/state','/hex/perception/gravity','/hex/cmd_vel','/hex/motor/command']):
                latest[topic]=message
                if topic=='/hex/motor/command' and message.option==1 and all(k in latest for k in [
                        '/hex/motor/state','/hex/servo/state','/hex/magnetic/state','/hex/perception/gravity']):
                    q=np.column_stack((read_motor(latest['/hex/motor/state'],'pos'),read_servo(latest['/hex/servo/state'])))
                    torque=read_motor(latest['/hex/motor/state'],'tau')
                    attached=read_magnetic(latest['/hex/magnetic/state'],'attached_')
                    gravity=np.asarray(latest['/hex/perception/gravity'].data)
                    command=latest.get('/hex/cmd_vel')
                    velocity=(np.array([command.linear.x,command.linear.y,command.angular.z]) *
                              np.array([config.velocity_scales[k] for k in ['vx','vy','wz']])) if command else np.zeros(3)
                    if np.isfinite(q).all() and np.isfinite(torque).all(): samples.append((q,torque,attached,gravity,velocity))
    else:
        for i in range(args.iterations):
            q=initial_q.copy(); q[:,:3]+=.01*np.sin(i*.07)
            samples.append((q,np.zeros((6,3)),initial.magnetic.astype(float),np.array([0.,0.,-1.]),np.array([.02,.1,.03])))
    if not samples: raise RuntimeError('No complete valid input samples')
    frames=[]; expected=[]; wall=[]; cpu=[]
    for i,(q,torque,attached,gravity,velocity) in enumerate(samples):
        frames.append(dict(q=q.tolist(),tau=torque.tolist(),attached=attached.tolist(),enabled=(attached>.85).tolist(),
                           gravity=gravity.tolist(),velocity=velocity.tolist(),sample=i+1))
        began=time.perf_counter();cpu_began=time.process_time()
        result=expert.RequestSingleStep(gravity,q,torque,attached,velocity)
        cpu.append((time.process_time()-cpu_began)*1e6);wall.append((time.perf_counter()-began)*1e6)
        expected.append(result)
    with tempfile.TemporaryDirectory(prefix='hex_cpp_benchmark_') as directory:
        data=yaml.safe_load(args.config.read_text());data['workspace_root']=str(root);data['robot_urdf']=config.robot_urdf
        data['operating_mode']=args.mode
        filename=Path(directory)/'runner.yaml';filename.write_text(yaml.safe_dump(data))
        request=dict(op='expert',config=str(filename),sim_info=str(Path(directory)/'initial.yaml'),
                     initial_q=initial_q.tolist(),initial_adhesions=initial.magnetic.tolist(),frames=frames)
        process=subprocess.run([str(args.probe)],input=json.dumps(request),text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        if process.returncode: raise RuntimeError(process.stderr)
        native=json.loads(process.stdout)['frames']
    max_q=max_tau=0.
    for actual,reference in zip(native,expected):
        np.testing.assert_allclose(actual['q'],reference[0],atol=1e-5,rtol=1e-6)
        np.testing.assert_allclose(actual['tau'],reference[1],atol=1e-5,rtol=1e-6)
        np.testing.assert_array_equal(actual['adhesions'],reference[2])
        np.testing.assert_array_equal(actual['servo_enabled'],reference[3])
        max_q=max(max_q,float(np.max(np.abs(np.asarray(actual['q'])-reference[0]))))
        max_tau=max(max_tau,float(np.max(np.abs(np.asarray(actual['tau'])-reference[1]))))
    warmup=min(30,len(native)//10)
    def timing(wall_us,cpu_us):
        return dict(mean_ms=float(np.mean(wall_us)/1000),p50_ms=float(np.percentile(wall_us,50)/1000),
                    p99_ms=float(np.percentile(wall_us,99)/1000),max_ms=float(np.max(wall_us)/1000),
                    mean_cpu_ms=float(np.mean(cpu_us)/1000))
    report=dict(status='PASS',mode=args.mode,samples=len(native),bag=str(args.bag) if args.bag else None,
                max_q_error=max_q,max_tau_error=max_tau,
                python=timing(wall[warmup:],cpu[warmup:]),
                cpp=timing([x['compute_us'] for x in native[warmup:]],[x['cpu_us'] for x in native[warmup:]]),
                note='Expert computation only; excludes ROS transport. Bag inputs are processed offline in recorded order.')
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
