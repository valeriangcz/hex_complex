#!/usr/bin/env python3
"""Exercise the shared runner against fake ROS devices or a real Gym process."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client


def wait_for(predicate, description, timeout=30):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("timeout waiting for " + description)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def fake_devices(mode):
    """Test-only provider, never installed/launched as a real hardware driver."""
    import numpy as np
    import rospy
    from dm_motor_fusion.msg import MultiLegCommand, MultiLegState
    from feetech_servo.msg import servo_cmd, servo_state
    from magnetic.msg import MagneticCmd as MagCommand, MagneticState as MagState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import SetBool, SetBoolResponse
    from hex_kinematic.ros_contract import read_motor, read_servo, read_servo_enabled, write_motor, write_servo, read_magnetic, write_magnetic
    from hex_kinematic.se3_utils import transform_to_xyzw

    rospy.init_node("test_devices")
    workspace = Path(__file__).resolve().parents[3]
    from hex_runner.config import RunnerConfig
    initial=RunnerConfig.load(workspace/'src/hex_runner/config'/(mode+'.yaml'),workspace/'data').initial_state
    q, pose=initial.q.copy(), initial.W_T_R.copy()
    state = dict(q=q, tau=np.zeros((6,3)), enabled=initial.magnetic.copy(), attached=initial.magnetic.astype(float),
                 invalid=False, change_at=0, ramp_start=initial.magnetic.astype(float))
    def motor(message):
        if message.option == 1:
            state['q'][:, :3] = read_motor(message, 'pos')
            state['tau'] = read_motor(message, 'tau')
    def servo(message):
        enabled=read_servo_enabled(message)
        state['q'][enabled, 3] = read_servo(message)[enabled]
    def magnet(message):
        desired = read_magnetic(message)
        if not np.array_equal(state['enabled'], desired):
            state['ramp_start'] = state['attached'].copy()
            state['enabled'] = desired
            state['change_at'] = time.monotonic()
    def invalidate(request):
        state['invalid'] = request.data
        return SetBoolResponse(success=True)
    subscribers = [rospy.Subscriber('hex/motor/command',MultiLegCommand,motor),
                   rospy.Subscriber('hex/servo/command',servo_cmd,servo),
                   rospy.Subscriber('hex/magnetic/command',MagCommand,magnet)]
    service = rospy.Service('test/invalidate',SetBool,invalidate)
    pubs = [rospy.Publisher(name,typ,queue_size=1) for name,typ in (
        ('hex/motor/state',MultiLegState),('hex/servo/state',servo_state),('hex/magnetic/state',MagState),
        ('hex/perception/pose',PoseStamped),('hex/perception/gravity',Float64MultiArray))]
    while not rospy.is_shutdown():
        fraction = min((time.monotonic() - state['change_at']) / .1, 1.)
        state['attached'] = state['ramp_start'] + fraction * (state['enabled'] - state['ramp_start'])
        stamp = rospy.Time.now()
        motor_msg = MultiLegState(option=1,valid=[True]*18)
        motor_msg.header.stamp = stamp
        for suffix, values in [('pos',state['q'][:,:3]),('tau',state['tau']),('vel',np.zeros((6,3)))]:
            write_motor(motor_msg,suffix,values)
        servo_msg = servo_state(valid=[True]*6); servo_msg.header.stamp = stamp
        write_servo(servo_msg,state['q'][:,3])
        magnetic = MagState()
        write_magnetic(magnetic, state['enabled'], 'enabled_')
        write_magnetic(magnetic, state['attached'], 'attached_')
        write_magnetic(magnetic, [not state['invalid']]*6, 'valid_')
        magnetic.header.stamp = stamp
        p,quat = transform_to_xyzw(pose)
        pose_msg = PoseStamped(); pose_msg.header.stamp=stamp; pose_msg.header.frame_id='world'
        pose_msg.pose.position.x,pose_msg.pose.position.y,pose_msg.pose.position.z=map(float,p)
        pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w=map(float,quat)
        gravity=Float64MultiArray(data=(pose[:3,:3].T @ np.array([0.,0.,-1.])).tolist())
        for pub,msg in zip(pubs,[motor_msg,servo_msg,magnetic,pose_msg,gravity]):pub.publish(msg)
        time.sleep(.01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['fake','isaac'],default='fake')
    parser.add_argument('--mode',choices=['climb','complex'],default='climb')
    parser.add_argument('--joystick',action='store_true',help='Use Joy buttons 1/0/4 for complex reset/start/pause')
    parser.add_argument('--pose-source',choices=['gravity','se3'])
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--climb-velocity',type=float,nargs=3,default=[.02,0.,0.],metavar=('VX','VY','WZ'),help='Physical climb command in m/s, m/s, rad/s')
    parser.add_argument('--motion-seconds',type=float,help='Run the full motion interval rather than ending after the required gait switches')
    parser.add_argument('--min-gait-switches',type=int,default=2)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--fake-device',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.joystick and args.mode != 'complex':
        parser.error('--joystick requires --mode complex')
    if args.fake_device:
        fake_devices(args.mode)
        return
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME',None)
    processes=[]
    runner_process=None
    workspace=Path(__file__).resolve().parents[3]
    checks=[]
    motion_metrics={}
    with tempfile.TemporaryDirectory(prefix='hex_ros_') as directory:
        os.environ['ROS_LOG_DIR']=directory
        with open(directory+'/process.log','w+') as log:
            def start(command):
                process=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True)
                processes.append(process)
                return process
            try:
                start(['roscore','-p',str(port)])
                def master_ready():
                    try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('test')[0]==1
                    except OSError:return False
                wait_for(master_ready,'master',15)
                import numpy as np
                import yaml
                import rospy
                from diagnostic_msgs.msg import DiagnosticArray
                from geometry_msgs.msg import Twist, PoseStamped
                from std_srvs.srv import Trigger,SetBool
                from dm_motor_fusion.msg import MultiLegCommand, MultiLegState
                from feetech_servo.msg import servo_cmd, servo_state
                from magnetic.msg import MagneticState
                from hex_kinematic.ros_contract import read_motor, read_servo, read_servo_enabled, read_magnetic
                from hex_kinematic.se3_utils import transform_from_xyzw
                from magnetic.msg import MagneticCmd
                rospy.init_node('hex_integration_test',disable_signals=True)
                observed_pose={}
                reset_stamps=[]
                frames={name:{} for name in ("motor", "servo", "pose", "magnetic")}
                latest={}; sim={}; counts={'motor':0,'servo':0,'magnetic':0}; commands={}
                def diagnostics(message):
                    for status in message.status:
                        target=latest if status.name=='hex_runner' else sim
                        target.update({entry.key:json.loads(entry.value) for entry in status.values})
                        target['phase']=status.message
                        if status.name=='hex_sim' and target.get('run_elapsed')==0 and target.get('reset_count',0)>0:
                            reset_stamps.append(message.header.stamp.to_nsec())
                def command(name,message):
                    counts[name]+=1;commands[name]=message
                subscribers=[rospy.Subscriber('hex/runner/diagnostics',DiagnosticArray,diagnostics),
                             rospy.Subscriber('hex/sim/diagnostics',DiagnosticArray,diagnostics)]
                for name,typ in [('motor',MultiLegCommand),('servo',servo_cmd),('magnetic',MagneticCmd)]:
                    subscribers.append(rospy.Subscriber('hex/'+name+'/command',typ,lambda msg,n=name:command(n,msg)))
                subscribers.append(rospy.Subscriber('hex/perception/pose',PoseStamped,lambda msg: observed_pose.update(x=msg.pose.position.x,y=msg.pose.position.y,z=msg.pose.position.z, quaternion=[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])))
                for name, topic, typ in [('motor','hex/motor/state',MultiLegState),
                                         ('servo','hex/servo/state',servo_state),
                                         ('pose','hex/perception/pose',PoseStamped),
                                         ('magnetic','hex/magnetic/state',MagneticState)]:
                    subscribers.append(rospy.Subscriber(topic,typ,lambda msg,n=name: frames[n].update({msg.header.stamp.to_nsec():msg})))
                velocity=rospy.Publisher('hex/cmd_vel',Twist,queue_size=1)
                if args.backend=='fake':
                    start([sys.executable,str(Path(__file__).resolve()),'--fake-device','--mode',args.mode])
                else:
                    start(['roslaunch','hex_isaacgym','backend.launch',
                           'config:='+str(workspace/'src/hex_isaacgym/config'/('plane.yaml' if args.mode=='climb' else 'complex_surface4.yaml')),
                           'device:='+args.device,'initial_config:='+str(workspace/'src/hex_runner/config'/(args.mode+'.yaml'))])
                config=yaml.safe_load((workspace/'src/hex_runner/config'/(args.mode+'.yaml')).read_text())
                if args.pose_source:config['pose_source']=args.pose_source
                config_file=directory+'/runner.yaml'
                with open(config_file,'w') as stream:yaml.safe_dump(config,stream)
                runner_process=start(['rosrun','hex_runner','runner_node.py','--config',config_file,
                                      '--data-root',str(workspace/'data'),'--output-dir',directory+'/runs'])
                wait_for(lambda:latest.get('feedback_ready'),'device feedback',90)
                assert latest['phase']=='WAITING' and not any(counts.values()), (latest,counts)
                import rosservice
                operations={name.rsplit('/',1)[1] for name in rosservice.get_service_list() if name.startswith('/hex/runner/')}
                assert operations == {'reset','start','pause','disable_actuators','release_magnets'},operations
                checks.append('no actuator commands before reset')
                if args.joystick:
                    from sensor_msgs.msg import Joy
                    start(['roslaunch','control_stick','climb.launch','start_joy:=false'])
                    joy=rospy.Publisher('joy',Joy,queue_size=1)
                    wait_for(lambda:joy.get_num_connections()>0,'joystick subscription')
                def service(name,success=True):
                    rospy.wait_for_service('hex/runner/'+name,timeout=5)
                    if args.joystick and success and name in ('reset','start','pause'):
                        joy.publish(Joy(axes=[0.,0.,0.],buttons=[0]*8))
                        time.sleep(.08)
                        pressed=[0]*8;pressed[{'reset':1,'start':0,'pause':4}[name]]=1
                        joy.publish(Joy(axes=[0.,0.,0.],buttons=pressed))
                        return
                    result=rospy.ServiceProxy('hex/runner/'+name,Trigger)()
                    assert result.success==success,result
                from hex_runner.config import RunnerConfig
                service('reset')
                wait_for(lambda:latest.get('phase')=='READY','reset READY')
                wait_for(lambda:all(counts.values()),'initial hold publications')
                assert commands['servo'].enable
                assert read_servo_enabled(commands['servo']).all()
                np.testing.assert_allclose(read_motor(commands['motor'], 'pos'),
                    RunnerConfig.load(workspace/'src/hex_runner/config'/(args.mode+'.yaml'), workspace/'data').initial_state.q[:,:3])
                checks.append('reset commands configured initial joint and magnetic targets')
                wait_for(lambda:velocity.get_num_connections()>0,'velocity subscription')
                if args.mode=='climb':
                    service('start',False);service('pause',False)
                    twist=Twist()
                    raw=[]
                    for axis,value in zip(('vx','vy','wz'),args.climb_velocity):
                        scale=config['velocity_scales'][axis]
                        if scale==0 and value!=0:raise ValueError('nonzero velocity requires nonzero '+axis+' scale')
                        raw.append(value/scale if scale else 0.)
                    twist.linear.x,twist.linear.y,twist.angular.z=raw
                    peak_switches=0
                    initial_position=np.array([observed_pose.get(k,0.) for k in ('x','y','z')])
                    initial_rotation=transform_from_xyzw([0.,0.,0.],observed_pose['quaternion'])[:3,:3]
                    began=time.monotonic()
                    end=began+(args.motion_seconds if args.motion_seconds is not None else ((160 if args.device.startswith('cuda') else 40) if args.backend=='isaac' else 1))
                    while time.monotonic()<end:
                        velocity.publish(twist);time.sleep(.05)
                        assert latest['phase']!='FAULT',latest
                        peak_switches=max(peak_switches,latest.get('gait_switch_count',0))
                        if args.motion_seconds is None and args.backend=='isaac' and peak_switches>=args.min_gait_switches and time.monotonic()-began>=10:
                            break
                    wait_for(lambda:latest.get('compute_count',0)>5,'climb control cycles')
                    moving_count=latest['compute_count']
                    motion_metrics=dict(compute_count_while_moving=moving_count, gait_switches_while_moving=peak_switches)
                    if args.backend=='isaac':
                        final_position=np.array([observed_pose.get(k,0.) for k in ('x','y','z')])
                        motion_metrics['displacement_m']=float(np.linalg.norm(final_position-initial_position))
                        assert peak_switches >= args.min_gait_switches, ('climb failed to alternate support groups',latest)
                        assert motion_metrics['displacement_m']>.001,motion_metrics
                        displacement_body=initial_rotation.T@(final_position-initial_position)
                        motion_metrics['displacement_body_start_m']=displacement_body.tolist()
                        if args.climb_velocity[2]==0 and np.linalg.norm(args.climb_velocity[:2])>0:
                            assert np.dot(displacement_body[:2],args.climb_velocity[:2])>0, ('robot moved opposite to command',motion_metrics)
                    before_silence=latest['compute_count'];time.sleep(.4)
                    assert latest['phase']=='RUNNING' and latest['compute_count']>before_silence,latest
                    velocity.publish(Twist())
                    wait_for(lambda:latest.get('phase')=='READY','explicit zero velocity',3)
                    frozen=latest['compute_count']; time.sleep(.3)
                    assert latest['compute_count']==frozen
                    velocity.publish(twist)
                    wait_for(lambda:latest.get('compute_count',0)>frozen,'velocity resume')
                    invalid=Twist();invalid.linear.x=float('nan');velocity.publish(invalid)
                    before_invalid=latest['compute_count'];time.sleep(.4)
                    assert latest['phase']=='RUNNING' and latest['compute_count']>before_invalid,latest
                    velocity.publish(Twist())
                    wait_for(lambda:latest.get('phase')=='READY','zero velocity after invalid input')
                    checks.append('climb retains velocity without new commands, explicit zero hold, resume and invalid input rejection')
                else:
                    service('start');wait_for(lambda:latest.get('compute_count',0)>200 or latest.get('phase')=='FAULT','complex execution',90)
                    assert latest.get('phase')!='FAULT',latest
                    motion_metrics['compute_count_before_pause']=latest['compute_count']
                    service('pause');wait_for(lambda:latest.get('phase')=='PAUSED','pause')
                    frozen=latest['compute_count'];time.sleep(.2)
                    assert latest['compute_count']==frozen
                    service('start');wait_for(lambda:latest.get('compute_count',0)>frozen,'resume',30)
                    checks.append('complex explicit start, pause and progress-preserving resume')
                    if args.joystick:checks.append('Joy buttons 1/0/4 reset, start, pause and resume through control_stick')
                if args.mode=='complex':
                    service('pause');wait_for(lambda:latest.get('phase')=='PAUSED','pause before reset')
                else:
                    velocity.publish(Twist());wait_for(lambda:latest.get('phase')=='READY','zero velocity before reset')
                time.sleep(.15)
                np.testing.assert_array_equal(read_servo_enabled(commands['servo']),~np.asarray(latest['gaits'],dtype=bool))
                checks.append('expert stance servos disabled, swing servos enabled, flags held while paused')
                service('reset');wait_for(lambda:latest.get('phase')=='READY','second reset')
                if args.backend=='fake':
                    if args.mode=='complex':
                        service('start');wait_for(lambda:latest.get('phase')=='RUNNING','run before disable')
                        service('pause');wait_for(lambda:latest.get('phase')=='PAUSED','pause before disable')
                        service('disable_actuators')
                        wait_for(lambda:latest.get('motor_servo_disabled'),'disable while paused')
                        service('start',False)
                        service('reset');wait_for(lambda:latest.get('phase')=='READY' and not latest.get('motor_servo_disabled'),'reset after pause/disable')
                        checks.append('pause permits reset or disable; disable latch prevents start until reset')
                    rospy.wait_for_service('test/invalidate',timeout=5)
                    invalidator=rospy.ServiceProxy('test/invalidate',SetBool)
                    from hex_runner.config import RunnerConfig
                    initial = RunnerConfig.load(workspace/'src/hex_runner/config'/(args.mode+'.yaml'), workspace/'data').initial_state
                    wait_for(lambda: latest.get('reset_count', 0) >= 2, 'reset applied before fault injection')
                    reset_counts = counts.copy()
                    wait_for(lambda: all(counts[name] > reset_counts[name] for name in counts),
                             'all output channels refreshed after reset')
                    wait_for(lambda: read_servo_enabled(commands['servo']).all() and
                             commands['motor'].option == 1 and
                             np.allclose(read_motor(commands['motor'], 'pos'), initial.q[:, :3]) and
                             np.allclose(read_motor(commands['motor'], 'tau'), 0) and
                             np.allclose(read_servo(commands['servo']), initial.q[:, 3]) and
                             np.array_equal(read_magnetic(commands['magnetic']), initial.magnetic),
                             'reset outputs received before fault injection')
                    old=commands.copy();invalidator(True)
                    wait_for(lambda:latest.get('phase')=='FAULT','invalid magnetic fault')
                    invalidator(False);time.sleep(.3)
                    assert latest['phase']=='FAULT'
                    for name in commands:
                        assert str(commands[name])==str(old[name]),name
                    service('reset');wait_for(lambda:latest.get('phase')=='READY','fault recovery')
                    stop(processes[1])  # Missing providers must become stale, not fabricate state.
                    wait_for(lambda:latest.get('phase')=='FAULT','feedback stream timeout',4)
                    service('reset',False)
                    checks.append('invalid/stale feedback latches fault, holds every output, explicit recovery')
                else:
                    stop(runner_process)
                    pubs=[rospy.Publisher('hex/motor/command',MultiLegCommand,queue_size=1),
                          rospy.Publisher('hex/servo/command',servo_cmd,queue_size=1),
                          rospy.Publisher('hex/magnetic/command',MagneticCmd,queue_size=1)]
                    wait_for(lambda:all(pub.get_num_connections() for pub in pubs),'manual sim publishers')
                    for _ in range(4):
                        pubs[0].publish(MultiLegCommand(option=0));pubs[1].publish(servo_cmd(enable=False))
                        pubs[2].publish(MagneticCmd());time.sleep(.05)
                    wait_for(lambda:sim.get('motor_mode')==0 and sim.get('servo_enabled') == [False]*6,'zero torque/servo disable')
                    assert sim['motor_torque_max']==0 and sim['servo_torque_max']==0,sim
                    pubs[0].publish(MultiLegCommand(option=2));time.sleep(.2)
                    assert sim['motor_mode']==0,'unsupported mode was not rejected'
                    resets=sim.get('reset_count',0)
                    rospy.ServiceProxy('hex/sim/reset',Trigger)()
                    wait_for(lambda:sim.get('reset_count',0)>resets,'physical reset')
                    wait_for(lambda:reset_stamps and all(reset_stamps[-1] in data for data in frames.values()),'exact reset feedback frame')
                    stamp=reset_stamps[-1]
                    from hex_runner.config import RunnerConfig
                    initial=RunnerConfig.load(workspace/'src/hex_runner/config'/(args.mode+'.yaml'),workspace/'data').initial_state
                    q=np.column_stack((read_motor(frames['motor'][stamp],'pos'),read_servo(frames['servo'][stamp])))
                    np.testing.assert_allclose(q,initial.q,atol=2e-6)
                    pose_msg=frames['pose'][stamp].pose
                    pose_matrix=transform_from_xyzw([pose_msg.position.x,pose_msg.position.y,pose_msg.position.z],
                        [pose_msg.orientation.x,pose_msg.orientation.y,pose_msg.orientation.z,pose_msg.orientation.w])
                    np.testing.assert_allclose(pose_matrix,initial.W_T_R,atol=2e-6)
                    np.testing.assert_array_equal(read_magnetic(frames['magnetic'][stamp],'enabled_'),initial.magnetic)
                    np.testing.assert_array_equal(read_magnetic(frames['magnetic'][stamp], 'attached_'),initial.magnetic)
                    checks.append('sim zero torque, servo disable, unsupported mode rejection and exact first reset feedback')
                report=dict(status='PASS',backend=args.backend,mode=args.mode,device=args.device,checks=checks,
                            compute_count=latest.get('compute_count'),gait_switch_count=latest.get('gait_switch_count'),
                            diagnostics=latest,simulation=sim,motion_metrics=motion_metrics)
                if args.report:
                    args.report.parent.mkdir(parents=True,exist_ok=True)
                    args.report.write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps(report,indent=2))
                rospy.signal_shutdown('test complete')
            except Exception:
                log.flush();log.seek(0);print(log.read())
                print('latest runner:',locals().get('latest'))
                raise
            finally:
                for process in reversed(processes):stop(process)


if __name__=='__main__':
    main()
