#!/usr/bin/env python3
"""End-to-end test on an isolated ROS master and test-only fake devices."""
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


def wait_for(predicate, description, timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.02)
    raise AssertionError('timeout: '+description)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait()


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
    os.environ['ROS_IP']='127.0.0.1'
    os.environ.pop('ROS_HOSTNAME',None)
    workspace=Path(__file__).resolve().parents[3]
    processes=[]
    with tempfile.TemporaryDirectory(prefix='hex_joy_test_') as directory:
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
                wait_for(master_ready,'master')
                import rospy
                from sensor_msgs.msg import Joy
                from geometry_msgs.msg import Twist
                from diagnostic_msgs.msg import DiagnosticArray
                from dm_motor_fusion.msg import MultiLegCommand
                from feetech_servo.msg import servo_cmd
                from magnetic.msg import MagneticCmd
                from std_srvs.srv import SetBool, Trigger, TriggerResponse
                from hex_kinematic.ros_contract import read_magnetic
                rospy.init_node('climb_joy_test',disable_signals=True)
                latest={};outputs={};velocities=[]
                sim_resets=[]
                def sim_reset(_request):
                    sim_resets.append(time.monotonic())
                    return TriggerResponse(success=True,message='test simulation reset accepted')
                sim_reset_service=rospy.Service('hex/sim/reset',Trigger,sim_reset)
                def diag(message):
                    for status in message.status:
                        latest.update({item.key:json.loads(item.value) for item in status.values})
                        latest['phase']=status.message
                subscribers=[rospy.Subscriber('hex/runner/diagnostics',DiagnosticArray,diag),
                             rospy.Subscriber('hex/cmd_vel',Twist,lambda msg:velocities.append((msg.linear.x,msg.linear.y,msg.angular.z)))]
                for name,typ in [('motor',MultiLegCommand),('servo',servo_cmd),('magnetic',MagneticCmd)]:
                    subscribers.append(rospy.Subscriber('hex/'+name+'/command',typ,lambda msg,n=name:outputs.update({n:msg})))
                start(['rosrun','hex_runner','runner_node.py','--config',str(workspace/'src/hex_runner/config/climb.yaml')])
                start(['roslaunch','control_stick','climb.launch','start_joy:=false'])
                joy=rospy.Publisher('/joy',Joy,queue_size=1)
                wait_for(lambda:joy.get_num_connections()>0 and latest.get('phase')=='WAITING','joystick and runner startup')
                assert not outputs,'startup emitted device commands'
                def send(*pressed,axes=None):
                    msg=Joy(axes=[0.,0.,0.] if axes is None else axes,buttons=[int(i in pressed) for i in range(8)])
                    joy.publish(msg)
                def press(*indices,axes=None):
                    send();time.sleep(.07);send(*indices,axes=axes)
                press(1,6,7)
                wait_for(lambda:latest.get('motor_servo_disabled') and latest.get('magnetic_forced_off'),'both disable requests before feedback')
                wait_for(lambda:all(name in outputs for name in ('motor','servo','magnetic')),'disable output')
                assert outputs['motor'].option==-1 and not outputs['servo'].enable
                assert not read_magnetic(outputs['magnetic']).any()
                assert latest['reset_count']==0
                start([sys.executable,str(workspace/'src/hex_runner/scripts/test_ros_integration.py'),'--fake-device','--mode','climb'])
                wait_for(lambda:latest.get('feedback_ready'),'fake feedback')
                def reset():
                    old=latest['reset_count']
                    press(1)
                    wait_for(lambda:latest.get('reset_count')==old+1 and latest.get('phase')=='READY','joystick reset')
                    # Hold the button: reset must not execute repeatedly.
                    for _ in range(6):send(1);time.sleep(.05)
                    assert latest['reset_count']==old+1
                    assert not latest['motor_servo_disabled'] and not latest['magnetic_forced_off']
                    wait_for(lambda:outputs['motor'].option==1 and outputs['servo'].enable and read_magnetic(outputs['magnetic']).all(),'reset initial outputs')
                    send()
                reset()
                reset_count=latest['reset_count']
                press(3)
                wait_for(lambda:len(sim_resets)==1,'button 3 calls simulation reset')
                for _ in range(6):send(3);time.sleep(.05)
                assert len(sim_resets)==1 and latest['reset_count']==reset_count
                send()
                from hex_runner.config import RunnerConfig
                scales=RunnerConfig.load(workspace/'src/hex_runner/config/climb.yaml').velocity_scales
                raw=[.02/scales['vx'],-.01/scales['vy'],.05/scales['wz']]
                for _ in range(10):send(axes=raw);time.sleep(.05)
                wait_for(lambda:all(abs(a-b)<1e-7 for a,b in zip(latest.get('command',[99]*3),[.02,-.01,.05])),'runner converts raw axes using its velocity_scales')
                # No new Joy for longer than the old timeout: keep sending the last command.
                time.sleep(.65)
                assert all(abs(a-b)<1e-7 for a,b in zip(velocities[-1],raw))
                assert latest.get('phase')=='RUNNING'
                send(axes=[0.,0.,0.])
                wait_for(lambda:velocities[-1]==(0.,0.,0.) and latest.get('phase')=='READY','explicit zero axes stop')
                previous_magnets=str(outputs['magnetic'])
                press(6,axes=[])
                wait_for(lambda:latest.get('motor_servo_disabled'),'invalid axes still disable')
                for _ in range(8):send(axes=[1.,1.,1.]);time.sleep(.05)
                assert outputs['motor'].option==-1 and not outputs['servo'].enable
                assert str(outputs['magnetic'])==previous_magnets
                reset()
                press(7)
                wait_for(lambda:latest.get('magnetic_forced_off'),'release magnets')
                time.sleep(.2)
                assert not read_magnetic(outputs['magnetic']).any()
                assert outputs['motor'].option==1 and outputs['servo'].enable
                for _ in range(6):send(axes=[1.,1.,1.]);time.sleep(.05)
                assert not read_magnetic(outputs['magnetic']).any()
                reset()
                rospy.wait_for_service('/test/invalidate',timeout=5)
                rospy.ServiceProxy('/test/invalidate',SetBool)(True)
                wait_for(lambda:latest.get('phase')=='FAULT','invalid feedback')
                press(6,7)
                wait_for(lambda:latest.get('motor_servo_disabled') and latest.get('magnetic_forced_off'),'disable during fault')
                old=latest['reset_count'];press(1);time.sleep(.4)
                assert latest['reset_count']==old
                assert latest['motor_servo_disabled'] and latest['magnetic_forced_off']
                report=dict(status='PASS',checks=['button 3 calls only simulation reset once per press','no startup device commands','simultaneous disable before feedback',
                    'button edge detection and reset priority','axis mapping, last-command hold without new Joy and explicit zero stop',
                    'invalid axes still process buttons','motor/servo disable latch preserves magnets',
                    'magnet release latch preserves joint hold','reset restores configured magnets',
                    'reset rejected on invalid feedback retains latches'])
                Path('/tmp/control_stick_integration.json').write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps(report,indent=2))
                rospy.signal_shutdown('test complete')
            except Exception:
                log.flush();log.seek(0);print(log.read())
                print('latest:',locals().get('latest'))
                raise
            finally:
                for process in reversed(processes):stop(process)

if __name__=='__main__':main()
