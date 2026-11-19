#!/usr/bin/env python3
"""Check initialization over ROS using fake devices or the Isaac Gym backend."""
import argparse,json,os,signal,socket,subprocess,sys,tempfile,time,xmlrpc.client
from pathlib import Path
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['climb','complex'],default='climb')
    parser.add_argument('--backend',choices=['fake','isaac'],default='fake')
    parser.add_argument('--method',choices=['set_init','set_init_by_se3'])
    parser.add_argument('--staged',choices=['true','false'],help='Override independent transition flag')
    parser.add_argument('--timeout',type=float,default=120)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[3]
    source=yaml.safe_load((root/'src/hex_runner/config'/(args.mode+'.yaml')).read_text())
    method=args.method or source.get('initialization',{}).get('method','set_init')
    staged=None if args.staged is None else args.staged=='true'
    if staged is None:
        staged=source.get('initialization',{}).get('staged',False)
    if args.mode=='climb' and method!='set_init':parser.error('climb requires set_init')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME',None)
    processes=[];latest={};modes=[];stages=[];servo_count=[0];zero_valid=[True];magnet_off=[False]
    with tempfile.TemporaryDirectory(prefix='hex_initialization_') as directory:
        os.environ.update(ROS_HOME=directory,ROS_LOG_DIR=directory+'/log')
        log=open(directory+'/process.log','w+')
        def start(cmd):
            p=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True);processes.append(p);return p
        def wait(predicate,description,timeout=None):
            until=time.monotonic()+(args.timeout if timeout is None else timeout)
            while time.monotonic()<until:
                if predicate():return
                if any(p.poll() is not None for p in processes):raise RuntimeError('process exited during '+description)
                time.sleep(.02)
            raise AssertionError('timeout: '+description+' '+json.dumps(latest))
        try:
            start(['roscore','-p',str(port)])
            def master_ready():
                try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('test')[0]==1
                except OSError:return False
            wait(master_ready,'master',15)
            import numpy as np
            import rospy
            from diagnostic_msgs.msg import DiagnosticArray
            from std_srvs.srv import Trigger
            from dm_motor_fusion.msg import MultiLegCommand
            from feetech_servo.msg import servo_cmd
            from magnetic.msg import MagneticCmd
            from hex_kinematic.ros_contract import read_motor,read_magnetic
            rospy.init_node('initialization_test',disable_signals=True)
            def diagnostics(msg):
                for s in msg.status:
                    if s.name=='hex_runner':
                        latest.update({v.key:json.loads(v.value) for v in s.values});latest['phase']=s.message
                        stage=latest.get('initialization_stage')
                        if stage and (not stages or stages[-1]!=stage):stages.append(stage);print('INIT_STAGE',stage,flush=True)
            def motor(msg):
                modes.append(msg.option)
                if msg.option==0:
                    zero_valid[0] &= all(np.all(read_motor(msg,k)==0) for k in ['kp','kd','tau','vel'])
            def magnetic(msg):
                if not read_magnetic(msg).any():magnet_off[0]=True
            subs=[rospy.Subscriber('/hex/runner/diagnostics',DiagnosticArray,diagnostics),
                  rospy.Subscriber('/hex/motor/command',MultiLegCommand,motor),
                  rospy.Subscriber('/hex/servo/command',servo_cmd,lambda msg:servo_count.__setitem__(0,servo_count[0]+1)),
                  rospy.Subscriber('/hex/magnetic/command',MagneticCmd,magnetic)]
            data=yaml.safe_load((root/'src/hex_runner/config'/(args.mode+'.yaml')).read_text())
            data['workspace_root']=str(root);data['initialization']={'method':method,'staged':staged}
            if args.mode=='complex' and method=='set_init':
                climb=yaml.safe_load((root/'src/hex_runner/config/climb.yaml').read_text())
                for k in ['mode','frame','feet']:data['initial_state'][k]=climb['initial_state'][k]
            config=Path(directory)/'runner.yaml';config.write_text(yaml.safe_dump(data))
            sim_info=Path(directory)/'to_sim_info.yaml'
            start(['rosrun','hex_runner','runner_node.py','--config',str(config),'--to-sim-info',str(sim_info)])
            wait(lambda:latest.get('expert_type')==args.mode,'runner created')
            rospy.wait_for_service('/hex/runner/reset',timeout=15)
            def reset():
                response=rospy.ServiceProxy('/hex/runner/reset',Trigger)()
                assert response.success,response.message
            if staged:
                reset();wait(lambda:len(modes)>=3 and magnet_off[0],'zero commands before devices',10)
                assert set(modes)=={0} and servo_count[0]==0 and zero_valid[0]
            if args.backend=='fake':
                start([sys.executable,str(root/'src/hex_runner/scripts/test_ros_integration.py'),
                       '--fake-device','--mode',args.mode,'--to-sim-info',str(sim_info)])
            else:
                backend=root/'src/hex_isaacgym/config'/('plane.yaml' if args.mode=='climb' else 'complex_surface4.yaml')
                start(['rosrun','hex_isaacgym','sim_node.py','--config',str(backend),'--to-sim-info',str(sim_info),
                       '--device','cpu','--headless','true'])
            wait(lambda:latest.get('feedback_ready'),'device feedback')
            if not staged:reset()
            wait(lambda:latest.get('phase') in ['READY','FAULT'],'initialization completion')
            assert latest['phase']=='READY',latest
            wait(lambda:1 in modes,'mode1 target command',5)
            assert set(modes)<={0,1} and zero_valid[0]
            if staged:assert {'LIFT','MOVE_TARGET','WAIT_ATTACH','COMPLETE'}<=set(stages),stages
            result=dict(status='PASS',mode=args.mode,method=method,staged=staged,backend=args.backend,
                        modes=sorted(set(modes)),zero_commands=modes.count(0),stages=stages,diagnostics=latest)
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2),flush=True)
        except Exception:
            log.flush();log.seek(0);print(log.read(),flush=True)
            print('LATEST',json.dumps(latest),flush=True)
            raise
        finally:
            for p in reversed(processes):
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGINT)
                    try:p.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.flush()
            args.report.with_suffix('.process.log').write_text(Path(directory+'/process.log').read_text())
            log.close()

if __name__=='__main__':main()
