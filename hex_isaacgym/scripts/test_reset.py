#!/usr/bin/env python3
"""Verify exact physical reset after movement, including non-symmetric adhesion."""
import argparse
import json
from pathlib import Path
import numpy as np
import rospkg
from hex_isaacgym.config import SimulationConfig
from hex_isaacgym.backend import IsaacGymBackend
from hex_isaacgym.types import ActuatorCommand


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    config=SimulationConfig.from_yaml(Path(rospkg.RosPack().get_path('hex_isaacgym'))/'config/plane.yaml',args.device,Path(rospkg.RosPack().get_path('hex_runner'))/'config/climb.yaml')
    initial=config.initial_state
    magnets=np.array([True,False,True,False,False,True])
    with IsaacGymBackend(config.backend,headless=True,initial_state=initial) as backend:
        command=ActuatorCommand(initial.q+.01,np.ones((6,3)),~magnets)
        for _ in range(5):backend.step(command)
        before=backend.get_state().sim_time
        for _ in range(2):
            state=backend.reset(initial,magnets)
            np.testing.assert_allclose(state.W_T_R,initial.W_T_R,atol=2e-6)
            np.testing.assert_allclose(state.q,initial.q,atol=2e-6)
            np.testing.assert_allclose(state.q_dot,initial.q_dot,atol=2e-6)
            np.testing.assert_allclose(state.q_torque,0)
            np.testing.assert_allclose(state.contact_forces,0)
            np.testing.assert_allclose(state.adhesion_force,magnets*config.backend.suction_force_max)
            force=backend.rb_forces[0,backend.magnetic_indices.reshape(-1),2].view(6,3).cpu().numpy()
            np.testing.assert_allclose(force,np.repeat(-magnets.astype(float)[:,None]*config.backend.suction_force_max/3.,3,axis=1))
            assert state.sim_time==before and state.run_elapsed==0
            next_state=backend.step(ActuatorCommand(initial.q,np.zeros((6,3)),magnets))
            assert next_state.sim_time>state.sim_time
            before=next_state.sim_time
        np.testing.assert_allclose(backend.reset(initial).adhesion_force,0)
        flags=np.array([True,False,True,False,False,True])
        targets=initial.q.copy();targets[:,3]+=.1
        backend.step(ActuatorCommand(targets,np.zeros((6,3)),magnets,servo_enabled=flags))
        indices=backend.command_indices.view(6,4)[:,3]
        torques=backend.last_torques[indices].cpu().numpy()
        np.testing.assert_array_equal(torques[~flags],0.)
        assert np.any(np.abs(torques[flags])>1e-6),torques
    report=dict(status='PASS',device=args.device,checks=['pose/q_init/q_dot','immediate adhesion_init forces',
                'cleared torque/contact history','monotonic sim time','post-reset stepping','default adhesion off','per-leg servo zero torque and active PD'])
    if args.report:args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
