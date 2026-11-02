"""Independent discrete actuator oracles and aerodynamic validation inputs."""
from pathlib import Path
import sys
import numpy as np
from scipy.io import loadmat, savemat
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from nearspace.config import load_config
from nearspace.model6 import VehicleParams
from nearspace.cmg import CMGParams, CMGCluster
from nearspace.torque_gap_authority import aero_moment_of_deflection

def main():
    cfg=load_config(str(ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml'))
    v,c=VehicleParams.from_config(cfg),CMGParams.from_config(cfg)
    dt=0.001; t=np.arange(8001)*dt
    cmd=np.zeros((len(t),4)); cmd[t>=.1]=[2, .2, -.2, -2]; cmd[t>=4]=[-2,-.2,.2,2]
    rate=np.zeros_like(cmd); gamma=np.zeros_like(cmd); gamma[0]=[2*np.pi-.02,0,0,0]
    accel=np.zeros_like(cmd)
    for k in range(len(t)):
        accel[k]=np.clip((np.clip(cmd[k],-c.gdot_max,c.gdot_max)-rate[k])/c.tau_g,-c.gddot_max,c.gddot_max)
        if k+1<len(t):
            rate[k+1]=np.clip(rate[k]+dt*accel[k],-c.gdot_max,c.gdot_max)
            gamma[k+1]=gamma[k]+dt*rate[k]
    scmd=np.zeros((len(t),3)); scmd[t>=.1]=2*v.delta_max; scmd[t>=4]=-2*v.delta_max
    delta=np.zeros_like(scmd); ddelta=np.zeros_like(scmd)
    for k in range(len(t)):
        ddelta[k]=np.clip((np.clip(scmd[k],-v.delta_max,v.delta_max)-delta[k])/v.tau_s,-v.delta_dot_max,v.delta_dot_max)
        if k+1<len(t): delta[k+1]=np.clip(delta[k]+dt*ddelta[k],-v.delta_max,v.delta_max)
    out=ROOT/'simulink/data/native_actuators';out.mkdir(parents=True,exist_ok=True)
    cl=CMGCluster(c);gamma_zero=gamma-gamma[0]
    momentum=np.array([cl.momentum(g) for g in gamma_zero])
    torque=np.array([cl.torque(g,n) for g,n in zip(gamma_zero,rate)])
    aero=np.array([aero_moment_of_deflection(1000,.1,.02,[.01,.02,.03],1000,d,v) for d in delta])
    savemat(out/'reference.mat',dict(t=t,dt=dt,cmg_cmd=cmd,nu=rate,gamma=gamma,accel=accel,
        surface_cmd=scmd,delta=delta,delta_dot=ddelta,gamma_zero=gamma_zero,H_integrated=momentum,
        torque_integrated=torque,aero_integrated=aero))
    assert gamma[:,0].max()>2*np.pi and abs(rate).max()<=c.gdot_max
    assert abs(accel).max()<=c.gddot_max and np.all(abs(delta)<=v.delta_max+1e-12)
    print('ACTUATOR_REFERENCE_PASS samples=',len(t))
if __name__=='__main__':main()
