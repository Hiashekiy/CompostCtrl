"""Freeze an actual production RTI linearization and its Python HPIPM solution."""
from pathlib import Path
import sys,json,time
import numpy as np
from scipy.io import savemat
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nearspace.torque_gap_ocp import _casadi
_casadi()
from nearspace.config import load_config
from nearspace.torque_gap_scenario import Scenario
from nearspace.torque_gap_rti import TorqueGapRTI

def main():
    arm='A' if '--surface-only' in sys.argv else 'B'
    case=ROOT/('configs/torque_gap_s08/cases/A4_descent_replay_S00_'+('no_cmg' if arm=='A' else 'with_cmg')+'.yaml')
    if '--config' in sys.argv:case=Path(sys.argv[sys.argv.index('--config')+1]).resolve()
    scn=Scenario(load_config(str(case))); rti=TorqueGapRTI.from_config(scn)
    x=scn.initial_state(); plan=scn.plan(scn.t_start,x)
    U=np.zeros((rti.nu,rti.N)); U=rti._clip(U,plan); X=rti._rollout(x,U,plan)
    sig=rti._sigma_profile(X)
    Q,q,R,r,lu,uu=rti._qp_matrices(X,U,plan,sig)
    A=np.array([np.asarray(rti._A(X[:,k],U[:,k],plan['cond']['tau_d'][k],rti.dts[k])) for k in range(rti.N)])
    B=np.array([np.asarray(rti._B(X[:,k],U[:,k],plan['cond']['tau_d'][k],rti.dts[k])) for k in range(rti.N)])
    start=time.perf_counter();dX,dU,status,iters=rti._iterate(X,U,plan,sig);elapsed=time.perf_counter()-start
    residuals=np.array([rti._solver.get(s) for s in ['max_res_stat','max_res_eq','max_res_ineq','max_res_comp']]).ravel()
    assert status==0,(status,residuals)
    objective=sum(.5*dX[:,k]@Q[k]@dX[:,k]+q[k]@dX[:,k] for k in range(rti.N+1))
    objective+=sum(.5*dU[:,k]@R[k]@dU[:,k]+r[k]@dU[:,k] for k in range(rti.N))
    out=ROOT/'simulink/tests/05_python_matlab_qp_equivalence';out.mkdir(parents=True,exist_ok=True)
    data=dict(N=rti.N,nx=rti.nx,nu=rti.nu,A=A.transpose(1,2,0),B=B.transpose(1,2,0),
        Q=Q.transpose(1,2,0),q=q.T,R=R.transpose(1,2,0),r=r.T,lu=lu.T,uu=uu.T,
        tr_J=rti._Jx_tr,tr_bound=rti._tr_bound_vec,dX_python=dX,dU_python=dU,
        status_python=status,objective_python=objective,residuals_python=residuals,time_python=elapsed,
        b=np.zeros((rti.nx,rti.N)),dts=rti.dts,X_nom=X,U_nom=U,
        q_ref=np.asarray(plan['cond']['q_ref']).T,omega_ref=np.asarray(plan['cond']['omega_ref']).T,
        tau_d=np.asarray(plan['cond']['tau_d']).T,tau_min=np.asarray(plan['box']['tau_min']).T,
        tau_max=np.asarray(plan['box']['tau_max']).T,weights=vars(rti.w),gdot=scn.prm.cmg.gdot_max,
        gddot=scn.prm.cmg.gddot_max,dt_ctrl=scn.dt_ctrl,J=scn.prm.J,
        tau_nom=np.asarray(plan['mod']['tau_nom']).T,gap=np.asarray(plan['mod']['d']).T,
        h0=scn.prm.cmg.h0,s0=scn.prm.cluster().s0,gxs0=scn.prm.cluster().gxs0)
    full=rti.solve(plan,x)
    data['online_command_python']=full.u0
    import os
    generated=ROOT/'simulink/hpipm/generated';generated.mkdir(parents=True,exist_ok=True)
    previous=Path.cwd()
    try:
        os.chdir(generated)
        for name,fun in [('native_F_'+arm,rti._F),('native_A_'+arm,rti._A),('native_B_'+arm,rti._B),('native_E_'+arm,rti._eof)]:
            fun.generate(name+'.c',{'mex':True})
    finally:os.chdir(previous)
    savemat(out/('production_online_A_qp.mat' if arm=='A' else 'production_online_qp.mat'),data)
    report=dict(status=status,N=rti.N,nx=rti.nx,nu=rti.nu,iterations=iters,objective=objective,residuals=residuals.tolist(),time_s=elapsed)
    (out/('python_report_online_'+arm+'.json')).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
