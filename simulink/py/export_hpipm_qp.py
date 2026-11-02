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
    case=ROOT/'configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml'
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
        b=np.zeros((rti.nx,rti.N)),dts=rti.dts)
    savemat(out/'production_qp.mat',data)
    report=dict(status=status,N=rti.N,nx=rti.nx,nu=rti.nu,iterations=iters,objective=objective,residuals=residuals.tolist(),time_s=elapsed)
    (out/'python_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
